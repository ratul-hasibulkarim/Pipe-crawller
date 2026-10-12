#include "control/drive_loop.h"

#include <cmath>

#include "bot_params.h"
#include "control/kinematics.h"

namespace ctl {

namespace {
constexpr float kTwoPi = 6.28318531f;
// Default trim gains: output is a speed correction in mm/s added to the feed-forward.
constexpr PidGains kDefaultGains{0.6f, 4.0f, 0.0f};
}  // namespace

DriveLoop::DriveLoop()
    : pid_{Pid(kDefaultGains, -0.3f * params::MAX_SPEED_MM_S, 0.3f * params::MAX_SPEED_MM_S, 0.3f * params::MAX_SPEED_MM_S, 30.0f),
           Pid(kDefaultGains, -0.3f * params::MAX_SPEED_MM_S, 0.3f * params::MAX_SPEED_MM_S, 0.3f * params::MAX_SPEED_MM_S, 30.0f),
           Pid(kDefaultGains, -0.3f * params::MAX_SPEED_MM_S, 0.3f * params::MAX_SPEED_MM_S, 0.3f * params::MAX_SPEED_MM_S, 30.0f)},
      wheel_dia_mm_(params::WHEEL_DIA_MM),
      current_a_(params::DRIVE_CURRENT_A) {}

void DriveLoop::setGains(const PidGains& g) {
  for (auto& p : pid_) p.setGains(g);
}

void DriveLoop::clearSlip() {
  slip_ = false;
  for (float& m : mismatch_s_) m = 0;
}

void DriveLoop::step(const hal::Hardware& hw, const DriveCmd& cmd, float dt) {
  const float mm_per_rad = wheel_dia_mm_ * 0.5f;
  // 1) measure wheel speeds: unwrap the absolute encoder, differentiate over a 10 ms window
  //    (AS5600 LSB noise / 1 ms would be ~20 mm/s of jitter)
  constexpr float kWindow = 0.010f;
  win_t_ += dt;
  const bool window_done = win_t_ >= kWindow;
  for (int i = 0; i < hal::kArms; ++i) {
    float a = 0;
    enc_ok_[i] = hw.wheel_enc[i]->read(a);
    if (!enc_ok_[i]) {
      have_angle_[i] = false;
      win_angle_[i] = 0;
      continue;
    }
    if (have_angle_[i]) {
      float da = a - last_angle_[i];
      if (da > kTwoPi * 0.5f) da -= kTwoPi;
      if (da < -kTwoPi * 0.5f) da += kTwoPi;
      win_angle_[i] += da;
    }
    if (window_done) {
      const float v = win_angle_[i] * mm_per_rad / win_t_;
      v_meas_[i] += 0.5f * (v - v_meas_[i]);
      win_angle_[i] = 0;
    }
    last_angle_[i] = a;
    have_angle_[i] = true;
  }
  if (window_done) win_t_ = 0;

  const bool run = cmd.mode == DriveMode::RUN;
  const bool hold = cmd.mode == DriveMode::HOLD;
  if (cmd.mode != last_mode_) {
    for (int i = 0; i < hal::kArms; ++i) {
      hw.stepper[i]->enable(run || hold);
      hw.stepper[i]->setCurrent(current_a_ * cmd.current_scale, params::HOLD_CURRENT_FRAC);
    }
    last_mode_ = cmd.mode;
  }

  const float usteps_per_mm = params::MOTOR_USTEPS_PER_REV * static_cast<float>(params::DRIVE_WORM_RATIO) /
                              (kin::kPi * wheel_dia_mm_);
  bool any_slip = false;
  for (int i = 0; i < hal::kArms; ++i) {
    float target = 0;
    if (run) {
      const float vmax = params::MAX_SPEED_MM_S * cmd.speed_scale;
      target = clampf(kin::armSpeed(i, cmd.v_mm_s, cmd.turn_deg_s, cmd.bend_dir_deg, cmd.diameter_mm), -vmax, vmax);
    }
    v_ramp_[i] = slew(v_ramp_[i], target, params::MAX_ACCEL_MM_S2, dt);
    float out = v_ramp_[i];
    if (run && enc_ok_[i]) {
      out += pid_[i].update(v_ramp_[i], v_meas_[i], dt);
    } else {
      pid_[i].reset();
    }
    if (!run) out = 0;
    hw.stepper[i]->setVelocity(out * usteps_per_mm);

    // stall / slip warning: big mismatch for SLIP_DETECT_MS while the driver is at its limit
    const float err = std::fabs(v_ramp_[i] - v_meas_[i]);
    const bool mismatch = run && enc_ok_[i] && err > params::SLIP_VEL_ERR_FRAC * std::fabs(v_ramp_[i]) + 5.0f;
    const hal::StepperStatus st = hw.stepper[i]->status();
    const bool at_limit = st.stalled || std::fabs(pid_[i].integral()) > 0.25f * params::MAX_SPEED_MM_S;
    mismatch_s_[i] = (mismatch && at_limit) ? mismatch_s_[i] + dt : 0.0f;
    if (mismatch_s_[i] * 1000.0f > static_cast<float>(params::SLIP_DETECT_MS)) any_slip = true;
  }
  if (any_slip) slip_ = true;
}

}  // namespace ctl
