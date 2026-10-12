#include "control/expansion_loop.h"

#include <cmath>

#include "bot_params.h"
#include "control/kinematics.h"
#include "control/pid.h"

namespace ctl {

namespace {
constexpr float kAngleTol = 0.2f * kin::kDeg;
constexpr float kTwoPi = 6.28318531f;
}  // namespace

float ExpansionLoop::diameter() const { return kin::diameterForTheta(theta_); }

float ExpansionLoop::gripMin() const { return std::fmin(grip_[0], std::fmin(grip_[1], grip_[2])); }

float ExpansionLoop::gripAvg() const { return (grip_[0] + grip_[1] + grip_[2]) / 3.0f; }

bool ExpansionLoop::allContact() const {
  return slide_[0] > params::CONTACT_SLIDE_MM && slide_[1] > params::CONTACT_SLIDE_MM && slide_[2] > params::CONTACT_SLIDE_MM;
}

void ExpansionLoop::clearFaults() {
  stall_ = over_current_ = no_contact_ = false;
  stall_s_ = oc_s_ = 0;
}

void ExpansionLoop::drive(const hal::Hardware& hw, float omega_target, float dt, float current_scale) {
  omega_ = slew(omega_, omega_target, params::EXPANSION_ACCEL_RAD_S2, dt);  // soft start / stop
  if (std::fabs(omega_) < 1e-6f && omega_target == 0.0f) omega_ = 0.0f;
  const bool want = omega_ != 0.0f;
  if (want != enabled_) {
    hw.stepper[3]->setCurrent((current_a_ > 0 ? current_a_ : params::EXPANSION_CURRENT_A) * current_scale, 0.0f);
    hw.stepper[3]->enable(want);
    hw.safety->expandEnable(want);
    enabled_ = want;
  }
  hw.stepper[3]->setVelocity(omega_ * params::EXPANSION_USTEPS_PER_RAD);
}

void ExpansionLoop::step(const hal::Hardware& hw, const ExpCmd& cmd, float dt) {
  // --- sense
  float a = 0;
  enc_ok_ = hw.lever_enc->read(a);
  if (enc_ok_) {
    if (a > kTwoPi * 0.5f) a -= kTwoPi;  // lever never passes +/-180 deg
    theta_ = a;
  }
  for (int i = 0; i < 3; ++i) {
    float s = 0;
    if (hw.sensors->gripSlideMm(i, s)) slide_[i] = s;
    grip_[i] = kin::gripForce(slide_[i]);
  }
  const hal::PowerData pw = hw.power->read();
  oc_s_ = (pw.expand_a > params::EXP_OVER_CURRENT_A) ? oc_s_ + dt : 0.0f;
  if (oc_s_ > 0.05f) over_current_ = true;

  if (cmd.mode != last_mode_) {
    done_ = false;
    no_contact_ = false;
    maintain_active_ = false;
    last_mode_ = cmd.mode;
  }
  const float wmax = params::EXPANSION_MAX_RAD_S;
  float w = 0;
  const bool fault = over_current_ || stall_ || !enc_ok_;
  if (!fault) {
    switch (cmd.mode) {
      case ExpMode::LOCKED:
        done_ = true;
        break;
      case ExpMode::DIAMETER:
      case ExpMode::RETRACT: {
        const float tgt = cmd.mode == ExpMode::RETRACT ? params::THETA_MIN_RAD : kin::thetaForDiameter(cmd.target);
        const float err = tgt - theta_;
        if (std::fabs(err) < kAngleTol) {
          done_ = true;
        } else {
          // proportional approach with a speed cap (accel-limited in drive())
          w = clampf(err * 4.0f, -wmax, wmax);
          if (std::fabs(w) < 0.05f * wmax) w = err > 0 ? 0.05f * wmax : -0.05f * wmax;
        }
        // never crush: stop extending once any arm is at the force limit
        if (w > 0 && (grip_[0] > params::MAX_GRIP_N || grip_[1] > params::MAX_GRIP_N || grip_[2] > params::MAX_GRIP_N)) {
          w = 0;
          done_ = true;
        }
        break;
      }
      case ExpMode::FORCE:
      case ExpMode::MAINTAIN: {
        const float tgt = clampf(cmd.target, params::SPRING_PRELOAD_N + 1.0f, params::MAX_GRIP_N);
        const float fmax = std::fmax(grip_[0], std::fmax(grip_[1], grip_[2]));
        if (cmd.mode == ExpMode::MAINTAIN) {
          // only act when out of band for a while (worms stay locked otherwise)
          const bool low = !allContact() || gripAvg() < tgt - params::REGRIP_DEADBAND_N || gripMin() < params::GRIP_LOW_N;
          const bool high = fmax > 0.95f * params::MAX_GRIP_N;
          maintain_s_ = (low || high) ? maintain_s_ + dt : 0.0f;
          if (maintain_s_ > 0.3f) maintain_active_ = true;
          if (!maintain_active_) {
            done_ = true;
            break;
          }
        }
        if (!allContact()) {
          if (theta_ >= params::THETA_MAX_RAD - kAngleTol) {
            no_contact_ = true;
            done_ = true;
            maintain_active_ = false;
          } else {
            w = wmax;  // approach at full speed; the force servo takes over at contact
          }
        } else {
          const float err = tgt - gripAvg();
          if (fmax > params::MAX_GRIP_N) {
            w = -0.1f * wmax;
          } else if (std::fabs(err) < 0.5f * params::REGRIP_DEADBAND_N) {
            done_ = true;
            maintain_active_ = false;
          } else {
            w = clampf(k_force_ * err, -0.3f * wmax, 0.3f * wmax);
          }
        }
        if (theta_ <= params::THETA_MIN_RAD && w < 0) w = 0;
        if (theta_ >= params::THETA_MAX_RAD && w > 0) w = 0;
        break;
      }
    }
  }
  drive(hw, w, dt, cmd.current_scale);

  // --- stall: motor turning but the lever not following (missed steps / jam)
  if (have_prev_ && dt > 0 && std::fabs(omega_) > 0.3f * wmax) {
    const float rate = (theta_ - prev_theta_) / dt;
    const bool lag = std::fabs(rate) < 0.2f * std::fabs(omega_);
    stall_s_ = lag ? stall_s_ + dt : 0.0f;
  } else {
    stall_s_ = 0;
  }
  if (stall_s_ > 0.3f || hw.stepper[3]->status().stalled) {
    stall_ = true;
    omega_ = 0;
    hw.stepper[3]->setVelocity(0);
  }
  prev_theta_ = theta_;
  have_prev_ = enc_ok_;
}

}  // namespace ctl
