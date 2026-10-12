#include "control/pid.h"

namespace ctl {

namespace {
constexpr float kTwoPi = 6.28318531f;
}

Pid::Pid(PidGains g, float out_min, float out_max, float i_limit, float d_cutoff_hz)
    : g_(g), out_min_(out_min), out_max_(out_max), i_limit_(i_limit), d_cutoff_hz_(d_cutoff_hz) {}

void Pid::reset() {
  i_ = 0;
  d_ = 0;
  first_ = true;
}

float Pid::update(float setpoint, float measured, float dt) {
  if (dt <= 0) return clampf(g_.kp * (setpoint - measured) + i_, out_min_, out_max_);
  const float err = setpoint - measured;
  // derivative on measurement (no kick on setpoint steps), low-pass filtered
  float d_raw = 0;
  if (!first_) d_raw = -(measured - prev_meas_) / dt;
  prev_meas_ = measured;
  first_ = false;
  const float alpha = (kTwoPi * d_cutoff_hz_ * dt) / (1.0f + kTwoPi * d_cutoff_hz_ * dt);
  d_ += alpha * (d_raw - d_);

  const float p = g_.kp * err;
  const float unsat = p + i_ + g_.kd * d_;
  const bool sat_hi = unsat > out_max_;
  const bool sat_lo = unsat < out_min_;
  // conditional integration: do not wind further into saturation
  if (!((sat_hi && err > 0) || (sat_lo && err < 0))) {
    i_ = clampf(i_ + g_.ki * err * dt, -i_limit_, i_limit_);
  }
  return clampf(p + i_ + g_.kd * d_, out_min_, out_max_);
}

}  // namespace ctl
