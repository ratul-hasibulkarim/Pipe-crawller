// PID with anti-windup (clamped + conditional integration), output clamp and a
// first-order filtered derivative on measurement.
#pragma once

namespace ctl {

struct PidGains {
  float kp = 0, ki = 0, kd = 0;
};

class Pid {
 public:
  Pid(PidGains g, float out_min, float out_max, float i_limit, float d_cutoff_hz);
  float update(float setpoint, float measured, float dt);
  void reset();
  void setGains(const PidGains& g) { g_ = g; }
  const PidGains& gains() const { return g_; }
  float integral() const { return i_; }

 private:
  PidGains g_;
  float out_min_, out_max_, i_limit_, d_cutoff_hz_;
  float i_ = 0, d_ = 0, prev_meas_ = 0;
  bool first_ = true;
};

// Clamp helper used across the controllers.
inline float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }

// Rate limiter: moves `current` toward `target` by at most rate*dt.
inline float slew(float current, float target, float rate, float dt) {
  const float step = rate * dt;
  if (target > current + step) return current + step;
  if (target < current - step) return current - step;
  return target;
}

}  // namespace ctl
