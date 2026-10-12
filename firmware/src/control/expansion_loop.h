// Expansion loop @ EXPANSION_LOOP_HZ: lever angle from the absolute AS5600 on the
// lever pivot (no homing), worm-sector drive in velocity mode.
//   DIAMETER  go to the lever angle for a pipe ID
//   FORCE     extend until all three arms touch, then servo the spring force
//   RETRACT   fully in
//   MAINTAIN  while gripped: hold, and re-grip if the pipe ID changes under the robot
// Soft start (accel-limited), stops (and locks) when done: the self-locking worms hold.
// Aborts on over-current, stall (lever not following the motor) or over-force.
#pragma once
#include <cstdint>

#include "hal/hal.h"

namespace ctl {

enum class ExpMode : uint8_t { LOCKED, DIAMETER, FORCE, RETRACT, MAINTAIN };

struct ExpCmd {
  ExpMode mode = ExpMode::LOCKED;
  float target = 0;  // mm or N
  float current_scale = 1.0f;
};

class ExpansionLoop {
 public:
  void step(const hal::Hardware& hw, const ExpCmd& cmd, float dt);
  float theta() const { return theta_; }
  float diameter() const;
  float grip(int arm) const { return grip_[arm]; }
  float slide(int arm) const { return slide_[arm]; }
  const float* slides() const { return slide_; }
  float gripMin() const;
  float gripAvg() const;
  bool allContact() const;
  bool done() const { return done_; }
  bool stalled() const { return stall_; }
  bool overCurrent() const { return over_current_; }
  bool noContact() const { return no_contact_; }
  bool encoderOk() const { return enc_ok_; }
  bool moving() const { return omega_ != 0.0f; }
  void clearFaults();
  void setForceGain(float k) { k_force_ = k; }
  float forceGain() const { return k_force_; }
  void setCurrentA(float a) { current_a_ = a; }

 private:
  float theta_ = 0, omega_ = 0, prev_theta_ = 0;
  float grip_[3] = {0, 0, 0}, slide_[3] = {0, 0, 0};
  bool done_ = true, stall_ = false, over_current_ = false, no_contact_ = false, enc_ok_ = true, have_prev_ = false;
  float stall_s_ = 0, oc_s_ = 0, maintain_s_ = 0;
  bool maintain_active_ = false;
  float k_force_ = 0.004f;  // rad/s per N of force error
  float current_a_ = 0;
  ExpMode last_mode_ = ExpMode::LOCKED;
  bool enabled_ = false;
  void drive(const hal::Hardware& hw, float omega_target, float dt, float current_scale);
};

}  // namespace ctl
