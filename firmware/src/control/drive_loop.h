// Drive loop @ DRIVE_LOOP_HZ: closed-loop wheel speed per arm (AS5600 wheel encoder +
// PID trim on top of a step-rate feed-forward), accel limiting, bend assist, holding
// current, and SLIP_WARNING (commanded vs measured mismatch for > SLIP_DETECT_MS
// while the driver is at its limit = stall / missed steps).
#pragma once
#include <cstdint>

#include "control/pid.h"
#include "hal/hal.h"

namespace ctl {

enum class DriveMode : uint8_t { OFF, HOLD, RUN };

struct DriveCmd {
  DriveMode mode = DriveMode::OFF;
  float v_mm_s = 0;
  float turn_deg_s = 0;
  float bend_dir_deg = 0;
  float diameter_mm = 0;
  float speed_scale = 1.0f;   // thermal derate
  float current_scale = 1.0f;
};

class DriveLoop {
 public:
  DriveLoop();
  void setGains(const PidGains& g);
  const PidGains& gains() const { return pid_[0].gains(); }
  void setWheelDia(float mm) { wheel_dia_mm_ = mm; }
  float wheelDia() const { return wheel_dia_mm_; }
  void setCurrentA(float a) { current_a_ = a; }
  void step(const hal::Hardware& hw, const DriveCmd& cmd, float dt);
  float measuredSpeed(int arm) const { return v_meas_[arm]; }
  bool encoderOk(int arm) const { return enc_ok_[arm]; }
  float commandedSpeed(int arm) const { return v_ramp_[arm]; }
  bool slipWarning() const { return slip_; }
  void clearSlip();

 private:
  Pid pid_[hal::kArms];
  float v_ramp_[hal::kArms] = {0, 0, 0};
  float v_meas_[hal::kArms] = {0, 0, 0};
  float last_angle_[hal::kArms] = {0, 0, 0};
  float win_angle_[hal::kArms] = {0, 0, 0};  // unwrapped angle accumulated over the speed window
  float win_t_ = 0;
  bool have_angle_[hal::kArms] = {false, false, false};
  bool enc_ok_[hal::kArms] = {true, true, true};
  float mismatch_s_[hal::kArms] = {0, 0, 0};
  bool slip_ = false;
  float wheel_dia_mm_;
  float current_a_;
  DriveMode last_mode_ = DriveMode::OFF;
};

}  // namespace ctl
