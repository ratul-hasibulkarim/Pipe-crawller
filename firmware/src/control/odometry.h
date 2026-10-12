// Odometry fusion: three wheel encoders (one per arm, through the self-locking drive)
// + front ToF range-rate + IMU. Wheel encoders measure wheel rotation, not ground
// motion, so slip shows up as (a) arms disagreeing or (b) ToF range-rate disagreeing.
#pragma once
#include <cstdint>

namespace ctl {

struct OdomInput {
  float arm_speed_mm_s[3];   // from wheel encoders
  bool arm_ok[3];
  bool tof_ok;
  float tof_mm;               // front range to a static target (pipe end / obstacle)
  float dt;
};

class Odometry {
 public:
  void reset();
  void update(const OdomInput& in);
  float distanceM() const { return dist_mm_ * 0.001f; }
  float speedMmS() const { return v_mm_s_; }
  float confidence() const { return conf_; }
  bool groundSlip() const { return ground_slip_; }

 private:
  float dist_mm_ = 0, v_mm_s_ = 0, conf_ = 1.0f;
  float tof_prev_ = 0, tof_rate_ = 0;
  bool tof_init_ = false, ground_slip_ = false;
  float slip_timer_ = 0;
};

}  // namespace ctl
