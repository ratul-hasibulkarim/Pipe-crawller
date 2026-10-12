#include "control/odometry.h"

#include <cmath>

#include "control/pid.h"

namespace ctl {

namespace {
float median3(float a, float b, float c) {
  if (a > b) {
    const float t = a;
    a = b;
    b = t;
  }
  if (b > c) b = c;
  return a > b ? a : b;
}
}  // namespace

void Odometry::reset() {
  dist_mm_ = 0;
  v_mm_s_ = 0;
  conf_ = 1.0f;
  tof_init_ = false;
  ground_slip_ = false;
  slip_timer_ = 0;
}

void Odometry::update(const OdomInput& in) {
  if (in.dt <= 0) return;
  // robust speed: median of healthy arms (one spinning / one stuck arm cannot fool it)
  float v[3];
  int n = 0;
  for (int i = 0; i < 3; ++i)
    if (in.arm_ok[i]) v[n++] = in.arm_speed_mm_s[i];
  float vm = 0, spread = 0;
  if (n == 3) {
    vm = median3(v[0], v[1], v[2]);
    for (int i = 0; i < 3; ++i) spread = std::fmax(spread, std::fabs(v[i] - vm));
  } else if (n == 2) {
    vm = 0.5f * (v[0] + v[1]);
    spread = std::fabs(v[0] - v[1]);
  } else if (n == 1) {
    vm = v[0];
  }
  v_mm_s_ = vm;
  dist_mm_ += vm * in.dt;

  // ToF range-rate (filtered); a static target ahead closes at -v
  bool tof_disagree = false;
  if (in.tof_ok) {
    if (tof_init_) {
      const float raw = (in.tof_mm - tof_prev_) / in.dt;
      tof_rate_ += 0.05f * (raw - tof_rate_);
      const float expect = -vm;
      if (std::fabs(vm) > 10.0f && std::fabs(tof_rate_ - expect) > 0.3f * std::fabs(vm) + 5.0f) tof_disagree = true;
    }
    tof_prev_ = in.tof_mm;
    tof_init_ = true;
  } else {
    tof_init_ = false;
  }

  const bool arms_disagree = std::fabs(vm) > 5.0f && spread > 0.25f * std::fabs(vm) + 5.0f;
  const bool suspicious = arms_disagree || tof_disagree || n < 2;
  slip_timer_ = suspicious ? slip_timer_ + in.dt : 0.0f;
  ground_slip_ = slip_timer_ > 0.3f;
  // confidence decays while suspicious, recovers slowly otherwise
  const float target = suspicious ? 0.3f : 1.0f;
  const float rate = suspicious ? 0.8f : 0.05f;
  conf_ = clampf(conf_ + (target - conf_) * clampf(rate * in.dt * 10.0f, 0.0f, 1.0f), 0.0f, 1.0f);
}

}  // namespace ctl
