// Arm geometry: pipe diameter <-> lever angle, grip force from the rocker spring,
// body offset (axis sag) from the three slide displacements, bend-assist speeds.
// Mirrors calc/mechanism.py; constants come from bot_params.h.
#pragma once
#include <cmath>

#include "bot_params.h"

namespace kin {

constexpr float kPi = 3.14159265f;
constexpr float kDeg = kPi / 180.0f;

inline float diameterForTheta(float theta) {
  using namespace params;
  return 2.0f * (PIVOT_R_MM + LEVER_LEN_MM * std::sin(theta) + ROCKER_OFFSET_MM + WHEEL_DIA_MM * 0.5f);
}

inline float thetaForDiameter(float d_mm) {
  using namespace params;
  float s = (d_mm * 0.5f - WHEEL_DIA_MM * 0.5f - ROCKER_OFFSET_MM - PIVOT_R_MM) / LEVER_LEN_MM;
  s = s < -1.0f ? -1.0f : (s > 1.0f ? 1.0f : s);
  float th = std::asin(s);
  if (th < THETA_MIN_RAD) th = THETA_MIN_RAD;
  if (th > THETA_MAX_RAD) th = THETA_MAX_RAD;
  return th;
}

// Spring force on one arm from its slide displacement (0 = rocker on the outer stop).
inline float gripForce(float slide_mm) {
  using namespace params;
  if (slide_mm <= CONTACT_SLIDE_MM) return 0.0f;  // below preload: contact unknown -> report 0
  return SPRING_PRELOAD_N + SPRING_RATE_N_MM * slide_mm;
}

// Body (axis) offset from the slide displacements: sum(s_i u_i) / 1.5 for 3 arms at 120 deg.
inline float axisOffsetMm(const float slide_mm[3]) {
  float x = 0, y = 0;
  for (int i = 0; i < 3; ++i) {
    const float a = static_cast<float>(i) * 120.0f * kDeg;
    x += slide_mm[i] * std::cos(a);
    y += slide_mm[i] * std::sin(a);
  }
  return std::sqrt(x * x + y * y) / 1.5f;
}

// Bend assist: arm i on the inside of a bend travels a shorter path.
// v_i = v - omega * r_contact * cos(phi_i - bend_dir)
inline float armSpeed(int arm, float v_mm_s, float turn_deg_s, float bend_dir_deg, float diameter_mm) {
  const float phi = static_cast<float>(arm) * 120.0f;
  const float omega = turn_deg_s * kDeg;
  return v_mm_s - omega * (diameter_mm * 0.5f) * std::cos((phi - bend_dir_deg) * kDeg);
}

}  // namespace kin
