#include "comm/telemetry.h"

#include <cmath>

#include "bot_params.h"
#include "control/kinematics.h"

namespace comm {

uint8_t TelemetryBuilder::batteryPercent(float pack_v) {
  if (params::BATTERY_SERIES <= 0 || pack_v <= 0) return 0;
  const float cell = pack_v / static_cast<float>(params::BATTERY_SERIES);
  // Li-ion OCV approximation between 3.3 V (0 %) and 4.15 V (100 %)
  float pct = (cell - 3.3f) / (4.15f - 3.3f) * 100.0f;
  pct = pct < 0 ? 0 : (pct > 100 ? 100 : pct);
  return static_cast<uint8_t>(pct + 0.5f);
}

proto::TelemetryMsg TelemetryBuilder::build(const TelemetryInputs& in) const {
  proto::TelemetryMsg t;
  t.t_ms = in.t_ms;
  t.state = static_cast<uint8_t>(in.state);
  t.faults = in.faults;
  t.est_diameter_mm = in.exp->diameter();
  t.lever_angle_deg = in.exp->theta() / kin::kDeg;
  for (int i = 0; i < 3; ++i) t.grip_force_n[i] = in.exp->grip(i);
  t.axis_offset_mm = kin::axisOffsetMm(in.exp->slides());
  t.linear_mm_s = in.odom->speedMmS();
  t.distance_m = in.odom->distanceM();
  t.odom_confidence = in.odom->confidence();
  t.pitch = in.imu.pitch_deg;
  t.roll = in.imu.roll_deg;
  t.yaw = in.imu.yaw_deg;
  t.pipe_orientation_deg = std::fabs(in.imu.pitch_deg);
  const float drive_each = in.power.drive_a / 3.0f;
  for (int i = 0; i < 3; ++i) t.motor_current_a[i] = in.drive->commandedSpeed(i) != 0.0f ? drive_each : drive_each * 0.5f;
  t.motor_current_a[3] = in.power.expand_a;
  t.driver_temp_c = in.power.driver_temp_c;
  t.battery_v = in.power.mode == proto::SupplyMode::BATTERY ? in.power.battery_v : in.power.bus_v;
  t.battery_pct = batteryPercent(in.power.battery_v);
  t.battery_current_a = in.power.drive_a + in.power.expand_a;
  t.supply_mode = static_cast<uint8_t>(in.power.mode);
  t.temp_c = in.env.temp_c;
  t.humidity_pct = in.env.humidity_pct;
  t.pressure_kpa = in.env.pressure_kpa;
  t.gas_ppm = in.env.gas_ppm;
  t.tof_front_mm = in.tof_front_ok ? in.tof_front : 0xFFFF;
  t.tof_rear_mm = in.tof_rear_ok ? in.tof_rear : 0xFFFF;
  t.pan_deg = in.pan;
  t.tilt_deg = in.tilt;
  t.white_pct = in.white;
  t.ir_pct = in.ir;
  t.last_seq = in.last_seq;
  t.rx_errors = in.rx_errors;
  return t;
}

}  // namespace comm
