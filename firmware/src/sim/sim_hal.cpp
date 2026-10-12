#include "sim/sim_hal.h"

#include <algorithm>
#include <cmath>

#include "bot_params.h"

namespace sim {

void SimStepper::setCurrent(float run_a, float) {
  run_a_ = run_a;
  const float nominal = i_ < 3 ? params::DRIVE_CURRENT_A : params::EXPANSION_CURRENT_A;
  p_.current_scale[i_] = nominal > 0 ? run_a / nominal : 1.0;
}

hal::StepperStatus SimStepper::status() {
  hal::StepperStatus s;
  s.stalled = (i_ < 3) ? p_.arm[i_].stalled : p_.exp_stalled;
  s.overtemp_warn = p_.driver_temp_c > 100;
  s.overtemp = p_.driver_temp_c > 140;
  return s;
}

bool SimAngle::read(float& rad) {
  if (fail || !p_.mcu_powered) return false;
  double a = arm_ < 0 ? p_.theta : p_.arm[arm_].wheel_angle;
  a += p_.noise(arm_ < 0 ? 0.0003 : 0.0015);  // AS5600 ~0.1 deg LSB + noise
  if (arm_ >= 0) {
    a = std::fmod(a, 6.283185307179586);
    if (a < 0) a += 6.283185307179586;
  }
  rad = static_cast<float>(a);
  return true;
}

hal::ImuData SimSensors::imu() {
  hal::ImuData d;
  d.ok = !imu_fail && p_.mcu_powered;
  d.pitch_deg = static_cast<float>(p_.sectionAt(p_.x_m).incline_deg + p_.noise(0.1));
  d.roll_deg = static_cast<float>(p_.cfg().roll_deg + p_.noise(0.1));
  d.yaw_deg = static_cast<float>(p_.noise(0.2));
  return d;
}

bool SimSensors::tofFront(uint16_t& mm) {
  const double nose_m = p_.x_m + params::BODY_LENGTH_MM * 1e-3;
  const double d = (p_.cfg().pipe_end_m - nose_m) * 1000.0 + p_.noise(3.0);
  if (d < 30 || d > 4000) return false;
  mm = static_cast<uint16_t>(d);
  return true;
}

bool SimSensors::tofRear(uint16_t& mm) {
  const double d = p_.x_m * 1000.0 + p_.noise(3.0);
  if (d < 30 || d > 4000) return false;
  mm = static_cast<uint16_t>(d);
  return true;
}

hal::EnvData SimSensors::env() {
  hal::EnvData e;
  const PipeSection& s = p_.sectionAt(p_.x_m);
  e.ok = true;
  e.temp_c = static_cast<float>(p_.cfg().ambient_c + 0.15 * (p_.driver_temp_c - p_.cfg().ambient_c) + p_.noise(0.1));
  e.humidity_pct = static_cast<float>((s.wet || p_.wet_override) ? 95.0 + p_.noise(0.5) : 55.0 + p_.noise(0.5));
  e.pressure_kpa = static_cast<float>(101.325 + 9.81 * s.water_depth_m + p_.noise(0.02));
  e.gas_ppm = static_cast<float>(std::max(0.0, s.gas_ppm + p_.noise(5.0)));
  return e;
}

bool SimSensors::gripSlideMm(int arm, float& mm) {
  if (!p_.mcu_powered) return false;
  mm = static_cast<float>(std::max(0.0, p_.arm[arm].slide_mm + p_.noise(0.02)));
  return true;
}

hal::PowerData SimPower::read() {
  hal::PowerData d;
  d.bus_v = static_cast<float>(p_.bus_v);
  d.drive_a = static_cast<float>(p_.drive_a);
  d.expand_a = static_cast<float>(p_.expand_a);
  d.battery_v = static_cast<float>(p_.battery_v);
  d.mode = p_.on_battery ? proto::SupplyMode::BATTERY : proto::SupplyMode::TETHER;
  d.driver_temp_c = static_cast<float>(p_.driver_temp_c);
  return d;
}

int SimLink::read(uint8_t* buf, int max) {
  int n = 0;
  while (n < max && !to_mcu.empty()) {
    buf[n++] = to_mcu.front();
    to_mcu.pop_front();
  }
  return n;
}

SimRig::SimRig(const PlantConfig& cfg)
    : plant(cfg),
      st0(plant, 0), st1(plant, 1), st2(plant, 2), st3(plant, 3),
      enc0(plant, 0), enc1(plant, 1), enc2(plant, 2), lever(plant, -1),
      sensors(plant), power(plant), gimbal(plant), lights(plant), safety(plant) {}

hal::Hardware SimRig::hw() {
  hal::Hardware h{};
  h.stepper[0] = &st0;
  h.stepper[1] = &st1;
  h.stepper[2] = &st2;
  h.stepper[3] = &st3;
  h.wheel_enc[0] = &enc0;
  h.wheel_enc[1] = &enc1;
  h.wheel_enc[2] = &enc2;
  h.lever_enc = &lever;
  h.sensors = &sensors;
  h.power = &power;
  h.gimbal = &gimbal;
  h.lights = &lights;
  h.safety = &safety;
  h.link = &link;
  return h;
}

}  // namespace sim
