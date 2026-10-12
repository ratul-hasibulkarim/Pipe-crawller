// HAL implementations backed by sim::Plant (native tests + SIL). The link is a pair of
// byte queues: the test/operator writes into `to_mcu` and reads `from_mcu`.
#pragma once
#include <cstdint>
#include <deque>
#include <vector>

#include "hal/hal.h"
#include "sim/plant.h"

namespace sim {

class SimStepper : public hal::IStepper {
 public:
  SimStepper(Plant& p, int idx) : p_(p), i_(idx) {}
  void setVelocity(float v) override { p_.stepper_vel[i_] = v; }
  // torque scales with the run current the firmware sets (thermal derate etc.)
  void setCurrent(float run_a, float) override;
  void enable(bool on) override { p_.stepper_en[i_] = on; }
  bool enabled() const override { return p_.stepper_en[i_]; }
  hal::StepperStatus status() override;

 private:
  Plant& p_;
  int i_;
  float run_a_ = 0;
};

class SimAngle : public hal::IAngleSensor {
 public:
  SimAngle(Plant& p, int arm) : p_(p), arm_(arm) {}  // arm = -1: lever encoder
  bool read(float& rad) override;
  bool fail = false;

 private:
  Plant& p_;
  int arm_;
};

class SimSensors : public hal::ISensors {
 public:
  explicit SimSensors(Plant& p) : p_(p) {}
  hal::ImuData imu() override;
  bool tofFront(uint16_t& mm) override;
  bool tofRear(uint16_t& mm) override;
  hal::EnvData env() override;
  bool gripSlideMm(int arm, float& mm) override;
  bool imu_fail = false;

 private:
  Plant& p_;
};

class SimPower : public hal::IPower {
 public:
  explicit SimPower(Plant& p) : p_(p) {}
  hal::PowerData read() override;

 private:
  Plant& p_;
};

class SimGimbal : public hal::IGimbal {
 public:
  explicit SimGimbal(Plant& p) : p_(p) {}
  void command(float pan, float tilt) override {
    p_.pan_cmd = pan;
    p_.tilt_cmd = tilt;
  }
  float pan() override { return static_cast<float>(p_.pan); }
  float tilt() override { return static_cast<float>(p_.tilt); }

 private:
  Plant& p_;
};

class SimLights : public hal::ILights {
 public:
  explicit SimLights(Plant& p) : p_(p) {}
  void set(uint8_t w, uint8_t ir) override {
    p_.white = w;
    p_.ir = ir;
  }

 private:
  Plant& p_;
};

class SimSafety : public hal::ISafetyHw {
 public:
  explicit SimSafety(Plant& p) : p_(p) {}
  void driveEnable(bool on) override { p_.drive_enable_line = on; }
  void expandEnable(bool on) override { p_.expand_enable_line = on; }
  bool estopInput() override { return p_.estop_loop_open; }
  void kickWatchdog() override { ++kicks; }
  bool bootedFromWatchdog() override { return watchdog_boot; }
  uint32_t kicks = 0;
  bool watchdog_boot = false;

 private:
  Plant& p_;
};

class SimLink : public hal::ISerial {
 public:
  int read(uint8_t* buf, int max) override;
  void write(const uint8_t* buf, int n) override { from_mcu.insert(from_mcu.end(), buf, buf + n); }
  std::deque<uint8_t> to_mcu;
  std::vector<uint8_t> from_mcu;
};

// Everything needed to run the firmware core against the plant.
struct SimRig {
  explicit SimRig(const PlantConfig& cfg);
  Plant plant;
  SimStepper st0, st1, st2, st3;
  SimAngle enc0, enc1, enc2, lever;
  SimSensors sensors;
  SimPower power;
  SimGimbal gimbal;
  SimLights lights;
  SimSafety safety;
  SimLink link;
  hal::Hardware hw() ;
};

}  // namespace sim
