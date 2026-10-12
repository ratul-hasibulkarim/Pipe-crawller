// Hardware abstraction. The control core only talks to these interfaces, so the same
// code runs on the RP2040 (src/hal/rp2040_*.cpp), in native unit tests and in the
// software-in-the-loop simulator (sim/).
#pragma once
#include <cstdint>

#include "protocol_gen.h"

namespace hal {

constexpr int kArms = 3;
constexpr int kMotors = 4;  // 0..2 drive (arm A/B/C), 3 expansion

struct StepperStatus {
  bool stalled = false;        // TMC StallGuard / missed steps
  bool overtemp_warn = false;  // driver pre-warning (~120 C die)
  bool overtemp = false;       // driver shut down
  bool comm_ok = true;         // UART/SPI link to the driver
};

// Velocity-mode stepper (TMC2209 VACTUAL over UART, TMC5160 ramp generator over SPI).
class IStepper {
 public:
  virtual ~IStepper() = default;
  virtual void setVelocity(float usteps_per_s) = 0;  // signed
  virtual void setCurrent(float run_a, float hold_frac) = 0;
  virtual void enable(bool on) = 0;
  virtual bool enabled() const = 0;
  virtual StepperStatus status() = 0;
};

// Absolute magnetic angle sensor (AS5600), 0..2pi.
class IAngleSensor {
 public:
  virtual ~IAngleSensor() = default;
  virtual bool read(float& rad) = 0;
};

struct ImuData {
  float pitch_deg = 0, roll_deg = 0, yaw_deg = 0;
  bool ok = false;
};
struct EnvData {
  float temp_c = 0, humidity_pct = 0, pressure_kpa = 0, gas_ppm = 0;
  bool ok = false;
};
struct PowerData {
  float bus_v = 0;            // input after the supply OR-ing
  float drive_a = 0;          // INA219 #1 (drive rail)
  float expand_a = 0;         // INA219 #2 (expansion motor)
  float battery_v = 0;        // pack voltage (0 if no pack)
  proto::SupplyMode mode = proto::SupplyMode::TETHER;
  float driver_temp_c = 25;   // hottest driver / board NTC
};

class ISensors {
 public:
  virtual ~ISensors() = default;
  virtual ImuData imu() = 0;
  virtual bool tofFront(uint16_t& mm) = 0;
  virtual bool tofRear(uint16_t& mm) = 0;
  virtual EnvData env() = 0;
  // Rocker slide displacement from the hall sensor (mm, 0 = at the outer stop).
  virtual bool gripSlideMm(int arm, float& mm) = 0;
};

class IPower {
 public:
  virtual ~IPower() = default;
  virtual PowerData read() = 0;
};

class IGimbal {
 public:
  virtual ~IGimbal() = default;
  virtual void command(float pan_deg, float tilt_deg) = 0;  // pan continuous, tilt clamped
  virtual float pan() = 0;
  virtual float tilt() = 0;
};

class ILights {
 public:
  virtual ~ILights() = default;
  virtual void set(uint8_t white_pct, uint8_t ir_pct) = 0;
};

// Hardware safety lines: driver ENABLE pins (E-stop cuts these in hardware too) and the MCU watchdog.
class ISafetyHw {
 public:
  virtual ~ISafetyHw() = default;
  virtual void driveEnable(bool on) = 0;
  virtual void expandEnable(bool on) = 0;
  virtual bool estopInput() = 0;  // true = external E-stop loop open
  virtual void kickWatchdog() = 0;
  virtual bool bootedFromWatchdog() = 0;
};

class ISerial {
 public:
  virtual ~ISerial() = default;
  virtual int read(uint8_t* buf, int max) = 0;
  virtual void write(const uint8_t* buf, int n) = 0;
};

struct Hardware {
  IStepper* stepper[kMotors];
  IAngleSensor* wheel_enc[kArms];
  IAngleSensor* lever_enc;
  ISensors* sensors;
  IPower* power;
  IGimbal* gimbal;
  ILights* lights;
  ISafetyHw* safety;
  ISerial* link;
};

}  // namespace hal
