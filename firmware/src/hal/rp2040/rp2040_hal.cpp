#ifdef ARDUINO
#include "hal/rp2040/rp2040_hal.h"

#include <Adafruit_BMP3XX.h>
#include <Adafruit_BNO08x.h>
#include <Adafruit_SHT31.h>
#include <Servo.h>
#include <TMCStepper.h>
#include <VL53L1X.h>
#include <Wire.h>
#include <hardware/watchdog.h>
#include <pico/sync.h>

#include <cmath>

#include "bot_params.h"
#include "pinmap.h"

namespace picohw {

namespace {
constexpr float kRSense = 0.11f;
constexpr float kTwoPi = 6.28318531f;
constexpr float kVactualHz = 0.715255737f;  // TMC2209 VACTUAL LSB in usteps/s (12 MHz internal clock)
critical_section_t g_cs;
SlowCache g_cache;

// ------------------------------------------------------------------ encoder bus (core 0)
bool muxSelect(uint8_t ch) {
  Wire1.beginTransmission(pins::kMuxAddr);
  Wire1.write(static_cast<uint8_t>(1u << ch));
  return Wire1.endTransmission() == 0;
}

bool as5600Read(uint8_t ch, float& rad) {
  if (!muxSelect(ch)) return false;
  Wire1.beginTransmission(pins::kAs5600Addr);
  Wire1.write(0x0C);  // RAW ANGLE (11:8), (7:0)
  if (Wire1.endTransmission(false) != 0) return false;
  if (Wire1.requestFrom(pins::kAs5600Addr, static_cast<uint8_t>(2)) != 2) return false;
  const uint16_t raw = static_cast<uint16_t>(((Wire1.read() & 0x0F) << 8) | Wire1.read());
  rad = static_cast<float>(raw) * (kTwoPi / 4096.0f);
  return true;
}

class As5600 : public hal::IAngleSensor {
 public:
  explicit As5600(uint8_t ch) : ch_(ch) {}
  bool read(float& rad) override { return as5600Read(ch_, rad); }

 private:
  uint8_t ch_;
};

// ------------------------------------------------------------------ steppers (core 0)
#if defined(PIPEBOT_SIZE_L)
class Stepper : public hal::IStepper {  // TMC5160 over SPI, internal ramp generator in velocity mode
 public:
  explicit Stepper(uint8_t idx) : drv_(pins::kSpiCs, kRSense * 0.6f, -1), idx_(idx) {}
  void begin() {
    drv_.begin();
    drv_.toff(4);
    drv_.microsteps(params::MOTOR_USTEPS_PER_REV / 200);
    drv_.en_pwm_mode(true);
    drv_.AMAX(20000);
    drv_.RAMPMODE(1);
    drv_.VMAX(0);
  }
  void setVelocity(float v) override {
    drv_.RAMPMODE(v >= 0 ? 1 : 2);
    drv_.VMAX(static_cast<uint32_t>(std::fabs(v) / 0.715f));
  }
  void setCurrent(float run_a, float hold_frac) override { drv_.rms_current(static_cast<uint16_t>(run_a * 1000), hold_frac); }
  void enable(bool on) override {
    on_ = on;
    drv_.toff(on ? 4 : 0);
  }
  bool enabled() const override { return on_; }
  hal::StepperStatus status() override {
    hal::StepperStatus s;
    const uint32_t st = drv_.DRV_STATUS();
    s.stalled = (st >> 24) & 1;
    s.overtemp_warn = (st >> 26) & 1;
    s.overtemp = (st >> 25) & 1;
    s.comm_ok = st != 0xFFFFFFFF;
    return s;
  }

 private:
  TMC5160Stepper drv_;
  uint8_t idx_;
  bool on_ = false;
};
#else
class Stepper : public hal::IStepper {  // TMC2209, single-wire UART, VACTUAL velocity mode
 public:
  explicit Stepper(uint8_t addr) : drv_(&Serial2, kRSense, addr) {}
  void begin() {
    drv_.begin();
    drv_.toff(4);
    drv_.blank_time(24);
    drv_.microsteps(params::MOTOR_USTEPS_PER_REV / 200);
    drv_.pwm_autoscale(true);
    drv_.en_spreadCycle(false);  // StealthChop: quiet, StallGuard4 available
    drv_.TCOOLTHRS(0xFFFFF);
    drv_.SGTHRS(60);
    drv_.VACTUAL(0);
  }
  void setVelocity(float v) override { drv_.VACTUAL(static_cast<int32_t>(v / kVactualHz)); }
  void setCurrent(float run_a, float hold_frac) override { drv_.rms_current(static_cast<uint16_t>(run_a * 1000), hold_frac); }
  void enable(bool on) override {
    on_ = on;
    drv_.toff(on ? 4 : 0);
  }
  bool enabled() const override { return on_; }
  hal::StepperStatus status() override {
    hal::StepperStatus s;
    const uint32_t st = drv_.DRV_STATUS();
    s.comm_ok = st != 0xFFFFFFFF;
    s.overtemp_warn = st & 1u;          // otpw
    s.overtemp = (st >> 1) & 1u;        // ot
    s.stalled = on_ && drv_.SG_RESULT() < 10;
    return s;
  }

 private:
  TMC2209Stepper drv_;
  bool on_ = false;
};
#endif

Stepper g_st0(0), g_st1(1), g_st2(2), g_st3(3);
As5600 g_enc0(pins::kMuxWheel[0]), g_enc1(pins::kMuxWheel[1]), g_enc2(pins::kMuxWheel[2]), g_lever(pins::kMuxLever);

// ------------------------------------------------------------------ sensors (read side, core 0)
class Sensors : public hal::ISensors {
 public:
  hal::ImuData imu() override { return board().snapshot().imu; }
  bool tofFront(uint16_t& mm) override {
    const SlowCache c = board().snapshot();
    mm = c.tof_front;
    return c.tof_front_ok;
  }
  bool tofRear(uint16_t& mm) override {
    const SlowCache c = board().snapshot();
    mm = c.tof_rear;
    return c.tof_rear_ok;
  }
  hal::EnvData env() override { return board().snapshot().env; }
  bool gripSlideMm(int arm, float& mm) override {
    const float v = analogRead(pins::kHall[arm]) * (3.3f / 4095.0f);
    if (v < 0.2f || v > 3.1f) return false;  // open / shorted sensor
    mm = (v - pins::kHallZeroV[arm]) / pins::kHallVPerMm[arm];
    if (mm < 0) mm = 0;
    return true;
  }
};

class Power : public hal::IPower {
 public:
  hal::PowerData read() override { return board().snapshot().power; }
};

class Gimbal : public hal::IGimbal {
 public:
  void begin() {
    tilt_.attach(pins::kTiltServo, 500, 2500);
    pinMode(pins::kPanIn1, OUTPUT);
    pinMode(pins::kPanIn2, OUTPUT);
  }
  void command(float pan_deg, float tilt_deg) override {
    tilt_deg_ = tilt_deg < -90 ? -90 : (tilt_deg > 90 ? 90 : tilt_deg);
    tilt_.writeMicroseconds(static_cast<int>(1500 + tilt_deg_ * (1000.0f / 90.0f)));
    float a;
    if (as5600Read(pins::kMuxPan, a)) pan_deg_ = a * (360.0f / kTwoPi) - 180.0f;
    float err = pan_deg - pan_deg_;
    while (err > 180) err -= 360;
    while (err < -180) err += 360;
    const int duty = static_cast<int>(std::fmin(1000.0f, std::fabs(err) * 40.0f));
    const bool fwd = err > 0;
    analogWrite(pins::kPanIn1, (std::fabs(err) > 1.0f && fwd) ? duty : 0);
    analogWrite(pins::kPanIn2, (std::fabs(err) > 1.0f && !fwd) ? duty : 0);
  }
  float pan() override { return pan_deg_; }
  float tilt() override { return tilt_deg_; }

 private:
  Servo tilt_;
  float pan_deg_ = 0, tilt_deg_ = 0;
};

class Lights : public hal::ILights {
 public:
  void set(uint8_t w, uint8_t ir) override {
    analogWrite(pins::kWhitePwm, w * 10);
    analogWrite(pins::kIrPwm, ir * 10);
  }
};

class Safety : public hal::ISafetyHw {
 public:
  void driveEnable(bool on) override { digitalWrite(pins::kDriveEn, on ? LOW : HIGH); }
  void expandEnable(bool on) override { digitalWrite(pins::kExpandEn, on ? LOW : HIGH); }
  bool estopInput() override { return digitalRead(pins::kEstopSense) == HIGH; }
  void kickWatchdog() override { watchdog_update(); }
  bool bootedFromWatchdog() override { return watchdog_caused_reboot(); }
};

class Link : public hal::ISerial {
 public:
  int read(uint8_t* buf, int max) override {
    int n = 0;
    while (n < max && Serial.available() > 0) buf[n++] = static_cast<uint8_t>(Serial.read());
    return n;
  }
  void write(const uint8_t* buf, int n) override { Serial.write(buf, n); }
};

Sensors g_sensors;
Power g_power;
Gimbal g_gimbal;
Lights g_lights;
Safety g_safety;
Link g_link;

// ------------------------------------------------------------------ slow sensors (core 1)
Adafruit_BNO08x g_bno(-1);
VL53L1X g_tof_front, g_tof_rear;
Adafruit_SHT31 g_sht(&Wire);
Adafruit_BMP3XX g_bmp;
bool g_have_bno = false, g_have_tof_f = false, g_have_tof_r = false, g_have_sht = false, g_have_bmp = false;

// INA219 read raw: shunt-voltage register (10 uV LSB) across a 10 mOhm shunt -> amps.
float inaAmps(uint8_t addr) {
  constexpr float kShuntOhm = 0.010f;
  Wire.beginTransmission(addr);
  Wire.write(0x01);
  if (Wire.endTransmission(false) != 0) return 0.0f;
  if (Wire.requestFrom(addr, static_cast<uint8_t>(2)) != 2) return 0.0f;
  const int16_t raw = static_cast<int16_t>((Wire.read() << 8) | Wire.read());
  return raw * 10e-6f / kShuntOhm;
}

int16_t adsRead(uint8_t ch) {
  // single-shot, +/-4.096 V, 860 SPS
  const uint16_t cfg = 0x8000 | static_cast<uint16_t>((0x4 + ch) << 12) | 0x0200 | 0x0100 | 0x00E0 | 0x0003;
  Wire.beginTransmission(pins::kAds1115);
  Wire.write(0x01);
  Wire.write(static_cast<uint8_t>(cfg >> 8));
  Wire.write(static_cast<uint8_t>(cfg));
  if (Wire.endTransmission() != 0) return 0;
  delayMicroseconds(1300);
  Wire.beginTransmission(pins::kAds1115);
  Wire.write(0x00);
  Wire.endTransmission(false);
  if (Wire.requestFrom(pins::kAds1115, static_cast<uint8_t>(2)) != 2) return 0;
  return static_cast<int16_t>((Wire.read() << 8) | Wire.read());
}
float adsVolts(uint8_t ch) { return adsRead(ch) * (4.096f / 32768.0f); }

}  // namespace

Board& board() {
  static Board b;
  return b;
}

SlowCache Board::snapshot() {
  critical_section_enter_blocking(&g_cs);
  SlowCache c = g_cache;
  critical_section_exit(&g_cs);
  return c;
}

void Board::publish(const SlowCache& c) {
  critical_section_enter_blocking(&g_cs);
  g_cache = c;
  critical_section_exit(&g_cs);
}

void Board::begin() {
  critical_section_init(&g_cs);
  pinMode(pins::kDriveEn, OUTPUT);
  pinMode(pins::kExpandEn, OUTPUT);
  digitalWrite(pins::kDriveEn, HIGH);  // disabled until the core enables them
  digitalWrite(pins::kExpandEn, HIGH);
  pinMode(pins::kEstopSense, INPUT_PULLUP);
  pinMode(pins::kOnBattery, INPUT);
  pinMode(pins::kHeater, OUTPUT);
  digitalWrite(pins::kHeater, HIGH);  // keep the window above dew point
  analogReadResolution(12);
  analogWriteFreq(20000);
  analogWriteRange(1000);
  Serial.begin(921600);
  Wire1.setSDA(pins::kEncSda);
  Wire1.setSCL(pins::kEncScl);
  Wire1.begin();
  Wire1.setClock(1000000);
#if defined(PIPEBOT_SIZE_L)
  SPI.setRX(pins::kSpiMiso);
  SPI.setTX(pins::kSpiMosi);
  SPI.setSCK(pins::kSpiSck);
  SPI.begin();
#else
  Serial2.setTX(pins::kTmcUartTx);
  Serial2.setRX(pins::kTmcUartRx);
  Serial2.begin(115200);
#endif
  g_st0.begin();
  g_st1.begin();
  g_st2.begin();
  g_st3.begin();
  g_gimbal.begin();
  watchdog_enable(100, true);  // 100 ms: the 1 kHz loop kicks it every tick
}

void Board::beginCore1() {
  Wire.setSDA(pins::kSensSda);
  Wire.setSCL(pins::kSensScl);
  Wire.begin();
  Wire.setClock(400000);
  pinMode(pins::kTofRearXshut, OUTPUT);
  digitalWrite(pins::kTofRearXshut, LOW);
  g_tof_front.setBus(&Wire);
  g_have_tof_f = g_tof_front.init();
  if (g_have_tof_f) {
    g_tof_front.setAddress(pins::kTofFront);
    g_tof_front.setDistanceMode(VL53L1X::Long);
    g_tof_front.startContinuous(20);
  }
  digitalWrite(pins::kTofRearXshut, HIGH);
  delay(5);
  g_tof_rear.setBus(&Wire);
  g_have_tof_r = g_tof_rear.init();
  if (g_have_tof_r) {
    g_tof_rear.setAddress(pins::kTofRear);
    g_tof_rear.setDistanceMode(VL53L1X::Long);
    g_tof_rear.startContinuous(20);
  }
  g_have_bno = g_bno.begin_I2C(pins::kBno085, &Wire);
  if (g_have_bno) g_bno.enableReport(SH2_GAME_ROTATION_VECTOR, 10000);
  g_have_sht = g_sht.begin(pins::kSht31);
  g_have_bmp = g_bmp.begin_I2C(pins::kBmp390, &Wire);
}

void Board::pollCore1() {
  static uint32_t last_env = 0;
  SlowCache c = snapshot();
  if (g_have_bno) {
    sh2_SensorValue_t v;
    if (g_bno.getSensorEvent(&v) && v.sensorId == SH2_GAME_ROTATION_VECTOR) {
      const float qr = v.un.gameRotationVector.real, qi = v.un.gameRotationVector.i;
      const float qj = v.un.gameRotationVector.j, qk = v.un.gameRotationVector.k;
      c.imu.roll_deg = std::atan2(2 * (qr * qi + qj * qk), 1 - 2 * (qi * qi + qj * qj)) * 57.2958f;
      c.imu.pitch_deg = std::asin(std::fmax(-1.0f, std::fmin(1.0f, 2 * (qr * qj - qk * qi)))) * 57.2958f;
      c.imu.yaw_deg = std::atan2(2 * (qr * qk + qi * qj), 1 - 2 * (qj * qj + qk * qk)) * 57.2958f;
      c.imu.ok = true;
    }
  } else {
    c.imu.ok = false;
  }
  if (g_have_tof_f && g_tof_front.dataReady()) {
    c.tof_front = g_tof_front.read(false);
    c.tof_front_ok = g_tof_front.ranging_data.range_status == VL53L1X::RangeValid;
  }
  if (g_have_tof_r && g_tof_rear.dataReady()) {
    c.tof_rear = g_tof_rear.read(false);
    c.tof_rear_ok = g_tof_rear.ranging_data.range_status == VL53L1X::RangeValid;
  }
  c.power.drive_a = inaAmps(pins::kInaDrive);
  c.power.expand_a = inaAmps(pins::kInaExpand);
  c.power.bus_v = adsVolts(pins::kAdsBus) * 20.0f;
  c.power.battery_v = adsVolts(pins::kAdsBattery) * 20.0f;
  c.power.mode = digitalRead(pins::kOnBattery) ? proto::SupplyMode::BATTERY : proto::SupplyMode::TETHER;
  {
    // 10 k NTC (B 3950) in a divider to 3.3 V with 10 k
    const float v = adsVolts(pins::kAdsNtc);
    const float r = 10000.0f * v / std::fmax(0.01f, 3.3f - v);
    c.power.driver_temp_c = 1.0f / (1.0f / 298.15f + std::log(r / 10000.0f) / 3950.0f) - 273.15f;
  }
  if (millis() - last_env > 500) {
    last_env = millis();
    c.env.ok = g_have_sht;
    if (g_have_sht) {
      c.env.temp_c = g_sht.readTemperature();
      c.env.humidity_pct = g_sht.readHumidity();
    }
    if (g_have_bmp && g_bmp.performReading()) c.env.pressure_kpa = static_cast<float>(g_bmp.pressure * 1e-3);
    // MQ-4: Rs/R0 -> ppm CH4 (datasheet curve: ppm = 1012 * (Rs/R0)^-2.786), R0 calibrated in clean air
    const float vg = adsVolts(pins::kAdsGas);
    const float rs = (5.0f - vg) / std::fmax(0.01f, vg);
    c.env.gas_ppm = 1012.0f * std::pow(std::fmax(0.05f, rs / 4.4f), -2.786f);
  }
  publish(c);
}

hal::Hardware Board::hardware() {
  hal::Hardware h{};
  h.stepper[0] = &g_st0;
  h.stepper[1] = &g_st1;
  h.stepper[2] = &g_st2;
  h.stepper[3] = &g_st3;
  h.wheel_enc[0] = &g_enc0;
  h.wheel_enc[1] = &g_enc1;
  h.wheel_enc[2] = &g_enc2;
  h.lever_enc = &g_lever;
  h.sensors = &g_sensors;
  h.power = &g_power;
  h.gimbal = &g_gimbal;
  h.lights = &g_lights;
  h.safety = &g_safety;
  h.link = &g_link;
  return h;
}

}  // namespace picohw
#endif
