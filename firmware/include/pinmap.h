// PIPEBOT-X controller pin map (RP2040 on the custom controller PCB, all 30 GPIOs broken out).
// SINGLE SOURCE OF TRUTH: docs/pinmap.md is generated from this file (tools/gen_pinmap_doc.py).
// Generator row format:  // PIN | <gpio> | <peripheral> | <function / alternate / notes>
#pragma once
#include <cstdint>

namespace pins {

// ---- Pi link -------------------------------------------------------------------------
// PIN | USB D+/D- | USB CDC | Pi <-> MCU protocol link (default)
// PIN | GP0 | UART0 TX | alternate Pi link, 921600 baud (build flag PIPEBOT_LINK_UART)
// PIN | GP1 | UART0 RX | alternate Pi link
constexpr uint8_t kPiUartTx = 0, kPiUartRx = 1;

// ---- Fast I2C, core 0 @ 1 MHz: magnetic encoders through a TCA9548A mux ----------------
// PIN | GP2 | I2C1 SDA | TCA9548A 0x70 -> AS5600 0x36 on ch0..4 (wheel A/B/C, lever, pan)
// PIN | GP3 | I2C1 SCL | encoder bus clock
constexpr uint8_t kEncSda = 2, kEncScl = 3;
constexpr uint8_t kMuxAddr = 0x70, kAs5600Addr = 0x36;
constexpr uint8_t kMuxWheel[3] = {0, 1, 2};
constexpr uint8_t kMuxLever = 3;
constexpr uint8_t kMuxPan = 4;

// ---- Stepper drivers -------------------------------------------------------------------
// PIN | GP4 | UART1 TX | TMC2209 single-wire UART, addresses 0..3 (1 k to PDN_UART); velocity via VACTUAL
// PIN | GP5 | UART1 RX | TMC2209 UART receive (same wire)
constexpr uint8_t kTmcUartTx = 4, kTmcUartRx = 5;
// PIN | GP6 | PIO0 SM0 | STEP drive A - fallback pulse train (normal: VACTUAL, no pulses)
// PIN | GP7 | PIO0 SM1 | STEP drive B - fallback
// PIN | GP8 | PIO0 SM2 | STEP drive C - fallback
// PIN | GP9 | PIO0 SM3 | STEP expansion - fallback
constexpr uint8_t kStep[4] = {6, 7, 8, 9};
// PIN | GP10 | PWM5 A | IR 850 nm LED driver DIM
constexpr uint8_t kIrPwm = 10;
// PIN | GP11 | GPIO in | E-stop loop sense (pull-up; low = loop closed = OK). Same loop drops the driver ENABLE relay.
constexpr uint8_t kEstopSense = 11;
// PIN | GP12 | GPIO in | supply path status (LTC4412 STAT; high = running from the battery)
constexpr uint8_t kOnBattery = 12;
// PIN | GP13 | GPIO out | camera/ToF window heater MOSFET (anti-fog)
constexpr uint8_t kHeater = 13;
// PIN | GP14 | GPIO out | DRIVE_EN, active low, 3 drive drivers (hardware E-stop relay in series)
// PIN | GP15 | GPIO out | EXPAND_EN, active low, expansion driver
constexpr uint8_t kDriveEn = 14, kExpandEn = 15;
// PIN | GP16 | SPI0 RX | TMC5160 SDO (size L) / spare
// PIN | GP17 | SPI0 CSn | TMC5160 chip select via 74HC138 (size L) / spare
// PIN | GP18 | SPI0 SCK | TMC5160 SCK (size L) / spare
// PIN | GP19 | SPI0 TX | TMC5160 SDI (size L) / spare
constexpr uint8_t kSpiMiso = 16, kSpiCs = 17, kSpiSck = 18, kSpiMosi = 19;

// ---- Slow I2C, core 1 @ 400 kHz: environment / IMU / power ------------------------------
// PIN | GP20 | I2C0 SDA | BNO085 0x4A, VL53L1X 0x29 (front) / 0x2A (rear), SHT31 0x44, BMP390 0x77, INA219 0x40/0x41, ADS1115 0x48
// PIN | GP21 | I2C0 SCL | sensor bus clock
constexpr uint8_t kSensSda = 20, kSensScl = 21;
constexpr uint8_t kBno085 = 0x4A, kTofFront = 0x29, kTofRear = 0x2A, kSht31 = 0x44, kBmp390 = 0x77;
constexpr uint8_t kInaDrive = 0x40, kInaExpand = 0x41, kAds1115 = 0x48;
// PIN | GP22 | GPIO out | VL53L1X rear XSHUT (low at boot so the front sensor is re-addressed first)
constexpr uint8_t kTofRearXshut = 22;
// ADS1115 channels: A0 MQ-4 gas, A1 bus voltage (1:20), A2 battery voltage (1:20), A3 driver NTC
constexpr uint8_t kAdsGas = 0, kAdsBus = 1, kAdsBattery = 2, kAdsNtc = 3;

// ---- Gimbal, lights ------------------------------------------------------------------
// PIN | GP23 | PWM3 B | tilt servo, 50 Hz
// PIN | GP24 | PWM4 A | pan gearmotor DRV8833 IN1
// PIN | GP25 | PWM4 B | pan gearmotor DRV8833 IN2
constexpr uint8_t kTiltServo = 23, kPanIn1 = 24, kPanIn2 = 25;

// ---- Analog -------------------------------------------------------------------------
// PIN | GP26 | ADC0 | grip hall sensor arm A (DRV5055A1; 0.6-2.7 V over the 10 mm slide)
// PIN | GP27 | ADC1 | grip hall sensor arm B
// PIN | GP28 | ADC2 | grip hall sensor arm C
constexpr uint8_t kHall[3] = {26, 27, 28};
// PIN | GP29 | PWM6 B | white LED driver DIM (4 x 1 W constant-current buck)
constexpr uint8_t kWhitePwm = 29;

// Hall calibration (per robot, written at assembly by tools/calibrate_hall.py; defaults here)
constexpr float kHallZeroV[3] = {0.60f, 0.60f, 0.60f};
constexpr float kHallVPerMm[3] = {0.21f, 0.21f, 0.21f};

}  // namespace pins
