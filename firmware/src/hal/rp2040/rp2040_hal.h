// RP2040 (Arduino-Pico core) implementation of the HAL.
//   core 0: control loops, encoder bus (I2C1 @ 1 MHz), stepper UART/SPI, hall ADCs, gimbal
//   core 1: slow sensors on I2C0 (IMU, ToF, env, INA219, ADS1115) -> double-buffered cache
#pragma once
#ifdef ARDUINO
#include <Arduino.h>

#include "hal/hal.h"

namespace rp2040 {

// Shared between core 1 (writer) and core 0 (reader), copied under a spinlock.
struct SlowCache {
  hal::ImuData imu;
  hal::EnvData env;
  hal::PowerData power;
  uint16_t tof_front = 0, tof_rear = 0;
  bool tof_front_ok = false, tof_rear_ok = false;
};

class Board {
 public:
  void begin();             // core 0 setup
  void beginCore1();        // core 1 setup
  void pollCore1();         // core 1 loop body
  hal::Hardware hardware();
  SlowCache snapshot();
  void publish(const SlowCache& c);
};

Board& board();

}  // namespace rp2040
#endif
