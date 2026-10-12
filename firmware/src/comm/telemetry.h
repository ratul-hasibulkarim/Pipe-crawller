// Builds the 20 Hz TELEMETRY packet from the controllers and sensors.
#pragma once
#include <cstdint>

#include "control/drive_loop.h"
#include "control/expansion_loop.h"
#include "control/odometry.h"
#include "hal/hal.h"
#include "protocol_gen.h"

namespace comm {

struct TelemetryInputs {
  uint32_t t_ms;
  proto::State state;
  uint32_t faults;
  const ctl::ExpansionLoop* exp;
  const ctl::DriveLoop* drive;
  const ctl::Odometry* odom;
  hal::ImuData imu;
  hal::EnvData env;
  hal::PowerData power;
  bool tof_front_ok, tof_rear_ok;
  uint16_t tof_front, tof_rear;
  float pan, tilt;
  uint8_t white, ir;
  uint16_t last_seq, rx_errors;
};

class TelemetryBuilder {
 public:
  proto::TelemetryMsg build(const TelemetryInputs& in) const;
  static uint8_t batteryPercent(float pack_v);
};

}  // namespace comm
