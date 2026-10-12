// Robot core: owns the control loops, state machine, safety and the Pi link.
// Statically allocated, no heap. tick() is called from the main loop (firmware) or the
// simulator; it schedules the drive loop (1 kHz), expansion loop (200 Hz), safety checks
// and telemetry (20 Hz) from the elapsed time it is given.
#pragma once
#include <cstdint>

#include "comm/serial_protocol.h"
#include "comm/telemetry.h"
#include "control/drive_loop.h"
#include "control/expansion_loop.h"
#include "control/odometry.h"
#include "control/state_machine.h"
#include "hal/hal.h"
#include "safety/watchdog.h"

namespace core {

class Robot {
 public:
  explicit Robot(const hal::Hardware& hw);
  void begin(uint32_t now_us);
  void tick(uint32_t now_us);

  // observers (tests, simulator)
  proto::State state() const { return sm_.state(); }
  uint32_t faults() const { return faults_; }
  bool hasFault(proto::Fault f) const { return (faults_ & proto::faultBit(f)) != 0; }
  const ctl::ExpansionLoop& expansion() const { return exp_; }
  const ctl::DriveLoop& driveLoop() const { return drive_; }
  const ctl::Odometry& odometry() const { return odom_; }
  uint16_t lastSeq() const { return last_seq_; }

 private:
  const hal::Hardware hw_;
  comm::FrameParser parser_;
  ctl::StateMachine sm_;
  ctl::DriveLoop drive_;
  ctl::ExpansionLoop exp_;
  ctl::Odometry odom_;
  safety::Heartbeat heartbeat_;
  safety::CommandTtl drive_ttl_, expand_ttl_;
  comm::TelemetryBuilder telem_;

  uint32_t now_ms_ = 0, last_us_ = 0, acc_drive_us_ = 0, acc_exp_us_ = 0, acc_tel_us_ = 0, acc_safety_us_ = 0;
  uint32_t faults_ = 0;
  uint16_t last_seq_ = 0;
  // latest commands
  float drive_v_ = 0, drive_turn_ = 0, drive_dir_ = 0;
  ctl::ExpMode exp_mode_ = ctl::ExpMode::LOCKED;
  float exp_target_ = 0, grip_target_;
  float pan_cmd_ = 0, tilt_cmd_ = 0;
  uint8_t white_ = 0, ir_ = 0;
  // safety timers
  float oc_drive_s_ = 0, low_v_s_ = 0, ok_v_s_ = 0, grip_low_s_ = 0, slip_hold_s_ = 0;
  bool thermal_derate_ = false;
  hal::PowerData power_{};

  void pollLink();
  void handleFrame(const comm::Frame& f);
  void ack(uint16_t seq, proto::MsgType cmd, proto::AckStatus st);
  void sendEvent(proto::Event code, uint32_t arg, float value);
  void setFault(proto::Fault f, bool latch_to_fault_state);
  void clearFlag(proto::Fault f);
  void driveStep(float dt);
  void expansionStep(float dt);
  void safetyStep(float dt);
  void telemetryStep();
  void estop();
  void reportStateChange();
  template <typename Msg>
  void send(const Msg& m);
};

}  // namespace core
