#include "core/robot.h"

#include <cmath>

#include "bot_params.h"
#include "control/kinematics.h"

namespace core {

using proto::AckStatus;
using proto::Fault;
using proto::MsgType;
using proto::State;

namespace {
constexpr uint32_t kDriveUs = 1000000u / params::DRIVE_LOOP_HZ;
constexpr uint32_t kExpUs = 1000000u / params::EXPANSION_LOOP_HZ;
constexpr uint32_t kTelUs = 1000000u / params::TELEMETRY_HZ;
constexpr uint32_t kSafetyUs = 10000u;  // 100 Hz
constexpr uint32_t kHoldToGrippedMs = 10000;
// faults that put the robot in FAULT and need CLEAR_FAULT
constexpr uint32_t kLatching = proto::faultBit(Fault::ESTOP) | proto::faultBit(Fault::OVERCURRENT) |
                               proto::faultBit(Fault::OVERTEMP) | proto::faultBit(Fault::EXPANSION_STALL) |
                               proto::faultBit(Fault::ENCODER_FAIL);
// flags cleared by CLEAR_FAULT (informational, latched until acknowledged)
constexpr uint32_t kAcknowledge = kLatching | proto::faultBit(Fault::SLIP_WARNING) | proto::faultBit(Fault::NO_CONTACT) |
                                  proto::faultBit(Fault::WATCHDOG_RESET) | proto::faultBit(Fault::COMMAND_STALE);
}  // namespace

Robot::Robot(const hal::Hardware& hw)
    : hw_(hw), heartbeat_(params::WATCHDOG_TIMEOUT_MS), grip_target_(params::DEFAULT_GRIP_TARGET_N) {}

template <typename Msg>
void Robot::send(const Msg& m) {
  uint8_t buf[proto::kMaxPayload + proto::kFrameOverhead];
  const size_t n = proto::encode(m, buf, sizeof(buf));
  if (n) hw_.link->write(buf, static_cast<int>(n));
}

void Robot::begin(uint32_t now_us) {
  last_us_ = now_us;
  hw_.safety->driveEnable(true);
  hw_.safety->expandEnable(false);
  for (auto* s : hw_.stepper) s->enable(false);
  if (hw_.safety->bootedFromWatchdog()) setFault(Fault::WATCHDOG_RESET, false);
  // A reset inside a pipe must not forget the grip: read the arms once.
  ctl::ExpCmd locked;
  exp_.step(hw_, locked, 0.0f);
  if (exp_.allContact()) sm_.assumeGripped(0);
  sendEvent(proto::Event::BOOT, proto::kVersion, exp_.diameter());
  reportStateChange();
}

void Robot::tick(uint32_t now_us) {
  uint32_t dt = now_us - last_us_;
  last_us_ = now_us;
  if (dt > 100000u) dt = 100000u;  // after a debugger halt: do not replay
  acc_drive_us_ += dt;
  acc_exp_us_ += dt;
  acc_tel_us_ += dt;
  acc_safety_us_ += dt;
  pollLink();
  while (acc_drive_us_ >= kDriveUs) {
    acc_drive_us_ -= kDriveUs;
    now_ms_ += kDriveUs / 1000u;
    driveStep(kDriveUs * 1e-6f);
  }
  while (acc_exp_us_ >= kExpUs) {
    acc_exp_us_ -= kExpUs;
    expansionStep(kExpUs * 1e-6f);
  }
  while (acc_safety_us_ >= kSafetyUs) {
    acc_safety_us_ -= kSafetyUs;
    safetyStep(kSafetyUs * 1e-6f);
  }
  if (acc_tel_us_ >= kTelUs) {
    acc_tel_us_ %= kTelUs;
    telemetryStep();
  }
  hw_.safety->kickWatchdog();
}

// ------------------------------------------------------------------------------- link
void Robot::pollLink() {
  uint8_t buf[64];
  for (int guard = 0; guard < 16; ++guard) {
    const int n = hw_.link->read(buf, sizeof(buf));
    if (n <= 0) break;
    for (int i = 0; i < n; ++i) {
      comm::Frame f{};
      if (parser_.feed(buf[i], f)) handleFrame(f);
    }
  }
}

void Robot::ack(uint16_t seq, MsgType cmd, AckStatus st) {
  proto::AckMsg a;
  a.seq = seq;
  a.cmd = static_cast<uint8_t>(cmd);
  a.status = static_cast<uint8_t>(st);
  send(a);
}

void Robot::sendEvent(proto::Event code, uint32_t arg, float value) {
  proto::EventMsg e;
  e.t_ms = now_ms_;
  e.code = static_cast<uint8_t>(code);
  e.state = static_cast<uint8_t>(sm_.state());
  e.arg = arg;
  e.value = value;
  send(e);
}

void Robot::handleFrame(const comm::Frame& f) {
  heartbeat_.touch(now_ms_);
  if (hasFault(Fault::HEARTBEAT_LOST)) clearFlag(Fault::HEARTBEAT_LOST);
  const State s = sm_.state();
  const bool faulted = s == State::FAULT;
  const AckStatus blocked = hasFault(Fault::ESTOP) ? AckStatus::ESTOPPED : AckStatus::FAULTED;
  switch (f.type) {
    case MsgType::DRIVE: {
      proto::DriveMsg m;
      if (!m.unpack(f.payload, f.len)) return;
      last_seq_ = m.seq;
      const float v = ctl::clampf(static_cast<float>(m.linear_mm_s), -params::MAX_SPEED_MM_S, params::MAX_SPEED_MM_S);
      const bool moving = v != 0.0f;
      if (moving && (faulted || s == State::LOW_POWER_SAFE)) {
        ack(m.seq, f.type, faulted ? blocked : AckStatus::REJECTED);
        return;
      }
      if (!sm_.drive(moving, now_ms_)) {
        ack(m.seq, f.type, AckStatus::REJECTED);
        return;
      }
      drive_v_ = v;
      drive_turn_ = m.turn_deg_s;
      drive_dir_ = m.bend_dir_deg;
      const uint32_t ttl = m.ttl_ms ? (m.ttl_ms > 1000 ? 1000u : m.ttl_ms) : static_cast<uint32_t>(params::DRIVE_TTL_MS);
      drive_ttl_.arm(now_ms_, ttl);
      if (moving) {
        clearFlag(Fault::SLIP_WARNING);
        clearFlag(Fault::COMMAND_STALE);
        drive_.clearSlip();
      }
      ack(m.seq, f.type, AckStatus::OK);
      return;
    }
    case MsgType::EXPAND: {
      proto::ExpandMsg m;
      if (!m.unpack(f.payload, f.len)) return;
      last_seq_ = m.seq;
      if (faulted || s == State::LOW_POWER_SAFE) {
        ack(m.seq, f.type, faulted ? blocked : AckStatus::REJECTED);
        return;
      }
      const auto mode = static_cast<proto::ExpandMode>(m.mode);
      bool ok_arg = true;
      if (mode == proto::ExpandMode::DIAMETER)
        ok_arg = m.target >= params::PIPE_ID_MIN_MM - 5.0f && m.target <= params::PIPE_ID_MAX_MM + 5.0f;
      else if (mode == proto::ExpandMode::FORCE)
        ok_arg = m.target > 0.0f && m.target <= params::MAX_GRIP_N;
      else if (mode != proto::ExpandMode::RETRACT)
        ok_arg = false;
      if (!ok_arg) {
        ack(m.seq, f.type, AckStatus::BAD_ARG);
        return;
      }
      // a refresh of the command already satisfied while gripped: keep the state, just ack
      const bool same = (s == State::GRIPPED || s == State::HOLDING) && mode == proto::ExpandMode::FORCE &&
                        exp_.allContact() && std::fabs(m.target - grip_target_) < 0.01f;
      if (same) {
        expand_ttl_.arm(now_ms_, m.ttl_ms ? m.ttl_ms : static_cast<uint32_t>(params::EXPAND_TTL_MS));
        ack(m.seq, f.type, AckStatus::OK);
        return;
      }
      if (!sm_.expand(mode == proto::ExpandMode::RETRACT, now_ms_)) {
        ack(m.seq, f.type, AckStatus::REJECTED);
        return;
      }
      exp_mode_ = mode == proto::ExpandMode::RETRACT ? ctl::ExpMode::RETRACT
                  : mode == proto::ExpandMode::FORCE ? ctl::ExpMode::FORCE
                                                      : ctl::ExpMode::DIAMETER;
      exp_target_ = m.target;
      if (mode == proto::ExpandMode::FORCE) grip_target_ = m.target;
      clearFlag(Fault::NO_CONTACT);
      expand_ttl_.arm(now_ms_, m.ttl_ms ? m.ttl_ms : static_cast<uint32_t>(params::EXPAND_TTL_MS));
      ack(m.seq, f.type, AckStatus::OK);
      return;
    }
    case MsgType::GIMBAL: {
      proto::GimbalMsg m;
      if (!m.unpack(f.payload, f.len)) return;
      last_seq_ = m.seq;
      if (s == State::LOW_POWER_SAFE) {
        ack(m.seq, f.type, AckStatus::REJECTED);
        return;
      }
      pan_cmd_ = m.pan_deg;
      tilt_cmd_ = ctl::clampf(m.tilt_deg, params::GIMBAL_TILT_MIN_DEG, params::GIMBAL_TILT_MAX_DEG);
      ack(m.seq, f.type, AckStatus::OK);
      return;
    }
    case MsgType::LIGHT: {
      proto::LightMsg m;
      if (!m.unpack(f.payload, f.len)) return;
      last_seq_ = m.seq;
      if (s == State::LOW_POWER_SAFE) {
        ack(m.seq, f.type, AckStatus::REJECTED);
        return;
      }
      white_ = m.white_pct > 100 ? 100 : m.white_pct;
      ir_ = m.ir_pct > 100 ? 100 : m.ir_pct;
      ack(m.seq, f.type, AckStatus::OK);
      return;
    }
    case MsgType::ESTOP: {
      proto::EstopMsg m;
      if (!m.unpack(f.payload, f.len)) return;
      last_seq_ = m.seq;
      estop();
      ack(m.seq, f.type, AckStatus::ESTOPPED);
      return;
    }
    case MsgType::CLEAR_FAULT: {
      proto::ClearFaultMsg m;
      if (!m.unpack(f.payload, f.len)) return;
      last_seq_ = m.seq;
      const bool cond_ok = !hw_.safety->estopInput() && exp_.encoderOk() && power_.driver_temp_c < params::THERMAL_TRIP_C;
      if (!cond_ok) {
        ack(m.seq, f.type, AckStatus::REJECTED);
        return;
      }
      for (int b = 0; b < 32; ++b) {
        if ((kAcknowledge & (1u << b)) && (faults_ & (1u << b))) clearFlag(static_cast<Fault>(b));
      }
      exp_.clearFaults();
      drive_.clearSlip();
      oc_drive_s_ = 0;
      hw_.safety->driveEnable(true);
      sm_.clearFault(true, now_ms_);
      ack(m.seq, f.type, AckStatus::OK);
      return;
    }
    case MsgType::PING: {
      proto::PingMsg m;
      if (!m.unpack(f.payload, f.len)) return;
      proto::PongMsg p;
      p.seq = m.seq;
      p.t_ms = m.t_ms;
      p.mcu_ms = now_ms_;
      send(p);
      return;
    }
    case MsgType::SET_PARAM: {
      proto::SetParamMsg m;
      if (!m.unpack(f.payload, f.len)) return;
      last_seq_ = m.seq;
      ctl::PidGains g = drive_.gains();
      bool ok = true;
      switch (static_cast<proto::Param>(m.param)) {
        case proto::Param::DRIVE_KP: g.kp = m.value; drive_.setGains(g); break;
        case proto::Param::DRIVE_KI: g.ki = m.value; drive_.setGains(g); break;
        case proto::Param::DRIVE_KD: g.kd = m.value; drive_.setGains(g); break;
        case proto::Param::EXPAND_KP: exp_.setForceGain(m.value); break;
        case proto::Param::EXPAND_KI: break;  // force loop is pure P + integrating actuator
        case proto::Param::DRIVE_CURRENT_A: drive_.setCurrentA(ctl::clampf(m.value, 0.1f, params::DRIVE_CURRENT_A)); break;
        case proto::Param::EXPAND_CURRENT_A: exp_.setCurrentA(ctl::clampf(m.value, 0.1f, params::EXPANSION_CURRENT_A)); break;
        case proto::Param::WHEEL_DIA_MM: ok = m.value > 0.5f * params::WHEEL_DIA_MM && m.value < 1.5f * params::WHEEL_DIA_MM; if (ok) drive_.setWheelDia(m.value); break;
        case proto::Param::GRIP_TARGET_N: ok = m.value > params::SPRING_PRELOAD_N && m.value <= params::MAX_GRIP_N; if (ok) grip_target_ = m.value; break;
        case proto::Param::ODOM_RESET: odom_.reset(); break;
        default: ok = false; break;
      }
      ack(m.seq, f.type, ok ? AckStatus::OK : AckStatus::BAD_ARG);
      if (ok) {
        proto::ParamValueMsg pv;
        pv.param = m.param;
        pv.value = m.value;
        send(pv);
      }
      return;
    }
    default:
      return;
  }
}

// ------------------------------------------------------------------------------- faults
void Robot::setFault(Fault f, bool latch) {
  const uint32_t bit = proto::faultBit(f);
  if (!(faults_ & bit)) {
    faults_ |= bit;
    sendEvent(proto::Event::FAULT_SET, static_cast<uint32_t>(f), 0.0f);
  }
  if (latch && sm_.state() != State::FAULT) {
    drive_v_ = 0;
    drive_ttl_.disarm();
    expand_ttl_.disarm();
    sm_.fault(now_ms_);
  }
}

void Robot::clearFlag(Fault f) {
  const uint32_t bit = proto::faultBit(f);
  if (faults_ & bit) {
    faults_ &= ~bit;
    sendEvent(proto::Event::FAULT_CLEARED, static_cast<uint32_t>(f), 0.0f);
  }
}

void Robot::estop() {
  hw_.safety->driveEnable(false);
  hw_.safety->expandEnable(false);
  for (auto* s : hw_.stepper) {
    s->setVelocity(0);
    s->enable(false);
  }
  setFault(Fault::ESTOP, true);
}

void Robot::reportStateChange() {
  State from, to;
  if (sm_.takeChanged(from, to)) sendEvent(proto::Event::STATE_CHANGE, static_cast<uint32_t>(from), static_cast<float>(to));
}

// ------------------------------------------------------------------------------- loops
void Robot::driveStep(float dt) {
  if (drive_ttl_.expiredNow(now_ms_)) {
    if (drive_v_ != 0.0f) setFault(Fault::COMMAND_STALE, false);
    drive_v_ = 0;
    sm_.drive(false, now_ms_);
  }
  ctl::DriveCmd cmd;
  const State s = sm_.state();
  cmd.mode = s == State::CRAWLING ? ctl::DriveMode::RUN : (s == State::HOLDING ? ctl::DriveMode::HOLD : ctl::DriveMode::OFF);
  cmd.v_mm_s = drive_v_;
  cmd.turn_deg_s = drive_turn_;
  cmd.bend_dir_deg = drive_dir_;
  cmd.diameter_mm = exp_.diameter();
  cmd.speed_scale = thermal_derate_ ? 0.5f : 1.0f;
  cmd.current_scale = thermal_derate_ ? 0.6f : 1.0f;
  drive_.step(hw_, cmd, dt);
  if (drive_.slipWarning() && !hasFault(Fault::SLIP_WARNING)) {
    setFault(Fault::SLIP_WARNING, false);
    drive_v_ = 0;
    drive_ttl_.disarm();
    sm_.drive(false, now_ms_);
  }
  ctl::OdomInput oi{};
  for (int i = 0; i < 3; ++i) {
    oi.arm_speed_mm_s[i] = drive_.measuredSpeed(i);
    oi.arm_ok[i] = drive_.encoderOk(i);
  }
  uint16_t tof = 0;
  oi.tof_ok = (s == State::CRAWLING) && hw_.sensors->tofFront(tof) && tof < 3500;
  oi.tof_mm = tof;
  oi.dt = dt;
  odom_.update(oi);
  if (odom_.groundSlip()) {
    if (!hasFault(Fault::GROUND_SLIP)) setFault(Fault::GROUND_SLIP, false);
  } else if (hasFault(Fault::GROUND_SLIP)) {
    clearFlag(Fault::GROUND_SLIP);
  }
  reportStateChange();
}

void Robot::expansionStep(float dt) {
  if (expand_ttl_.expiredNow(now_ms_)) sm_.expansionStale(exp_.allContact(), now_ms_);
  const State s = sm_.state();
  ctl::ExpCmd cmd;
  cmd.current_scale = thermal_derate_ ? 0.6f : 1.0f;
  if ((s == State::EXPANDING || s == State::RETRACTING) && expand_ttl_.active(now_ms_)) {
    cmd.mode = exp_mode_;
    cmd.target = exp_target_;
  } else if (s == State::GRIPPED || s == State::HOLDING || s == State::CRAWLING) {
    cmd.mode = ctl::ExpMode::MAINTAIN;
    cmd.target = grip_target_;
  }
  exp_.step(hw_, cmd, dt);
  if (exp_.stalled()) setFault(Fault::EXPANSION_STALL, true);
  if (exp_.overCurrent()) setFault(Fault::OVERCURRENT, true);
  if (!exp_.encoderOk()) setFault(Fault::ENCODER_FAIL, true);
  if ((s == State::EXPANDING || s == State::RETRACTING) && exp_.done() && cmd.mode != ctl::ExpMode::LOCKED) {
    if (exp_.noContact()) setFault(Fault::NO_CONTACT, false);
    if (s == State::EXPANDING && exp_.allContact()) sendEvent(proto::Event::CONTACT, 0, exp_.gripAvg());
    expand_ttl_.disarm();
    sm_.expansionDone(exp_.allContact(), now_ms_);
  }
  // grip monitor while gripped: stop crawling and let MAINTAIN re-grip
  const bool gripped_state = s == State::GRIPPED || s == State::HOLDING || s == State::CRAWLING;
  grip_low_s_ = (gripped_state && exp_.gripMin() < params::GRIP_LOW_N) ? grip_low_s_ + dt : 0.0f;
  if (grip_low_s_ > 0.1f && !hasFault(Fault::GRIP_LOW)) {
    setFault(Fault::GRIP_LOW, false);
    sendEvent(proto::Event::REGRIP, 0, exp_.gripMin());
    if (s == State::CRAWLING) {
      drive_v_ = 0;
      drive_ttl_.disarm();
      sm_.drive(false, now_ms_);
    }
  } else if (grip_low_s_ == 0.0f && hasFault(Fault::GRIP_LOW) && exp_.gripMin() >= params::GRIP_LOW_N) {
    clearFlag(Fault::GRIP_LOW);
  }
  if (gripped_state && exp_.noContact() && !hasFault(Fault::NO_CONTACT)) setFault(Fault::NO_CONTACT, false);
  reportStateChange();
}

void Robot::safetyStep(float dt) {
  power_ = hw_.power->read();
  if (hw_.safety->estopInput() && !hasFault(Fault::ESTOP)) estop();
  // drive over-current
  oc_drive_s_ = power_.drive_a > params::DRIVE_OVER_CURRENT_A ? oc_drive_s_ + dt : 0.0f;
  if (oc_drive_s_ > 0.1f) setFault(Fault::OVERCURRENT, true);
  // thermal
  if (power_.driver_temp_c > params::THERMAL_TRIP_C) setFault(Fault::OVERTEMP, true);
  const bool derate = power_.driver_temp_c > params::THERMAL_DERATE_C;
  if (derate != thermal_derate_) {
    thermal_derate_ = derate;
    if (derate) {
      setFault(Fault::THERMAL_DERATE, false);
    } else {
      clearFlag(Fault::THERMAL_DERATE);
    }
  }
  // supply: battery pack or tether
  bool low = false, critical = false;
  if (power_.mode == proto::SupplyMode::BATTERY && params::BATTERY_SERIES > 0) {
    low = power_.battery_v < params::BATTERY_LOW_V;
    critical = power_.battery_v < params::BATTERY_CRITICAL_V;
    if (low) setFault(Fault::LOW_BATTERY, false);
    else if (power_.battery_v > params::BATTERY_LOW_V + 0.3f) clearFlag(Fault::LOW_BATTERY);
  } else {
    critical = power_.bus_v < params::SUPPLY_MIN_V;
  }
  low_v_s_ = critical ? low_v_s_ + dt : 0.0f;
  ok_v_s_ = critical ? 0.0f : ok_v_s_ + dt;
  if (low_v_s_ > 0.5f) {
    setFault(power_.mode == proto::SupplyMode::BATTERY ? Fault::CRITICAL_BATTERY : Fault::SUPPLY_UNDERVOLT, false);
    if (sm_.state() != State::LOW_POWER_SAFE && sm_.state() != State::FAULT) {
      drive_v_ = 0;
      drive_ttl_.disarm();
      expand_ttl_.disarm();
      sm_.lowPower(true, now_ms_);
    }
  } else if (ok_v_s_ > 2.0f && sm_.state() == State::LOW_POWER_SAFE) {
    clearFlag(Fault::CRITICAL_BATTERY);
    clearFlag(Fault::SUPPLY_UNDERVOLT);
    sm_.lowPower(false, now_ms_);
  }
  // heartbeat: Pi silent -> stop driving, keep the grip
  if (heartbeat_.lost(now_ms_)) {
    if (!hasFault(Fault::HEARTBEAT_LOST)) setFault(Fault::HEARTBEAT_LOST, false);
    if (sm_.state() == State::CRAWLING) {
      drive_v_ = 0;
      drive_ttl_.disarm();
      sm_.drive(false, now_ms_);
    }
  }
  const hal::ImuData imu = hw_.sensors->imu();
  if (!imu.ok) {
    if (!hasFault(Fault::IMU_FAIL)) setFault(Fault::IMU_FAIL, false);
  } else if (hasFault(Fault::IMU_FAIL)) {
    clearFlag(Fault::IMU_FAIL);
  }
  sm_.holdTimeout(now_ms_, kHoldToGrippedMs);
  const bool lp = sm_.state() == State::LOW_POWER_SAFE;
  hw_.lights->set(lp ? 0 : white_, lp ? 0 : ir_);
  if (!lp) hw_.gimbal->command(pan_cmd_, tilt_cmd_);
  reportStateChange();
}

void Robot::telemetryStep() {
  comm::TelemetryInputs in{};
  in.t_ms = now_ms_;
  in.state = sm_.state();
  in.faults = faults_;
  in.exp = &exp_;
  in.drive = &drive_;
  in.odom = &odom_;
  in.imu = hw_.sensors->imu();
  in.env = hw_.sensors->env();
  in.power = power_;
  in.tof_front_ok = hw_.sensors->tofFront(in.tof_front);
  in.tof_rear_ok = hw_.sensors->tofRear(in.tof_rear);
  in.pan = hw_.gimbal->pan();
  in.tilt = hw_.gimbal->tilt();
  in.white = white_;
  in.ir = ir_;
  in.last_seq = last_seq_;
  in.rx_errors = parser_.errors();
  send(telem_.build(in));
}

}  // namespace core
