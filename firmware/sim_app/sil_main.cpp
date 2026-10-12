// PIPEBOT-X software-in-the-loop simulator.
//
// Runs the real firmware core (src/core, src/control, src/comm) against the physics
// plant (src/sim) with the same wire protocol as the hardware.
//
//   pipebot_sil_<S|M|L> --scenario file.json --out log.csv [--summary out.json]
//        fast-time: a scripted operator plays the scenario's command timeline
//   pipebot_sil_<S|M|L> --scenario file.json --tcp 7000 [--speed 1.0]
//        real-time: the Pi software connects over TCP and drives the robot
#include <arpa/inet.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <sys/socket.h>
#include <unistd.h>

#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <memory>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include "bot_params.h"
#include "comm/serial_protocol.h"
#include "core/robot.h"
#include "json.h"
#include "protocol_gen.h"
#include "sim/sim_hal.h"

using proto::State;

namespace {

const char* kStateNames[] = {"IDLE", "EXPANDING", "GRIPPED", "CRAWLING", "HOLDING", "RETRACTING", "FAULT", "LOW_POWER_SAFE"};

State stateFromName(const std::string& s) {
  for (int i = 0; i < 8; ++i)
    if (s == kStateNames[i]) return static_cast<State>(i);
  return State::IDLE;
}

struct Step {
  std::string type;
  mjson::Value v;
  double t_min = 0;
};

struct EventSpec {
  double t = 0;
  std::string type;
  double value = 0;
  double duration = 0;
};

sim::PlantConfig plantConfig(const mjson::Value& p) {
  sim::PlantConfig c;
  for (const auto& s : p["sections"].a) {
    sim::PipeSection ps;
    ps.start_m = s.num("start_m", 0);
    ps.end_m = s.num("end_m", 1e9);
    ps.id_mm = s.num("id_mm", 0.5 * (params::PIPE_ID_MIN_MM + params::PIPE_ID_MAX_MM));
    ps.incline_deg = s.num("incline_deg", 0);
    ps.wet = s.boolean("wet", false);
    ps.water_depth_m = s.num("water_depth_m", 0);
    ps.gas_ppm = s.num("gas_ppm", 0);
    ps.mu_dry = s.num("mu_dry", -1);
    ps.mu_wet = s.num("mu_wet", -1);
    c.sections.push_back(ps);
  }
  c.pipe_end_m = p.num("pipe_end_m", 1e9);
  c.x0_m = p.num("x0_m", 0.5);
  c.theta0 = p.num("theta0", -1);
  c.start_gripped = p.boolean("start_gripped", false);
  c.roll_deg = p.num("roll_deg", 0);
  c.tether = p.boolean("tether", true);
  c.ambient_c = p.num("ambient_c", 25);
  c.tether_drag_n_per_m = p.num("tether_drag_n_per_m", 0.05);
  c.extra_mass_kg = p.num("extra_mass_kg", 0);
  c.battery_soc = p.num("battery_soc", 1.0);
  c.noise = p.num("noise", 1.0);
  c.seed = static_cast<uint32_t>(p.num("seed", 1));
  return c;
}

class Sil {
 public:
  Sil(const mjson::Value& sc, double link_ber) : rig_(plantConfig(sc["plant"])), ber_(link_ber) {
    for (const auto& s : sc["operator"].a) steps_.push_back({s.str("cmd", "WAIT"), s, s.num("t", 0)});
    for (const auto& e : sc["events"].a) events_.push_back({e.num("t", 0), e.str("type", ""), e.num("value", 0), e.num("duration_s", 0)});
    duration_ = sc.num("duration_s", 30);
    log_dt_ = 1.0 / sc.num("log_hz", params::TELEMETRY_HZ);
    hw_ = rig_.hw();
  }

  sim::SimRig rig_;
  hal::Hardware hw_{};
  std::unique_ptr<core::Robot> robot_;
  double t_ = 0, duration_ = 30, log_dt_ = 0.05, next_log_ = 0;
  uint64_t us_ = 0;
  double ber_;
  uint32_t rng_ = 12345;
  // operator
  std::vector<Step> steps_;
  std::vector<EventSpec> events_;
  size_t step_i_ = 0;
  double step_t0_ = -1, next_send_ = 0, next_ping_ = 0, dist0_ = 0, silent_until_ = -1, pull_until_ = -1;
  bool step_seen_busy_ = false;
  uint16_t seq_ = 1;
  comm::FrameParser rx_;
  proto::TelemetryMsg tel_{};
  bool have_tel_ = false;
  uint32_t tel_count_ = 0, boots_ = 0;
  double first_tel_t_ = -1;
  std::vector<std::string> step_results_, event_log_;
  // metrics
  double min_bus_v_ = 1e9, max_temp_ = -1e9, max_axis_off_ = 0;
  std::FILE* csv_ = nullptr;

  uint32_t rnd() { return rng_ = rng_ * 1664525u + 1013904223u; }
  void corrupt(uint8_t* b, size_t n) {
    if (ber_ <= 0) return;
    for (size_t i = 0; i < n; ++i)
      for (int k = 0; k < 8; ++k)
        if ((rnd() >> 8) * (1.0 / 16777216.0) < ber_) b[i] ^= static_cast<uint8_t>(1u << k);
  }

  template <typename Msg>
  void send(Msg m) {
    if (t_ < silent_until_) return;
    uint8_t buf[proto::kMaxPayload + proto::kFrameOverhead];
    const size_t n = proto::encode(m, buf, sizeof(buf));
    corrupt(buf, n);
    rig_.link.to_mcu.insert(rig_.link.to_mcu.end(), buf, buf + n);
  }

  void bootIfNeeded() {
    if (rig_.plant.mcu_powered && !robot_) {
      robot_ = std::make_unique<core::Robot>(hw_);
      robot_->begin(static_cast<uint32_t>(us_));
      ++boots_;
      rig_.link.to_mcu.clear();
    } else if (!rig_.plant.mcu_powered && robot_) {
      robot_.reset();  // brown-out: MCU dead, motors unpowered, worms hold
      rig_.link.from_mcu.clear();
      for (int i = 0; i < 4; ++i) {
        rig_.plant.stepper_en[i] = false;
        rig_.plant.stepper_vel[i] = 0;
      }
    }
  }

  void applyEvents() {
    for (auto& e : events_) {
      if (e.t < 0 || t_ < e.t) continue;
      auto& p = rig_.plant;
      if (e.type == "POWER_LOSS") p.tether_connected = false;
      else if (e.type == "POWER_RESTORE") p.tether_connected = true;
      else if (e.type == "PULL") { p.pull_force_n = e.value; pull_until_ = e.duration > 0 ? t_ + e.duration : -1; }
      else if (e.type == "EXTRA_MASS") p.extra_mass_kg = e.value;
      else if (e.type == "WET") p.wet_override = e.value != 0;
      else if (e.type == "ESTOP_LOOP") p.estop_loop_open = e.value != 0;
      else if (e.type == "PI_SILENT") silent_until_ = t_ + e.value;
      else if (e.type == "IMU_FAIL") rig_.sensors.imu_fail = e.value != 0;
      else if (e.type == "LEVER_ENC_FAIL") rig_.lever.fail = e.value != 0;
      else if (e.type == "AMBIENT") p.ambient_c = e.value;
      else if (e.type == "LINK_BER") ber_ = e.value;
      char buf[160];
      std::snprintf(buf, sizeof(buf), "{\"t\":%.3f,\"sim_event\":\"%s\",\"value\":%g}", t_, e.type.c_str(), e.value);
      event_log_.push_back(buf);
      e.t = -1;  // consumed
    }
    if (pull_until_ > 0 && t_ >= pull_until_) {
      rig_.plant.pull_force_n = 0;
      pull_until_ = -1;
    }
  }

  void readRobotOutput() {
    auto& out = rig_.link.from_mcu;
    corrupt(out.data(), out.size());
    for (uint8_t b : out) {
      comm::Frame f{};
      if (!rx_.feed(b, f)) continue;
      if (f.type == proto::MsgType::TELEMETRY && tel_.unpack(f.payload, f.len)) {
        have_tel_ = true;
        if (first_tel_t_ < 0) first_tel_t_ = t_;
        ++tel_count_;
      } else if (f.type == proto::MsgType::EVENT) {
        proto::EventMsg e;
        if (e.unpack(f.payload, f.len)) {
          char buf[200];
          std::snprintf(buf, sizeof(buf), "{\"t\":%.3f,\"event\":%u,\"state\":\"%s\",\"arg\":%u,\"value\":%g}", t_, e.code,
                        kStateNames[e.state < 8 ? e.state : 0], e.arg, e.value);
          event_log_.push_back(buf);
        }
      } else if (f.type == proto::MsgType::ACK) {
        proto::AckMsg a;
        if (a.unpack(f.payload, f.len) && a.status != 0) {
          char buf[120];
          std::snprintf(buf, sizeof(buf), "{\"t\":%.3f,\"ack\":%u,\"cmd\":%u,\"status\":%u}", t_, a.seq, a.cmd, a.status);
          event_log_.push_back(buf);
        }
      }
    }
    out.clear();
  }

  State robotState() const { return have_tel_ ? static_cast<State>(tel_.state) : State::IDLE; }

  void finishStep(bool ok, const std::string& note) {
    char buf[260];
    std::snprintf(buf, sizeof(buf), "{\"i\":%zu,\"cmd\":\"%s\",\"t_start\":%.3f,\"t_end\":%.3f,\"state\":\"%s\",\"ok\":%s,\"note\":\"%s\"}",
                  step_i_, steps_[step_i_].type.c_str(), step_t0_, t_, kStateNames[static_cast<int>(robotState())], ok ? "true" : "false",
                  note.c_str());
    step_results_.push_back(buf);
    ++step_i_;
    step_t0_ = -1;
  }

  void operatorTick() {
    if (t_ >= next_ping_) {
      proto::PingMsg p;
      p.seq = seq_++;
      p.t_ms = static_cast<uint32_t>(t_ * 1000);
      send(p);
      next_ping_ = t_ + 0.25;
    }
    if (step_i_ >= steps_.size()) return;
    Step& s = steps_[step_i_];
    if (step_t0_ < 0) {
      if (t_ < s.t_min) return;
      step_t0_ = t_;
      next_send_ = t_;
      step_seen_busy_ = false;
      dist0_ = have_tel_ ? tel_.distance_m : 0;
    }
    const double el = t_ - step_t0_;
    const std::string& ty = s.type;
    const double refresh = s.v.num("refresh_ms", ty == "DRIVE" ? 100 : 150) / 1000.0;
    if (ty == "WAIT") {
      if (el >= s.v.num("duration_s", 1)) finishStep(true, "");
    } else if (ty == "WAIT_STATE") {
      if (robotState() == stateFromName(s.v.str("state", "IDLE"))) finishStep(true, "");
      else if (el > s.v.num("timeout_s", 30)) finishStep(false, "timeout");
    } else if (ty == "EXPAND") {
      const std::string mode = s.v.str("mode", "FORCE");
      const State st = robotState();
      const bool busy = st == State::EXPANDING || st == State::RETRACTING;
      if (busy) step_seen_busy_ = true;
      // same rule as the Pi's refresh job: done once it was busy and stopped, or never got
      // busy within 1 s (the firmware skips a grip that is already on target)
      if (!busy && ((step_seen_busy_ && el > 0.3) || el > 1.0)) {
        finishStep(true, "");
        return;
      }
      if (el > s.v.num("timeout_s", 40)) {
        finishStep(false, "timeout");
        return;
      }
      if (t_ >= next_send_) {
        proto::ExpandMsg m;
        m.seq = seq_++;
        m.mode = static_cast<uint8_t>(mode == "DIAMETER" ? proto::ExpandMode::DIAMETER
                                      : mode == "RETRACT"  ? proto::ExpandMode::RETRACT
                                                           : proto::ExpandMode::FORCE);
        m.target = static_cast<float>(s.v.num("target", params::DEFAULT_GRIP_TARGET_N));
        m.ttl_ms = static_cast<uint16_t>(s.v.num("ttl_ms", 500));
        send(m);
        next_send_ = t_ + refresh;
      }
    } else if (ty == "DRIVE") {
      const double dur = s.v.num("duration_s", 1e9);
      const double dist = s.v.num("distance_m", 1e9);
      const bool reached = have_tel_ && std::fabs(tel_.distance_m - dist0_) >= dist;
      if (el >= dur || reached || el > s.v.num("timeout_s", 600)) {
        proto::DriveMsg m;
        m.seq = seq_++;
        m.ttl_ms = 200;
        send(m);
        finishStep(el < s.v.num("timeout_s", 600), reached ? "distance" : "duration");
        return;
      }
      if (t_ >= next_send_) {
        proto::DriveMsg m;
        m.seq = seq_++;
        m.linear_mm_s = static_cast<int16_t>(s.v.num("speed", 50));
        m.turn_deg_s = static_cast<int16_t>(s.v.num("turn", 0));
        m.bend_dir_deg = static_cast<int16_t>(s.v.num("bend_dir", 0));
        m.ttl_ms = static_cast<uint16_t>(s.v.num("ttl_ms", 300));
        send(m);
        next_send_ = t_ + refresh;
      }
    } else if (ty == "GIMBAL") {
      proto::GimbalMsg m;
      m.seq = seq_++;
      m.pan_deg = static_cast<float>(s.v.num("pan", 0));
      m.tilt_deg = static_cast<float>(s.v.num("tilt", 0));
      send(m);
      finishStep(true, "");
    } else if (ty == "LIGHT") {
      proto::LightMsg m;
      m.seq = seq_++;
      m.white_pct = static_cast<uint8_t>(s.v.num("white", 0));
      m.ir_pct = static_cast<uint8_t>(s.v.num("ir", 0));
      send(m);
      finishStep(true, "");
    } else if (ty == "ESTOP") {
      proto::EstopMsg m;
      m.seq = seq_++;
      send(m);
      finishStep(true, "");
    } else if (ty == "CLEAR_FAULT") {
      proto::ClearFaultMsg m;
      m.seq = seq_++;
      send(m);
      finishStep(true, "");
    } else if (ty == "SET_PARAM") {
      proto::SetParamMsg m;
      m.seq = seq_++;
      m.param = static_cast<uint16_t>(s.v.num("param", 0));
      m.value = static_cast<float>(s.v.num("value", 0));
      send(m);
      finishStep(true, "");
    } else {
      finishStep(false, "unknown step");
    }
  }

  void logRow() {
    if (!csv_ || !have_tel_) return;
    const auto& p = rig_.plant;
    const auto& tl = tel_;
    std::fprintf(csv_,
                 "%.3f,%s,%u,%.2f,%.3f,%.2f,%.2f,%.2f,%.2f,%.2f,%.4f,%.3f,%.2f,%.2f,%u,%u,%.3f,%.3f,%.3f,%.3f,%.1f,%u,%u,%.1f,%.1f,%u,%u,%u,"
                 "%.5f,%.5f,%d,%.1f,%.2f,%.2f,%.2f,%.3f,%.3f,%.3f,%d,%.2f,%.4f,%.5f,%.2f,%.2f,%d\n",
                 t_, kStateNames[tl.state < 8 ? tl.state : 0], tl.faults, tl.est_diameter_mm, tl.lever_angle_deg, tl.grip_force_n[0],
                 tl.grip_force_n[1], tl.grip_force_n[2], tl.axis_offset_mm, tl.linear_mm_s, tl.distance_m, tl.odom_confidence, tl.pitch,
                 tl.battery_v, tl.battery_pct, tl.supply_mode, tl.motor_current_a[0], tl.motor_current_a[1], tl.motor_current_a[2],
                 tl.motor_current_a[3], tl.driver_temp_c, tl.tof_front_mm, tl.tof_rear_mm, tl.pan_deg, tl.tilt_deg, tl.last_seq, tl.rx_errors,
                 tel_count_, p.x_m, p.v_m_s, p.slipping ? 1 : 0, p.pipeIdAt(p.x_m), p.arm[0].force_n, p.arm[1].force_n, p.arm[2].force_n,
                 p.offset_x_mm, p.offset_y_mm, p.max_slip_mm, p.mcu_powered ? 1 : 0, p.bus_v, p.soc, p.theta, p.driver_temp_c,
                 p.extra_mass_kg, boots_);
  }

  void step1ms() {
    const double dt = 0.001;
    applyEvents();
    rig_.plant.step(dt);
    bootIfNeeded();
    us_ += 1000;
    if (robot_) robot_->tick(static_cast<uint32_t>(us_));
    t_ += dt;
    min_bus_v_ = std::fmin(min_bus_v_, rig_.plant.bus_v);
    max_temp_ = std::fmax(max_temp_, rig_.plant.driver_temp_c);
    max_axis_off_ = std::fmax(max_axis_off_, std::hypot(rig_.plant.offset_x_mm, rig_.plant.offset_y_mm));
  }

  int runScripted(const std::string& csv_path, const std::string& summary_path, const std::string& events_path) {
    csv_ = csv_path.empty() ? nullptr : std::fopen(csv_path.c_str(), "w");
    if (csv_)
      std::fprintf(csv_,
                   "t,state,faults,est_diameter_mm,lever_deg,grip0,grip1,grip2,axis_offset_mm,linear_mm_s,distance_m,odom_conf,pitch,"
                   "battery_v,battery_pct,supply,cur0,cur1,cur2,cur3,driver_temp_c,tof_front,tof_rear,pan,tilt,last_seq,rx_errors,tel_count,"
                   "x_m,v_m_s,slipping,pipe_id_mm,f0_true,f1_true,f2_true,off_x_mm,off_y_mm,max_slip_mm,mcu_powered,bus_v,soc,theta,"
                   "driver_temp_true,extra_mass_kg,boots\n");
    while (t_ < duration_) {
      step1ms();
      readRobotOutput();
      operatorTick();
      if (t_ >= next_log_) {
        logRow();
        next_log_ += log_dt_;
      }
    }
    if (csv_) std::fclose(csv_);
    if (!events_path.empty()) {
      std::ofstream ev(events_path);
      for (const auto& e : event_log_) ev << e << "\n";
    }
    std::ofstream sm(summary_path.empty() ? "/dev/null" : summary_path);
    const auto& p = rig_.plant;
    sm << "{\n \"size\": \"" << params::NAME << "\",\n \"duration_s\": " << t_ << ",\n \"final_state\": \""
       << kStateNames[static_cast<int>(robotState())] << "\",\n \"faults\": " << (have_tel_ ? tel_.faults : 0)
       << ",\n \"telemetry_packets\": " << tel_count_ << ",\n \"first_telemetry_s\": " << first_tel_t_
       << ",\n \"expected_packets\": " << static_cast<long>((t_ - std::fmax(0.0, first_tel_t_)) * params::TELEMETRY_HZ) + 1
       << ",\n \"rx_errors\": " << (have_tel_ ? tel_.rx_errors : 0) << ",\n \"host_crc_rejects\": " << rx_.errors() << ",\n \"boots\": " << boots_ << ",\n \"x_m\": " << p.x_m
       << ",\n \"distance_m\": " << (have_tel_ ? tel_.distance_m : 0) << ",\n \"max_slip_mm\": " << p.max_slip_mm
       << ",\n \"min_bus_v\": " << min_bus_v_ << ",\n \"max_driver_temp_c\": " << max_temp_ << ",\n \"max_axis_offset_mm\": "
       << max_axis_off_ << ",\n \"soc\": " << p.soc << ",\n \"steps\": [\n";
    for (size_t i = 0; i < step_results_.size(); ++i) sm << "  " << step_results_[i] << (i + 1 < step_results_.size() ? ",\n" : "\n");
    sm << " ]\n}\n";
    std::printf("SIL %c: %.1f s simulated, final state %s, %u telemetry packets, %u boots\n", params::NAME, t_,
                kStateNames[static_cast<int>(robotState())], tel_count_, boots_);
    return 0;
  }

  int runTcp(int port, double speed) {
    int srv = socket(AF_INET, SOCK_STREAM, 0);
    int one = 1;
    setsockopt(srv, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));
    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    addr.sin_port = htons(static_cast<uint16_t>(port));
    if (bind(srv, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0 || listen(srv, 1) < 0) {
      std::perror("bind/listen");
      return 2;
    }
    fcntl(srv, F_SETFL, O_NONBLOCK);
    std::printf("SIL %c listening on tcp://0.0.0.0:%d (speed x%.1f)\n", params::NAME, port, speed);
    std::fflush(stdout);
    int cli = -1;
    auto t0 = std::chrono::steady_clock::now();
    double next_status = 0;
    while (true) {
      if (cli < 0) {
        cli = accept(srv, nullptr, nullptr);
        if (cli >= 0) {
          fcntl(cli, F_SETFL, O_NONBLOCK);
          setsockopt(cli, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));
          std::printf("client connected\n");
          std::fflush(stdout);
        }
      }
      const double wall = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count() * speed;
      int guard = 0;
      while (t_ < wall && guard++ < 50) {
        step1ms();
        if (cli >= 0) {
          uint8_t buf[512];
          ssize_t n = recv(cli, buf, sizeof(buf), 0);
          if (n > 0) rig_.link.to_mcu.insert(rig_.link.to_mcu.end(), buf, buf + n);
          else if (n == 0) {
            close(cli);
            cli = -1;
            std::printf("client disconnected\n");
          }
          auto& out = rig_.link.from_mcu;
          if (!out.empty()) {
            ::send(cli, out.data(), out.size(), MSG_NOSIGNAL);
            out.clear();
          }
        } else {
          rig_.link.from_mcu.clear();
        }
      }
      if (t_ > next_status) {
        const auto& p = rig_.plant;
        std::printf("t=%7.1f x=%6.2f m v=%5.0f mm/s pipe=%3.0f mm theta=%5.1f deg F=[%5.1f %5.1f %5.1f] bus=%4.1f V %s\n", t_, p.x_m,
                    p.v_m_s * 1000, p.pipeIdAt(p.x_m), p.theta * 57.2958, p.arm[0].force_n, p.arm[1].force_n, p.arm[2].force_n, p.bus_v,
                    robot_ ? "" : "(MCU off)");
        std::fflush(stdout);
        next_status = t_ + 5.0;
      }
      applyEvents();
      std::this_thread::sleep_for(std::chrono::microseconds(500));
    }
  }
};

}  // namespace

int main(int argc, char** argv) {
  std::string scen, out, summary, events;
  int port = 0;
  double speed = 1.0, ber = 0.0;
  for (int i = 1; i < argc; ++i) {
    std::string a = argv[i];
    auto next = [&]() { return i + 1 < argc ? std::string(argv[++i]) : std::string(); };
    if (a == "--scenario") scen = next();
    else if (a == "--out") out = next();
    else if (a == "--summary") summary = next();
    else if (a == "--events") events = next();
    else if (a == "--tcp") port = std::stoi(next());
    else if (a == "--speed") speed = std::stod(next());
    else if (a == "--ber") ber = std::stod(next());
    else {
      std::fprintf(stderr, "usage: %s --scenario f.json [--out log.csv --summary s.json --events e.jsonl | --tcp PORT --speed X]\n", argv[0]);
      return 1;
    }
  }
  std::string txt = "{\"plant\":{\"sections\":[{}]},\"duration_s\":1e9}";
  if (!scen.empty()) {
    std::ifstream f(scen);
    if (!f) {
      std::fprintf(stderr, "cannot read %s\n", scen.c_str());
      return 1;
    }
    std::stringstream ss;
    ss << f.rdbuf();
    txt = ss.str();
  }
  mjson::Value sc;
  try {
    sc = mjson::Parser(txt).parse();
  } catch (const std::exception& e) {
    std::fprintf(stderr, "scenario: %s\n", e.what());
    return 1;
  }
  auto sil = std::make_unique<Sil>(sc, ber);
  if (port) return sil->runTcp(port, speed);
  return sil->runScripted(out, summary, events);
}
