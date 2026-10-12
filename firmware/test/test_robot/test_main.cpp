// Firmware core against the physics plant: TTL, watchdog, E-stop, brown-out, re-grip, hold.
#include <unity.h>

#include <memory>

#include "bot_params.h"
#include "comm/serial_protocol.h"
#include "core/robot.h"
#include "sim/sim_hal.h"

using proto::Fault;
using proto::State;

namespace {
struct Harness {
  std::unique_ptr<sim::SimRig> rig;
  hal::Hardware hw{};
  std::unique_ptr<core::Robot> robot;
  uint64_t us = 0;
  uint16_t seq = 1;
  comm::FrameParser rx;
  int acks_ok = 0, acks_bad = 0;

  explicit Harness(sim::PlantConfig cfg) {
    rig = std::make_unique<sim::SimRig>(cfg);
    hw = rig->hw();
    rig->plant.step(0.001);  // sensors settle before the MCU boots
    robot = std::make_unique<core::Robot>(hw);
    robot->begin(0);
  }
  template <typename M>
  void send(M m) {
    uint8_t b[256];
    const size_t n = proto::encode(m, b, sizeof(b));
    rig->link.to_mcu.insert(rig->link.to_mcu.end(), b, b + n);
  }
  void ping() {
    proto::PingMsg p;
    p.seq = seq++;
    send(p);
  }
  void drive(int v, uint16_t ttl = 300) {
    proto::DriveMsg d;
    d.seq = seq++;
    d.linear_mm_s = static_cast<int16_t>(v);
    d.ttl_ms = ttl;
    send(d);
  }
  void expandForce(float n) {
    proto::ExpandMsg e;
    e.seq = seq++;
    e.mode = static_cast<uint8_t>(proto::ExpandMode::FORCE);
    e.target = n;
    e.ttl_ms = 500;
    send(e);
  }
  void run(double s, bool keep_alive = true) {
    for (int i = 0; i < static_cast<int>(s * 1000); ++i) {
      rig->plant.step(0.001);
      us += 1000;
      if (keep_alive && i % 250 == 0) ping();
      robot->tick(static_cast<uint32_t>(us));
      for (uint8_t b : rig->link.from_mcu) {
        comm::Frame f{};
        if (rx.feed(b, f) && f.type == proto::MsgType::ACK) {
          proto::AckMsg a;
          a.unpack(f.payload, f.len);
          (a.status == 0 ? acks_ok : acks_bad)++;
        }
      }
      rig->link.from_mcu.clear();
    }
  }
  // keep sending a command while time passes (dead-man refresh)
  template <typename F>
  void runWith(double s, double period, F f) {
    for (double t = 0; t < s; t += period) {
      f();
      run(period);
    }
  }
};

sim::PlantConfig vertical(bool gripped) {
  sim::PlantConfig c;
  sim::PipeSection p;
  p.id_mm = 0.5 * (params::PIPE_ID_MIN_MM + params::PIPE_ID_MAX_MM);
  p.incline_deg = 90;
  c.sections.push_back(p);
  c.x0_m = 1.0;
  c.start_gripped = gripped;
  return c;
}
}  // namespace

void setUp() {}
void tearDown() {}

void test_boot_inside_pipe_assumes_grip() {
  Harness h(vertical(true));
  TEST_ASSERT_EQUAL(State::HOLDING, h.robot->state());
}

void test_expand_force_then_climb_and_ttl_stop() {
  Harness h(vertical(true));
  h.runWith(3.0, 0.15, [&] { h.expandForce(params::DEFAULT_GRIP_TARGET_N); });
  TEST_ASSERT_TRUE(h.robot->state() == State::GRIPPED || h.robot->state() == State::HOLDING);
  TEST_ASSERT_TRUE(h.robot->expansion().allContact());
  const double x0 = h.rig->plant.x_m;
  h.runWith(2.0, 0.1, [&] { h.drive(50); });
  TEST_ASSERT_EQUAL(State::CRAWLING, h.robot->state());
  TEST_ASSERT_TRUE(h.rig->plant.x_m - x0 > 0.07);  // climbed against gravity
  h.run(0.6);                                       // stop refreshing: TTL (300 ms) must stop it
  TEST_ASSERT_EQUAL(State::HOLDING, h.robot->state());
  const double x1 = h.rig->plant.x_m;
  h.run(2.0);
  TEST_ASSERT_DOUBLE_WITHIN(0.0005, x1, h.rig->plant.x_m);  // holds on a vertical wall
}

void test_heartbeat_loss_stops_but_keeps_grip() {
  Harness h(vertical(true));
  h.runWith(2.0, 0.1, [&] { h.drive(40, 1000); });
  TEST_ASSERT_EQUAL(State::CRAWLING, h.robot->state());
  h.run(1.5, false);  // Pi silent
  TEST_ASSERT_TRUE(h.robot->hasFault(Fault::HEARTBEAT_LOST));
  TEST_ASSERT_EQUAL(State::HOLDING, h.robot->state());
  TEST_ASSERT_TRUE(h.robot->expansion().allContact());
}

void test_estop_cuts_enable_lines_and_needs_clear() {
  Harness h(vertical(true));
  h.runWith(1.0, 0.1, [&] { h.drive(40); });
  proto::EstopMsg e;
  e.seq = h.seq++;
  h.send(e);
  h.run(0.05);
  TEST_ASSERT_EQUAL(State::FAULT, h.robot->state());
  TEST_ASSERT_FALSE(h.rig->plant.drive_enable_line);
  const int bad = h.acks_bad;
  h.drive(40);
  h.run(0.05);
  TEST_ASSERT_TRUE(h.acks_bad > bad);  // motion refused while E-stopped
  const double x = h.rig->plant.x_m;
  h.run(2.0);
  TEST_ASSERT_DOUBLE_WITHIN(0.0005, x, h.rig->plant.x_m);  // still gripping
  proto::ClearFaultMsg c;
  c.seq = h.seq++;
  h.send(c);
  h.run(0.05);
  TEST_ASSERT_EQUAL(State::HOLDING, h.robot->state());
  TEST_ASSERT_TRUE(h.rig->plant.drive_enable_line);
}

void test_tether_brownout_on_tether_only_robot_holds_mechanically() {
  sim::PlantConfig c = vertical(true);
  Harness h(c);
  h.rig->plant.tether_connected = false;
  for (int i = 0; i < 3000; ++i) h.rig->plant.step(0.001);  // MCU may be dead: plant only
  TEST_ASSERT_TRUE(h.rig->plant.max_slip_mm < 0.5);
}

void test_regrip_when_pipe_widens() {
  sim::PlantConfig c = vertical(true);
  c.sections[0].end_m = 1.15;
  sim::PipeSection wide = c.sections[0];
  wide.start_m = 1.15;
  wide.end_m = 100;
  wide.id_mm = c.sections[0].id_mm + 6;  // a 6 mm step (joint / reducer)
  c.sections.push_back(wide);
  c.sections[0].incline_deg = c.sections[1].incline_deg = 0;
  Harness h(c);
  h.runWith(2.0, 0.15, [&] { h.expandForce(params::DEFAULT_GRIP_TARGET_N); });
  h.runWith(4.0, 0.1, [&] { h.drive(50); });
  h.run(3.0);  // MAINTAIN re-grips in the wider section
  TEST_ASSERT_TRUE(h.rig->plant.x_m > 1.15);
  TEST_ASSERT_TRUE(h.robot->expansion().allContact());
  TEST_ASSERT_TRUE(h.robot->expansion().gripMin() > params::GRIP_LOW_N);
}

void test_diameter_estimate_while_gripping() {
  for (const double incline : {90.0, 0.0}) {
    sim::PlantConfig c = vertical(true);
    c.sections[0].incline_deg = incline;
    Harness h(c);
    h.runWith(3.0, 0.15, [&] { h.expandForce(params::DEFAULT_GRIP_TARGET_N); });
    // the slides are mid-travel when gripping: ignoring them reads ~2 x 5 mm too large
    TEST_ASSERT_FLOAT_WITHIN(1.5f, c.sections[0].id_mm, h.robot->expansion().diameter());
  }
}

int main() {
  UNITY_BEGIN();
  RUN_TEST(test_boot_inside_pipe_assumes_grip);
  RUN_TEST(test_expand_force_then_climb_and_ttl_stop);
  RUN_TEST(test_heartbeat_loss_stops_but_keeps_grip);
  RUN_TEST(test_estop_cuts_enable_lines_and_needs_clear);
  RUN_TEST(test_tether_brownout_on_tether_only_robot_holds_mechanically);
  RUN_TEST(test_regrip_when_pipe_widens);
  RUN_TEST(test_diameter_estimate_while_gripping);
  return UNITY_END();
}
