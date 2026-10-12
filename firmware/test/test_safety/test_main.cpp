// Heartbeat watchdog, command TTL, odometry robustness.
#include <unity.h>

#include "control/odometry.h"
#include "safety/watchdog.h"

void setUp() {}
void tearDown() {}

void test_heartbeat() {
  safety::Heartbeat hb(1000);
  TEST_ASSERT_TRUE(hb.lost(0));  // never heard from the Pi
  hb.touch(100);
  TEST_ASSERT_FALSE(hb.lost(1100));
  TEST_ASSERT_TRUE(hb.lost(1101));
}

void test_heartbeat_wraps() {
  safety::Heartbeat hb(1000);
  hb.touch(0xFFFFFF00u);
  TEST_ASSERT_FALSE(hb.lost(0x00000100u));  // across the 32-bit wrap
}

void test_ttl() {
  safety::CommandTtl t;
  TEST_ASSERT_FALSE(t.active(0));
  t.arm(1000, 200);
  TEST_ASSERT_TRUE(t.active(1200));
  TEST_ASSERT_FALSE(t.expiredNow(1200));
  TEST_ASSERT_TRUE(t.expiredNow(1201));
  TEST_ASSERT_FALSE(t.expiredNow(1300));  // reported once
  TEST_ASSERT_FALSE(t.active(1300));
}

static ctl::OdomInput in3(float a, float b, float c) {
  ctl::OdomInput i{};
  i.arm_speed_mm_s[0] = a;
  i.arm_speed_mm_s[1] = b;
  i.arm_speed_mm_s[2] = c;
  i.arm_ok[0] = i.arm_ok[1] = i.arm_ok[2] = true;
  i.dt = 0.001f;
  return i;
}

void test_odometry_median_ignores_one_spinning_arm() {
  ctl::Odometry o;
  for (int k = 0; k < 1000; ++k) o.update(in3(50, 50, 300));
  TEST_ASSERT_FLOAT_WITHIN(0.5f, 50.0f, o.speedMmS());
  TEST_ASSERT_FLOAT_WITHIN(0.002f, 0.050f, o.distanceM());
  TEST_ASSERT_TRUE(o.groundSlip());
  TEST_ASSERT_TRUE(o.confidence() < 0.6f);
}

void test_odometry_consistent_arms_high_confidence() {
  ctl::Odometry o;
  for (int k = 0; k < 2000; ++k) o.update(in3(80, 81, 79));
  TEST_ASSERT_FALSE(o.groundSlip());
  TEST_ASSERT_TRUE(o.confidence() > 0.95f);
  TEST_ASSERT_FLOAT_WITHIN(0.003f, 0.160f, o.distanceM());
}

void test_odometry_tof_disagreement_flags_slip() {
  ctl::Odometry o;
  float tof = 3000;
  for (int k = 0; k < 2000; ++k) {
    auto i = in3(80, 80, 80);  // wheels say 80 mm/s ...
    i.tof_ok = true;
    i.tof_mm = tof;            // ... but the range to the pipe end does not change: spinning in place
    o.update(i);
  }
  TEST_ASSERT_TRUE(o.groundSlip());
}

int main() {
  UNITY_BEGIN();
  RUN_TEST(test_heartbeat);
  RUN_TEST(test_heartbeat_wraps);
  RUN_TEST(test_ttl);
  RUN_TEST(test_odometry_median_ignores_one_spinning_arm);
  RUN_TEST(test_odometry_consistent_arms_high_confidence);
  RUN_TEST(test_odometry_tof_disagreement_flags_slip);
  return UNITY_END();
}
