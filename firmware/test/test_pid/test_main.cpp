// PID: proportional response, output clamp, anti-windup, derivative on measurement.
#include <unity.h>

#include "control/pid.h"

using ctl::Pid;
using ctl::PidGains;

void setUp() {}
void tearDown() {}

void test_proportional_only() {
  Pid p(PidGains{2.0f, 0, 0}, -100, 100, 100, 50);
  TEST_ASSERT_FLOAT_WITHIN(1e-5f, 20.0f, p.update(10, 0, 0.01f));
}

void test_output_clamped() {
  Pid p(PidGains{100.0f, 0, 0}, -5, 5, 100, 50);
  TEST_ASSERT_EQUAL_FLOAT(5.0f, p.update(10, 0, 0.01f));
  TEST_ASSERT_EQUAL_FLOAT(-5.0f, p.update(-10, 0, 0.01f));
}

void test_integral_clamped_and_no_windup() {
  Pid p(PidGains{1.0f, 50.0f, 0}, -2, 2, 1.5f, 50);
  for (int i = 0; i < 1000; ++i) p.update(10, 0, 0.01f);  // saturated for 10 s
  TEST_ASSERT_TRUE(p.integral() <= 1.5f + 1e-6f);
  // the error reverses: output must leave saturation immediately (no wound-up integral)
  const float out = p.update(-1, 0, 0.01f);
  TEST_ASSERT_TRUE(out < 2.0f);
}

void test_integral_converges() {
  Pid p(PidGains{0.5f, 2.0f, 0}, -10, 10, 10, 50);
  float y = 0;
  for (int i = 0; i < 5000; ++i) y += (p.update(3.0f, y, 0.01f) - 0.2f * y) * 0.01f * 5.0f;
  TEST_ASSERT_FLOAT_WITHIN(0.05f, 3.0f, y);
}

void test_no_derivative_kick_on_setpoint_step() {
  Pid p(PidGains{0, 0, 10.0f}, -1000, 1000, 100, 1000);
  p.update(0, 0, 0.01f);
  TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.0f, p.update(100, 0, 0.01f));
}

void test_slew_and_clamp_helpers() {
  TEST_ASSERT_EQUAL_FLOAT(1.0f, ctl::slew(0, 10, 100, 0.01f));
  TEST_ASSERT_EQUAL_FLOAT(10.0f, ctl::slew(9.5f, 10, 100, 0.01f));
  TEST_ASSERT_EQUAL_FLOAT(3.0f, ctl::clampf(7, 0, 3));
}

int main() {
  UNITY_BEGIN();
  RUN_TEST(test_proportional_only);
  RUN_TEST(test_output_clamped);
  RUN_TEST(test_integral_clamped_and_no_windup);
  RUN_TEST(test_integral_converges);
  RUN_TEST(test_no_derivative_kick_on_setpoint_step);
  RUN_TEST(test_slew_and_clamp_helpers);
  return UNITY_END();
}
