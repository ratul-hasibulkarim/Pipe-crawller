// State machine transitions, including the never-release-the-grip rules.
#include <unity.h>

#include "control/state_machine.h"

using ctl::StateMachine;
using proto::State;

void setUp() {}
void tearDown() {}

static StateMachine gripped() {
  StateMachine sm;
  sm.expand(false, 0);
  sm.expansionDone(true, 1);
  return sm;
}

void test_expand_to_gripped_and_crawl() {
  StateMachine sm;
  TEST_ASSERT_TRUE(sm.expand(false, 0));
  TEST_ASSERT_EQUAL(State::EXPANDING, sm.state());
  sm.expansionDone(true, 10);
  TEST_ASSERT_EQUAL(State::GRIPPED, sm.state());
  TEST_ASSERT_TRUE(sm.drive(true, 20));
  TEST_ASSERT_EQUAL(State::CRAWLING, sm.state());
  sm.drive(false, 30);
  TEST_ASSERT_EQUAL(State::HOLDING, sm.state());
}

void test_no_contact_returns_idle() {
  StateMachine sm;
  sm.expand(false, 0);
  sm.expansionDone(false, 10);
  TEST_ASSERT_EQUAL(State::IDLE, sm.state());
  TEST_ASSERT_FALSE(sm.gripped());
}

void test_cannot_move_when_not_gripped() {
  StateMachine sm;
  TEST_ASSERT_FALSE(sm.drive(true, 0));
  TEST_ASSERT_TRUE(sm.drive(false, 0));  // zero speed always harmless
  TEST_ASSERT_EQUAL(State::IDLE, sm.state());
}

void test_cannot_expand_while_crawling() {
  StateMachine sm = gripped();
  sm.drive(true, 2);
  TEST_ASSERT_FALSE(sm.expand(true, 3));
  TEST_ASSERT_EQUAL(State::CRAWLING, sm.state());
}

void test_fault_keeps_grip_and_clears_to_holding() {
  StateMachine sm = gripped();
  sm.drive(true, 2);
  sm.fault(3);
  TEST_ASSERT_EQUAL(State::FAULT, sm.state());
  TEST_ASSERT_TRUE(sm.gripped());
  TEST_ASSERT_FALSE(sm.expand(true, 4));  // no retract from FAULT
  TEST_ASSERT_FALSE(sm.clearFault(false, 5));
  TEST_ASSERT_TRUE(sm.clearFault(true, 6));
  TEST_ASSERT_EQUAL(State::HOLDING, sm.state());
}

void test_low_power_and_recovery() {
  StateMachine sm = gripped();
  sm.lowPower(true, 5);
  TEST_ASSERT_EQUAL(State::LOW_POWER_SAFE, sm.state());
  TEST_ASSERT_FALSE(sm.drive(true, 6));
  sm.lowPower(false, 7);
  TEST_ASSERT_EQUAL(State::HOLDING, sm.state());
}

void test_fault_has_priority_over_low_power() {
  StateMachine sm = gripped();
  sm.fault(1);
  sm.lowPower(true, 2);
  TEST_ASSERT_EQUAL(State::FAULT, sm.state());
}

void test_hold_timeout_drops_to_gripped() {
  StateMachine sm = gripped();
  sm.drive(true, 100);
  sm.drive(false, 200);
  sm.holdTimeout(5000, 10000);
  TEST_ASSERT_EQUAL(State::HOLDING, sm.state());
  sm.holdTimeout(10300, 10000);
  TEST_ASSERT_EQUAL(State::GRIPPED, sm.state());
}

void test_retract_and_reboot_assume_grip() {
  StateMachine sm = gripped();
  TEST_ASSERT_TRUE(sm.expand(true, 2));
  TEST_ASSERT_EQUAL(State::RETRACTING, sm.state());
  sm.expansionDone(false, 3);
  TEST_ASSERT_EQUAL(State::IDLE, sm.state());
  StateMachine boot;
  boot.assumeGripped(0);
  TEST_ASSERT_EQUAL(State::HOLDING, boot.state());
}

void test_change_notification_once() {
  StateMachine sm;
  sm.expand(false, 0);
  State a, b;
  TEST_ASSERT_TRUE(sm.takeChanged(a, b));
  TEST_ASSERT_EQUAL(State::IDLE, a);
  TEST_ASSERT_EQUAL(State::EXPANDING, b);
  TEST_ASSERT_FALSE(sm.takeChanged(a, b));
}

int main() {
  UNITY_BEGIN();
  RUN_TEST(test_expand_to_gripped_and_crawl);
  RUN_TEST(test_no_contact_returns_idle);
  RUN_TEST(test_cannot_move_when_not_gripped);
  RUN_TEST(test_cannot_expand_while_crawling);
  RUN_TEST(test_fault_keeps_grip_and_clears_to_holding);
  RUN_TEST(test_low_power_and_recovery);
  RUN_TEST(test_fault_has_priority_over_low_power);
  RUN_TEST(test_hold_timeout_drops_to_gripped);
  RUN_TEST(test_retract_and_reboot_assume_grip);
  RUN_TEST(test_change_notification_once);
  return UNITY_END();
}
