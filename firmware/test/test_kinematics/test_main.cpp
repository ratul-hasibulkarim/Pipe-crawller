// Arm geometry: same numbers as calc/mechanism.py (LUT generated from it), grip, sag, bend assist.
#include <unity.h>

#include <cmath>

#include "bot_params.h"
#include "control/kinematics.h"

void setUp() {}
void tearDown() {}

void test_lut_matches_closed_form() {
  for (int i = 0; i < params::DIAMETER_LUT_N; ++i) {
    TEST_ASSERT_FLOAT_WITHIN(0.01f, params::DIAMETER_LUT_MM[i], kin::diameterForTheta(params::DIAMETER_LUT_THETA[i]));
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, params::DIAMETER_LUT_THETA[i], kin::thetaForDiameter(params::DIAMETER_LUT_MM[i]));
  }
}

void test_range_endpoints_and_clamping() {
  TEST_ASSERT_FLOAT_WITHIN(0.01f, params::PIPE_ID_MIN_MM, kin::diameterForTheta(params::THETA_MIN_RAD));
  TEST_ASSERT_FLOAT_WITHIN(0.01f, params::PIPE_ID_MAX_MM, kin::diameterForTheta(params::THETA_MAX_RAD));
  TEST_ASSERT_EQUAL_FLOAT(params::THETA_MAX_RAD, kin::thetaForDiameter(10000));
  TEST_ASSERT_EQUAL_FLOAT(params::THETA_MIN_RAD, kin::thetaForDiameter(1));
}

void test_grip_force_spring_model() {
  TEST_ASSERT_EQUAL_FLOAT(0.0f, kin::gripForce(0.1f));
  const float mid = kin::gripForce(params::COMPLIANCE_TRAVEL_MM);
  TEST_ASSERT_FLOAT_WITHIN(0.1f, params::WORKING_GRIP_N, mid);  // mid-travel = working grip
  TEST_ASSERT_TRUE(kin::gripForce(2 * params::COMPLIANCE_TRAVEL_MM) <= params::MAX_GRIP_N + 0.01f);
}

void test_axis_offset() {
  const float even[3] = {4, 4, 4};
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.0f, kin::axisOffsetMm(even));
  const float sag[3] = {2, 6, 6};  // arm A unloaded: body sagged away from it
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 4.0f / 1.5f, kin::axisOffsetMm(sag));
}

void test_bend_assist_inner_arm_slower() {
  const float d = 200;
  const float inner = kin::armSpeed(0, 50, 10, 0, d);    // bend centre toward arm A
  const float outer = kin::armSpeed(1, 50, 10, 0, d);
  TEST_ASSERT_TRUE(inner < 50 && outer > 50);
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, 50.0f, kin::armSpeed(2, 50, 0, 0, d));
}

int main() {
  UNITY_BEGIN();
  RUN_TEST(test_lut_matches_closed_form);
  RUN_TEST(test_range_endpoints_and_clamping);
  RUN_TEST(test_grip_force_spring_model);
  RUN_TEST(test_axis_offset);
  RUN_TEST(test_bend_assist_inner_arm_slower);
  return UNITY_END();
}
