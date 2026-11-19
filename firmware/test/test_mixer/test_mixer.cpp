#include <unity.h>

#include "mixer.h"

using namespace ctl;

void setUp() {}
void tearDown() {}

void test_center_and_idle() {
    MixerConfig c;
    const ActuatorPulses p = mixRudderMotor(0.0f, 0.0f, c);
    TEST_ASSERT_EQUAL_UINT16(1500, p.rudderUs);
    TEST_ASSERT_EQUAL_UINT16(1000, p.escUs);
}

void test_rudder_full_and_reversed() {
    MixerConfig c;
    TEST_ASSERT_EQUAL_UINT16(1900, mixRudderMotor(1.0f, 0.0f, c).rudderUs);
    TEST_ASSERT_EQUAL_UINT16(1100, mixRudderMotor(-5.0f, 0.0f, c).rudderUs);
    c.servoReversed = true;
    TEST_ASSERT_EQUAL_UINT16(1100, mixRudderMotor(1.0f, 0.0f, c).rudderUs);
}

void test_trim() {
    MixerConfig c;
    c.rudderTrim = 0.1f;
    TEST_ASSERT_EQUAL_UINT16(1540, mixRudderMotor(0.0f, 0.0f, c).rudderUs);
}

void test_throttle_limited_to_30_percent() {
    MixerConfig c;  // maxThrottle = 0.30 за замовчуванням
    TEST_ASSERT_EQUAL_UINT16(1300, mixRudderMotor(0.0f, 1.0f, c).escUs);
    TEST_ASSERT_EQUAL_UINT16(1200, mixRudderMotor(0.0f, 0.2f, c).escUs);
    TEST_ASSERT_EQUAL_UINT16(1000, mixRudderMotor(0.0f, -0.5f, c).escUs);
}

void test_nan_is_safe() {
    MixerConfig c;
    const ActuatorPulses p = mixRudderMotor(NAN, NAN, c);
    TEST_ASSERT_EQUAL_UINT16(1500, p.rudderUs);
    TEST_ASSERT_EQUAL_UINT16(1000, p.escUs);
}

void test_rudder_speed_scale_linear() {
    TEST_ASSERT_FLOAT_WITHIN(1e-5, 1.0f, rudderSpeedScale(0.0f, 0.4f));   // на місці — повний
    TEST_ASSERT_FLOAT_WITHIN(1e-5, 0.7f, rudderSpeedScale(0.5f, 0.4f));   // половина — посередині
    TEST_ASSERT_FLOAT_WITHIN(1e-5, 0.4f, rudderSpeedScale(1.0f, 0.4f));   // повна — мінімум
    TEST_ASSERT_FLOAT_WITHIN(1e-5, 0.4f, rudderSpeedScale(3.0f, 0.4f));   // швидше — не менше мін.
    TEST_ASSERT_FLOAT_WITHIN(1e-5, 1.0f, rudderSpeedScale(-1.0f, 0.4f));  // назад — як на місці
    TEST_ASSERT_FLOAT_WITHIN(1e-5, 1.0f, rudderSpeedScale(0.8f, 1.0f));   // min = 1 → вимкнено
    TEST_ASSERT_FLOAT_WITHIN(1e-5, 1.0f, rudderSpeedScale(NAN, 0.4f));
}

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_rudder_speed_scale_linear);
    RUN_TEST(test_center_and_idle);
    RUN_TEST(test_rudder_full_and_reversed);
    RUN_TEST(test_trim);
    RUN_TEST(test_throttle_limited_to_30_percent);
    RUN_TEST(test_nan_is_safe);
    return UNITY_END();
}
