#include <unity.h>

#include <cmath>

#include "speed_controller.h"

using namespace ctl;

void setUp() {}
void tearDown() {}

static SpeedConfig cfg(float kp, float ki, float ff) {
    SpeedConfig c;
    c.kp = kp;
    c.ki = ki;
    c.kd = 0.0f;
    c.ff = ff;
    c.slewRate = 0.0f;
    return c;
}

void test_too_slow_gives_more_throttle() {
    SpeedController sc(cfg(0.1f, 0.0f, 0.0f));
    // хочемо 2 м/с, їдемо 1 м/с → газ 0.1
    TEST_ASSERT_FLOAT_WITHIN(1e-4, 0.1f, sc.update(2.0f, 1.0f, true, 0.2f));
}

void test_too_fast_never_negative() {
    SpeedController sc(cfg(0.5f, 0.0f, 0.0f));
    TEST_ASSERT_EQUAL_FLOAT(0.0f, sc.update(1.0f, 3.0f, true, 0.2f));
}

void test_feedforward_added() {
    SpeedController sc(cfg(0.0f, 0.0f, 0.1f));
    TEST_ASSERT_FLOAT_WITHIN(1e-4, 0.2f, sc.update(2.0f, 2.0f, true, 0.2f));
}

void test_invalid_gps_cuts_throttle() {
    SpeedController sc(cfg(0.1f, 0.1f, 0.1f));
    for (int i = 0; i < 20; ++i) sc.update(2.0f, 1.0f, true, 0.2f);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, sc.update(2.0f, 1.0f, false, 0.2f));
    TEST_ASSERT_TRUE(sc.dataFault());
    TEST_ASSERT_EQUAL_FLOAT(0.0f, sc.update(2.0f, NAN, true, 0.2f));
    // інтеграл скинутий — після повернення GPS немає ривка від старого I
    sc.update(2.0f, 2.0f, true, 0.2f);
    TEST_ASSERT_FALSE(sc.dataFault());
    TEST_ASSERT_TRUE(sc.terms().i < 0.05f);
}

void test_zero_setpoint_stops() {
    SpeedController sc(cfg(0.1f, 0.1f, 0.1f));
    TEST_ASSERT_EQUAL_FLOAT(0.0f, sc.update(0.0f, 1.0f, true, 0.2f));
}

void test_engage_bumpless() {
    SpeedController sc(cfg(0.1f, 0.05f, 0.05f));
    sc.engage(1.5f, 1.2f, 0.22f);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.22f, sc.update(1.5f, 1.2f, true, 0.02f));
}

void test_closed_loop_with_gps_5hz() {
    SpeedController sc(cfg(0.1f, 0.05f, 0.15f));
    // човен: усталена швидкість = 5 м/с * газ, інерція 3 с
    float v = 0.0f;
    float gpsSpeed = 0.0f;
    const float dt = 0.02f;
    float thr = 0.0f;
    for (int i = 0; i < 90 * 50; ++i) {
        if (i % 10 == 0) gpsSpeed = v;  // GPS оновлюється 5 разів на секунду
        thr = sc.update(1.0f, gpsSpeed, true, dt);
        v += (5.0f * thr - v) * dt / 3.0f;
    }
    TEST_ASSERT_FLOAT_WITHIN(0.05f, 1.0f, v);
    TEST_ASSERT_TRUE(thr >= 0.0f && thr <= 1.0f);
}

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_too_slow_gives_more_throttle);
    RUN_TEST(test_too_fast_never_negative);
    RUN_TEST(test_feedforward_added);
    RUN_TEST(test_invalid_gps_cuts_throttle);
    RUN_TEST(test_zero_setpoint_stops);
    RUN_TEST(test_engage_bumpless);
    RUN_TEST(test_closed_loop_with_gps_5hz);
    return UNITY_END();
}
