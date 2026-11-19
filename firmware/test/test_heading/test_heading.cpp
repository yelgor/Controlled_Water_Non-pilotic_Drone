#include <unity.h>

#include <cmath>

#include "angle.h"
#include "heading_controller.h"

using namespace ctl;

void setUp() {}
void tearDown() {}

static HeadingConfig cfgP(float kp) {
    HeadingConfig c;
    c.kp = kp;
    c.ki = 0.0f;
    c.kd = 0.0f;
    c.slewRate = 0.0f;
    return c;
}

// Дуже проста модель аероглісера: швидкість повороту наздоганяє
// K * газ * кермо з інерцією T. Цього досить, щоб перевірити знаки і збіжність.
struct BoatYawModel {
    float headingDeg = 0.0f;
    float yawRate = 0.0f;  // °/с
    float K = 60.0f;       // °/с при повному кермі і повному газі
    float T = 0.5f;        // с
    void step(float rudder, float throttle, float dt) {
        const float target = K * throttle * rudder;
        yawRate += (target - yawRate) * dt / T;
        headingDeg = wrap360(headingDeg + yawRate * dt);
    }
};

void test_turns_shortest_way_350_from_10() {
    HeadingController hc(cfgP(0.01f));
    // setpoint 350°, курс 10° → помилка -20°, кермо вліво (мінус)
    const float rudder = hc.update(350.0f, 10.0f, 0.3f, 0.02f);
    TEST_ASSERT_FLOAT_WITHIN(1e-4, -20.0f, hc.lastError());
    TEST_ASSERT_FLOAT_WITHIN(1e-4, -0.2f, rudder);
}

void test_turns_right_when_setpoint_is_right() {
    HeadingController hc(cfgP(0.01f));
    TEST_ASSERT_TRUE(hc.update(30.0f, 0.0f, 0.3f, 0.02f) > 0.0f);
    TEST_ASSERT_TRUE(hc.update(330.0f, 0.0f, 0.3f, 0.02f) < 0.0f);
}

void test_rudder_saturates_at_one() {
    HeadingController hc(cfgP(0.05f));
    TEST_ASSERT_EQUAL_FLOAT(1.0f, hc.update(120.0f, 0.0f, 0.3f, 0.02f));
    TEST_ASSERT_EQUAL_FLOAT(-1.0f, hc.update(240.0f, 0.0f, 0.3f, 0.02f));
}

void test_integral_frozen_without_airflow() {
    HeadingConfig c = cfgP(0.0f);
    c.ki = 0.1f;
    c.minSteerThrottle = 0.1f;
    HeadingController hc(c);
    for (int i = 0; i < 500; ++i) hc.update(90.0f, 0.0f, 0.0f, 0.02f);  // газ 0
    TEST_ASSERT_EQUAL_FLOAT(0.0f, hc.terms().i);
    for (int i = 0; i < 10; ++i) hc.update(10.0f, 0.0f, 0.3f, 0.02f);   // газ є
    TEST_ASSERT_TRUE(hc.terms().i > 0.0f);
}

void test_engage_is_bumpless() {
    HeadingConfig c = cfgP(0.01f);
    c.ki = 0.02f;
    HeadingController hc(c);
    hc.engage(45.0f, 45.0f, 0.25f);  // пілот тримав кермо 0.25
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.25f, hc.update(45.0f, 45.0f, 0.3f, 0.02f));
}

void test_closed_loop_reaches_setpoint() {
    HeadingController hc(cfgP(0.01f));
    BoatYawModel boat;
    const float dt = 0.02f;
    float maxHeading = 0.0f;
    for (int i = 0; i < 60 * 50; ++i) {  // 60 с
        const float r = hc.update(90.0f, boat.headingDeg, 0.3f, dt);
        boat.step(r, 0.3f, dt);
        if (boat.headingDeg < 180.0f && boat.headingDeg > maxHeading) maxHeading = boat.headingDeg;
    }
    TEST_ASSERT_FLOAT_WITHIN(3.0f, 90.0f, boat.headingDeg);
    TEST_ASSERT_TRUE(maxHeading < 99.0f);  // перерегулювання < 10%
}

void test_closed_loop_crosses_north_short_way() {
    HeadingController hc(cfgP(0.01f));
    BoatYawModel boat;
    boat.headingDeg = 10.0f;
    const float dt = 0.02f;
    for (int i = 0; i < 60 * 50; ++i) {
        const float r = hc.update(350.0f, boat.headingDeg, 0.3f, dt);
        boat.step(r, 0.3f, dt);
        // ніколи не повертаємо «довгим шляхом» через 180°
        TEST_ASSERT_TRUE(std::fabs(wrap180(boat.headingDeg)) < 25.0f);
    }
    TEST_ASSERT_FLOAT_WITHIN(3.0f, 0.0f, headingError(350.0f, boat.headingDeg));
}

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_turns_shortest_way_350_from_10);
    RUN_TEST(test_turns_right_when_setpoint_is_right);
    RUN_TEST(test_rudder_saturates_at_one);
    RUN_TEST(test_integral_frozen_without_airflow);
    RUN_TEST(test_engage_is_bumpless);
    RUN_TEST(test_closed_loop_reaches_setpoint);
    RUN_TEST(test_closed_loop_crosses_north_short_way);
    return UNITY_END();
}
