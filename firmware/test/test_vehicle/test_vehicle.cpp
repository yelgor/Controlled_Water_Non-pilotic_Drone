#include <unity.h>

#include <cmath>

#include "vehicle.h"

using namespace core;

void setUp() {}
void tearDown() {}

static VehicleInputs inputs(uint32_t t) {
    VehicleInputs in;
    in.nowMs = t;
    in.rc.linkOk = true;
    in.imuOk = true;
    in.headingDeg = 90.0f;
    in.gpsOk = true;
    in.speedMps = 1.0f;
    return in;
}

// Армимо: тумблер OFF → ON при газі 0.
static uint32_t arm(Vehicle& v, uint32_t t) {
    VehicleInputs in = inputs(t);
    v.update(in);
    in.nowMs += 20;
    in.rc.armSwitch = true;
    v.update(in);
    return in.nowMs;
}

void test_starts_disarmed_motor_off() {
    Vehicle v;
    VehicleInputs in = inputs(0);
    in.rc.throttle = 1.0f;
    const VehicleStatus& s = v.update(in);
    TEST_ASSERT_EQUAL(static_cast<int>(Mode::Disarmed), static_cast<int>(s.mode));
    TEST_ASSERT_EQUAL_UINT16(1000, s.pulses.escUs);
}

void test_no_link_is_failsafe() {
    Vehicle v;
    VehicleInputs in = inputs(0);
    in.rc.linkOk = false;
    in.rc.rudder = 1.0f;
    const VehicleStatus& s = v.update(in);
    TEST_ASSERT_EQUAL(static_cast<int>(Mode::Failsafe), static_cast<int>(s.mode));
    TEST_ASSERT_EQUAL_UINT16(1000, s.pulses.escUs);
    TEST_ASSERT_EQUAL_UINT16(1500, s.pulses.rudderUs);
}

void test_boot_with_arm_switch_on_does_not_arm() {
    Vehicle v;
    VehicleInputs in = inputs(0);
    in.rc.armSwitch = true;
    v.update(in);
    in.nowMs = 20;
    TEST_ASSERT_FALSE(v.armed());
    TEST_ASSERT_EQUAL(static_cast<int>(Mode::Disarmed), static_cast<int>(v.update(in).mode));
}

void test_arm_requires_low_throttle() {
    Vehicle v;
    VehicleInputs in = inputs(0);
    in.rc.throttle = 0.5f;
    v.update(in);
    in.rc.armSwitch = true;
    in.nowMs = 20;
    v.update(in);
    TEST_ASSERT_FALSE(v.armed());
    // зменшили газ, але тумблер не перещелкнули — все одно не армимо
    in.rc.throttle = 0.0f;
    in.nowMs = 40;
    v.update(in);
    TEST_ASSERT_FALSE(v.armed());
}

void test_manual_throttle_limited_by_thr_max() {
    Vehicle v;
    uint32_t t = arm(v, 0);
    VehicleInputs in = inputs(t + 20);
    in.rc.armSwitch = true;
    in.rc.throttle = 1.0f;
    const VehicleStatus& s = v.update(in);
    TEST_ASSERT_EQUAL(static_cast<int>(Mode::Manual), static_cast<int>(s.mode));
    TEST_ASSERT_FLOAT_WITHIN(1e-4, 0.30f, s.throttle);
    TEST_ASSERT_EQUAL_UINT16(1300, s.pulses.escUs);
}

void test_manual_throttle_scaled_over_full_stick() {
    // Стік на половині → половина від thr.max, а не вже максимум.
    Vehicle v;
    uint32_t t = arm(v, 0);
    VehicleInputs in = inputs(t + 20);
    in.rc.armSwitch = true;
    in.rc.throttle = 0.5f;
    const VehicleStatus& s = v.update(in);
    TEST_ASSERT_FLOAT_WITHIN(1e-4, 0.15f, s.throttle);
    TEST_ASSERT_EQUAL_UINT16(1150, s.pulses.escUs);
}

void test_link_loss_disarms_and_needs_rearm() {
    Vehicle v;
    uint32_t t = arm(v, 0);
    VehicleInputs in = inputs(t + 20);
    in.rc.armSwitch = true;
    in.rc.linkOk = false;
    v.update(in);
    TEST_ASSERT_FALSE(v.armed());
    in.rc.linkOk = true;  // зв'язок повернувся, тумблер досі ON
    in.nowMs += 20;
    TEST_ASSERT_EQUAL(static_cast<int>(Mode::Disarmed), static_cast<int>(v.update(in).mode));
}

void test_hold_captures_current_heading() {
    Vehicle v;
    uint32_t t = arm(v, 0);
    VehicleInputs in = inputs(t + 20);
    in.rc.armSwitch = true;
    in.rc.holdSwitch = true;
    in.rc.throttle = 0.2f;
    in.headingDeg = 123.0f;
    const VehicleStatus& s = v.update(in);
    TEST_ASSERT_EQUAL(static_cast<int>(Mode::Hold), static_cast<int>(s.mode));
    TEST_ASSERT_FLOAT_WITHIN(1e-4, 123.0f, s.headingSp);
    // човен звернув праворуч (курс 133) → кермо вліво
    for (int i = 0; i < 50; ++i) {
        in.nowMs += 20;
        in.headingDeg = 133.0f;
        v.update(in);
    }
    TEST_ASSERT_TRUE(v.status().rudder < 0.0f);
    // газ — зі стіка, бо spd.en = 0 (масштабований на thr.max = 0.3)
    TEST_ASSERT_FLOAT_WITHIN(1e-4, 0.2f * 0.3f, v.status().throttle);
}

void test_hold_is_bumpless_from_stick() {
    Params p;
    p.hdgKi = 0.01f;
    p.hdgSlew = 0.0f;
    Vehicle v;
    v.applyParams(p);
    uint32_t t = arm(v, 0);
    VehicleInputs in = inputs(t + 20);
    in.rc.armSwitch = true;
    in.rc.throttle = 0.2f;
    in.rc.rudder = 0.3f;  // пілот тримає кермо
    v.update(in);
    in.rc.holdSwitch = true;
    in.nowMs += 20;
    TEST_ASSERT_FLOAT_WITHIN(0.02f, 0.3f, v.update(in).rudder);
}

void test_hold_without_compass_falls_back_to_manual() {
    Vehicle v;
    uint32_t t = arm(v, 0);
    VehicleInputs in = inputs(t + 20);
    in.rc.armSwitch = true;
    in.rc.holdSwitch = true;
    in.imuOk = false;
    in.rc.rudder = -0.4f;
    const VehicleStatus& s = v.update(in);
    TEST_ASSERT_EQUAL(static_cast<int>(Mode::Manual), static_cast<int>(s.mode));
    TEST_ASSERT_FLOAT_WITHIN(1e-4, -0.4f, s.rudder);
}

void test_speed_hold_uses_gps_and_falls_back_to_stick() {
    Params p;
    p.spdEn = 1.0f;
    p.spdSp = 2.0f;
    p.spdSlew = 0.0f;
    Vehicle v;
    v.applyParams(p);
    uint32_t t = arm(v, 0);
    VehicleInputs in = inputs(t + 20);
    in.rc.armSwitch = true;
    in.rc.holdSwitch = true;
    in.rc.throttle = 0.1f;
    in.speedMps = 1.0f;
    const VehicleStatus& s = v.update(in);
    TEST_ASSERT_TRUE(s.speedHold);
    // GPS пропав → газ знову зі стіка
    in.gpsOk = false;
    in.nowMs += 20;
    v.update(in);
    TEST_ASSERT_FALSE(v.status().speedHold);
    TEST_ASSERT_FLOAT_WITHIN(1e-4, 0.1f * 0.3f, v.status().throttle);  // стік × thr.max
}

void test_set_heading_only_in_hold() {
    Vehicle v;
    TEST_ASSERT_FALSE(v.setHeadingSetpoint(10.0f));
    uint32_t t = arm(v, 0);
    VehicleInputs in = inputs(t + 20);
    in.rc.armSwitch = true;
    in.rc.holdSwitch = true;
    v.update(in);
    TEST_ASSERT_TRUE(v.setHeadingSetpoint(-10.0f));
    TEST_ASSERT_FLOAT_WITHIN(1e-4, 350.0f, v.status().headingSp);
}

void test_nan_sticks_are_safe() {
    Vehicle v;
    uint32_t t = arm(v, 0);
    VehicleInputs in = inputs(t + 20);
    in.rc.armSwitch = true;
    in.rc.throttle = NAN;
    in.rc.rudder = NAN;
    const VehicleStatus& s = v.update(in);
    TEST_ASSERT_EQUAL_UINT16(1000, s.pulses.escUs);
    TEST_ASSERT_EQUAL_UINT16(1500, s.pulses.rudderUs);
}

// --- тестовий режим без тумблера ARM ---

void test_bypass_arms_without_switch_at_zero_throttle() {
    Vehicle v;
    v.setArmSwitchBypass(true);
    VehicleInputs in = inputs(0);  // тумблер ARM вимкнений
    v.update(in);
    TEST_ASSERT_TRUE(v.armed());
    in.nowMs = 20;
    in.rc.throttle = 0.2f;
    in.rc.rudder = 1.0f;
    in.speedMps = 0.0f;  // на місці — повний хід керма
    const VehicleStatus& s = v.update(in);
    TEST_ASSERT_EQUAL(static_cast<int>(Mode::Manual), static_cast<int>(s.mode));
    TEST_ASSERT_TRUE(s.pulses.escUs > 1000);
    TEST_ASSERT_EQUAL_UINT16(1900, s.pulses.rudderUs);
}

void test_bypass_does_not_arm_with_throttle_up() {
    Vehicle v;
    v.setArmSwitchBypass(true);
    VehicleInputs in = inputs(0);
    in.rc.throttle = 0.5f;
    const VehicleStatus& s = v.update(in);
    TEST_ASSERT_FALSE(v.armed());
    TEST_ASSERT_EQUAL(static_cast<int>(Mode::Disarmed), static_cast<int>(s.mode));
    TEST_ASSERT_EQUAL_UINT16(1000, s.pulses.escUs);
}

void test_bypass_link_loss_disarms_then_rearms_at_zero() {
    Vehicle v;
    v.setArmSwitchBypass(true);
    VehicleInputs in = inputs(0);
    v.update(in);
    TEST_ASSERT_TRUE(v.armed());

    in.nowMs = 20;
    in.rc.linkOk = false;
    v.update(in);
    TEST_ASSERT_FALSE(v.armed());

    in.nowMs = 40;  // зв'язок повернувся, але газ піднятий — мотор не стартує
    in.rc.linkOk = true;
    in.rc.throttle = 0.5f;
    TEST_ASSERT_EQUAL_UINT16(1000, v.update(in).pulses.escUs);
    TEST_ASSERT_FALSE(v.armed());

    in.nowMs = 60;
    in.rc.throttle = 0.0f;
    v.update(in);
    TEST_ASSERT_TRUE(v.armed());
}

// --- хід керма від швидкості ---

void test_rudder_reduced_with_gps_speed() {
    Params p;  // rud.min 0.4, rud.vmax 6
    Vehicle v;
    v.applyParams(p);
    uint32_t t = arm(v, 0);
    VehicleInputs in = inputs(t + 20);
    in.rc.armSwitch = true;
    in.rc.rudder = 1.0f;
    in.speedMps = 0.0f;
    TEST_ASSERT_EQUAL_UINT16(1900, v.update(in).pulses.rudderUs);  // на місці — повний хід
    in.nowMs += 20;
    in.speedMps = 3.0f;  // половина rud.vmax → 0.7 ходу
    TEST_ASSERT_EQUAL_UINT16(1500 + 280, v.update(in).pulses.rudderUs);
    in.nowMs += 20;
    in.speedMps = 10.0f;  // швидше за rud.vmax → 0.4 ходу
    TEST_ASSERT_EQUAL_UINT16(1500 + 160, v.update(in).pulses.rudderUs);
}

void test_rudder_reduced_with_throttle_without_gps() {
    Vehicle v;
    uint32_t t = arm(v, 0);
    VehicleInputs in = inputs(t + 20);
    in.rc.armSwitch = true;
    in.gpsOk = false;
    in.rc.rudder = -1.0f;
    in.rc.throttle = 1.0f;  // повний газ = «повна швидкість» → 0.4 ходу
    const VehicleStatus& s = v.update(in);
    TEST_ASSERT_FLOAT_WITHIN(1e-4, 0.4f, s.rudderScale);
    TEST_ASSERT_EQUAL_UINT16(1500 - 160, s.pulses.rudderUs);
}

void test_rudder_scale_off_when_min_is_one() {
    Params p;
    p.rudMin = 1.0f;
    Vehicle v;
    v.applyParams(p);
    uint32_t t = arm(v, 0);
    VehicleInputs in = inputs(t + 20);
    in.rc.armSwitch = true;
    in.rc.rudder = 1.0f;
    in.speedMps = 10.0f;
    TEST_ASSERT_EQUAL_UINT16(1900, v.update(in).pulses.rudderUs);
}

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_starts_disarmed_motor_off);
    RUN_TEST(test_no_link_is_failsafe);
    RUN_TEST(test_boot_with_arm_switch_on_does_not_arm);
    RUN_TEST(test_arm_requires_low_throttle);
    RUN_TEST(test_manual_throttle_limited_by_thr_max);
    RUN_TEST(test_manual_throttle_scaled_over_full_stick);
    RUN_TEST(test_link_loss_disarms_and_needs_rearm);
    RUN_TEST(test_hold_captures_current_heading);
    RUN_TEST(test_hold_is_bumpless_from_stick);
    RUN_TEST(test_hold_without_compass_falls_back_to_manual);
    RUN_TEST(test_speed_hold_uses_gps_and_falls_back_to_stick);
    RUN_TEST(test_set_heading_only_in_hold);
    RUN_TEST(test_nan_sticks_are_safe);
    RUN_TEST(test_rudder_reduced_with_gps_speed);
    RUN_TEST(test_rudder_reduced_with_throttle_without_gps);
    RUN_TEST(test_rudder_scale_off_when_min_is_one);
    RUN_TEST(test_bypass_arms_without_switch_at_zero_throttle);
    RUN_TEST(test_bypass_does_not_arm_with_throttle_up);
    RUN_TEST(test_bypass_link_loss_disarms_then_rearms_at_zero);
    return UNITY_END();
}
