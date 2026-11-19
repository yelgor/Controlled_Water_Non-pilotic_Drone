#include <unity.h>

#include <cmath>
#include <limits>

#include "angle.h"
#include "pid.h"

using namespace ctl;

static PidConfig baseCfg() {
    PidConfig c;
    c.kp = 1.0f;
    c.ki = 0.0f;
    c.kd = 0.0f;
    c.outMin = -1.0f;
    c.outMax = 1.0f;
    c.dFilterTau = 0.0f;
    return c;
}

void setUp() {}
void tearDown() {}

// --- кути ---

void test_wrap180() {
    TEST_ASSERT_FLOAT_WITHIN(1e-4, -20.0f, wrap180(340.0f));
    TEST_ASSERT_FLOAT_WITHIN(1e-4, 20.0f, wrap180(-340.0f));
    TEST_ASSERT_FLOAT_WITHIN(1e-4, 0.0f, wrap180(720.0f));
    TEST_ASSERT_FLOAT_WITHIN(1e-4, -180.0f, wrap180(180.0f));
    TEST_ASSERT_FLOAT_WITHIN(1e-4, 350.0f, wrap360(-10.0f));
}

void test_heading_error_350_to_10() {
    // setpoint 350°, курс 10° → повертати на -20°, а не на +340°
    TEST_ASSERT_FLOAT_WITHIN(1e-4, -20.0f, headingError(350.0f, 10.0f));
    TEST_ASSERT_FLOAT_WITHIN(1e-4, 20.0f, headingError(10.0f, 350.0f));
}

// --- стрибок setpoint ---

void test_step_response_sign_and_magnitude() {
    PidConfig c = baseCfg();
    c.kp = 0.5f;
    Pid pid(c);
    // помилка +1 → вихід +0.5
    TEST_ASSERT_FLOAT_WITHIN(1e-5, 0.5f, pid.update(1.0f, 0.0f, 0.02f));
    // помилка -1 → вихід -0.5
    TEST_ASSERT_FLOAT_WITHIN(1e-5, -0.5f, pid.update(-1.0f, 0.0f, 0.02f));
}

void test_integral_accumulates_with_real_dt() {
    PidConfig c = baseCfg();
    c.kp = 0.0f;
    c.ki = 0.5f;
    Pid pid(c);
    // помилка 1, 10 кроків по 0.1 с = 1 с → I = 0.5
    for (int i = 0; i < 10; ++i) pid.update(1.0f, 0.0f, 0.1f);
    TEST_ASSERT_FLOAT_WITHIN(1e-4, 0.5f, pid.terms().i);
    // інший dt — та сама сумарна тривалість → той самий інтеграл
    Pid pid2(c);
    for (int i = 0; i < 50; ++i) pid2.update(1.0f, 0.0f, 0.02f);
    TEST_ASSERT_FLOAT_WITHIN(1e-4, 0.5f, pid2.terms().i);
}

void test_derivative_on_measurement_no_kick_on_setpoint_step() {
    PidConfig c = baseCfg();
    c.kp = 0.0f;
    c.kd = 1.0f;
    Pid pid(c);
    pid.update(0.0f, 0.0f, 0.02f);
    // стрибок setpoint при незмінному вимірі — D має лишитись 0
    pid.update(10.0f, 0.0f, 0.02f);
    TEST_ASSERT_FLOAT_WITHIN(1e-6, 0.0f, pid.terms().d);
    // вимір росте → D гальмує (від'ємний)
    pid.update(10.0f, 0.01f, 0.02f);
    TEST_ASSERT_TRUE(pid.terms().d < 0.0f);
}

void test_derivative_filter_smooths() {
    PidConfig c = baseCfg();
    c.kp = 0.0f;
    c.kd = 1.0f;
    c.outMin = -100.0f;
    c.outMax = 100.0f;
    PidConfig cf = c;
    cf.dFilterTau = 0.2f;
    Pid raw(c), filt(cf);
    raw.update(0, 0, 0.02f);
    filt.update(0, 0, 0.02f);
    raw.update(0, 0.1f, 0.02f);
    filt.update(0, 0.1f, 0.02f);
    TEST_ASSERT_TRUE(std::fabs(filt.terms().d) < std::fabs(raw.terms().d));
    TEST_ASSERT_TRUE(filt.terms().d < 0.0f);
}

void test_derivative_wraps_angles() {
    PidConfig c = baseCfg();
    c.kp = 0.0f;
    c.kd = 1.0f;
    c.wrap = 360.0f;
    c.outMin = -1000.0f;
    c.outMax = 1000.0f;
    Pid pid(c);
    pid.update(0.0f, 359.0f, 0.1f);
    pid.update(0.0f, 1.0f, 0.1f);  // реально повернули на +2°, а не на -358°
    TEST_ASSERT_FLOAT_WITHIN(1e-3, -20.0f, pid.terms().d);
}

// --- насичення та anti-windup ---

void test_output_saturates() {
    PidConfig c = baseCfg();
    c.kp = 10.0f;
    Pid pid(c);
    TEST_ASSERT_EQUAL_FLOAT(1.0f, pid.update(5.0f, 0.0f, 0.02f));
    TEST_ASSERT_EQUAL_FLOAT(-1.0f, pid.update(-5.0f, 0.0f, 0.02f));
}

void test_anti_windup_after_long_saturation() {
    PidConfig c = baseCfg();
    c.kp = 0.5f;
    c.ki = 1.0f;
    Pid pid(c);
    // 60 с великої помилки — вихід увесь час у межі
    for (int i = 0; i < 3000; ++i) pid.update(10.0f, 0.0f, 0.02f);
    TEST_ASSERT_EQUAL_FLOAT(1.0f, pid.output());
    TEST_ASSERT_TRUE(pid.terms().i <= 1.0f);
    // помилка змінила знак — вихід має піти вниз одразу, а не через хвилину
    const float out = pid.update(-1.0f, 0.0f, 0.02f);
    TEST_ASSERT_TRUE(out < 0.6f);
    for (int i = 0; i < 10; ++i) pid.update(-1.0f, 0.0f, 0.02f);
    TEST_ASSERT_TRUE(pid.output() < 0.5f);
}

void test_slew_rate_limits_change() {
    PidConfig c = baseCfg();
    c.kp = 10.0f;
    c.slewRate = 1.0f;  // 1 одиниця за секунду
    Pid pid(c);
    const float out = pid.update(10.0f, 0.0f, 0.1f);
    TEST_ASSERT_FLOAT_WITHIN(1e-5, 0.1f, out);
    TEST_ASSERT_FLOAT_WITHIN(1e-5, 0.2f, pid.update(10.0f, 0.0f, 0.1f));
}

// --- dt ---

void test_dt_zero_keeps_output() {
    PidConfig c = baseCfg();
    c.ki = 1.0f;
    Pid pid(c);
    const float a = pid.update(0.5f, 0.0f, 0.02f);
    const float i = pid.terms().i;
    TEST_ASSERT_EQUAL_FLOAT(a, pid.update(0.9f, 0.0f, 0.0f));
    TEST_ASSERT_EQUAL_FLOAT(i, pid.terms().i);
}

void test_dt_negative_keeps_output() {
    PidConfig c = baseCfg();
    c.ki = 1.0f;
    Pid pid(c);
    const float a = pid.update(0.5f, 0.0f, 0.02f);
    TEST_ASSERT_EQUAL_FLOAT(a, pid.update(0.9f, 0.0f, -1.0f));
}

void test_dt_huge_is_limited() {
    PidConfig c = baseCfg();
    c.kp = 0.0f;
    c.ki = 1.0f;
    c.kd = 1.0f;
    c.maxDt = 0.5f;
    c.outMin = -100.0f;
    c.outMax = 100.0f;
    Pid pid(c);
    pid.update(1.0f, 0.0f, 0.02f);
    const float iBefore = pid.terms().i;
    pid.update(1.0f, 5.0f, 1000.0f);  // 1000 с паузи
    // інтеграл виріс не більше ніж на Ki*|e|*maxDt
    TEST_ASSERT_TRUE(std::fabs(pid.terms().i - iBefore) <= 1.0f * 4.0f * 0.5f + 1e-4f);
    // похідна по стрибку виміру після паузи не рахується
    TEST_ASSERT_EQUAL_FLOAT(0.0f, pid.terms().d);
    TEST_ASSERT_TRUE(std::isfinite(pid.output()));
}

// --- NaN ---

void test_nan_input_gives_safe_output() {
    PidConfig c = baseCfg();
    c.ki = 1.0f;
    c.safeOutput = 0.0f;
    Pid pid(c);
    pid.update(0.5f, 0.0f, 0.1f);
    const float iBefore = pid.terms().i;
    const float nan = std::numeric_limits<float>::quiet_NaN();
    const float inf = std::numeric_limits<float>::infinity();
    TEST_ASSERT_EQUAL_FLOAT(0.0f, pid.update(nan, 0.0f, 0.1f));
    TEST_ASSERT_TRUE(pid.lastInputInvalid());
    TEST_ASSERT_EQUAL_FLOAT(0.0f, pid.update(0.0f, nan, 0.1f));
    TEST_ASSERT_EQUAL_FLOAT(0.0f, pid.update(0.0f, 0.0f, nan));
    TEST_ASSERT_EQUAL_FLOAT(0.0f, pid.update(inf, 0.0f, 0.1f));
    // інтеграл не зіпсований
    TEST_ASSERT_EQUAL_FLOAT(iBefore, pid.terms().i);
    // після нормальних даних — знову працює і без NaN
    const float out = pid.update(0.5f, 0.0f, 0.1f);
    TEST_ASSERT_TRUE(std::isfinite(out));
    TEST_ASSERT_FALSE(pid.lastInputInvalid());
}

void test_safe_output_respects_limits() {
    PidConfig c = baseCfg();
    c.outMin = 0.0f;
    c.outMax = 1.0f;
    c.safeOutput = -5.0f;
    Pid pid(c);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, pid.update(NAN, 0.0f, 0.1f));
}

// --- reset і bumpless ---

void test_reset_clears_state() {
    PidConfig c = baseCfg();
    c.ki = 1.0f;
    c.kd = 1.0f;
    Pid pid(c);
    for (int i = 0; i < 20; ++i) pid.update(0.5f, 0.0f, 0.05f);
    pid.reset();
    TEST_ASSERT_EQUAL_FLOAT(0.0f, pid.terms().i);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, pid.output());
    // після reset перший крок = чисте P
    TEST_ASSERT_FLOAT_WITHIN(1e-4, 0.1f + 1.0f * 0.1f * 0.05f, pid.update(0.1f, 0.0f, 0.05f));
}

void test_bumpless_transfer_with_integral() {
    PidConfig c = baseCfg();
    c.kp = 0.5f;
    c.ki = 0.2f;
    Pid pid(c);
    // у ручному режимі стік тримав 0.4; вмикаємо регулятор при помилці 0.2
    pid.initialize(1.2f, 1.0f, 0.4f);
    const float out = pid.update(1.2f, 1.0f, 0.02f);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.4f, out);
}

void test_bumpless_transfer_without_integral_uses_slew() {
    PidConfig c = baseCfg();
    c.kp = 1.0f;
    c.ki = 0.0f;
    c.slewRate = 2.0f;
    Pid pid(c);
    pid.initialize(0.0f, 0.0f, 0.6f);
    // P хоче 0, але вихід спускається плавно: 0.6 - 2*0.02 = 0.56
    TEST_ASSERT_FLOAT_WITHIN(1e-4, 0.56f, pid.update(0.0f, 0.0f, 0.02f));
}

void test_gain_change_is_bumpless() {
    PidConfig c = baseCfg();
    c.kp = 0.0f;
    c.ki = 1.0f;
    Pid pid(c);
    for (int i = 0; i < 25; ++i) pid.update(1.0f, 0.0f, 0.02f);  // I = 0.5
    const float before = pid.output();
    c.ki = 5.0f;  // різко збільшили Ki
    pid.setConfig(c);
    const float after = pid.update(1.0f, 0.0f, 0.001f);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, before, after);
}

void test_integrator_hold() {
    PidConfig c = baseCfg();
    c.kp = 0.0f;
    c.ki = 1.0f;
    Pid pid(c);
    pid.setIntegratorHold(true);
    for (int i = 0; i < 50; ++i) pid.update(1.0f, 0.0f, 0.02f);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, pid.terms().i);
}

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_wrap180);
    RUN_TEST(test_heading_error_350_to_10);
    RUN_TEST(test_step_response_sign_and_magnitude);
    RUN_TEST(test_integral_accumulates_with_real_dt);
    RUN_TEST(test_derivative_on_measurement_no_kick_on_setpoint_step);
    RUN_TEST(test_derivative_filter_smooths);
    RUN_TEST(test_derivative_wraps_angles);
    RUN_TEST(test_output_saturates);
    RUN_TEST(test_anti_windup_after_long_saturation);
    RUN_TEST(test_slew_rate_limits_change);
    RUN_TEST(test_dt_zero_keeps_output);
    RUN_TEST(test_dt_negative_keeps_output);
    RUN_TEST(test_dt_huge_is_limited);
    RUN_TEST(test_nan_input_gives_safe_output);
    RUN_TEST(test_safe_output_respects_limits);
    RUN_TEST(test_reset_clears_state);
    RUN_TEST(test_bumpless_transfer_with_integral);
    RUN_TEST(test_bumpless_transfer_without_integral_uses_slew);
    RUN_TEST(test_gain_change_is_bumpless);
    RUN_TEST(test_integrator_hold);
    return UNITY_END();
}
