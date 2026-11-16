#include <unity.h>

#include <cmath>

#include "angle.h"
#include "crsf_parser.h"
#include "heading_math.h"
#include "mag_calibration.h"

using namespace nav;

void setUp() {}
void tearDown() {}

// ---------- компас ----------

// Будує, що побачать сенсори човна при заданих курсі/крені/тангажі.
// Світ NED: поле Землі — на північ і вниз (як в Україні).
static void bodyVectors(float yawDeg, float pitchDeg, float rollDeg, Vec3& down, Vec3& mag) {
    const float d2r = 0.0174532925f;
    const float y = yawDeg * d2r, p = pitchDeg * d2r, r = rollDeg * d2r;
    const float wb[3] = {0.20f, 0.0f, 0.45f};  // Гс: горизонтальна і вертикальна складові
    const float wg[3] = {0.0f, 0.0f, 1.0f};
    // R = Rz(y) Ry(p) Rx(r); body = R^T * world
    const float cy = std::cos(y), sy = std::sin(y), cp = std::cos(p), sp = std::sin(p),
                cr = std::cos(r), sr = std::sin(r);
    const float R[3][3] = {
        {cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr},
        {sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr},
        {-sp, cp * sr, cp * cr},
    };
    float b[3], g[3];
    for (int i = 0; i < 3; ++i) {
        b[i] = R[0][i] * wb[0] + R[1][i] * wb[1] + R[2][i] * wb[2];
        g[i] = R[0][i] * wg[0] + R[1][i] * wg[1] + R[2][i] * wg[2];
    }
    down = Vec3{g[0], g[1], g[2]};
    mag = Vec3{b[0], b[1], b[2]};
}

void test_heading_level_cardinal_points() {
    const float yaws[] = {0.0f, 45.0f, 90.0f, 180.0f, 270.0f, 359.0f};
    for (float yaw : yaws) {
        Vec3 d, m;
        bodyVectors(yaw, 0, 0, d, m);
        const float h = tiltCompensatedHeading(d, m, 0.0f);
        TEST_ASSERT_FLOAT_WITHIN(0.1f, 0.0f, ctl::headingError(yaw, h));
    }
}

void test_heading_tilt_compensated() {
    // крен до 25° і тангаж до 15° (хвилі) — курс не повинен «гуляти»
    for (float yaw = 0; yaw < 360; yaw += 30) {
        for (float roll = -25; roll <= 25; roll += 12.5f) {
            Vec3 d, m;
            bodyVectors(yaw, 15.0f, roll, d, m);
            const float h = tiltCompensatedHeading(d, m, 0.0f);
            TEST_ASSERT_FLOAT_WITHIN(0.5f, 0.0f, ctl::headingError(yaw, h));
        }
    }
}

void test_heading_declination_and_bad_input() {
    Vec3 d, m;
    bodyVectors(0, 0, 0, d, m);
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 8.0f, tiltCompensatedHeading(d, m, 8.0f));
    TEST_ASSERT_TRUE(std::isnan(tiltCompensatedHeading(Vec3{}, m, 0.0f)));
    TEST_ASSERT_TRUE(std::isnan(tiltCompensatedHeading(d, Vec3{}, 0.0f)));
}

void test_mag_calibration_removes_offset() {
    // поле від заліза на човні: зміщення (0.3, -0.1), вісь Y «стиснута» до 0.8
    MagCalibrator cal;
    cal.start();
    for (int i = 0; i < 360; ++i) {
        const float a = i * 0.0174532925f;
        cal.add(Vec3{0.3f + 0.5f * std::cos(a), -0.1f + 0.4f * std::sin(a), 0.2f});
    }
    MagCal c;
    TEST_ASSERT_EQUAL(static_cast<int>(CalResult::Ok), static_cast<int>(cal.finish(c)));
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.3f, c.offset.x);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, -0.1f, c.offset.y);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, c.offset.z);  // Z не крутили — не чіпаємо
    // після калібрування коло знову кругле
    const Vec3 px = applyMagCal(c, Vec3{0.8f, -0.1f, 0.2f});
    const Vec3 py = applyMagCal(c, Vec3{0.3f, 0.3f, 0.2f});
    TEST_ASSERT_FLOAT_WITHIN(0.01f, px.x, py.y);
}

void test_mag_calibration_rejects_bad_data() {
    MagCalibrator cal;
    cal.start();
    for (int i = 0; i < 10; ++i) cal.add(Vec3{0.1f, 0.1f, 0.1f});
    MagCal c;
    TEST_ASSERT_EQUAL(static_cast<int>(CalResult::NotEnoughSamples), static_cast<int>(cal.finish(c)));
    cal.start();
    for (int i = 0; i < 500; ++i) cal.add(Vec3{0.1f + 0.0001f * i, 0.1f, 0.1f});
    TEST_ASSERT_EQUAL(static_cast<int>(CalResult::NotEnoughRotation), static_cast<int>(cal.finish(c)));
}

// ---------- CRSF ----------

void test_crsf_crc_reference() {
    // CRC-8/DVB-S2, контрольне значення для "123456789" = 0xBC
    const uint8_t s[] = {'1', '2', '3', '4', '5', '6', '7', '8', '9'};
    TEST_ASSERT_EQUAL_HEX8(0xBC, rc::CrsfParser::crc8(s, sizeof(s)));
}

static size_t buildChannelsFrame(const uint16_t ch[16], uint8_t* f) {
    f[0] = 0xC8;
    f[1] = 24;
    f[2] = 0x16;
    rc::CrsfParser::packChannels(ch, &f[3]);
    f[25] = rc::CrsfParser::crc8(&f[2], 23);
    return 26;
}

void test_crsf_parses_channels() {
    uint16_t ch[16];
    for (int i = 0; i < 16; ++i) ch[i] = static_cast<uint16_t>(172 + i * 100);
    uint8_t f[26];
    const size_t n = buildChannelsFrame(ch, f);
    rc::CrsfParser p;
    p.push(0x00);  // сміття перед кадром
    p.push(0x55);
    bool got = false;
    for (size_t i = 0; i < n; ++i) got = p.push(f[i]);
    TEST_ASSERT_TRUE(got);
    for (int i = 0; i < 16; ++i) TEST_ASSERT_EQUAL_UINT16(ch[i], p.channel(i));
    TEST_ASSERT_EQUAL_UINT32(1, p.goodFrames());
}

void test_crsf_rejects_bad_crc() {
    uint16_t ch[16];
    for (int i = 0; i < 16; ++i) ch[i] = 1811;
    uint8_t f[26];
    buildChannelsFrame(ch, f);
    f[10] ^= 0x01;
    rc::CrsfParser p;
    bool got = false;
    for (uint8_t b : f) got = p.push(b);
    TEST_ASSERT_FALSE(got);
    TEST_ASSERT_EQUAL_UINT16(992, p.channel(0));
    TEST_ASSERT_EQUAL_UINT32(1, p.badFrames());
}

void test_crsf_conversions() {
    TEST_ASSERT_FLOAT_WITHIN(1e-3, 0.0f, rc::CrsfParser::toBipolar(992));
    TEST_ASSERT_FLOAT_WITHIN(1e-3, 1.0f, rc::CrsfParser::toBipolar(1811));
    TEST_ASSERT_FLOAT_WITHIN(1e-3, -1.0f, rc::CrsfParser::toBipolar(172));
    TEST_ASSERT_FLOAT_WITHIN(1e-3, 0.0f, rc::CrsfParser::toUnit(172));
    TEST_ASSERT_FLOAT_WITHIN(1e-3, 1.0f, rc::CrsfParser::toUnit(1811));
    TEST_ASSERT_TRUE(rc::CrsfParser::toSwitch(1811));
    TEST_ASSERT_FALSE(rc::CrsfParser::toSwitch(172));
}

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_heading_level_cardinal_points);
    RUN_TEST(test_heading_tilt_compensated);
    RUN_TEST(test_heading_declination_and_bad_input);
    RUN_TEST(test_mag_calibration_removes_offset);
    RUN_TEST(test_mag_calibration_rejects_bad_data);
    RUN_TEST(test_crsf_crc_reference);
    RUN_TEST(test_crsf_parses_channels);
    RUN_TEST(test_crsf_rejects_bad_crc);
    RUN_TEST(test_crsf_conversions);
    return UNITY_END();
}
