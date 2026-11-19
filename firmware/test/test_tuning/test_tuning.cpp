#include <unity.h>

#include <cstring>

#include "command_parser.h"
#include "param_table.h"
#include "telemetry_csv.h"

using namespace tuning;

void setUp() {}
void tearDown() {}

void test_parse_set() {
    Command c = parseCommand("set hdg.kp 0.02\r");
    TEST_ASSERT_EQUAL(static_cast<int>(CmdType::Set), static_cast<int>(c.type));
    TEST_ASSERT_EQUAL_STRING("hdg.kp", c.name);
    TEST_ASSERT_FLOAT_WITHIN(1e-6, 0.02f, c.value);
}

void test_parse_is_case_insensitive_and_trims() {
    Command c = parseCommand("   SET   Spd.Sp   1.5  ");
    TEST_ASSERT_EQUAL(static_cast<int>(CmdType::Set), static_cast<int>(c.type));
    TEST_ASSERT_EQUAL_STRING("spd.sp", c.name);
}

void test_parse_errors() {
    TEST_ASSERT_EQUAL(static_cast<int>(CmdType::BadArgs),
                      static_cast<int>(parseCommand("set hdg.kp abc").type));
    TEST_ASSERT_EQUAL(static_cast<int>(CmdType::BadArgs),
                      static_cast<int>(parseCommand("set hdg.kp").type));
    TEST_ASSERT_EQUAL(static_cast<int>(CmdType::BadArgs),
                      static_cast<int>(parseCommand("set hdg.kp nan").type));
    TEST_ASSERT_EQUAL(static_cast<int>(CmdType::Unknown),
                      static_cast<int>(parseCommand("fly").type));
    TEST_ASSERT_EQUAL(static_cast<int>(CmdType::Empty),
                      static_cast<int>(parseCommand("   ").type));
    TEST_ASSERT_EQUAL(static_cast<int>(CmdType::BadArgs),
                      static_cast<int>(parseCommand("set aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa 1").type));
}

void test_parse_other_commands() {
    TEST_ASSERT_EQUAL(static_cast<int>(CmdType::Get), static_cast<int>(parseCommand("get").type));
    Command g = parseCommand("get thr.max");
    TEST_ASSERT_TRUE(g.hasName);
    TEST_ASSERT_EQUAL(static_cast<int>(CmdType::Save), static_cast<int>(parseCommand("save").type));
    TEST_ASSERT_EQUAL(static_cast<int>(CmdType::CalStart),
                      static_cast<int>(parseCommand("cal start").type));
    Command d = parseCommand("log dump 3");
    TEST_ASSERT_EQUAL(static_cast<int>(CmdType::LogDump), static_cast<int>(d.type));
    TEST_ASSERT_EQUAL_FLOAT(3.0f, d.value);
    TEST_ASSERT_EQUAL(static_cast<int>(CmdType::BadArgs),
                      static_cast<int>(parseCommand("log dump 1.5").type));
    TEST_ASSERT_EQUAL(static_cast<int>(CmdType::Heading),
                      static_cast<int>(parseCommand("hdg 270").type));
    TEST_ASSERT_EQUAL(static_cast<int>(CmdType::I2cScan), static_cast<int>(parseCommand("i2c").type));
    TEST_ASSERT_EQUAL(static_cast<int>(CmdType::ImuInfo), static_cast<int>(parseCommand("imu").type));
    TEST_ASSERT_EQUAL(static_cast<int>(CmdType::RawOn), static_cast<int>(parseCommand("raw on").type));
    TEST_ASSERT_EQUAL(static_cast<int>(CmdType::RawOff), static_cast<int>(parseCommand("raw off").type));
    TEST_ASSERT_EQUAL(static_cast<int>(CmdType::BadArgs), static_cast<int>(parseCommand("raw").type));
    TEST_ASSERT_EQUAL(static_cast<int>(CmdType::ServoTest),
                      static_cast<int>(parseCommand("servo").type));
}

void test_set_param_ok_and_range() {
    core::Params p;
    TEST_ASSERT_EQUAL(static_cast<int>(SetResult::Ok), static_cast<int>(setParam(p, "hdg.kp", 0.03f)));
    TEST_ASSERT_EQUAL_FLOAT(0.03f, p.hdgKp);
    TEST_ASSERT_EQUAL(static_cast<int>(SetResult::OutOfRange),
                      static_cast<int>(setParam(p, "thr.max", 1.5f)));
    TEST_ASSERT_EQUAL_FLOAT(0.30f, p.thrMax);
    TEST_ASSERT_EQUAL(static_cast<int>(SetResult::UnknownName),
                      static_cast<int>(setParam(p, "foo", 1.0f)));
    float v = 0;
    TEST_ASSERT_TRUE(getParam(p, "hdg.kp", v));
    TEST_ASSERT_EQUAL_FLOAT(0.03f, v);
}

void test_defaults_are_conservative_and_valid() {
    core::Params p;
    TEST_ASSERT_TRUE(paramsValid(p));
    TEST_ASSERT_EQUAL_FLOAT(0.0f, p.hdgKi);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, p.hdgKd);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, p.spdKi);
    TEST_ASSERT_TRUE(p.thrMax <= 0.30f);
    p.hdgKp = NAN;
    TEST_ASSERT_FALSE(paramsValid(p));
}

void test_param_names_unique() {
    for (size_t i = 0; i < paramCount(); ++i)
        for (size_t j = i + 1; j < paramCount(); ++j)
            TEST_ASSERT_TRUE(std::strcmp(paramAt(i).name, paramAt(j).name) != 0);
}

void test_csv_header_and_row_have_same_columns() {
    char h[256], r[256];
    telemetry::formatCsvHeader(h, sizeof(h));
    core::VehicleStatus s;
    s.mode = core::Mode::Hold;
    s.headingSp = 350.0f;
    s.heading = 10.0f;
    const int n = telemetry::formatCsvRow(r, sizeof(r), 12345, s, 7);
    TEST_ASSERT_TRUE(n > 0 && n < static_cast<int>(sizeof(r)));
    int ch = 0, cr = 0;
    for (char* c = h; *c; ++c) ch += (*c == ',');
    for (char* c = r; *c; ++c) cr += (*c == ',');
    TEST_ASSERT_EQUAL(ch, cr);
    TEST_ASSERT_EQUAL(0, std::strncmp(r, "12345,HOLD,350.0,10.0,", 22));
    // координати з 7 знаками і в кінці рядка
    telemetry::formatCsvRow(r, sizeof(r), 1, s, 9, 49.8397123, 24.0297456);
    TEST_ASSERT_NOT_NULL(std::strstr(r, ",49.8397123,24.0297456,"));
    // сирі дані компаса й курс GPS — у кінці
    telemetry::CsvExtra ex;
    ex.mx = 0.1234f; ex.my = -0.5f; ex.mz = -1.3912f; ex.dx = 0.01f; ex.dy = -0.02f; ex.dz = 0.999f;
    ex.cog = 271.25f;
    telemetry::formatCsvRow(r, sizeof(r), 1, s, 9, 49.8397123, 24.0297456, ex);
    TEST_ASSERT_NOT_NULL(std::strstr(r, ",0.1234,-0.5000,-1.3912,0.010,-0.020,0.999,271.2\n"));
    cr = 0;
    for (char* c = r; *c; ++c) cr += (*c == ',');
    TEST_ASSERT_EQUAL(ch, cr);
}

void test_csv_worst_case_row_fits_log_line() {
    // Рядок логу обмежений 256 байтами (FlightLog::kLineMax) — найдовші значення мають влазити.
    char h[256], r[512];
    TEST_ASSERT_TRUE(telemetry::formatCsvHeader(h, sizeof(h)) < 256);
    core::VehicleStatus s;
    s.mode = core::Mode::Disarmed;
    s.headingSp = -359.9f; s.heading = -359.9f;
    s.hdg.p = s.hdg.i = s.hdg.d = -99.9999f; s.rudder = -1.0f;
    s.speedSp = s.speed = -99.99f;
    s.spd.p = s.spd.i = s.spd.d = s.spd.ff = -99.9999f; s.throttle = -1.0f;
    telemetry::CsvExtra ex;
    ex.mx = ex.my = ex.mz = -9.9999f; ex.dx = ex.dy = ex.dz = -1.0f; ex.cog = 359.9f;
    const int n = telemetry::formatCsvRow(r, sizeof(r), 4294967295u, s, 99, -89.9999999, -179.9999999, ex);
    TEST_ASSERT_TRUE(n < 256);
}

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_parse_set);
    RUN_TEST(test_parse_is_case_insensitive_and_trims);
    RUN_TEST(test_parse_errors);
    RUN_TEST(test_parse_other_commands);
    RUN_TEST(test_set_param_ok_and_range);
    RUN_TEST(test_defaults_are_conservative_and_valid);
    RUN_TEST(test_param_names_unique);
    RUN_TEST(test_csv_header_and_row_have_same_columns);
    RUN_TEST(test_csv_worst_case_row_fits_log_line);
    return UNITY_END();
}
