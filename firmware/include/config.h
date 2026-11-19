#pragma once
// Апаратна конфігурація: піни, канали пульта, таймаути.
// ПЕРЕВІР піни під свою плату ESP32-S3 перед першим ввімкненням!
// Не використовувати: GPIO 0, 3, 45, 46 (strapping), 19/20 (USB),
// 26–37 (flash/PSRAM у N16R8), 43/44 (UART0 = Serial на USB).

#include <cstdint>

namespace cfg {

// --- Компас LSM9DS1 або FXOS8700 (I2C), визначається автоматично ---
constexpr int kI2cSda = 8;
constexpr int kI2cScl = 9;

// --- GPS NEO-6M (UART1). RX плати ← TX GPS ---
constexpr int kGpsRx = 16;
constexpr int kGpsTx = 15;
constexpr uint32_t kGpsBaud = 9600;

// --- ELRS приймач, CRSF (UART2). RX плати ← TX приймача ---
constexpr int kRcRx = 18;
constexpr int kRcTx = 17;
constexpr uint32_t kRcBaud = 420000;

// --- PWM ---
constexpr int kServoPin = 4;
constexpr int kEscPin = 5;

// --- Канали пульта (нумерація з 0: CH1 = 0) ---
constexpr int kChRudder = 0;    // CH1 — стік керма
constexpr int kChThrottle = 2;  // CH3 — газ
constexpr int kChArm = 4;       // CH5 — тумблер ARM
constexpr int kChHold = 7;      // CH8 — тумблер HOLD (тримати курс); на TX12 це тумблер SC

// ТИМЧАСОВО ДЛЯ ТЕСТІВ: true = тумблери не потрібні. ARM (CH5) ігнорується — човен
// армиться сам, щойно є пульт і газ на нулі; HOLD (CH6) ігнорується — завжди MANUAL
// (газ і кермо прямо зі стіків). ПЕРЕД ВИХОДОМ НА ВОДУ ПОВЕРНУТИ false!
constexpr bool kArmSwitchBypass = true;
// Тумблер HOLD (CH8) працює навіть у тестовому режимі без ARM: катер армиться сам,
// а HOLD вмикає утримання курсу. false = як раніше, тільки MANUAL.
constexpr bool kHoldSwitchInBypass = true;

// --- Таймінги ---
constexpr uint32_t kLoopPeriodMs = 20;   // 50 Гц
constexpr uint32_t kRcTimeoutMs = 500;   // немає кадрів пульта → FAILSAFE
constexpr uint32_t kGpsTimeoutMs = 1500;
constexpr uint32_t kImuTimeoutMs = 200;
constexpr uint32_t kWatchdogS = 2;
constexpr unsigned kGpsMinSats = 5;

// --- Як осі компаса лягають на осі човна (x — ніс, y — правий борт, z — вниз) ---
// Для кожної осі човна: {номер осі сенсора 0=x 1=y 2=z, знак}.
// За замовчуванням: плата лежить чипом догори, стрілка X сенсора — на ніс.
// Перевірка напрямків — docs/pid-tuning.md, розділ «Компас».
struct AxisMap {
    uint8_t src;
    int8_t sign;
};
constexpr AxisMap kAccelMap[3] = {{0, +1}, {1, -1}, {2, -1}};
constexpr AxisMap kMagMap[3] = {{0, +1}, {1, -1}, {2, -1}};

}  // namespace cfg
