// Перший тест IMU: ESP32-S3 + IMU по I2C, фільтр Madgwick, CSV у Serial.
// Підтримувані модулі (визначаються автоматично на старті):
//   LSM9DS1                          гіро/акс 0x6B|0x6A, магнітометр 0x1E|0x1C, 238 Гц
//   Adafruit NXP 9-DOF (FXOS8700 +   гіро FXAS21002 0x21|0x20,
//                       FXAS21002)   акс/магнітометр FXOS8700 0x1F|0x1E|0x1D|0x1C, 200 Гц
//
// Потік:
//   1. Старт: пошук модуля на шині, налаштування.
//   2. Калібрування зміщення гіроскопа (1 с пауза + 5 с, плата має лежати нерухомо).
//   3. Цикл за готовністю даних гіроскопа: читання, Madgwick, рядок CSV.
//
// Рядки, що починаються з '#', це коментарі (налаштування, зміщення, повідомлення).
// Команди з клавіатури (у моніторі):
//   c  повторити калібрування гіроскопа
//   m  перемкнути 6DOF (гіро+акс) / 9DOF (гіро+акс+магнітометр)
//   r  скинути орієнтацію фільтра (за акселерометром, yaw = 0)
//   h  ще раз надрукувати заголовок CSV

#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include <TinyGPSPlus.h>

// ---------- Налаштування ----------
static const int   PIN_SDA     = 8;        // типові піни Wire на ESP32-S3 DevKitC
static const int   PIN_SCL     = 9;
static const uint32_t I2C_HZ   = 400000;
static const uint32_t BAUD     = 921600;
static const int   CALIB_MS    = 5000;     // тривалість калібрування гіро
static const float BETA        = 0.1f;     // підсилення Madgwick
static bool        useMag      = false;
// GPS (той самий модуль, що в gps_tracker): TX модуля -> GPIO15, RX модуля -> GPIO16 (можна не під'єднувати),
// VCC -> 5V або 3V3 (як дозволяє модуль), GND -> GND. Рядок у CSV:
//   GPS,t_us,lat,lon,sats,hdop,speed_kmh,course_deg
static const int   PIN_GPS_RX  = 15;
static const int   PIN_GPS_TX  = 16;
static const uint32_t GPS_BAUD = 9600;    // старт у 6DOF: курс = чистий інтеграл гіро (міряємо дрейф)

// ---------- I2C ----------
static bool writeReg(uint8_t addr, uint8_t reg, uint8_t val) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write(val);
  return Wire.endTransmission() == 0;
}

static bool readRegs(uint8_t addr, uint8_t reg, uint8_t *buf, uint8_t n) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(addr, n) != n) return false;
  for (uint8_t i = 0; i < n; i++) buf[i] = Wire.read();
  return true;
}

static uint8_t readReg(uint8_t addr, uint8_t reg) {
  uint8_t v = 0;
  readRegs(addr, reg, &v, 1);
  return v;
}

// Без зайвих транзакцій: перевіряємо, чи пристрій відповідає (ACK), перед читанням WHO_AM_I
static bool present(uint8_t addr) {
  Wire.beginTransmission(addr);
  return Wire.endTransmission() == 0;
}

static void unpackLE(const uint8_t *b, int16_t *v) {   // молодший байт першим (LSM9DS1)
  for (int i = 0; i < 3; i++) v[i] = (int16_t)(b[2 * i] | (b[2 * i + 1] << 8));
}

static void unpackBE(const uint8_t *b, int16_t *v) {   // старший байт першим (NXP)
  for (int i = 0; i < 3; i++) v[i] = (int16_t)((b[2 * i] << 8) | b[2 * i + 1]);
}

static void scanBus() {
  Serial.print("# I2C scan:");
  for (uint8_t a = 1; a < 127; a++)
    if (present(a)) Serial.printf(" 0x%02X", a);
  Serial.println();
}

// ---------- Дані ----------
struct Sample { float g[3], a[3], m[3]; };
static float gyroBias[3] = {0, 0, 0};
static float magLast[3]  = {0, 0, 0};
// Калібрування акселерометра (tools/calib_accel6.py). Нулі/одиниці = вимкнено.
static float accOffset[3] = {0, 0, 0};
static float accScale[3]  = {1, 1, 1};

enum ImuType { IMU_NONE, IMU_LSM9DS1, IMU_NXP };
static ImuType imu = IMU_NONE;
static uint8_t gAddr = 0, mAddr = 0;   // LSM9DS1: gAddr = гіро+акс; NXP: gAddr = FXAS21002, mAddr = FXOS8700
static float   odrHz = 0;

// ===== LSM9DS1 =====
namespace lsm {
const uint8_t WHO_AM_I = 0x0F;      // AG: 0x68, M: 0x3D
const uint8_t CTRL_REG1_G = 0x10, STATUS_REG = 0x17, OUT_X_L_G = 0x18;
const uint8_t CTRL_REG6_XL = 0x20, CTRL_REG8 = 0x22, OUT_X_L_XL = 0x28;
const uint8_t CTRL_REG1_M = 0x20, CTRL_REG2_M = 0x21, CTRL_REG3_M = 0x22, CTRL_REG4_M = 0x23,
              CTRL_REG5_M = 0x24, STATUS_REG_M = 0x27, OUT_X_L_M = 0x28;
const float G_SENS = 0.00875f;      // dps/LSB, ±245 dps
const float A_SENS = 0.000061f;     // g/LSB,   ±2 g
const float M_SENS = 0.00014f;      // gauss/LSB, ±4 гаус

bool find() {
  uint8_t ag = 0, m = 0;
  for (uint8_t a : {0x6B, 0x6A}) if (present(a) && readReg(a, WHO_AM_I) == 0x68) { ag = a; break; }
  for (uint8_t a : {0x1E, 0x1C}) if (present(a) && readReg(a, WHO_AM_I) == 0x3D) { m = a; break; }
  if (!ag || !m) return false;
  gAddr = ag; mAddr = m;
  return true;
}

void configure() {
  writeReg(gAddr, CTRL_REG8, 0x05);       // SW_RESET + IF_ADD_INC
  delay(10);
  writeReg(gAddr, CTRL_REG1_G, 0x80);     // 238 Гц, 245 dps
  writeReg(gAddr, CTRL_REG6_XL, 0x80);    // 238 Гц, ±2 g
  writeReg(gAddr, CTRL_REG8, 0x44);       // BDU + IF_ADD_INC
  writeReg(mAddr, CTRL_REG2_M, 0x0C);     // SOFT_RST + REBOOT
  delay(10);
  writeReg(mAddr, CTRL_REG1_M, 0xFC);     // TEMP_COMP, ultra-high XY, 80 Гц
  writeReg(mAddr, CTRL_REG2_M, 0x00);     // ±4 гаус
  writeReg(mAddr, CTRL_REG3_M, 0x00);     // неперервний режим
  writeReg(mAddr, CTRL_REG4_M, 0x0C);     // ultra-high Z
  writeReg(mAddr, CTRL_REG5_M, 0x40);     // BDU
  delay(50);
  odrHz = 238;
  Serial.println("# cfg: LSM9DS1, ODR 238 Hz, gyro 245 dps, accel 2 g, mag 4 gauss @ 80 Hz");
}

bool ready() { return readReg(gAddr, STATUS_REG) & 0x02; }

bool read(Sample &s) {
  uint8_t b[6];
  int16_t v[3];
  if (!readRegs(gAddr, OUT_X_L_G, b, 6)) return false;
  unpackLE(b, v);
  for (int i = 0; i < 3; i++) s.g[i] = v[i] * G_SENS;
  if (!readRegs(gAddr, OUT_X_L_XL, b, 6)) return false;
  unpackLE(b, v);
  for (int i = 0; i < 3; i++) s.a[i] = v[i] * A_SENS;
  // Магнітометр 80 Гц: читаємо, коли є нові дані. Багатобайтне читання: старший біт адреси.
  if (readReg(mAddr, STATUS_REG_M) & 0x08 && readRegs(mAddr, OUT_X_L_M | 0x80, b, 6)) {
    unpackLE(b, v);
    for (int i = 0; i < 3; i++) magLast[i] = v[i] * M_SENS;
  }
  return true;
}

// Вісь X магнітометра LSM9DS1 протилежна осі X гіро/акселерометра (рис. 1 даташиту).
// Перевірити на столі: при повороті навколо Z yaw у 9DOF має йти в той самий бік, що й у 6DOF.
void magToBody(const float m[3], float out[3]) { out[0] = -m[0]; out[1] = m[1]; out[2] = m[2]; }
}  // namespace lsm

// ===== Adafruit NXP 9-DOF: FXAS21002 (гіро) + FXOS8700 (акс + магнітометр) =====
namespace nxp {
// FXAS21002
const uint8_t G_WHO_AM_I = 0x0C;    // 0xD7
const uint8_t G_DR_STATUS = 0x07, G_OUT_X_MSB = 0x01, G_CTRL_REG0 = 0x0D, G_CTRL_REG1 = 0x13;
// FXOS8700
const uint8_t A_WHO_AM_I = 0x0D;    // 0xC7
const uint8_t A_STATUS = 0x00, A_XYZ_DATA_CFG = 0x0E, A_CTRL_REG1 = 0x2A, A_CTRL_REG2 = 0x2B;
const uint8_t M_CTRL_REG1 = 0x5B, M_CTRL_REG2 = 0x5C;
const float G_SENS = 0.0078125f;    // dps/LSB, ±250 dps
const float A_SENS = 0.000244f;     // g/LSB, ±2 g, 14 біт
const float M_SENS = 0.001f;        // gauss/LSB (0,1 мкТл)

bool find() {
  uint8_t g = 0, am = 0;
  for (uint8_t a : {0x21, 0x20}) if (present(a) && readReg(a, G_WHO_AM_I) == 0xD7) { g = a; break; }
  for (uint8_t a : {0x1F, 0x1E, 0x1D, 0x1C}) if (present(a) && readReg(a, A_WHO_AM_I) == 0xC7) { am = a; break; }
  if (!g || !am) return false;
  gAddr = g; mAddr = am;
  return true;
}

void configure() {
  // Гіроскоп: standby -> ±250 dps -> 200 Гц + active
  writeReg(gAddr, G_CTRL_REG1, 0x00);
  writeReg(gAddr, G_CTRL_REG1, 0x40);     // RST
  delay(10);
  writeReg(gAddr, G_CTRL_REG0, 0x03);     // FS = ±250 dps
  writeReg(gAddr, G_CTRL_REG1, 0x0A);     // DR = 200 Гц, ACTIVE
  // Акселерометр + магнітометр: гібридний режим, 200 Гц на кожен
  writeReg(mAddr, A_CTRL_REG1, 0x00);     // standby
  writeReg(mAddr, A_XYZ_DATA_CFG, 0x00);  // ±2 g
  writeReg(mAddr, A_CTRL_REG2, 0x02);     // high resolution
  writeReg(mAddr, M_CTRL_REG1, 0x1F);     // гібрид акс+маг, макс. оверсемплінг
  writeReg(mAddr, M_CTRL_REG2, 0x20);     // автоінкремент 0x06 -> 0x33 (маг одразу після акс)
  writeReg(mAddr, A_CTRL_REG1, 0x0D);     // ODR 400 Гц (у гібриді 200), low noise, ACTIVE
  delay(100);
  odrHz = 200;
  Serial.println("# cfg: NXP FXAS21002+FXOS8700, ODR 200 Hz, gyro 250 dps, accel 2 g, mag 12 gauss @ 200 Hz");
}

bool ready() { return readReg(gAddr, G_DR_STATUS) & 0x08; }

bool read(Sample &s) {
  uint8_t b[13];
  int16_t v[3];
  if (!readRegs(gAddr, G_OUT_X_MSB, b, 6)) return false;
  unpackBE(b, v);
  for (int i = 0; i < 3; i++) s.g[i] = v[i] * G_SENS;
  // STATUS, акс X..Z (14 біт, вирівняні вліво), маг X..Z
  if (!readRegs(mAddr, A_STATUS, b, 13)) return false;
  unpackBE(b + 1, v);
  for (int i = 0; i < 3; i++) s.a[i] = (v[i] >> 2) * A_SENS;
  unpackBE(b + 7, v);
  for (int i = 0; i < 3; i++) magLast[i] = v[i] * M_SENS;
  return true;
}

// На платі Adafruit осі магнітометра FXOS8700 збігаються з осями акселерометра.
void magToBody(const float m[3], float out[3]) { out[0] = m[0]; out[1] = m[1]; out[2] = m[2]; }
}  // namespace nxp

// ---------- Спільний інтерфейс ----------
static bool findImu() {
  if (lsm::find()) { imu = IMU_LSM9DS1; return true; }
  if (nxp::find()) { imu = IMU_NXP; return true; }
  return false;
}

static void configureImu() { if (imu == IMU_LSM9DS1) lsm::configure(); else nxp::configure(); }

static bool gyroReady() { return imu == IMU_LSM9DS1 ? lsm::ready() : nxp::ready(); }

static void magToBody(const float m[3], float out[3]) {
  if (imu == IMU_LSM9DS1) lsm::magToBody(m, out); else nxp::magToBody(m, out);
}

static bool readSample(Sample &s) {
  bool ok = imu == IMU_LSM9DS1 ? lsm::read(s) : nxp::read(s);
  if (!ok) return false;
  for (int i = 0; i < 3; i++) {
    s.g[i] -= gyroBias[i];
    s.a[i] = (s.a[i] - accOffset[i]) * accScale[i];
    s.m[i] = magLast[i];
  }
  return true;
}

// Чекає на свіжий відлік (до 50 мс)
static bool readFresh(Sample &s) {
  uint32_t t0 = millis();
  while (!gyroReady()) if (millis() - t0 > 50) return false;
  return readSample(s);
}

// ---------- Madgwick ----------
static float q0 = 1, q1 = 0, q2 = 0, q3 = 0;

static void madgwickIMU(float gx, float gy, float gz, float ax, float ay, float az, float dt) {
  float qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
  float qDot2 = 0.5f * ( q0 * gx + q2 * gz - q3 * gy);
  float qDot3 = 0.5f * ( q0 * gy - q1 * gz + q3 * gx);
  float qDot4 = 0.5f * ( q0 * gz + q1 * gy - q2 * gx);

  float an = sqrtf(ax * ax + ay * ay + az * az);
  if (an > 0.0f) {
    ax /= an; ay /= an; az /= an;
    float _2q0 = 2 * q0, _2q1 = 2 * q1, _2q2 = 2 * q2, _2q3 = 2 * q3;
    float _4q0 = 4 * q0, _4q1 = 4 * q1, _4q2 = 4 * q2;
    float _8q1 = 8 * q1, _8q2 = 8 * q2;
    float q0q0 = q0 * q0, q1q1 = q1 * q1, q2q2 = q2 * q2, q3q3 = q3 * q3;
    float s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
    float s1 = _4q1 * q3q3 - _2q3 * ax + 4 * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
    float s2 = 4 * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
    float s3 = 4 * q1q1 * q3 - _2q1 * ax + 4 * q2q2 * q3 - _2q2 * ay;
    float sn = sqrtf(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
    if (sn > 0.0f) {
      qDot1 -= BETA * s0 / sn; qDot2 -= BETA * s1 / sn;
      qDot3 -= BETA * s2 / sn; qDot4 -= BETA * s3 / sn;
    }
  }
  q0 += qDot1 * dt; q1 += qDot2 * dt; q2 += qDot3 * dt; q3 += qDot4 * dt;
  float n = sqrtf(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
  q0 /= n; q1 /= n; q2 /= n; q3 /= n;
}

static void madgwickAHRS(float gx, float gy, float gz, float ax, float ay, float az,
                         float mx, float my, float mz, float dt) {
  float mn = sqrtf(mx * mx + my * my + mz * mz);
  float an = sqrtf(ax * ax + ay * ay + az * az);
  if (mn == 0.0f || an == 0.0f) { madgwickIMU(gx, gy, gz, ax, ay, az, dt); return; }

  float qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
  float qDot2 = 0.5f * ( q0 * gx + q2 * gz - q3 * gy);
  float qDot3 = 0.5f * ( q0 * gy - q1 * gz + q3 * gx);
  float qDot4 = 0.5f * ( q0 * gz + q1 * gy - q2 * gx);

  ax /= an; ay /= an; az /= an;
  mx /= mn; my /= mn; mz /= mn;

  float _2q0mx = 2 * q0 * mx, _2q0my = 2 * q0 * my, _2q0mz = 2 * q0 * mz, _2q1mx = 2 * q1 * mx;
  float _2q0 = 2 * q0, _2q1 = 2 * q1, _2q2 = 2 * q2, _2q3 = 2 * q3;
  float _2q0q2 = 2 * q0 * q2, _2q2q3 = 2 * q2 * q3;
  float q0q0 = q0 * q0, q0q1 = q0 * q1, q0q2 = q0 * q2, q0q3 = q0 * q3;
  float q1q1 = q1 * q1, q1q2 = q1 * q2, q1q3 = q1 * q3;
  float q2q2 = q2 * q2, q2q3 = q2 * q3, q3q3 = q3 * q3;

  // Напрям магнітного поля Землі в системі «світу»
  float hx = mx * q0q0 - _2q0my * q3 + _2q0mz * q2 + mx * q1q1 + _2q1 * my * q2 + _2q1 * mz * q3 - mx * q2q2 - mx * q3q3;
  float hy = _2q0mx * q3 + my * q0q0 - _2q0mz * q1 + _2q1mx * q2 - my * q1q1 + my * q2q2 + _2q2 * mz * q3 - my * q3q3;
  float _2bx = sqrtf(hx * hx + hy * hy);
  float _2bz = -_2q0mx * q2 + _2q0my * q1 + mz * q0q0 + _2q1mx * q3 - mz * q1q1 + _2q2 * my * q3 - mz * q2q2 + mz * q3q3;
  float _4bx = 2 * _2bx, _4bz = 2 * _2bz;

  float s0 = -_2q2 * (2 * q1q3 - _2q0q2 - ax) + _2q1 * (2 * q0q1 + _2q2q3 - ay)
           - _2bz * q2 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx)
           + (-_2bx * q3 + _2bz * q1) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my)
           + _2bx * q2 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
  float s1 = _2q3 * (2 * q1q3 - _2q0q2 - ax) + _2q0 * (2 * q0q1 + _2q2q3 - ay)
           - 4 * q1 * (1 - 2 * q1q1 - 2 * q2q2 - az)
           + _2bz * q3 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx)
           + (_2bx * q2 + _2bz * q0) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my)
           + (_2bx * q3 - _4bz * q1) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
  float s2 = -_2q0 * (2 * q1q3 - _2q0q2 - ax) + _2q3 * (2 * q0q1 + _2q2q3 - ay)
           - 4 * q2 * (1 - 2 * q1q1 - 2 * q2q2 - az)
           + (-_4bx * q2 - _2bz * q0) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx)
           + (_2bx * q1 + _2bz * q3) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my)
           + (_2bx * q0 - _4bz * q2) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
  float s3 = _2q1 * (2 * q1q3 - _2q0q2 - ax) + _2q2 * (2 * q0q1 + _2q2q3 - ay)
           + (-_4bx * q3 + _2bz * q1) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx)
           + (-_2bx * q0 + _2bz * q2) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my)
           + _2bx * q1 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
  float sn = sqrtf(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
  if (sn > 0.0f) {
    qDot1 -= BETA * s0 / sn; qDot2 -= BETA * s1 / sn;
    qDot3 -= BETA * s2 / sn; qDot4 -= BETA * s3 / sn;
  }
  q0 += qDot1 * dt; q1 += qDot2 * dt; q2 += qDot3 * dt; q3 += qDot4 * dt;
  float n = sqrtf(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
  q0 /= n; q1 /= n; q2 /= n; q3 /= n;
}

// Початкова орієнтація з акселерометра (roll, pitch), yaw = 0
static void resetOrientation(const float a[3]) {
  float roll  = atan2f(a[1], a[2]);
  float pitch = atan2f(-a[0], sqrtf(a[1] * a[1] + a[2] * a[2]));
  float cr = cosf(roll / 2), sr = sinf(roll / 2);
  float cp = cosf(pitch / 2), sp = sinf(pitch / 2);
  q0 = cr * cp; q1 = sr * cp; q2 = cr * sp; q3 = -sr * sp;
}

static void toEuler(float &roll, float &pitch, float &yaw) {
  const float R2D = 57.29578f;
  roll  = atan2f(2 * (q0 * q1 + q2 * q3), 1 - 2 * (q1 * q1 + q2 * q2)) * R2D;
  float sp = 2 * (q0 * q2 - q3 * q1);
  sp = sp > 1 ? 1 : (sp < -1 ? -1 : sp);
  pitch = asinf(sp) * R2D;
  yaw   = atan2f(2 * (q0 * q3 + q1 * q2), 1 - 2 * (q2 * q2 + q3 * q3)) * R2D;
}

// ---------- Калібрування ----------
static void calibrateGyro() {
  Serial.printf("# calib: тримайте плату нерухомо %d мс...\n", CALIB_MS);
  for (int i = 0; i < 3; i++) gyroBias[i] = 0;
  double sum[3] = {0, 0, 0}, sum2[3] = {0, 0, 0};
  int n = 0;
  uint32_t t0 = millis();
  Sample s;
  while (millis() - t0 < (uint32_t)CALIB_MS) {
    if (!gyroReady() || !readSample(s)) continue;
    for (int i = 0; i < 3; i++) { sum[i] += s.g[i]; sum2[i] += s.g[i] * s.g[i]; }
    n++;
  }
  if (n < 10) { Serial.println("# calib: ПОМИЛКА, замало відліків"); return; }
  float sd[3];
  for (int i = 0; i < 3; i++) {
    gyroBias[i] = sum[i] / n;
    sd[i] = sqrt(fmax(0.0, sum2[i] / n - (sum[i] / n) * (sum[i] / n)));
  }
  Serial.printf("# gyro_bias_dps,%.5f,%.5f,%.5f\n", gyroBias[0], gyroBias[1], gyroBias[2]);
  Serial.printf("# gyro_std_dps,%.5f,%.5f,%.5f,n=%d\n", sd[0], sd[1], sd[2], n);
  if (sd[0] > 0.5f || sd[1] > 0.5f || sd[2] > 0.5f)
    Serial.println("# calib: УВАГА, великий розкид, плата рухалась? Надішліть 'c' ще раз");
  // Початкова орієнтація після калібрування
  if (readFresh(s)) resetOrientation(s.a);
}

static void printHeader() {
  Serial.printf("# imu,%s\n", imu == IMU_LSM9DS1 ? "LSM9DS1" : "NXP_FXOS8700_FXAS21002");
  Serial.printf("# mode,%s\n", useMag ? "9dof" : "6dof");
  Serial.println("t_us,gx_dps,gy_dps,gz_dps,ax_g,ay_g,az_g,mx_gauss,my_gauss,mz_gauss,"
                 "roll_deg,pitch_deg,yaw_deg,calc_us");
}

// ---------- GPS ----------
static TinyGPSPlus gps;
static uint32_t gpsNoFixMs = 0;

static void pollGps() {
  while (Serial1.available()) gps.encode(Serial1.read());
  if (gps.location.isUpdated() && gps.location.isValid()) {
    Serial.printf("GPS,%lu,%.7f,%.7f,%lu,%.2f,%.2f,%.1f\n", (unsigned long)micros(),
                  gps.location.lat(), gps.location.lng(), (unsigned long)gps.satellites.value(),
                  gps.hdop.hdop(), gps.speed.kmph(), gps.course.deg());
  } else if (millis() - gpsNoFixMs > 5000) {
    gpsNoFixMs = millis();
    if (!gps.location.isValid())
      Serial.printf("# gps nofix, sats=%lu, chars=%lu\n", (unsigned long)gps.satellites.value(),
                    (unsigned long)gps.charsProcessed());
  }
}

// ---------- Arduino ----------
static uint32_t tPrev = 0;

void setup() {
  Serial.setTxBufferSize(4096);
  Serial.begin(BAUD);
  // Нативний USB: чекаємо до 3 с, поки ноутбук відкриє порт, інакше перші рядки губляться
  uint32_t tw = millis();
  while (!Serial && millis() - tw < 3000) delay(10);
  delay(300);
  Serial.println("# imu-test: ESP32-S3 + LSM9DS1 / NXP 9-DOF, Madgwick");
  Wire.begin(PIN_SDA, PIN_SCL, I2C_HZ);
  Serial1.begin(GPS_BAUD, SERIAL_8N1, PIN_GPS_RX, PIN_GPS_TX);

  while (!findImu()) {
    Serial.printf("# IMU не знайдено (SDA=%d, SCL=%d)\n", PIN_SDA, PIN_SCL);
    scanBus();
    delay(2000);
  }
  Serial.printf("# found %s: gyro=0x%02X mag=0x%02X\n",
                imu == IMU_LSM9DS1 ? "LSM9DS1" : "NXP 9-DOF", gAddr, mAddr);
  configureImu();
  delay(1000);   // гіроскоп після увімкнення ще «прогрівається», не калібруємо одразу
  calibrateGyro();
  printHeader();
  tPrev = micros();
}

static void handleCommand(int c) {
  Sample s;
  switch (c) {
    case 'c': calibrateGyro(); printHeader(); tPrev = micros(); break;
    case 'm': useMag = !useMag; printHeader(); break;
    case 'r': if (readFresh(s)) resetOrientation(s.a); Serial.println("# reset"); break;
    case 'h': printHeader(); break;
    default: break;
  }
}

void loop() {
  while (Serial.available()) handleCommand(Serial.read());
  pollGps();

  if (!gyroReady()) return;
  uint32_t t = micros();
  Sample s;
  if (!readSample(s)) { Serial.println("# i2c read error"); return; }
  float dt = (t - tPrev) * 1e-6f;
  tPrev = t;
  if (dt <= 0 || dt > 0.1f) dt = 1.0f / odrHz;

  const float D2R = 0.01745329f;
  uint32_t c0 = micros();
  if (useMag) {
    float mb[3];
    magToBody(s.m, mb);
    madgwickAHRS(s.g[0] * D2R, s.g[1] * D2R, s.g[2] * D2R, s.a[0], s.a[1], s.a[2],
                 mb[0], mb[1], mb[2], dt);
  } else {
    madgwickIMU(s.g[0] * D2R, s.g[1] * D2R, s.g[2] * D2R, s.a[0], s.a[1], s.a[2], dt);
  }
  float roll, pitch, yaw;
  toEuler(roll, pitch, yaw);
  uint32_t calc = micros() - c0;

  Serial.printf("%lu,%.4f,%.4f,%.4f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.3f,%.3f,%.3f,%lu\n",
                (unsigned long)t, s.g[0], s.g[1], s.g[2], s.a[0], s.a[1], s.a[2],
                s.m[0], s.m[1], s.m[2], roll, pitch, yaw, (unsigned long)calc);
}
