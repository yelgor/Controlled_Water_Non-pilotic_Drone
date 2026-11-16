#include "imu.h"

#include <Wire.h>

#include <cmath>

#include "config.h"

namespace {
nav::Vec3 remap(const cfg::AxisMap m[3], float x, float y, float z) {
    const float s[3] = {x, y, z};
    return nav::Vec3{m[0].sign * s[m[0].src], m[1].sign * s[m[1].src], m[2].sign * s[m[2].src]};
}
// Фільтр акселерометра: вібрація мотора не повинна «хитати» курс.
constexpr float kAccelAlpha = 0.1f;  // при 50 Гц стала часу ≈ 0.2 с

// --- FXOS8700 (даташит NXP FXOS8700CQ) ---
constexpr uint8_t kFxosStatus = 0x00;
constexpr uint8_t kFxosWhoAmI = 0x0D;
constexpr uint8_t kFxosWhoAmIValue = 0xC7;
constexpr uint8_t kFxosXyzDataCfg = 0x0E;
constexpr uint8_t kFxosCtrlReg1 = 0x2A;
constexpr uint8_t kFxosCtrlReg2 = 0x2B;
constexpr uint8_t kFxosMCtrlReg1 = 0x5B;
constexpr uint8_t kFxosMCtrlReg2 = 0x5C;
constexpr float kFxosAccelMs2PerLsb = 0.000488f * 9.80665f;  // ±4g, 14 біт
constexpr float kFxosMagUtPerLsb = 0.1f;

bool writeReg(uint8_t addr, uint8_t reg, uint8_t val) {
    Wire.beginTransmission(addr);
    Wire.write(reg);
    Wire.write(val);
    return Wire.endTransmission() == 0;
}

bool readRegs(uint8_t addr, uint8_t reg, uint8_t* buf, size_t n) {
    Wire.beginTransmission(addr);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0) return false;
    if (Wire.requestFrom(addr, static_cast<uint8_t>(n)) != n) return false;
    for (size_t i = 0; i < n; ++i) buf[i] = static_cast<uint8_t>(Wire.read());
    return true;
}

int16_t be16(const uint8_t* p) { return static_cast<int16_t>((p[0] << 8) | p[1]); }
}  // namespace

uint8_t Imu::fxosWhoAmI(uint8_t addr) {
    uint8_t v = 0;
    return readRegs(addr, kFxosWhoAmI, &v, 1) ? v : 0;
}

const char* Imu::chipName() const {
    switch (chip_) {
        case Chip::Lsm9ds1: return "LSM9DS1";
        case Chip::Fxos8700: return "FXOS8700";
        default: return "немає";
    }
}

bool Imu::begin(int sda, int scl) {
    Wire.begin(sda, scl);
    Wire.setClock(400000);
    chip_ = Chip::None;
    if (lsm_.begin()) {
        lsm_.setupAccel(lsm_.LSM9DS1_ACCELRANGE_4G);
        lsm_.setupMag(lsm_.LSM9DS1_MAGGAIN_4GAUSS);
        lsm_.setupGyro(lsm_.LSM9DS1_GYROSCALE_245DPS);
        chip_ = Chip::Lsm9ds1;
        return true;
    }
    if (beginFxos()) {
        chip_ = Chip::Fxos8700;
        return true;
    }
    return false;
}

bool Imu::beginFxos() {
    for (uint8_t a = 0x1C; a <= 0x1F; ++a) {
        if (fxosWhoAmI(a) == kFxosWhoAmIValue) {
            fxosAddr_ = a;
            break;
        }
    }
    if (fxosAddr_ == 0) return false;
    // Налаштування — тільки в режимі standby.
    return writeReg(fxosAddr_, kFxosCtrlReg1, 0x00) &&
           writeReg(fxosAddr_, kFxosXyzDataCfg, 0x01) &&  // ±4g
           writeReg(fxosAddr_, kFxosCtrlReg2, 0x02) &&    // high resolution
           writeReg(fxosAddr_, kFxosMCtrlReg1, 0x1F) &&   // гібрид акс+маг, OSR=7
           writeReg(fxosAddr_, kFxosMCtrlReg2, 0x20) &&   // автоінкремент акс → маг
           writeReg(fxosAddr_, kFxosCtrlReg1, 0x15);      // 100 Гц (гібрид), low noise, active
}

bool Imu::readLsm(float acc[3], float mag[3]) {
    lsm_.read();
    sensors_event_t a, m, g, t;
    lsm_.getEvent(&a, &m, &g, &t);
    acc[0] = a.acceleration.x;
    acc[1] = a.acceleration.y;
    acc[2] = a.acceleration.z;
    mag[0] = m.magnetic.x;
    mag[1] = m.magnetic.y;
    mag[2] = m.magnetic.z;
    return true;
}

bool Imu::readFxos(float acc[3], float mag[3]) {
    // STATUS, акселерометр X/Y/Z (MSB першим), магнітометр X/Y/Z — одним читанням.
    uint8_t b[13];
    if (!readRegs(fxosAddr_, kFxosStatus, b, sizeof(b))) return false;
    for (int i = 0; i < 3; ++i) {
        acc[i] = static_cast<float>(be16(&b[1 + 2 * i]) >> 2) * kFxosAccelMs2PerLsb;
        mag[i] = static_cast<float>(be16(&b[7 + 2 * i])) * kFxosMagUtPerLsb;
    }
    return true;
}

bool Imu::read(uint32_t nowMs) {
    float a[3], m[3];
    bool got = false;
    if (chip_ == Chip::Lsm9ds1) got = readLsm(a, m);
    else if (chip_ == Chip::Fxos8700) got = readFxos(a, m);
    if (!got) return false;

    const nav::Vec3 acc = remap(cfg::kAccelMap, a[0], a[1], a[2]);
    const nav::Vec3 mag = remap(cfg::kMagMap, m[0], m[1], m[2]);
    const float an = std::sqrt(acc.x * acc.x + acc.y * acc.y + acc.z * acc.z);
    const float mn = std::sqrt(mag.x * mag.x + mag.y * mag.y + mag.z * mag.z);
    // Перевірка на правдоподібність: ~1g і не нульове поле.
    if (!std::isfinite(an) || an < 4.0f || an > 20.0f || !std::isfinite(mn) || mn < 1e-3f) {
        return false;
    }
    // down = -accel (акселерометр у спокої показує «вгору»)
    const nav::Vec3 d{-acc.x / an, -acc.y / an, -acc.z / an};
    if (!haveAccel_) {
        down_ = d;
        haveAccel_ = true;
    } else {
        down_.x += kAccelAlpha * (d.x - down_.x);
        down_.y += kAccelAlpha * (d.y - down_.y);
        down_.z += kAccelAlpha * (d.z - down_.z);
    }
    mag_ = mag;
    lastOkMs_ = nowMs;
    return true;
}

bool Imu::ok(uint32_t nowMs) const {
    return chip_ != Chip::None && haveAccel_ && (nowMs - lastOkMs_) < cfg::kImuTimeoutMs;
}

float Imu::heading(const nav::MagCal& cal, float declinationDeg) const {
    return nav::tiltCompensatedHeading(down_, nav::applyMagCal(cal, mag_), declinationDeg);
}
