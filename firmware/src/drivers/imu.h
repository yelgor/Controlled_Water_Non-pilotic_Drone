#pragma once
// Компас: акселерометр + магнітометр → курс з компенсацією нахилу.
// Чип визначається автоматично при старті:
//   - LSM9DS1 (0x6B + 0x1E), через бібліотеку Adafruit;
//   - NXP FXOS8700 (0x1C..0x1F, WHO_AM_I = 0xC7), напряму через регістри.
//     Часто стоїть у парі з гіроскопом FXAS21002C (0x20/0x21) — для курсу він не потрібен.

#include <Adafruit_LSM9DS1.h>

#include "heading_math.h"
#include "mag_calibration.h"

class Imu {
public:
    enum class Chip { None, Lsm9ds1, Fxos8700 };

    bool begin(int sda, int scl);
    // Прочитати сенсор (викликати в основному циклі).
    bool read(uint32_t nowMs);
    bool ok(uint32_t nowMs) const;
    Chip chip() const { return chip_; }
    const char* chipName() const;

    // Магнітне поле в осях човна, без калібрування (для `cal`), гаус.
    const nav::Vec3& rawMag() const { return mag_; }
    // Відфільтрований напрям «вниз» в осях човна (рівно → 0, 0, 1). Для діагностики.
    const nav::Vec3& down() const { return down_; }
    // Курс 0..360 або NaN.
    float heading(const nav::MagCal& cal, float declinationDeg) const;

    // WHO_AM_I FXOS8700 за адресою addr; 0 — не відповідає (для команди `i2c`).
    static uint8_t fxosWhoAmI(uint8_t addr);

private:
    bool beginFxos();
    bool readLsm(float acc[3], float mag[3]);
    bool readFxos(float acc[3], float mag[3]);

    Adafruit_LSM9DS1 lsm_;
    Chip chip_ = Chip::None;
    uint8_t fxosAddr_ = 0;
    uint32_t lastOkMs_ = 0;
    nav::Vec3 down_{0.0f, 0.0f, 1.0f};  // відфільтрований напрям «вниз»
    nav::Vec3 mag_{};
    bool haveAccel_ = false;
};
