#pragma once
// Усі налаштування, які можна міняти через serial без перепрошивки.
// Значення за замовчуванням — консервативні: малий Kp, Ki = Kd = 0, газ ≤ 30%.
// Усі поля float — так простіше зберігати і міняти командою `set`.

#include <cstdint>

#include "heading_controller.h"
#include "mag_calibration.h"
#include "mixer.h"
#include "speed_controller.h"

namespace core {

struct Params {
    // --- курс (hdg.*) ---
    float hdgKp = 0.01f;   // 30° помилки → кермо 0.3
    float hdgKi = 0.0f;
    float hdgKd = 0.0f;
    float hdgDtau = 0.10f;
    float hdgSlew = 2.0f;
    float hdgMinThr = 0.10f;

    // --- швидкість (spd.*) ---
    float spdEn = 0.0f;  // 0 = газ зі стіка, 1 = тримати spd.sp
    float spdSp = 1.0f;  // м/с
    float spdKp = 0.10f;
    float spdKi = 0.0f;
    float spdKd = 0.0f;
    float spdFf = 0.05f;
    float spdSlew = 0.5f;

    // --- безпека й актуатори ---
    float thrMax = 0.30f;
    float rudTrim = 0.0f;
    float rudRev = 0.0f;
    // Чим швидше — тим менший хід керма (лінійно): на місці 100%,
    // на швидкості rud.vmax і вище — rud.min. rud.min = 1 вимикає ефект.
    // Без GPS замість швидкості береться газ (газ = thr.max ≈ «повна швидкість»).
    float rudMin = 0.4f;
    float rudVmax = 6.0f;  // м/с
    // Загальний хід керма (частка від ±400 мкс) і швидкість, з якою серво може рухатись,
    // у всіх режимах. rud.rate — ходів за секунду: 2.5 → від центру до упору за 0.4 с.
    // rud.rate = 0 вимикає обмеження швидкості.
    float rudMax = 0.8f;
    float rudRate = 2.5f;

    // --- компас (mag.*) ---
    float magDecl = 0.0f;  // магнітне схилення, °
    float magOffX = 0.0f, magOffY = 0.0f, magOffZ = 0.0f;
    float magSclX = 1.0f, magSclY = 1.0f, magSclZ = 1.0f;

    // --- лог ---
    float logHz = 10.0f;
};

// Версія формату для збереження у flash. Міняти, коли змінюється Params.
constexpr uint32_t kParamsVersion = 3;  // 2: rud.min, rud.vmax; 3: rud.max, rud.rate

inline ctl::HeadingConfig headingConfig(const Params& p) {
    ctl::HeadingConfig c;
    c.kp = p.hdgKp;
    c.ki = p.hdgKi;
    c.kd = p.hdgKd;
    c.dFilterTau = p.hdgDtau;
    c.slewRate = p.hdgSlew;
    c.minSteerThrottle = p.hdgMinThr;
    return c;
}

inline ctl::SpeedConfig speedConfig(const Params& p) {
    ctl::SpeedConfig c;
    c.kp = p.spdKp;
    c.ki = p.spdKi;
    c.kd = p.spdKd;
    c.ff = p.spdFf;
    c.slewRate = p.spdSlew;
    return c;
}

inline ctl::MixerConfig mixerConfig(const Params& p) {
    ctl::MixerConfig c;
    c.maxThrottle = p.thrMax;
    c.rudderTrim = p.rudTrim;
    c.servoReversed = p.rudRev > 0.5f;
    return c;
}

inline nav::MagCal magCal(const Params& p) {
    nav::MagCal c;
    c.offset = nav::Vec3{p.magOffX, p.magOffY, p.magOffZ};
    c.scale = nav::Vec3{p.magSclX, p.magSclY, p.magSclZ};
    return c;
}

}  // namespace core
