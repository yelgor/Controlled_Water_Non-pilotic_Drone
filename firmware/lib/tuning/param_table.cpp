#include "param_table.h"

#include <cmath>
#include <cstring>

namespace tuning {

using core::Params;

namespace {
const ParamDef kParams[] = {
    {"hdg.kp", &Params::hdgKp, 0.0f, 0.2f, "курс: Kp, керма на 1 градус"},
    {"hdg.ki", &Params::hdgKi, 0.0f, 0.1f, "курс: Ki"},
    {"hdg.kd", &Params::hdgKd, 0.0f, 0.2f, "курс: Kd, керма на 1 град/с"},
    {"hdg.dtau", &Params::hdgDtau, 0.0f, 2.0f, "курс: фільтр D, с"},
    {"hdg.slew", &Params::hdgSlew, 0.0f, 20.0f, "курс: швидкість керма, од/с (0=без обмеж.)"},
    {"hdg.minthr", &Params::hdgMinThr, 0.0f, 1.0f, "курс: мін. газ, щоб інтегрувати"},
    {"spd.en", &Params::spdEn, 0.0f, 1.0f, "швидкість: 1=тримати spd.sp, 0=газ зі стіка"},
    {"spd.sp", &Params::spdSp, 0.0f, 15.0f, "швидкість: задана, м/с"},
    {"spd.kp", &Params::spdKp, 0.0f, 2.0f, "швидкість: Kp, газу на 1 м/с"},
    {"spd.ki", &Params::spdKi, 0.0f, 1.0f, "швидкість: Ki"},
    {"spd.kd", &Params::spdKd, 0.0f, 1.0f, "швидкість: Kd"},
    {"spd.ff", &Params::spdFf, 0.0f, 0.5f, "швидкість: feedforward, газу на 1 м/с"},
    {"spd.slew", &Params::spdSlew, 0.0f, 5.0f, "швидкість: зміна газу, од/с"},
    {"thr.max", &Params::thrMax, 0.0f, 1.0f, "БЕЗПЕКА: максимальний газ 0..1"},
    {"rud.trim", &Params::rudTrim, -0.5f, 0.5f, "кермо: трим"},
    {"rud.rev", &Params::rudRev, 0.0f, 1.0f, "кермо: 1=реверс серво"},
    {"rud.min", &Params::rudMin, 0.1f, 1.0f, "кермо: частка ходу на повній швидкості (1=вимк.)"},
    {"rud.vmax", &Params::rudVmax, 0.5f, 30.0f, "кермо: «повна» швидкість для rud.min, м/с"},
    {"mag.decl", &Params::magDecl, -30.0f, 30.0f, "компас: магнітне схилення, град"},
    {"mag.offx", &Params::magOffX, -2.0f, 2.0f, "компас: зміщення X (з cal)"},
    {"mag.offy", &Params::magOffY, -2.0f, 2.0f, "компас: зміщення Y (з cal)"},
    {"mag.offz", &Params::magOffZ, -2.0f, 2.0f, "компас: зміщення Z (з cal)"},
    {"mag.sclx", &Params::magSclX, 0.2f, 5.0f, "компас: масштаб X (з cal)"},
    {"mag.scly", &Params::magSclY, 0.2f, 5.0f, "компас: масштаб Y (з cal)"},
    {"mag.sclz", &Params::magSclZ, 0.2f, 5.0f, "компас: масштаб Z (з cal)"},
    {"log.hz", &Params::logHz, 1.0f, 50.0f, "лог: рядків за секунду"},
};
constexpr size_t kCount = sizeof(kParams) / sizeof(kParams[0]);
}  // namespace

size_t paramCount() { return kCount; }
const ParamDef& paramAt(size_t i) { return kParams[i < kCount ? i : 0]; }

const ParamDef* findParam(const char* name) {
    if (!name) return nullptr;
    for (const ParamDef& d : kParams) {
        if (std::strcmp(d.name, name) == 0) return &d;
    }
    return nullptr;
}

SetResult setParam(Params& p, const char* name, float value) {
    const ParamDef* d = findParam(name);
    if (!d) return SetResult::UnknownName;
    if (!std::isfinite(value) || value < d->min || value > d->max) return SetResult::OutOfRange;
    p.*(d->field) = value;
    return SetResult::Ok;
}

bool getParam(const Params& p, const char* name, float& out) {
    const ParamDef* d = findParam(name);
    if (!d) return false;
    out = p.*(d->field);
    return true;
}

bool paramsValid(const Params& p) {
    for (const ParamDef& d : kParams) {
        const float v = p.*(d.field);
        if (!std::isfinite(v) || v < d.min || v > d.max) return false;
    }
    return true;
}

}  // namespace tuning
