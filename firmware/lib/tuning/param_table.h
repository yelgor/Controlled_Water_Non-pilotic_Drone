#pragma once
// Таблиця параметрів: назва для serial-команди ↔ поле Params, з допустимим діапазоном.

#include <cstddef>

#include "params.h"

namespace tuning {

struct ParamDef {
    const char* name;
    float core::Params::*field;
    float min;
    float max;
    const char* help;
};

enum class SetResult { Ok, UnknownName, OutOfRange };

size_t paramCount();
const ParamDef& paramAt(size_t i);
const ParamDef* findParam(const char* name);

SetResult setParam(core::Params& p, const char* name, float value);
bool getParam(const core::Params& p, const char* name, float& out);

// Перевіряє, що всі значення в межах (наприклад, після читання з flash).
bool paramsValid(const core::Params& p);

}  // namespace tuning
