#pragma once
// Збереження параметрів у NVS (енергонезалежна пам'ять ESP32).
// Якщо дані пошкоджені або старої версії — беремо значення за замовчуванням.

#include "params.h"

namespace storage {

bool loadParams(core::Params& out);
bool saveParams(const core::Params& p);

}  // namespace storage
