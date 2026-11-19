#pragma once
// Робота з кутами в градусах.

#include <cmath>

namespace ctl {

// Будь-який кут → [-180, 180). Приклад: 340 → -20.
inline float wrap180(float deg) {
    if (!std::isfinite(deg)) return deg;
    float r = std::fmod(deg + 180.0f, 360.0f);
    if (r < 0.0f) r += 360.0f;
    return r - 180.0f;
}

// Будь-який кут → [0, 360). Приклад: -10 → 350.
inline float wrap360(float deg) {
    if (!std::isfinite(deg)) return deg;
    float r = std::fmod(deg, 360.0f);
    if (r < 0.0f) r += 360.0f;
    return r;
}

// Найкоротша помилка курсу: куди і на скільки повернути від heading до setpoint.
// Плюс — праворуч (за годинниковою), мінус — ліворуч.
inline float headingError(float setpointDeg, float headingDeg) {
    return wrap180(setpointDeg - headingDeg);
}

template <typename T>
inline T clamp(T v, T lo, T hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}

}  // namespace ctl
