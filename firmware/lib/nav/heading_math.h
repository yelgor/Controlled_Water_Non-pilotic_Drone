#pragma once
// Курс з компаса з компенсацією нахилу (човен хитається на хвилях).
//
// Система координат човна: x — вперед (ніс), y — праворуч (правий борт), z — вниз.
//   down — напрям сили тяжіння в цих осях; човен рівно → (0, 0, 1).
//          (Акселерометр у спокої показує протилежне: down = -accel.)
//   mag  — відкалібрований вектор магнітного поля в тих самих осях.
// Повертає курс 0..360° (0 = північ, 90 = схід) або NaN, якщо дані погані.

#include <cmath>

namespace nav {

struct Vec3 {
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
};

inline float tiltCompensatedHeading(const Vec3& down, const Vec3& mag, float declinationDeg) {
    constexpr float kRad2Deg = 57.29577951f;
    const float gNorm = std::sqrt(down.x * down.x + down.y * down.y + down.z * down.z);
    const float mNorm = std::sqrt(mag.x * mag.x + mag.y * mag.y + mag.z * mag.z);
    if (!(gNorm > 1e-6f) || !(mNorm > 1e-9f) || !std::isfinite(declinationDeg)) return NAN;

    const float roll = std::atan2(down.y, down.z);
    const float sr = std::sin(roll), cr = std::cos(roll);
    const float pitch = std::atan2(-down.x, down.y * sr + down.z * cr);
    const float sp = std::sin(pitch), cp = std::cos(pitch);

    // Проєкція магнітного поля на горизонтальну площину.
    const float bx = mag.x * cp + mag.y * sp * sr + mag.z * sp * cr;
    const float by = mag.z * sr - mag.y * cr;
    if (std::fabs(bx) < 1e-9f && std::fabs(by) < 1e-9f) return NAN;

    float hdg = std::atan2(by, bx) * kRad2Deg + declinationDeg;
    hdg = std::fmod(hdg, 360.0f);
    if (hdg < 0.0f) hdg += 360.0f;
    return hdg;
}

}  // namespace nav
