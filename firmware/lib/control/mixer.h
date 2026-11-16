#pragma once
// Мікшер «кермо + один мотор»: команди (-1..1, 0..1) → імпульси PWM у мікросекундах.
// Тут же застосовується загальний ліміт газу thr.max — у будь-якому режимі.

#include <cmath>
#include <cstdint>

#include "angle.h"

namespace ctl {

struct MixerConfig {
    uint16_t servoCenterUs = 1500;
    uint16_t servoHalfRangeUs = 400;  // ±400 мкс → 1100..1900
    bool servoReversed = false;
    float rudderTrim = 0.0f;  // -0.3..0.3, щоб човен ішов прямо при rudder = 0
    uint16_t escMinUs = 1000;
    uint16_t escMaxUs = 2000;
    float maxThrottle = 0.30f;  // 0..1, обмеження тяги
};

struct ActuatorPulses {
    uint16_t rudderUs;
    uint16_t escUs;
};

// Чим швидше — тим менший хід керма. fraction = частка «повної» швидкості (0..1).
// Лінійно: fraction 0 → 1.0 (повний хід), fraction 1 і більше → minScale.
inline float rudderSpeedScale(float fraction, float minScale) {
    if (!std::isfinite(fraction)) fraction = 0.0f;
    if (!std::isfinite(minScale)) minScale = 1.0f;
    const float f = clamp(fraction, 0.0f, 1.0f);
    const float m = clamp(minScale, 0.0f, 1.0f);
    return 1.0f - (1.0f - m) * f;
}

inline ActuatorPulses mixRudderMotor(float rudder, float throttle, const MixerConfig& c) {
    if (!std::isfinite(rudder)) rudder = 0.0f;
    if (!std::isfinite(throttle)) throttle = 0.0f;

    float r = clamp(rudder, -1.0f, 1.0f);
    if (c.servoReversed) r = -r;
    r = clamp(r + clamp(c.rudderTrim, -0.5f, 0.5f), -1.0f, 1.0f);

    const float maxThr = clamp(c.maxThrottle, 0.0f, 1.0f);
    const float t = clamp(throttle, 0.0f, maxThr);

    ActuatorPulses out;
    out.rudderUs = static_cast<uint16_t>(std::lround(c.servoCenterUs + r * c.servoHalfRangeUs));
    out.escUs = static_cast<uint16_t>(
        std::lround(c.escMinUs + t * static_cast<float>(c.escMaxUs - c.escMinUs)));
    return out;
}

}  // namespace ctl
