#pragma once
// Регулятор курсу: курс (з компаса) → положення керма.
//
// Вихід: rudder у [-1, 1], +1 = повний поворот праворуч (курс росте).
// Аероглісер: кермо діє тільки в потоці від пропелера, тому при газі
// нижче minSteerThrottle інтеграл заморожується (інакше «накрутиться» на місці).

#include "pid.h"

namespace ctl {

struct HeadingConfig {
    float kp = 0.01f;  // одиниць керма на 1° помилки
    float ki = 0.0f;
    float kd = 0.0f;
    float dFilterTau = 0.1f;
    float slewRate = 2.0f;  // кермо від центру до краю за 0.5 с
    float minSteerThrottle = 0.10f;
};

class HeadingController {
public:
    HeadingController() { setConfig(HeadingConfig{}); }
    explicit HeadingController(const HeadingConfig& cfg) { setConfig(cfg); }

    void setConfig(const HeadingConfig& cfg);

    // setpoint і курс у градусах (будь-які, нормалізуються), throttle 0..1, dt у с.
    float update(float setpointDeg, float headingDeg, float throttle, float dt);

    // Плавне ввімкнення: кермо продовжує з того місця, де його тримав пілот.
    void engage(float setpointDeg, float headingDeg, float currentRudder);
    void reset() { pid_.reset(); }

    const PidTerms& terms() const { return pid_.terms(); }
    float lastError() const { return lastError_; }

private:
    Pid pid_;
    float minSteerThrottle_ = 0.1f;
    float lastError_ = 0.0f;
};

}  // namespace ctl
