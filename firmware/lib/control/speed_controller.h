#pragma once
// Регулятор швидкості: швидкість з GPS (м/с) → газ 0..1.
//
// feedforward = ff * setpoint — «приблизний газ для цієї швидкості»,
// PID лише доправляє різницю. Якщо GPS недостовірний → газ 0.

#include "pid.h"

namespace ctl {

struct SpeedConfig {
    float kp = 0.10f;  // газу на 1 м/с помилки
    float ki = 0.0f;
    float kd = 0.0f;
    float ff = 0.05f;  // газу на 1 м/с заданої швидкості
    float dFilterTau = 0.3f;
    float slewRate = 0.5f;  // газ 0→100% не швидше ніж за 2 с
};

class SpeedController {
public:
    SpeedController() { setConfig(SpeedConfig{}); }
    explicit SpeedController(const SpeedConfig& cfg) { setConfig(cfg); }

    void setConfig(const SpeedConfig& cfg);

    // speedValid = false (немає фіксу або дані застарілі) → 0 і скидання.
    float update(float setpointMps, float speedMps, bool speedValid, float dt);

    void engage(float setpointMps, float speedMps, float currentThrottle);
    void reset() { pid_.reset(); }

    const PidTerms& terms() const { return pid_.terms(); }
    bool dataFault() const { return fault_; }

private:
    Pid pid_;
    float ff_ = 0.0f;
    bool fault_ = false;
};

}  // namespace ctl
