#include "speed_controller.h"

#include <cmath>

namespace ctl {

void SpeedController::setConfig(const SpeedConfig& cfg) {
    PidConfig p;
    p.kp = cfg.kp;
    p.ki = cfg.ki;
    p.kd = cfg.kd;
    p.outMin = 0.0f;  // мотор не реверсивний
    p.outMax = 1.0f;
    p.dFilterTau = cfg.dFilterTau;
    p.slewRate = cfg.slewRate;
    p.safeOutput = 0.0f;
    p.maxDt = 1.0f;  // GPS 1–5 Гц
    pid_.setConfig(p);
    ff_ = cfg.ff;
}

float SpeedController::update(float setpointMps, float speedMps, bool speedValid, float dt) {
    if (!speedValid || !std::isfinite(speedMps)) {
        fault_ = true;
        pid_.reset();
        return 0.0f;
    }
    fault_ = false;
    if (!std::isfinite(setpointMps) || setpointMps <= 0.0f) {
        pid_.reset();
        return 0.0f;
    }
    return pid_.update(setpointMps, speedMps, dt, ff_ * setpointMps);
}

void SpeedController::engage(float setpointMps, float speedMps, float currentThrottle) {
    fault_ = false;
    pid_.initialize(setpointMps, speedMps, currentThrottle, ff_ * setpointMps);
}

}  // namespace ctl
