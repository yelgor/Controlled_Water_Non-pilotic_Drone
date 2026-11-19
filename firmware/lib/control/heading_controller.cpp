#include "heading_controller.h"

#include <cmath>

#include "angle.h"

namespace ctl {

void HeadingController::setConfig(const HeadingConfig& cfg) {
    PidConfig p;
    p.kp = cfg.kp;
    p.ki = cfg.ki;
    p.kd = cfg.kd;
    p.outMin = -1.0f;
    p.outMax = 1.0f;
    p.dFilterTau = cfg.dFilterTau;
    p.slewRate = cfg.slewRate;
    p.safeOutput = 0.0f;  // кермо в центр
    p.wrap = 360.0f;
    pid_.setConfig(p);
    minSteerThrottle_ = cfg.minSteerThrottle;
}

float HeadingController::update(float setpointDeg, float headingDeg, float throttle, float dt) {
    const float sp = wrap360(setpointDeg);
    const float hdg = wrap360(headingDeg);
    lastError_ = headingError(sp, hdg);
    const bool noAirflow = !std::isfinite(throttle) || throttle < minSteerThrottle_;
    pid_.setIntegratorHold(noAirflow);
    return pid_.update(sp, hdg, dt);
}

void HeadingController::engage(float setpointDeg, float headingDeg, float currentRudder) {
    pid_.initialize(wrap360(setpointDeg), wrap360(headingDeg), currentRudder);
}

}  // namespace ctl
