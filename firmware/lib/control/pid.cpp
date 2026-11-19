#include "pid.h"

#include <cmath>

#include "angle.h"

namespace ctl {

namespace {
bool finite(float v) { return std::isfinite(v); }
}  // namespace

void Pid::setConfig(const PidConfig& cfg) {
    const bool kiWasZero = (cfg_.ki == 0.0f);
    cfg_ = cfg;
    if (cfg_.outMin > cfg_.outMax) {
        const float t = cfg_.outMin;
        cfg_.outMin = cfg_.outMax;
        cfg_.outMax = t;
    }
    if (cfg_.dFilterTau < 0.0f) cfg_.dFilterTau = 0.0f;
    if (cfg_.slewRate < 0.0f) cfg_.slewRate = 0.0f;
    if (!(cfg_.maxDt > 0.0f)) cfg_.maxDt = 0.5f;
    // Ki вимкнули — прибираємо «застиглий» інтеграл, щоб він не жив вічно.
    if (cfg_.ki == 0.0f && !kiWasZero) integral_ = 0.0f;
    integral_ = clamp(integral_, cfg_.outMin, cfg_.outMax);
    lastOut_ = clamp(lastOut_, cfg_.outMin, cfg_.outMax);
}

float Pid::error(float setpoint, float measurement) const {
    const float e = setpoint - measurement;
    return cfg_.wrap > 0.0f ? wrap180(e * (360.0f / cfg_.wrap)) * (cfg_.wrap / 360.0f) : e;
}

float Pid::measurementDelta(float now, float prev) const {
    return error(now, prev);  // та сама логіка «найкоротшої різниці»
}

float Pid::update(float setpoint, float measurement, float dt, float feedforward) {
    if (!finite(setpoint) || !finite(measurement) || !finite(dt) || !finite(feedforward)) {
        invalid_ = true;
        hasPrev_ = false;  // старий вимір більше не довіряємо
        dTerm_ = 0.0f;
        lastOut_ = clamp(cfg_.safeOutput, cfg_.outMin, cfg_.outMax);
        terms_ = PidTerms{0.0f, integral_, 0.0f, 0.0f, lastOut_};
        return lastOut_;
    }
    invalid_ = false;

    if (dt <= 0.0f) return lastOut_;  // час не пройшов (або пішов назад) — нічого не міняємо

    if (dt > cfg_.maxDt) {  // довга пауза: похідна безглузда, крок інтегралу обмежуємо
        dt = cfg_.maxDt;
        hasPrev_ = false;
    }

    const float e = error(setpoint, measurement);
    const float p = cfg_.kp * e;

    // D від виміру + НЧ-фільтр першого порядку.
    if (hasPrev_ && cfg_.kd != 0.0f) {
        const float raw = -cfg_.kd * measurementDelta(measurement, prevMeas_) / dt;
        const float alpha = cfg_.dFilterTau > 0.0f ? dt / (cfg_.dFilterTau + dt) : 1.0f;
        dTerm_ += alpha * (raw - dTerm_);
    } else {
        dTerm_ = 0.0f;
    }
    prevMeas_ = measurement;
    hasPrev_ = true;

    // Інтеграл з anti-windup: не інтегруємо, якщо це ще сильніше заганяє вихід у межу.
    if (!hold_ && cfg_.ki != 0.0f) {
        const float candidate = integral_ + cfg_.ki * e * dt;
        const float u = p + candidate + dTerm_ + feedforward;
        const bool pushHigh = (u > cfg_.outMax) && (e > 0.0f);
        const bool pushLow = (u < cfg_.outMin) && (e < 0.0f);
        if (!pushHigh && !pushLow) integral_ = candidate;
    }
    integral_ = clamp(integral_, cfg_.outMin, cfg_.outMax);

    float out = clamp(p + integral_ + dTerm_ + feedforward, cfg_.outMin, cfg_.outMax);

    if (cfg_.slewRate > 0.0f) {
        const float step = cfg_.slewRate * dt;
        out = clamp(out, lastOut_ - step, lastOut_ + step);
    }

    lastOut_ = out;
    terms_ = PidTerms{p, integral_, dTerm_, feedforward, out};
    return out;
}

void Pid::reset() {
    integral_ = 0.0f;
    dTerm_ = 0.0f;
    prevMeas_ = 0.0f;
    hasPrev_ = false;
    invalid_ = false;
    lastOut_ = clamp(0.0f, cfg_.outMin, cfg_.outMax);
    terms_ = PidTerms{0.0f, 0.0f, 0.0f, 0.0f, lastOut_};
}

void Pid::initialize(float setpoint, float measurement, float currentOutput, float feedforward) {
    reset();
    if (!finite(setpoint) || !finite(measurement) || !finite(currentOutput) ||
        !finite(feedforward)) {
        return;
    }
    lastOut_ = clamp(currentOutput, cfg_.outMin, cfg_.outMax);
    prevMeas_ = measurement;
    hasPrev_ = true;
    if (cfg_.ki != 0.0f) {
        const float p = cfg_.kp * error(setpoint, measurement);
        integral_ = clamp(lastOut_ - p - feedforward, cfg_.outMin, cfg_.outMax);
    }
    terms_ = PidTerms{0.0f, integral_, 0.0f, feedforward, lastOut_};
}

}  // namespace ctl
