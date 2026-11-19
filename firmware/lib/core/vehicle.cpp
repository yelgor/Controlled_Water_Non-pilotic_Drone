#include "vehicle.h"

#include <cmath>

#include "angle.h"

namespace core {

const char* modeName(Mode m) {
    switch (m) {
        case Mode::Disarmed: return "DISARMED";
        case Mode::Manual: return "MANUAL";
        case Mode::Hold: return "HOLD";
        case Mode::Failsafe: return "FAILSAFE";
    }
    return "?";
}

namespace {
float sane(float v, float lo, float hi, float fallback) {
    return std::isfinite(v) ? ctl::clamp(v, lo, hi) : fallback;
}
}  // namespace

void Vehicle::applyParams(const Params& p) {
    params_ = p;
    heading_.setConfig(headingConfig(p));
    speed_.setConfig(speedConfig(p));
    mixer_ = mixerConfig(p);
}

bool Vehicle::setHeadingSetpoint(float deg) {
    if (st_.mode != Mode::Hold || !std::isfinite(deg)) return false;
    st_.headingSp = ctl::wrap360(deg);
    return true;
}

const VehicleStatus& Vehicle::update(const VehicleInputs& in) {
    // Реальний dt. Віднімання uint32 коректне і при переповненні millis().
    float dt = 0.0f;
    if (hasLast_) dt = static_cast<float>(static_cast<uint32_t>(in.nowMs - lastMs_)) / 1000.0f;
    lastMs_ = in.nowMs;
    hasLast_ = true;

    const float stickThr = sane(in.rc.throttle, 0.0f, 1.0f, 0.0f);
    const float stickRud = sane(in.rc.rudder, -1.0f, 1.0f, 0.0f);
    const bool imuOk = in.imuOk && std::isfinite(in.headingDeg);
    const bool gpsOk = in.gpsOk && std::isfinite(in.speedMps);
    const float maxThr = ctl::clamp(params_.thrMax, 0.0f, 1.0f);

    // --- вибір режиму ---
    Mode next;
    if (!in.rc.linkOk) {
        armed_ = false;
        prevArmSwitch_ = true;  // після повернення зв'язку — перещелкнути ARM
        next = Mode::Failsafe;
    } else {
        if (armBypass_) {
            // Тест без тумблера: армимось, коли газ на нулі (мотор не стартує ривком).
            if (!armed_ && stickThr < kArmThrottleMax) armed_ = true;
        } else {
            const bool rising = in.rc.armSwitch && !prevArmSwitch_;
            prevArmSwitch_ = in.rc.armSwitch;
            if (!in.rc.armSwitch) armed_ = false;
            else if (rising && stickThr < kArmThrottleMax) armed_ = true;
        }

        if (!armed_) next = Mode::Disarmed;
        else if (in.rc.holdSwitch && imuOk) next = Mode::Hold;
        else next = Mode::Manual;
    }

    // --- переходи ---
    if (next == Mode::Hold && st_.mode != Mode::Hold) {
        st_.headingSp = ctl::wrap360(in.headingDeg);  // тримаємо курс, який є зараз
        heading_.engage(st_.headingSp, in.headingDeg, lastRudder_);
        speedEngaged_ = false;
    }
    if (next != Mode::Hold && st_.mode == Mode::Hold) {
        heading_.reset();
        speed_.reset();
        speedEngaged_ = false;
    }

    // --- команди ---
    float rudder = 0.0f;
    float throttle = 0.0f;
    bool speedHold = false;
    switch (next) {
        case Mode::Failsafe:
            break;
        case Mode::Disarmed:
            rudder = stickRud;
            break;
        case Mode::Manual:
            rudder = stickRud;
            throttle = stickThr * maxThr;  // увесь хід стіка → 0..thr.max (не обрізаємо)
            break;
        case Mode::Hold: {
            speedHold = params_.spdEn > 0.5f && gpsOk;
            if (speedHold) {
                if (!speedEngaged_) {
                    speed_.engage(params_.spdSp, in.speedMps, lastThrottle_);
                    speedEngaged_ = true;
                }
                throttle = speed_.update(params_.spdSp, in.speedMps, gpsOk, dt);
            } else {
                speedEngaged_ = false;
                throttle = stickThr * maxThr;
            }
            throttle = ctl::clamp(throttle, 0.0f, maxThr);
            rudder = heading_.update(st_.headingSp, in.headingDeg, throttle, dt);
            break;
        }
    }
    throttle = ctl::clamp(throttle, 0.0f, maxThr);

    // Чим швидше — тим менший хід керма. Швидкість з GPS; без GPS — газ як її замінник
    // (аероглісер: більше газу = сильніший потік на кермо).
    float speedFraction = 0.0f;
    if (gpsOk && params_.rudVmax > 0.0f) speedFraction = in.speedMps / params_.rudVmax;
    else if (maxThr > 0.0f) speedFraction = throttle / maxThr;
    const float rudScale = ctl::rudderSpeedScale(speedFraction, params_.rudMin);

    // PID і bumpless працюють з немасштабованим кермом; масштаб — тільки на серво.
    lastRudder_ = rudder;
    lastThrottle_ = throttle;

    st_.mode = next;
    st_.heading = in.headingDeg;
    st_.hdg = (next == Mode::Hold) ? heading_.terms() : ctl::PidTerms{};
    st_.rudder = rudder;
    st_.speedHold = speedHold;
    st_.speedSp = params_.spdSp;
    st_.speed = in.speedMps;
    st_.spd = speedHold ? speed_.terms() : ctl::PidTerms{};
    st_.throttle = throttle;
    st_.rudderScale = rudScale;
    st_.pulses = ctl::mixRudderMotor(rudder * rudScale, throttle, mixer_);
    st_.rcOk = in.rc.linkOk;
    st_.imuOk = imuOk;
    st_.gpsOk = gpsOk;
    return st_;
}

}  // namespace core
