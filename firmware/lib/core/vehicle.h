#pragma once
// «Мозок» човна: режими, безпека і з'єднання регуляторів.
// Не знає нічого про залізо — отримує готові числа, віддає імпульси PWM.
//
// Режими:
//   FAILSAFE — немає зв'язку з пультом: газ 0, кермо в центр, DISARM.
//   DISARMED — мотор вимкнено, кермо можна рухати стіком (перевірка серво).
//   MANUAL   — пілот керує сам.
//   HOLD     — тумблер HOLD: човен тримає курс, який був у момент вмикання;
//              газ — зі стіка, або з регулятора швидкості, якщо spd.en = 1 і є GPS.
//
// ARM: тумблер ARM переводиться з OFF в ON при газі < 5%.
// Після втрати зв'язку або ввімкнення плати з тумблером у ON — треба перещелкнути.

#include <cstdint>

#include "heading_controller.h"
#include "mixer.h"
#include "params.h"
#include "speed_controller.h"

namespace core {

enum class Mode : uint8_t { Disarmed = 0, Manual = 1, Hold = 2, Failsafe = 3 };

const char* modeName(Mode m);

struct RcInput {
    bool linkOk = false;
    float throttle = 0.0f;  // 0..1
    float rudder = 0.0f;    // -1..1
    bool armSwitch = false;
    bool holdSwitch = false;
};

struct VehicleInputs {
    uint32_t nowMs = 0;
    RcInput rc;
    bool imuOk = false;
    float headingDeg = 0.0f;
    bool gpsOk = false;
    float speedMps = 0.0f;
};

struct VehicleStatus {
    Mode mode = Mode::Failsafe;
    float headingSp = 0.0f;
    float heading = 0.0f;
    ctl::PidTerms hdg{};
    float rudder = 0.0f;       // команда керма (стік або PID), до масштабу швидкості
    float rudderScale = 1.0f;  // хід керма від швидкості: 1 = повний, rud.min = на повній
    bool speedHold = false;
    float speedSp = 0.0f;
    float speed = 0.0f;
    ctl::PidTerms spd{};
    float throttle = 0.0f;
    ctl::ActuatorPulses pulses{1500, 1000};
    bool rcOk = false;
    bool imuOk = false;
    bool gpsOk = false;
};

class Vehicle {
public:
    Vehicle() { applyParams(Params{}); }

    void applyParams(const Params& p);
    const VehicleStatus& update(const VehicleInputs& in);

    // Змінити заданий курс (тільки в HOLD). Повертає false, якщо не в HOLD.
    bool setHeadingSetpoint(float deg);

    // ТІЛЬКИ ДЛЯ ТЕСТІВ: тумблер ARM не потрібен — човен армиться сам, щойно є пульт
    // і газ на нулі. Втрата пульта так само дизармить; після неї — знову газ у нуль.
    void setArmSwitchBypass(bool on) { armBypass_ = on; }
    bool armSwitchBypass() const { return armBypass_; }

    const VehicleStatus& status() const { return st_; }
    bool armed() const { return armed_; }

    static constexpr float kArmThrottleMax = 0.05f;

private:
    Params params_{};
    ctl::HeadingController heading_;
    ctl::SpeedController speed_;
    ctl::MixerConfig mixer_{};
    VehicleStatus st_{};

    uint32_t lastMs_ = 0;
    bool hasLast_ = false;
    bool armed_ = false;
    bool armBypass_ = false;
    bool prevArmSwitch_ = true;  // старт з тумблером у ON не армить
    bool speedEngaged_ = false;
    float lastRudder_ = 0.0f;
    float lastThrottle_ = 0.0f;
    float servoCmd_ = 0.0f;  // що реально подано на серво (після rud.max і rud.rate)
    bool hasServoCmd_ = false;
};

}  // namespace core
