#pragma once
// Універсальний PID-регулятор.
//
//   output = P + I + D + feedforward
//   P = Kp * error
//   I = накопичена сума Ki * error * dt (зберігається вже в одиницях виходу)
//   D = -Kd * d(measurement)/dt, пропущене через НЧ-фільтр
//
// Особливості:
//  - D рахується від виміру, а не від помилки → немає «удару» при зміні setpoint;
//  - anti-windup: інтеграл не росте, коли вихід уже впирається в межу в той самий бік;
//  - slew rate: вихід змінюється не швидше за slewRate одиниць/с;
//  - NaN/Inf на вході → safeOutput, стан не псується;
//  - без динамічної пам'яті.

namespace ctl {

struct PidConfig {
    float kp = 0.0f;
    float ki = 0.0f;
    float kd = 0.0f;
    float outMin = -1.0f;
    float outMax = 1.0f;
    float dFilterTau = 0.1f;  // с, стала часу фільтра D; 0 = без фільтра
    float slewRate = 0.0f;    // одиниць виходу за секунду; 0 = вимкнено
    float maxDt = 0.5f;       // с; більший dt = «дані зникали», D скидається
    float safeOutput = 0.0f;  // вихід при некоректних вхідних даних
    float wrap = 0.0f;        // 0 = звичайна величина; 360 = кут у градусах
};

struct PidTerms {
    float p = 0.0f;
    float i = 0.0f;
    float d = 0.0f;
    float ff = 0.0f;
    float out = 0.0f;
};

class Pid {
public:
    Pid() = default;
    explicit Pid(const PidConfig& cfg) { setConfig(cfg); }

    // Можна викликати на ходу: стан зберігається, ривка немає.
    void setConfig(const PidConfig& cfg);
    const PidConfig& config() const { return cfg_; }

    // Один крок. dt у секундах. feedforward додається до виходу.
    float update(float setpoint, float measurement, float dt, float feedforward = 0.0f);

    // Повне скидання (вихід стає 0, обмежений межами).
    void reset();

    // Bumpless transfer: підготувати регулятор так, щоб наступний вихід
    // був ≈ currentOutput (наприклад, положення стіка в ручному режимі).
    // Якщо Ki = 0, плавність забезпечує тільки slew rate.
    void initialize(float setpoint, float measurement, float currentOutput,
                    float feedforward = 0.0f);

    // Заморозити інтеграл (наприклад, коли кермо не діє).
    void setIntegratorHold(bool hold) { hold_ = hold; }

    const PidTerms& terms() const { return terms_; }
    float output() const { return lastOut_; }
    bool lastInputInvalid() const { return invalid_; }

private:
    float error(float setpoint, float measurement) const;
    float measurementDelta(float now, float prev) const;

    PidConfig cfg_{};
    PidTerms terms_{};
    float integral_ = 0.0f;
    float dTerm_ = 0.0f;
    float prevMeas_ = 0.0f;
    float lastOut_ = 0.0f;
    bool hasPrev_ = false;
    bool hold_ = false;
    bool invalid_ = false;
};

}  // namespace ctl
