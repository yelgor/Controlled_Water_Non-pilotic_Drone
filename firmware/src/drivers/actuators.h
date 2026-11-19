#pragma once
// Серво керма і ESC (Skywalker 40A): імпульси 50 Гц.

#include <ESP32Servo.h>

#include "mixer.h"

class Actuators {
public:
    // Одразу після ввімкнення шле «газ 0, кермо в центр» — ESC не стартує.
    void begin(int servoPin, int escPin);
    void write(const ctl::ActuatorPulses& p);
    void safe();
    // Тільки серво (для команди `servo`); ESC тримається на 1000 мкс.
    void servoOnly(uint16_t us);

private:
    Servo rudder_;
    Servo esc_;
};
