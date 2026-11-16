#include "actuators.h"

void Actuators::begin(int servoPin, int escPin) {
    ESP32PWM::allocateTimer(0);
    ESP32PWM::allocateTimer(1);
    rudder_.setPeriodHertz(50);
    esc_.setPeriodHertz(50);
    rudder_.attach(servoPin, 1000, 2000);
    esc_.attach(escPin, 1000, 2000);
    safe();
}

void Actuators::write(const ctl::ActuatorPulses& p) {
    rudder_.writeMicroseconds(p.rudderUs);
    esc_.writeMicroseconds(p.escUs);
}

void Actuators::servoOnly(uint16_t us) {
    rudder_.writeMicroseconds(us);
    esc_.writeMicroseconds(1000);
}

void Actuators::safe() {
    rudder_.writeMicroseconds(1500);
    esc_.writeMicroseconds(1000);
}
