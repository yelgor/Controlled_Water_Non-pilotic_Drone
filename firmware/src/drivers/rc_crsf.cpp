#include "rc_crsf.h"

#include "config.h"

void RcCrsf::begin(HardwareSerial& port, int rx, int tx, uint32_t baud) {
    port_ = &port;
    port_->begin(baud, SERIAL_8N1, rx, tx);
}

void RcCrsf::poll(uint32_t nowMs) {
    if (!port_) return;
    while (port_->available() > 0) {
        if (parser_.push(static_cast<uint8_t>(port_->read()))) {
            lastFrameMs_ = nowMs;
            everReceived_ = true;
        }
    }
}

core::RcInput RcCrsf::input(uint32_t nowMs) const {
    core::RcInput in;
    in.linkOk = everReceived_ && (nowMs - lastFrameMs_) < cfg::kRcTimeoutMs;
    if (!in.linkOk) return in;  // усе вимкнено
    in.rudder = rc::CrsfParser::toBipolar(parser_.channel(cfg::kChRudder));
    in.throttle = rc::CrsfParser::toUnit(parser_.channel(cfg::kChThrottle));
    in.armSwitch = rc::CrsfParser::toSwitch(parser_.channel(cfg::kChArm));
    in.holdSwitch = rc::CrsfParser::toSwitch(parser_.channel(cfg::kChHold));
    return in;
}
