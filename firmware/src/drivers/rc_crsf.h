#pragma once
// ELRS-приймач по CRSF → стіки і тумблери для Vehicle.
// У налаштуваннях приймача failsafe має бути «No Pulses» (за замовчуванням для CRSF):
// тоді при втраті зв'язку кадри перестають іти, і через 0.5 с вмикається FAILSAFE.

#include <Arduino.h>

#include "crsf_parser.h"
#include "vehicle.h"

class RcCrsf {
public:
    void begin(HardwareSerial& port, int rx, int tx, uint32_t baud);
    void poll(uint32_t nowMs);
    core::RcInput input(uint32_t nowMs) const;
    uint8_t linkQuality() const { return parser_.linkQuality(); }
    const rc::CrsfParser& parser() const { return parser_; }

private:
    HardwareSerial* port_ = nullptr;
    rc::CrsfParser parser_;
    uint32_t lastFrameMs_ = 0;
    bool everReceived_ = false;
};
