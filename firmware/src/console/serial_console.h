#pragma once
// Serial-консоль: команди з ноутбука на березі (USB, 115200).
// Список команд — `help` або lib/tuning/command_parser.h.

#include <Arduino.h>

#include "command_parser.h"
#include "mag_calibration.h"
#include "params.h"
#include "vehicle.h"

class Actuators;
class FlightLog;
class Imu;
class GpsNeo6m;
class RcCrsf;

struct ConsoleContext {
    core::Params& params;
    core::Vehicle& vehicle;
    FlightLog& log;
    Imu& imu;
    GpsNeo6m& gps;
    RcCrsf& rc;
    Actuators& act;
    nav::MagCalibrator& magCal;
    void (*feedWatchdog)();
};

class SerialConsole {
public:
    SerialConsole(Stream& io, ConsoleContext ctx) : io_(io), ctx_(ctx) {}
    void poll(uint32_t nowMs);
    bool telemetryOn() const { return tele_; }
    bool rawOn() const { return raw_; }

private:
    void execute(const tuning::Command& c, uint32_t nowMs);
    void printParam(const char* name);
    void printAll();
    void printStatus(uint32_t nowMs);
    void applyParams();
    bool requireDisarmed();

    Stream& io_;
    ConsoleContext ctx_;
    char line_[96] = {};
    size_t len_ = 0;
    bool overflow_ = false;
    bool tele_ = false;
    bool raw_ = false;
};
