// Водний дрон-аероглісер: утримання курсу і швидкості на PID.
//
// main.cpp тільки збирає модулі разом:
//   пульт (rc_crsf) ─┐
//   компас (imu)  ───┼─→ Vehicle (режими + PID) ─→ actuators (серво + ESC)
//   GPS (gps)     ───┘            │
//                                 └─→ flight_log (CSV у flash), serial_console
//
// Уся логіка — у lib/ (тестується на ПК: pio test -e native).

#include <Arduino.h>
#include <esp_task_wdt.h>

#include "config.h"
#include "console/serial_console.h"
#include "drivers/actuators.h"
#include "drivers/gps_neo6m.h"
#include "drivers/imu.h"
#include "drivers/rc_crsf.h"
#include "mag_calibration.h"
#include "params.h"
#include "storage/flight_log.h"
#include "storage/param_store.h"
#include "telemetry_csv.h"
#include "vehicle.h"

namespace {

core::Params params;
core::Vehicle vehicle;
Actuators actuators;
Imu imu;
GpsNeo6m gps;
RcCrsf rcLink;
FlightLog flightLog;
nav::MagCalibrator magCalibrator;

void feedWatchdog() { esp_task_wdt_reset(); }

SerialConsole console(Serial, ConsoleContext{params, vehicle, flightLog, imu, gps, rcLink,
                                             actuators, magCalibrator, feedWatchdog});

uint32_t lastTickMs = 0;
uint32_t lastLogMs = 0;
char csvLine[256];

void startWatchdog() {
    esp_task_wdt_init(cfg::kWatchdogS, true);  // завис цикл > 2 с → перезавантаження
    esp_task_wdt_add(nullptr);
}

void handleLogging(uint32_t now, const core::VehicleStatus& st) {
    // Новий файл логу на кожен ARM, закриваємо при DISARM.
    if (vehicle.armed() && !flightLog.active()) {
        telemetry::formatCsvHeader(csvLine, sizeof(csvLine));
        flightLog.start(csvLine);  // «лог NNN: запис» надрукує задача запису
    } else if (!vehicle.armed() && flightLog.active()) {
        flightLog.stop();
    }

    const uint32_t period = static_cast<uint32_t>(1000.0f / params.logHz);
    if (now - lastLogMs < period) return;
    lastLogMs = now;

    const bool needLine = flightLog.active() || console.telemetryOn();
    if (!needLine) return;
    telemetry::formatCsvRow(csvLine, sizeof(csvLine), now, st, gps.satellites(), gps.lat(),
                            gps.lon());
    if (flightLog.active()) flightLog.write(csvLine);
    if (console.telemetryOn()) Serial.print(csvLine);
    flightLog.flushIfDue(now);
}

}  // namespace

void setup() {
    // Перше — безпечні виходи: ESC бачить «газ 0» і не стартує.
    actuators.begin(cfg::kServoPin, cfg::kEscPin);

#if ARDUINO_USB_CDC_ON_BOOT
    // Консоль через вбудований USB. Більший буфер — щоб `help` і `log dump` не обрізались;
    // короткий таймаут — щоб без ноутбука (на воді) друк не гальмував керування.
    Serial.setTxBufferSize(4096);
    Serial.setTxTimeoutMs(20);
#endif
    Serial.begin(115200);
    // Дати ноутбуку до 2 с підключитись, щоб побачити повідомлення старту.
    const uint32_t waitStart = millis();
    while (!Serial && millis() - waitStart < 2000) delay(10);

    Serial.println("\n=== Водний дрон: PID курсу і швидкості ===");

    if (storage::loadParams(params)) Serial.println("[1/5] параметри: з flash");
    else Serial.println("[1/5] параметри: заводські (обережні)");
    vehicle.applyParams(params);
    vehicle.setArmSwitchBypass(cfg::kArmSwitchBypass);
    if (cfg::kArmSwitchBypass) {
        Serial.println("!!! ТЕСТ: тумблери ARM і HOLD вимкнено — армиться сам при газі 0,"
                       " газ і кермо зі стіків (config.h)");
    }

    Serial.println("[2/5] компас...");
    if (imu.begin(cfg::kI2cSda, cfg::kI2cScl)) Serial.printf("      OK: %s\n", imu.chipName());
    else Serial.println("      УВАГА: не знайдено! Перевір дроти (команда i2c)");

    Serial.println("[3/5] GPS...");
    gps.begin(Serial1, cfg::kGpsRx, cfg::kGpsTx, cfg::kGpsBaud);
    Serial.println("[4/5] пульт ELRS...");
    rcLink.begin(Serial2, cfg::kRcRx, cfg::kRcTx, cfg::kRcBaud);

    Serial.println("[5/5] пам'ять для логів...");
    if (flightLog.begin(Serial)) Serial.println("      OK");
    else Serial.println("      УВАГА: не вдалося, лог вимкнено");

    Serial.printf("Готово. Ліміт газу: %.0f%%. Введи 'help' для списку команд.\n",
                  params.thrMax * 100.0f);
    startWatchdog();
}

void loop() {
    const uint32_t now = millis();
    rcLink.poll(now);
    gps.poll(now);
    console.poll(now);

    if (now - lastTickMs >= cfg::kLoopPeriodMs) {
        lastTickMs = now;

        imu.read(now);
        if (magCalibrator.active()) magCalibrator.add(imu.rawMag());

        core::VehicleInputs in;
        in.nowMs = now;
        in.rc = rcLink.input(now);
        if (cfg::kArmSwitchBypass) in.rc.holdSwitch = false;  // тест: тільки MANUAL
        in.imuOk = imu.ok(now);
        in.headingDeg = imu.heading(core::magCal(params), params.magDecl);
        in.gpsOk = gps.ok(now);
        in.speedMps = gps.speedMps();

        const core::VehicleStatus& st = vehicle.update(in);
        actuators.write(st.pulses);
        handleLogging(now, st);
    }

    feedWatchdog();
}
