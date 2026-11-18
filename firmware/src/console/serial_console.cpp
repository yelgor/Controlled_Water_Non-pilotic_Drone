#include "serial_console.h"

#include <Wire.h>

#include <cmath>

#include "drivers/actuators.h"
#include "drivers/gps_neo6m.h"
#include "drivers/imu.h"
#include "drivers/rc_crsf.h"
#include "param_table.h"
#include "storage/flight_log.h"
#include "storage/param_store.h"

using tuning::CmdType;

namespace {
const char* kHelp =
    "Команди:\n"
    "  status                  стан: режим, сенсори, курс, швидкість\n"
    "  get [назва]             параметри (всі або один)\n"
    "  set <назва> <число>     змінити параметр одразу\n"
    "  save | load | defaults  зберегти у flash / прочитати / заводські\n"
    "  hdg <градуси>           новий заданий курс (тільки в HOLD)\n"
    "  cal start | cal stop    калібрування компаса (крутити човен між ними)\n"
    "  log list | log dump <n> | log clear\n"
    "  tele on | tele off      живий CSV у цей порт\n"
    "  i2c                     пристрої на шині I2C (компас LSM9DS1 або FXOS8700)\n"
    "  imu                     сирі дані компаса: нахил, поле до/після калібрування\n"
    "  raw on | raw off        потік RAW,mx,my,mz,dx,dy,dz (Гс, осі човна) для калібрування на ПК\n"
    "  servo                   прогнати серво 1100/1500/1900 мкс (тільки DISARMED, мотор стоїть)\n";
}  // namespace

void SerialConsole::poll(uint32_t nowMs) {
    while (io_.available() > 0) {
        const char ch = static_cast<char>(io_.read());
        if (ch == '\n' || ch == '\r') {
            if (overflow_) {
                io_.println("ERR задовгий рядок");
            } else if (len_ > 0) {
                line_[len_] = '\0';
                execute(tuning::parseCommand(line_), nowMs);
            }
            len_ = 0;
            overflow_ = false;
        } else if (len_ + 1 < sizeof(line_)) {
            line_[len_++] = ch;
        } else {
            overflow_ = true;
        }
    }
}

void SerialConsole::applyParams() { ctx_.vehicle.applyParams(ctx_.params); }

bool SerialConsole::requireDisarmed() {
    if (ctx_.vehicle.armed()) {
        io_.println("ERR спершу DISARM (тумблер ARM вниз)");
        return false;
    }
    return true;
}

void SerialConsole::printParam(const char* name) {
    const tuning::ParamDef* d = tuning::findParam(name);
    if (!d) {
        io_.printf("ERR невідомий параметр '%s' (див. get)\n", name);
        return;
    }
    io_.printf("%-11s = %-9.4f [%g..%g]  %s\n", d->name, ctx_.params.*(d->field), d->min, d->max,
               d->help);
}

void SerialConsole::printAll() {
    for (size_t i = 0; i < tuning::paramCount(); ++i) printParam(tuning::paramAt(i).name);
}

void SerialConsole::printStatus(uint32_t nowMs) {
    const core::VehicleStatus& s = ctx_.vehicle.status();
    io_.printf("режим: %s   пульт: %s (LQ %u%%)   компас: %s   GPS: %s (%u супутн.)\n",
               core::modeName(s.mode), s.rcOk ? "OK" : "НЕМА", ctx_.rc.linkQuality(),
               s.imuOk ? "OK" : "НЕМА", s.gpsOk ? "OK" : "НЕМА", ctx_.gps.satellites());
    io_.printf("курс: %.1f  (задано %.1f)   кермо: %.2f (хід %.0f%%)   швидкість: %.2f м/с"
               "   газ: %.2f\n",
               s.heading, s.headingSp, s.rudder, s.rudderScale * 100.0f, s.speed, s.throttle);
    const core::RcInput rc = ctx_.rc.input(nowMs);
    io_.printf("стіки: газ %.2f%s  кермо %+.2f   ARM(CH5): %s   HOLD(CH6): %s\n", rc.throttle,
               rc.throttle < core::Vehicle::kArmThrottleMax ? " (нуль)" : " (НЕ нуль!)",
               rc.rudder, rc.armSwitch ? "ON" : "OFF", rc.holdSwitch ? "ON" : "OFF");
    io_.print("канали, мкс:");
    for (size_t i = 0; i < 8; ++i) {
        const int us = 1500 + (static_cast<int>(ctx_.rc.parser().channel(i)) - 992) * 5 / 8;
        io_.printf(" CH%u=%d", static_cast<unsigned>(i + 1), us);
    }
    io_.println();
    io_.printf("GPS дані: %lu байт, речень OK %lu, з помилкою %lu%s\n",
               static_cast<unsigned long>(ctx_.gps.charsReceived()),
               static_cast<unsigned long>(ctx_.gps.sentencesOk()),
               static_cast<unsigned long>(ctx_.gps.sentencesBad()),
               ctx_.gps.charsReceived() == 0 ? "  ← немає даних: перевір TX/RX (15/16) і живлення GPS"
                                             : "");
    io_.printf("лог: %s   час: %lu мс\n", ctx_.log.active() ? "пише" : "ні",
               static_cast<unsigned long>(nowMs));
}

void SerialConsole::execute(const tuning::Command& c, uint32_t nowMs) {
    switch (c.type) {
        case CmdType::Empty:
            return;
        case CmdType::Help:
            io_.print(kHelp);
            return;
        case CmdType::Status:
            printStatus(nowMs);
            return;
        case CmdType::Get:
            if (c.hasName) printParam(c.name);
            else printAll();
            return;
        case CmdType::Set: {
            const tuning::SetResult r = tuning::setParam(ctx_.params, c.name, c.value);
            if (r == tuning::SetResult::UnknownName) {
                io_.printf("ERR невідомий параметр '%s'\n", c.name);
            } else if (r == tuning::SetResult::OutOfRange) {
                io_.println("ERR значення поза межами:");
                printParam(c.name);
            } else {
                applyParams();
                io_.print("OK ");
                printParam(c.name);
                io_.println("   (щоб не втратити після перезапуску — save)");
            }
            return;
        }
        case CmdType::Save:
            io_.println(storage::saveParams(ctx_.params) ? "OK збережено" : "ERR не вдалося");
            return;
        case CmdType::Load:
            if (!requireDisarmed()) return;
            if (storage::loadParams(ctx_.params)) {
                applyParams();
                io_.println("OK прочитано з flash");
            } else {
                io_.println("ERR у flash немає збережених параметрів");
            }
            return;
        case CmdType::Defaults:
            if (!requireDisarmed()) return;
            ctx_.params = core::Params{};
            applyParams();
            io_.println("OK заводські значення (не збережені — save)");
            return;
        case CmdType::Heading:
            io_.println(ctx_.vehicle.setHeadingSetpoint(c.value) ? "OK" : "ERR тільки в режимі HOLD");
            return;
        case CmdType::CalStart:
            if (!requireDisarmed()) return;
            ctx_.magCal.start();
            io_.println("OK калібрування: повільно покрути човен по колу і нахили в усі боки,"
                        " потім 'cal stop'");
            return;
        case CmdType::CalStop: {
            if (!ctx_.magCal.active()) {
                io_.println("ERR калібрування не запущене");
                return;
            }
            nav::MagCal cal;
            const nav::CalResult r = ctx_.magCal.finish(cal);
            if (r == nav::CalResult::NotEnoughSamples) {
                io_.println("ERR замало даних — крути довше (20–30 с)");
            } else if (r == nav::CalResult::NotEnoughRotation) {
                io_.println("ERR замало обертання — зроби повне коло");
            } else {
                core::Params p = ctx_.params;
                p.magOffX = cal.offset.x; p.magOffY = cal.offset.y; p.magOffZ = cal.offset.z;
                p.magSclX = cal.scale.x;  p.magSclY = cal.scale.y;  p.magSclZ = cal.scale.z;
                if (!tuning::paramsValid(p)) {
                    io_.println("ERR дивні результати — магніт поруч? Спробуй ще раз");
                    return;
                }
                ctx_.params = p;
                applyParams();
                io_.printf("OK зміщення %.3f %.3f %.3f, масштаб %.3f %.3f %.3f — тепер save\n",
                           p.magOffX, p.magOffY, p.magOffZ, p.magSclX, p.magSclY, p.magSclZ);
            }
            return;
        }
        case CmdType::LogList:
            ctx_.log.list(io_, ctx_.feedWatchdog);
            return;
        case CmdType::LogDump:
            if (!requireDisarmed()) return;
            if (!ctx_.log.dump(io_, static_cast<unsigned>(c.value), ctx_.feedWatchdog))
                io_.println("ERR немає такого логу (log list)");
            return;
        case CmdType::LogClear:
            if (!requireDisarmed()) return;
            io_.println(ctx_.log.clear(ctx_.feedWatchdog) ? "OK логи видалено" : "ERR");
            return;
        case CmdType::TeleOn:
            tele_ = true;
            return;
        case CmdType::RawOn:
            raw_ = true;
            return;
        case CmdType::RawOff:
            raw_ = false;
            return;
        case CmdType::TeleOff:
            tele_ = false;
            return;
        case CmdType::I2cScan: {
            int found = 0;
            for (uint8_t a = 1; a < 127; ++a) {
                // Зламана шина (SDA/SCL замкнені, датчик без живлення) → кожна адреса чекає
                // таймаут, і весь перебір триває секунди — без цього спрацьовує watchdog.
                ctx_.feedWatchdog();
                Wire.beginTransmission(a);
                if (Wire.endTransmission() == 0) {
                    const char* what = "";
                    if (a >= 0x1C && a <= 0x1F && Imu::fxosWhoAmI(a) == 0xC7) {
                        what = "  (FXOS8700 акселерометр/магнітометр)";
                    } else if (a == 0x1E || a == 0x1C) {
                        what = "  (LSM9DS1 магнітометр?)";
                    } else if (a == 0x6B || a == 0x6A) {
                        what = "  (LSM9DS1 акселерометр/гіроскоп)";
                    } else if (a == 0x20 || a == 0x21) {
                        what = "  (FXAS21002 гіроскоп?)";
                    }
                    io_.printf("  знайдено 0x%02X%s\n", a, what);
                    ++found;
                }
            }
            if (found == 0) io_.println("  нічого не знайдено: перевір 3V3, GND, SDA=8, SCL=9");
            else io_.printf("  компас у прошивці: %s (потрібно LSM9DS1 0x1E+0x6B або FXOS8700 0x1C..0x1F)\n",
                            ctx_.imu.chipName());
            return;
        }
        case CmdType::ImuInfo: {
            // Діагностика компаса: осі човна x — ніс, y — правий борт, z — вниз.
            const nav::Vec3 d = ctx_.imu.down();
            const nav::Vec3 r = ctx_.imu.rawMag();
            const nav::Vec3 m = nav::applyMagCal(core::magCal(ctx_.params), r);
            constexpr float kDeg = 57.29578f;
            const float roll = std::atan2(d.y, d.z) * kDeg;
            const float pitch = std::atan2(-d.x, std::sqrt(d.y * d.y + d.z * d.z)) * kDeg;
            io_.printf("чип: %s   вниз: %+.2f %+.2f %+.2f   крен %+.1f°  тангаж %+.1f°\n",
                       ctx_.imu.chipName(), d.x, d.y, d.z, roll, pitch);
            io_.printf("поле сире, Гс:     x %+.3f  y %+.3f  z %+.3f  |B| %.3f\n", r.x, r.y, r.z,
                       std::sqrt(r.x * r.x + r.y * r.y + r.z * r.z));
            io_.printf("поле після cal, Гс: x %+.3f  y %+.3f  z %+.3f  |B| %.3f  гориз. %.3f\n", m.x, m.y,
                       m.z, std::sqrt(m.x * m.x + m.y * m.y + m.z * m.z), std::sqrt(m.x * m.x + m.y * m.y));
            io_.printf("курс: %.1f   (поле Землі тут ≈ 0.5 Гс, горизонтальне ≈ 0.2 Гс)\n",
                       ctx_.imu.heading(core::magCal(ctx_.params), ctx_.params.magDecl));
            return;
        }
        case CmdType::ServoTest: {
            if (!requireDisarmed()) return;
            static constexpr uint16_t kSteps[] = {1500, 1100, 1500, 1900, 1500};
            for (uint16_t us : kSteps) {
                io_.printf("  серво %u мкс\n", us);
                ctx_.act.servoOnly(us);
                for (int i = 0; i < 8; ++i) {  // ~0.8 с, watchdog не спрацює
                    delay(100);
                    ctx_.feedWatchdog();
                }
            }
            io_.println("OK. Не рухалось → перевір 5 В на серво, спільний GND, дріт на GPIO 4");
            return;
        }
        case CmdType::BadArgs:
            io_.println("ERR неправильні аргументи (help)");
            return;
        case CmdType::Unknown:
            io_.println("ERR невідома команда (help)");
            return;
    }
}
