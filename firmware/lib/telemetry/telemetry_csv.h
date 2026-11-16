#pragma once
// Формат рядка телеметрії CSV. Однаковий для логу у flash і для `tele on`.
// Колонки: час, режим, курс (setpoint, вимір, P, I, D, вихід=кермо),
//          швидкість (setpoint, вимір, P, I, D, FF, вихід=газ), стан сенсорів.

#include <cstddef>
#include <cstdint>
#include <cstdio>

#include "vehicle.h"

namespace telemetry {

inline int formatCsvHeader(char* buf, size_t size) {
    return std::snprintf(buf, size,
                         "t_ms,mode,hdg_sp,hdg,hdg_p,hdg_i,hdg_d,rudder,"
                         "spd_hold,spd_sp,spd,spd_p,spd_i,spd_d,spd_ff,throttle,"
                         "rudder_us,esc_us,rc_ok,imu_ok,gps_ok,sats,lat,lon\n");
}

// lat/lon — градуси WGS84 (з GPS), 0 якщо фіксу немає. 7 знаків ≈ 1 см.
inline int formatCsvRow(char* buf, size_t size, uint32_t tMs, const core::VehicleStatus& s,
                        unsigned sats, double lat = 0.0, double lon = 0.0) {
    return std::snprintf(
        buf, size,
        "%lu,%s,%.1f,%.1f,%.4f,%.4f,%.4f,%.3f,"
        "%d,%.2f,%.2f,%.4f,%.4f,%.4f,%.4f,%.3f,"
        "%u,%u,%d,%d,%d,%u,%.7f,%.7f\n",
        static_cast<unsigned long>(tMs), core::modeName(s.mode), s.headingSp, s.heading, s.hdg.p,
        s.hdg.i, s.hdg.d, s.rudder, s.speedHold ? 1 : 0, s.speedSp, s.speed, s.spd.p, s.spd.i,
        s.spd.d, s.spd.ff, s.throttle, static_cast<unsigned>(s.pulses.rudderUs),
        static_cast<unsigned>(s.pulses.escUs), s.rcOk ? 1 : 0, s.imuOk ? 1 : 0, s.gpsOk ? 1 : 0,
        sats, lat, lon);
}

}  // namespace telemetry
