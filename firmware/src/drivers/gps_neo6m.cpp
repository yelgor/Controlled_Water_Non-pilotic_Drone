#include "gps_neo6m.h"

#include "config.h"

void GpsNeo6m::sendUbx(uint8_t cls, uint8_t id, const uint8_t* payload, uint16_t len) {
    uint8_t ckA = 0, ckB = 0;
    auto put = [&](uint8_t b) {
        port_->write(b);
        ckA += b;
        ckB += ckA;
    };
    port_->write(0xB5);
    port_->write(0x62);
    put(cls);
    put(id);
    put(static_cast<uint8_t>(len & 0xFF));
    put(static_cast<uint8_t>(len >> 8));
    for (uint16_t i = 0; i < len; ++i) put(payload[i]);
    port_->write(ckA);
    port_->write(ckB);
    port_->flush();
    delay(20);
}

void GpsNeo6m::begin(HardwareSerial& port, int rx, int tx, uint32_t baud) {
    port_ = &port;
    port_->begin(baud, SERIAL_8N1, rx, tx);
    delay(100);
    // Вимкнути GLL, GSA, GSV, VTG — лишаються GGA (фікс, супутники) і RMC (швидкість).
    const uint8_t off[][3] = {{0xF0, 0x01, 0}, {0xF0, 0x02, 0}, {0xF0, 0x03, 0}, {0xF0, 0x05, 0}};
    for (const auto& m : off) sendUbx(0x06, 0x01, m, 3);
    // CFG-RATE: вимір кожні 200 мс (5 Гц), навігація кожен вимір, час GPS.
    const uint8_t rate[] = {0xC8, 0x00, 0x01, 0x00, 0x01, 0x00};
    sendUbx(0x06, 0x08, rate, sizeof(rate));
}

void GpsNeo6m::poll(uint32_t nowMs) {
    if (!port_) return;
    while (port_->available() > 0) {
        gps_.encode(static_cast<char>(port_->read()));
        if (gps_.speed.isUpdated()) {
            (void)gps_.speed.mps();  // скидає прапорець isUpdated
            lastSpeedMs_ = nowMs;
            haveSpeed_ = true;
        }
    }
}

bool GpsNeo6m::ok(uint32_t nowMs) const {
    return haveSpeed_ && gps_.location.isValid() && gps_.speed.isValid() &&
           gps_.satellites.value() >= cfg::kGpsMinSats &&
           (nowMs - lastSpeedMs_) < cfg::kGpsTimeoutMs;
}

float GpsNeo6m::speedMps() const {
    return static_cast<float>(gps_.speed.mps());
}

unsigned GpsNeo6m::satellites() const {
    return gps_.satellites.value();
}

bool GpsNeo6m::hasLocation() const { return gps_.location.isValid(); }

double GpsNeo6m::lat() const { return hasLocation() ? gps_.location.lat() : 0.0; }

double GpsNeo6m::lon() const { return hasLocation() ? gps_.location.lng() : 0.0; }

float GpsNeo6m::courseDeg() const {
    return gps_.course.isValid() ? static_cast<float>(gps_.course.deg()) : -1.0f;
}
