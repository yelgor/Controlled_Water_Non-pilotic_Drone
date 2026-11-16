#pragma once
// GPS NEO-6M: швидкість над землею. При старті налаштовуємо 5 Гц і
// вимикаємо зайві NMEA-повідомлення, щоб 9600 бод вистачало.

#include <Arduino.h>
#include <TinyGPSPlus.h>

class GpsNeo6m {
public:
    void begin(HardwareSerial& port, int rx, int tx, uint32_t baud);
    void poll(uint32_t nowMs);  // читати всі байти, що прийшли
    bool ok(uint32_t nowMs) const;
    float speedMps() const;
    unsigned satellites() const;
    // Координати, градуси WGS84. hasLocation() = false → lat/lon = 0.
    bool hasLocation() const;
    double lat() const;
    double lon() const;

private:
    void sendUbx(uint8_t cls, uint8_t id, const uint8_t* payload, uint16_t len);

    HardwareSerial* port_ = nullptr;
    mutable TinyGPSPlus gps_;  // у TinyGPSPlus геттери не const
    uint32_t lastSpeedMs_ = 0;
    bool haveSpeed_ = false;
};
