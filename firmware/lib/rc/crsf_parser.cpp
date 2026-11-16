#include "crsf_parser.h"

namespace rc {

uint8_t CrsfParser::crc8(const uint8_t* data, size_t len) {
    uint8_t crc = 0;
    for (size_t i = 0; i < len; ++i) {
        crc ^= data[i];
        for (int b = 0; b < 8; ++b) crc = (crc & 0x80) ? static_cast<uint8_t>((crc << 1) ^ 0xD5)
                                                       : static_cast<uint8_t>(crc << 1);
    }
    return crc;
}

void CrsfParser::unpackChannels(const uint8_t* p, uint16_t out[kNumChannels]) {
    uint32_t bits = 0;
    int count = 0;
    size_t ch = 0;
    for (size_t i = 0; i < 22 && ch < kNumChannels; ++i) {
        bits |= static_cast<uint32_t>(p[i]) << count;
        count += 8;
        while (count >= 11 && ch < kNumChannels) {
            out[ch++] = static_cast<uint16_t>(bits & 0x7FF);
            bits >>= 11;
            count -= 11;
        }
    }
}

void CrsfParser::packChannels(const uint16_t in[kNumChannels], uint8_t p[22]) {
    uint32_t bits = 0;
    int count = 0;
    size_t idx = 0;
    for (size_t ch = 0; ch < kNumChannels; ++ch) {
        bits |= static_cast<uint32_t>(in[ch] & 0x7FF) << count;
        count += 11;
        while (count >= 8) {
            p[idx++] = static_cast<uint8_t>(bits & 0xFF);
            bits >>= 8;
            count -= 8;
        }
    }
}

float CrsfParser::toBipolar(uint16_t v) {
    float f = (static_cast<float>(v) - kMid) / static_cast<float>(kMax - kMid);
    return f < -1.0f ? -1.0f : (f > 1.0f ? 1.0f : f);
}

float CrsfParser::toUnit(uint16_t v) {
    float f = (static_cast<float>(v) - kMin) / static_cast<float>(kMax - kMin);
    return f < 0.0f ? 0.0f : (f > 1.0f ? 1.0f : f);
}

bool CrsfParser::push(uint8_t b) {
    newChannels_ = false;
    if (pos_ == 0) {
        // Чекаємо початок кадру. ELRS шле на адресу 0xC8; 0xEE/0xEA теж трапляються.
        if (b == kAddrFc || b == 0xEE || b == 0xEA) buf_[pos_++] = b;
        return false;
    }
    if (pos_ == 1) {
        if (b < 2 || b > 62) {  // неможлива довжина — шукаємо наступний кадр
            pos_ = 0;
            ++bad_;
            return false;
        }
        expected_ = static_cast<size_t>(b) + 2;
        buf_[pos_++] = b;
        return false;
    }
    buf_[pos_++] = b;
    if (pos_ < expected_) return false;

    handleFrame();
    pos_ = 0;
    return newChannels_;
}

void CrsfParser::handleFrame() {
    const size_t len = buf_[1];  // тип + дані + CRC
    const uint8_t crc = crc8(&buf_[2], len - 1);
    if (crc != buf_[expected_ - 1]) {
        ++bad_;
        return;
    }
    ++good_;
    const uint8_t type = buf_[2];
    if (type == kTypeChannels && len == 24) {
        unpackChannels(&buf_[3], ch_);
        newChannels_ = true;
    } else if (type == kTypeLinkStats && len >= 4) {
        lq_ = buf_[3 + 2];  // uplink link quality, %
    }
}

}  // namespace rc
