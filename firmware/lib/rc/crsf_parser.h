#pragma once
// Парсер протоколу CRSF (ELRS-приймач → контролер, UART 420000 бод).
//
// Кадр: [адреса][довжина][тип][дані...][CRC8]
//   довжина = тип + дані + CRC (у байтах)
//   CRC8 (поліном 0xD5) рахується по типу і даних.
// Канали (тип 0x16): 16 каналів × 11 біт, 22 байти, молодші біти першими.
// Значення 172..1811 ≈ 988..2012 мкс, центр 992 = 1500 мкс.

#include <cstddef>
#include <cstdint>

namespace rc {

class CrsfParser {
public:
    static constexpr uint8_t kAddrFc = 0xC8;
    static constexpr uint8_t kTypeChannels = 0x16;
    static constexpr uint8_t kTypeLinkStats = 0x14;
    static constexpr size_t kNumChannels = 16;
    static constexpr uint16_t kMin = 172;
    static constexpr uint16_t kMid = 992;
    static constexpr uint16_t kMax = 1811;

    // Подати один байт. true — щойно прийшов новий кадр з каналами.
    bool push(uint8_t b);

    const uint16_t* channels() const { return ch_; }
    uint16_t channel(size_t i) const { return i < kNumChannels ? ch_[i] : kMid; }
    uint8_t linkQuality() const { return lq_; }  // 0..100 %, з кадру link stats
    uint32_t goodFrames() const { return good_; }
    uint32_t badFrames() const { return bad_; }

    static uint8_t crc8(const uint8_t* data, size_t len);
    static void unpackChannels(const uint8_t* payload22, uint16_t out[kNumChannels]);
    static void packChannels(const uint16_t in[kNumChannels], uint8_t payload22[22]);

    // Перетворення: стік -1..1, газ 0..1, тумблер (вище середини = ON).
    static float toBipolar(uint16_t v);
    static float toUnit(uint16_t v);
    static bool toSwitch(uint16_t v) { return v > 1400; }

private:
    void handleFrame();

    uint8_t buf_[64] = {};
    size_t pos_ = 0;
    size_t expected_ = 0;
    uint16_t ch_[kNumChannels] = {kMid, kMid, kMin, kMid, kMin, kMin, kMin, kMin,
                                  kMin, kMin, kMin, kMin, kMin, kMin, kMin, kMin};
    uint8_t lq_ = 0;
    uint32_t good_ = 0;
    uint32_t bad_ = 0;
    bool newChannels_ = false;
};

}  // namespace rc
