#pragma once
// Калібрування магнітометра (hard iron + спрощений soft iron).
//
// Як: `cal start`, повільно покрутити зібраний човен у всі боки (повне коло
// по горизонталі + нахили), `cal stop`. Знаходимо min/max по кожній осі:
//   offset = (max + min) / 2   — прибирає постійне поле від заліза на човні
//   scale  = середній радіус / радіус осі — вирівнює «сплюснуте» коло

#include <cstddef>

#include "heading_math.h"

namespace nav {

struct MagCal {
    Vec3 offset{0.0f, 0.0f, 0.0f};
    Vec3 scale{1.0f, 1.0f, 1.0f};
};

inline Vec3 applyMagCal(const MagCal& c, const Vec3& raw) {
    return Vec3{(raw.x - c.offset.x) * c.scale.x, (raw.y - c.offset.y) * c.scale.y,
                (raw.z - c.offset.z) * c.scale.z};
}

enum class CalResult { Ok, NotEnoughSamples, NotEnoughRotation };

class MagCalibrator {
public:
    static constexpr size_t kMinSamples = 200;
    static constexpr float kMinRange = 0.2f;  // гаус; поле Землі ~0.5 Гс → розмах ~1 Гс

    void start() {
        active_ = true;
        n_ = 0;
        min_ = Vec3{1e9f, 1e9f, 1e9f};
        max_ = Vec3{-1e9f, -1e9f, -1e9f};
    }
    bool active() const { return active_; }
    size_t samples() const { return n_; }

    void add(const Vec3& v) {
        if (!active_ || !std::isfinite(v.x) || !std::isfinite(v.y) || !std::isfinite(v.z)) return;
        ++n_;
        min_.x = std::fmin(min_.x, v.x); max_.x = std::fmax(max_.x, v.x);
        min_.y = std::fmin(min_.y, v.y); max_.y = std::fmax(max_.y, v.y);
        min_.z = std::fmin(min_.z, v.z); max_.z = std::fmax(max_.z, v.z);
    }

    // X і Y обов'язкові (горизонтальне коло). Z — якщо човен нахиляли достатньо,
    // інакше лишається без змін (на рівній воді він майже не впливає).
    CalResult finish(MagCal& out) {
        active_ = false;
        if (n_ < kMinSamples) return CalResult::NotEnoughSamples;
        const float rx = (max_.x - min_.x) * 0.5f;
        const float ry = (max_.y - min_.y) * 0.5f;
        const float rz = (max_.z - min_.z) * 0.5f;
        if (rx * 2.0f < kMinRange || ry * 2.0f < kMinRange) return CalResult::NotEnoughRotation;
        const bool zOk = rz * 2.0f >= kMinRange;

        MagCal c;
        c.offset.x = (max_.x + min_.x) * 0.5f;
        c.offset.y = (max_.y + min_.y) * 0.5f;
        c.offset.z = zOk ? (max_.z + min_.z) * 0.5f : 0.0f;
        const float avg = zOk ? (rx + ry + rz) / 3.0f : (rx + ry) * 0.5f;
        c.scale.x = avg / rx;
        c.scale.y = avg / ry;
        c.scale.z = zOk ? avg / rz : 1.0f;
        out = c;
        return CalResult::Ok;
    }

private:
    bool active_ = false;
    size_t n_ = 0;
    Vec3 min_{};
    Vec3 max_{};
};

}  // namespace nav
