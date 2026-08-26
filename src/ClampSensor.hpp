#pragma once
#include <Arduino.h>
#include "Config.hpp"

class ClampSensor {
public:
    explicit ClampSensor(const ClampCfg& cfg)
        : cfg_(cfg) {}

    void setup() {
        pinMode(cfg_.pin, INPUT_PULLUP);
        last_raw_ = readRaw();
        stable_ = last_raw_;
        last_edge_ms_ = millis();
        last_change_ms_ = last_edge_ms_;
        last_emit_ms_ = last_edge_ms_;
    }

    void tick() {
        const bool raw = readRaw();
        const uint32_t now = millis();

        if (raw != last_raw_) {
            last_raw_ = raw;
            last_edge_ms_ = now;
        }

        if (static_cast<uint32_t>(now - last_edge_ms_) >= cfg_.debounce_ms &&
            stable_ != raw) {
            stable_ = raw;
            last_change_ms_ = now;
        }
    }

    // Preserve the currently working installed polarity exactly. The existing
    // firmware treated LOW as clamped regardless of active_high. Diagnostics
    // must observe, not reinterpret, that deployed behavior.
    bool isClamped() const { return !stable_; }

    bool rawLevel() const { return last_raw_; }
    bool stableLevel() const { return stable_; }
    uint32_t lastChangeMs() const { return last_change_ms_; }
    uint32_t debounceMs() const { return cfg_.debounce_ms; }

private:
    bool readRaw() const { return digitalRead(cfg_.pin) == HIGH; }

    const ClampCfg& cfg_;
    bool last_raw_ = false;
    bool stable_ = false;
    uint32_t last_edge_ms_ = 0;
    uint32_t last_change_ms_ = 0;
    uint32_t last_emit_ms_ = 0;
};
