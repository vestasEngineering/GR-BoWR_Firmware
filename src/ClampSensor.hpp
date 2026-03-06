#pragma once
#include <Arduino.h>
#include "Config.hpp"

class ClampSensor {
public:
    explicit ClampSensor(const ClampCfg& cfg)
    : cfg_(cfg) {}

    void setup() {
        pinMode(cfg_.pin, INPUT_PULLUP); // Use internal pull-up (OK with open collector)
        last_raw_ = readRaw();
        stable_   = last_raw_;
        last_change_ms_ = millis();
        last_emit_ms_   = last_change_ms_;
    }

    void tick() {
        const bool raw = readRaw();
        const uint32_t now = millis();

        if (raw != last_raw_) {
            last_raw_ = raw;
            last_edge_ms_ = now;
        }

        // debounce
        if ((now - last_edge_ms_) >= cfg_.debounce_ms) {
            if (stable_ != raw) {
                stable_ = raw;
                last_change_ms_ = now;
            }
        }

        // Optional: periodic debug emit (comment out if noisy)
        /*
        if (cfg_.emit_ms > 0 && (now - last_emit_ms_) >= cfg_.emit_ms) {
            last_emit_ms_ = now;
            Serial.print("{\"type\":\"clamp\",\"ts_ms\":");
            Serial.print(now);
            Serial.print(",\"clamped\":");
            Serial.print(isClamped() ? "true" : "false");
            Serial.println("}");
        }
        */
    }

    /// Returns true when the clamp is confirmed engaged (debounced).
    bool isClamped() const {
        // If active_high = true, HIGH means clamped; else invert
        return cfg_.active_high ? !stable_ : !stable_;
    }

    /// Raw read (no debounce, no polarity)
    bool rawLevel() const { return last_raw_; }

    uint32_t lastChangeMs() const { return last_change_ms_; }

private:
    bool readRaw() const {
        int v = digitalRead(cfg_.pin);
        return (v == HIGH);
    }

    const ClampCfg& cfg_;

    // Debounce state
    bool     last_raw_       = false;
    bool     stable_         = false;
    uint32_t last_edge_ms_   = 0;
    uint32_t last_change_ms_ = 0;
    uint32_t last_emit_ms_   = 0;
};