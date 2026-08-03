#pragma once
#include <Arduino.h>
#include "Config.hpp"

class ContactorMonitor {
public:
  explicit ContactorMonitor(const ContactorCfg& cfg = CFG.contactor) : cfg_(cfg) {}

  void setup() {
    pinMode(cfg_.pin, cfg_.pullup ? INPUT_PULLUP : INPUT);
    rawPowerPresent_ = readPowerPresent();
    stablePowerPresent_ = rawPowerPresent_;
    rawChangedAtMs_ = millis();
    initialized_ = true;
  }

  void tick() {
    const bool now = readPowerPresent();
    const uint32_t ms = millis();
    if (now != rawPowerPresent_) {
      rawPowerPresent_ = now;
      rawChangedAtMs_ = ms;
    }
    if (initialized_ && now != stablePowerPresent_ &&
        static_cast<uint32_t>(ms - rawChangedAtMs_) >= cfg_.debounce_ms) {
      stablePowerPresent_ = now;
      if (stablePowerPresent_) rose_ = true; else fell_ = true;
    }
  }

  bool powerPresent() const { return stablePowerPresent_; }
  bool powerLostEdge() { const bool v = fell_; fell_ = false; return v; }
  bool powerRestoredEdge() { const bool v = rose_; rose_ = false; return v; }

private:
  bool readPowerPresent() const {
    const bool high = digitalRead(cfg_.pin) == HIGH;
    // Installed wiring: HIGH=open/unpowered; LOW=energized/powered.
    return cfg_.high_means_power_lost ? !high : high;
  }
  ContactorCfg cfg_;
  bool initialized_ = false;
  bool rawPowerPresent_ = false;
  bool stablePowerPresent_ = false;
  bool rose_ = false;
  bool fell_ = false;
  uint32_t rawChangedAtMs_ = 0;
};
