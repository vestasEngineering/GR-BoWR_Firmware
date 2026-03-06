#ifndef ESTOP_HPP
#define ESTOP_HPP

#include <Arduino.h>
#include "Config.hpp"

// ===== Debug switch (uncomment to enable) =====
//#define ESTOP_DEBUG 1

class EStop {
public:
  struct Config {
    enum class Pull : uint8_t { None, Pullup, Pulldown };

    uint8_t     pin        = D0;              // GPIO input
    bool        activeHigh = false;           // false => LOW means ACTIVE
    Pull        pull       = Pull::Pullup;    // Pull-up typical for active-low wiring
    bool        latch      = false;           // Software latch until clearLatch()
  };

  // --- Constructors ---
  /// Default: auto-load from CFG.estop
  EStop() { loadFromCFG(cfg_); }

  /// Explicit: bypass CFG with custom config
  explicit EStop(const Config& cfg) : cfg_(cfg) {}

  // --- Setup ---
  /// Configure pin and establish baseline state (stable==raw).
  void setup() {
    switch (cfg_.pull) {
      case Config::Pull::None:     pinMode(cfg_.pin, INPUT); break;
      case Config::Pull::Pullup:   pinMode(cfg_.pin, INPUT_PULLUP); break;
      case Config::Pull::Pulldown: pinMode(cfg_.pin, INPUT_PULLDOWN); break;
    }

    const bool raw = rawActiveInstant();
    stable_             = raw;                 // No filtering: stable == raw
    lastStableChangeMs_ = millis();

    if (cfg_.latch && stable_) latched_ = true;
  }

  // --- Runtime ---
  /// Call frequently to sample input, detect edges, and maintain latch.
  void tick() {
    const bool raw = rawActiveInstant();

    // Edge detection on polarity-adjusted raw value
    if (raw != stable_) {
      stable_             = raw;
      lastStableChangeMs_ = millis();

      // Latch sets when becoming ACTIVE; clear is explicit via clearLatch()
      if (cfg_.latch && stable_) latched_ = true;

      if (stable_) edgeRising_ = true; else edgeFalling_ = true;

      if (onChange_) onChange_(stable_);
    }

#ifdef ESTOP_DEBUG
    Serial.print(F("{\"estop_dbg\":{"));
    Serial.print(F("\"raw\":"));       Serial.print(raw ? "1" : "0");
    Serial.print(F(",\"stable\":"));   Serial.print(stable_ ? "1" : "0");
    Serial.print(F(",\"latched\":"));  Serial.print(latched_ ? "1" : "0");
    Serial.print(F(",\"ms\":"));       Serial.print(millis());
    Serial.println(F("}}"));
#endif
  }

  // --- Query API ---
  /// True if E-Stop is ACTIVE (raw/stable OR latched).
  bool isActive()     const { return stable_ || latched_; }

  /// Debounced/stable concept is equal to raw here (no filters).
  bool stableActive() const { return stable_; }

  /// Raw (polarity applied) pin read (no debounce, no latch).
  bool rawActive()    const { return rawActiveInstant(); }

  /// One-shot edge flags (based on debounced==raw state)
  bool risingEdge()         { bool e = edgeRising_;  edgeRising_  = false; return e; }
  bool fallingEdge()        { bool e = edgeFalling_; edgeFalling_ = false; return e; }

  /// Latch controls
  bool isLatched()    const { return latched_; }
  void clearLatch()         { latched_ = false; }

  /// Timestamp of last stable/raw change
  uint32_t lastChangeMs() const { return lastStableChangeMs_; }

  // --- Hooks / Diagnostics ---
  /// Optional: set a callback invoked when the debounced/raw state flips.
  /// Signature: void onChange(bool activeNow)
  void setOnChange(void (*cb)(bool)) { onChange_ = cb; }

  /// Emit compact JSON status line to Serial (no ArduinoJson dependency).
  /// Example: {"type":"estop_status","active":true,"stable":true,"latched":true,"raw":true,"ms":12345}
  void emitStatusJson() const {
    Serial.print(F("{\"type\":\"estop_status\",\"active\":"));
    Serial.print(isActive() ? F("true") : F("false"));
    Serial.print(F(",\"stable\":"));
    Serial.print(stable_ ? F("true") : F("false"));
    Serial.print(F(",\"latched\":"));
    Serial.print(latched_ ? F("true") : F("false"));
    Serial.print(F(",\"raw\":"));
    Serial.print(rawActiveInstant() ? F("true") : F("false"));
    Serial.print(F(",\"ms\":"));
    Serial.print((unsigned long)millis());
    Serial.println(F("}"));
  }

  /// Access/modify config; reload from global CFG
  const Config& config() const { return cfg_; }
  void setConfig(const Config& cfg) { cfg_ = cfg; }
  void reloadFromCFG() { loadFromCFG(cfg_); }

private:
  // Map RobotConfig -> EStop::Config
  static void loadFromCFG(Config& out) {
    out.pin        = static_cast<uint8_t>(CFG.estop.pin);
    out.activeHigh = CFG.estop.active_high;
    switch (CFG.estop.pull_mode) {
      case 0: out.pull = Config::Pull::None; break;
      case 1: out.pull = Config::Pull::Pullup; break;
      case 2: out.pull = Config::Pull::Pulldown; break;
      default: out.pull = Config::Pull::Pullup; break;
    }
    out.latch      = CFG.estop.latch;
  }

  // Polarity-adjusted instantaneous read
  bool rawActiveInstant() const {
    const bool high = (digitalRead(cfg_.pin) == HIGH);
    return cfg_.activeHigh ? high : !high;
  }

  // --- Internals ---
  Config   cfg_{};

  bool     stable_             = false;  // == rawActiveInstant()
  bool     latched_            = false;

  uint32_t lastStableChangeMs_ = 0;

  bool     edgeRising_         = false;
  bool     edgeFalling_        = false;

  void   (*onChange_)(bool)    = nullptr;
};

#endif // ESTOP_HPP