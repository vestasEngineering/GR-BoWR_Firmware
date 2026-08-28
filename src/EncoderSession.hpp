#pragma once
#include <Arduino.h>
#include <ArduinoJson.h>
#include "ContactorMonitor.hpp"
#include "Motors.hpp"

class EncoderSession {
public:
  enum class State : uint8_t {
    STARTUP_RECOVERY,
    POWER_LOST,
    WAITING_FOR_RESTORE,
    RESTORING,
    VALID,
    RESTORE_FAILED
  };

  EncoderSession(ContactorMonitor& contactor, Motors& motors, Stream& io)
      : contactor_(contactor), motors_(motors), io_(io) {}

  void setup() {
    sessionId_ = 1;
    valid_ = false;
    restoreRequired_ = true;
    state_ = contactor_.powerPresent() ? State::WAITING_FOR_RESTORE : State::POWER_LOST;
    motors_.setMotionInhibited(true);
    emitState("startup");
  }

  void tick(bool& ultrasonicEnabled) {
    contactor_.tick();
    if (contactor_.powerLostEdge()) {
      ++sessionId_;
      valid_ = false;
      restoreRequired_ = true;
      state_ = State::POWER_LOST;
      ultrasonicEnabled = false;
      motors_.setMotionInhibited(true);
      motors_.BRAKE_STOP();
      emitState("motor_controller_power_lost");
    }
    if (contactor_.powerRestoredEdge()) {
      valid_ = false;
      restoreRequired_ = true;
      state_ = State::WAITING_FOR_RESTORE;
      ultrasonicEnabled = false;
      motors_.setMotionInhibited(true);
      emitState("motor_controller_power_restored");
    }
  }

  bool beginTransaction(uint32_t expectedSession) {
    if (!contactor_.powerPresent() || expectedSession != sessionId_) return false;
    state_ = State::RESTORING;
    valid_ = false;
    restoreRequired_ = true;
    motors_.setMotionInhibited(true);
    emitState("encoder_restoration_started");
    return true;
  }

  void configurationChanged(const char* reason, const char* transactionId = nullptr) {
    ++sessionId_;
    valid_ = false;
    restoreRequired_ = true;
    state_ = contactor_.powerPresent() ? State::WAITING_FOR_RESTORE : State::POWER_LOST;
    motors_.setMotionInhibited(true);
    motors_.BRAKE_STOP();
    emitState(reason, transactionId);
  }

  void transactionSucceeded(const char* source, const char* transactionId) {
    valid_ = true;
    restoreRequired_ = false;
    state_ = State::VALID;
    motors_.setMotionInhibited(false);
    emitState(source, transactionId);
  }

  void transactionFailed(const char* reason, const char* transactionId) {
    valid_ = false;
    restoreRequired_ = true;
    state_ = State::RESTORE_FAILED;
    motors_.setMotionInhibited(true);
    emitState(reason, transactionId);
  }

  bool valid() const { return valid_ && contactor_.powerPresent(); }
  bool restoreRequired() const { return restoreRequired_; }
  bool motorPowerPresent() const { return contactor_.powerPresent(); }
  uint32_t sessionId() const { return sessionId_; }

  const char* stateString() const {
    switch (state_) {
      case State::STARTUP_RECOVERY: return "startup_recovery";
      case State::POWER_LOST: return "power_lost";
      case State::WAITING_FOR_RESTORE: return "waiting_for_restore";
      case State::RESTORING: return "restoring";
      case State::VALID: return "valid";
      case State::RESTORE_FAILED: return "restore_failed";
    }
    return "unknown";
  }

  void emitState(const char* reason, const char* transactionId = nullptr) {
    StaticJsonDocument<320> doc;
    doc["type"] = "encoder_session";
    doc["state"] = stateString();
    doc["valid"] = valid();
    doc["restore_required"] = restoreRequired_;
    doc["motor_power_present"] = contactor_.powerPresent();
    doc["encoder_session_id"] = sessionId_;
    doc["reason"] = reason;
    doc["ts_ms"] = millis();
    if (transactionId && transactionId[0]) doc["transaction_id"] = transactionId;
    serializeJson(doc, io_);
    io_.println();
  }

private:
  ContactorMonitor& contactor_;
  Motors& motors_;
  Stream& io_;
  State state_ = State::STARTUP_RECOVERY;
  uint32_t sessionId_ = 1;
  bool valid_ = false;
  bool restoreRequired_ = true;
};
