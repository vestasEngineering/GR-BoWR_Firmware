#include "AndonManager.hpp"
#include "AndonLight.hpp"
#include "MySerial.hpp"
#include "MotorCP.hpp"
#include "Actuator.hpp"
#include "JogControl.hpp"
#include "BatteryMonitor.hpp"
#include "Ultrasonic.hpp"
#include "Config.hpp"
#include "EStop.hpp"
#include "ContactorMonitor.hpp"

constexpr AndonManager::Config AndonManager::kDefaultCfg;
extern bool ultrasonicEnabled;

const char* AndonManager::faultToKey(FaultCode f) {
  switch (f) {
    case FaultCode::EStopActive:           return "estop_active";
    case FaultCode::ContactorPowerLost:    return "motor_contactor_power_lost";
    case FaultCode::BatteryCritical:       return "battery_critical";
    case FaultCode::UltrasonicPersistent:  return "ultrasonic_bad";
    case FaultCode::ActuatorFault:         return "actuator_fault";
    case FaultCode::Motor0Fault:           return "motor0_fault";
    case FaultCode::Motor1Fault:           return "motor1_fault";
    default:                               return "unknown_fault";
  }
}

const char* AndonManager::faultToModuleId(FaultCode f) {
  switch (f) {
    case FaultCode::EStopActive:            return "estop";
    case FaultCode::ContactorPowerLost:     return "estop";
    case FaultCode::BatteryCritical:        return "battery";
    case FaultCode::UltrasonicPersistent:   return "ultrasonic";
    case FaultCode::ActuatorFault:          return "actuator_1";
    case FaultCode::Motor0Fault:            return "motor_1";
    case FaultCode::Motor1Fault:            return "motor_2";
    default:                                return "unknown";
  }
}

AndonManager::AndonManager(AndonLight& light,
                           MySerial& link,
                           MotorCP& motors,
                           ActuatorControl& actuator,
                           JogControl& jog,
                           BatteryMonitor& battery,
                           Ultrasonic& ultrasonic,
                           EStop& estop,
                           ContactorMonitor& contactor,
                           bool& hmiConnected,
                           Config cfg)
: light_(light), link_(link), motors_(motors), actuator_(actuator),
  jog_(jog), battery_(battery), ultrasonic_(ultrasonic),
  cfg_(cfg), estop_(estop), contactor_(contactor),
  hmi_connected_(hmiConnected) {}

void AndonManager::setup() {
  last_tick_ms_ = millis();
  last_change_ms_ = last_tick_ms_;
  has_override_ = false;
  current_ = AndonLight::YELLOW;
  last_raw_ = AndonLight::YELLOW;
  startup_grace_until_ = last_tick_ms_ + CFG.andonMgr.comms_grace_ms;
  ut_verify_done_ = false;
  ut_verify_passed_ = false;
  ut_active_since_ms_ = 0;
  ut_good_consec_ = 0;
}

void AndonManager::setOverride(AndonLight::States s) { override_state_ = s; has_override_ = true; }
void AndonManager::clearOverride() { has_override_ = false; }
AndonLight::States AndonManager::currentState() const { return current_; }

void AndonManager::tick() {
  const auto now = millis();
  if (now - last_tick_ms_ < CFG.andonMgr.tick_period_ms) return;
  last_tick_ms_ = now;

  AndonLight::States raw;
  if (has_override_) {
    if (isEStop() || hasFault() || isCommsLost()) raw = compute(now);
    else raw = override_state_;
  } else {
    raw = compute(now);
  }

  if (raw != last_raw_) { last_raw_ = raw; last_change_ms_ = now; }
  if (now - last_change_ms_ < CFG.andonMgr.stable_ms) return;

  if (raw != current_) {
    current_ = raw;
    light_.setState(current_);

    std::vector<FaultCode> faults;
    collectFaults(faults);

    link_.sendAndonDiagnostics(
        current_, now, has_override_, override_state_, isEStop(),
        !faults.empty(), isCommsLost(), isBlockedOrStarved(),
        isPausedOrJog(), isBatteryLow(), isRunning(), faults);
  }
}

AndonLight::States AndonManager::compute(uint32_t /*now*/) {
  if (isEStop())                return AndonLight::RED;
  if (hasFault())               return AndonLight::BLINK_RED;
  if (isJogging())              return AndonLight::BLINK_BLUE;
  if (isActuatorDisconnected()) return AndonLight::BLUE;
  if (isCommsLost())            return AndonLight::YELLOW;
  if (isHmiDisconnected())      return AndonLight::YELLOW;
  if (isBlockedOrStarved())     return AndonLight::BLINK_YELLOW;
  if (isBatteryLow())           return AndonLight::BLINK_YELLOW;
  if (isRunning())              return AndonLight::BLINK_GREEN;
  return AndonLight::GREEN;
}

bool AndonManager::isEStop() const { return estop_.isActive(); }
bool AndonManager::isContactorPowerLost() const { return !contactor_.powerPresent(); }

bool AndonManager::motionStopRequired() const {
  return isEStop() || isContactorPowerLost() || hasFault();
}

void AndonManager::enforceMotionSafety() {
  motors_.setMotionInhibited(motionStopRequired());
}

bool AndonManager::hasFault() const {
  if (isContactorPowerLost()) return true;
  if (battery_.isCritical(CFG.battery.critical_pct)) return true;
  return false;
}

void AndonManager::collectFaults(std::vector<AndonManager::FaultCode>& out) const {
  out.clear();
  if (isEStop()) out.push_back(FaultCode::EStopActive);
  if (isContactorPowerLost()) out.push_back(FaultCode::ContactorPowerLost);
  if (battery_.isCritical(CFG.battery.critical_pct)) out.push_back(FaultCode::BatteryCritical);
}

bool AndonManager::isCommsLost() const { return !link_.commsAlive(); }
bool AndonManager::isHmiDisconnected() const { return !hmi_connected_; }
bool AndonManager::isBlockedOrStarved() const { return false; }
bool AndonManager::isJogging() const { return jog_.isActive(); }
bool AndonManager::isActuatorDisconnected() const { return actuator_.hasPCBFault(); }
bool AndonManager::isPausedOrJog() const { return isJogging() || isActuatorDisconnected(); }
bool AndonManager::isBatteryLow() const { return battery_.isLow(20.0f); }

bool AndonManager::isRunning() const {
  const uint32_t kDwellMs = 300;
  static bool latched = false;
  static uint32_t runningUntil = 0;
  const uint32_t now = millis();
  const bool processActive = ultrasonicEnabled && !actuator_.hasPCBFault();

  if (processActive) {
    latched = true;
    runningUntil = now + kDwellMs;
  } else if (latched && static_cast<int32_t>(now - runningUntil) > 0) {
    latched = false;
  }
  return latched;
}
