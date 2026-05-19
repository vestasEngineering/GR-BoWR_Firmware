#include "AndonManager.hpp"
#include "AndonLight.hpp"
#include "MySerial.hpp"
#include "Motors.hpp"
#include "Actuator.hpp"
#include "JogControl.hpp"
#include "BatteryMonitor.hpp"
#include "Ultrasonic.hpp"
#include "UltrasonicServo.hpp"
#include "Config.hpp"
#include "EStop.hpp"
#include "ClampSensor.hpp"

static constexpr uint16_t kUltrasonicBadStreakThreshold = 20;
static constexpr float    kUTServoTooCloseThreshold     = 60.0f;
constexpr AndonManager::Config AndonManager::kDefaultCfg;
extern bool ultrasonicEnabled;

const char* AndonManager::faultToKey(FaultCode f) {
  switch (f) {
    case FaultCode::BatteryCritical:     return "battery_critical";
    case FaultCode::UltrasonicPersistent:return "ultrasonic_bad";
    case FaultCode::UltrasonicServoFault:  return "ultrasonic_servo_fault";
    case FaultCode::ActuatorFault:       return "actuator_fault";
    case FaultCode::ClampUnclamped:        return "clamp_unclamped";
    case FaultCode::Motor0Fault:         return "motor0_fault";
    case FaultCode::Motor1Fault:         return "motor1_fault";
    case FaultCode::Motor2Fault:         return "motor2_fault";
    case FaultCode::Motor3Fault:         return "motor3_fault";
    default:                             return "unknown_fault";
  }
}

const char* AndonManager::faultToModuleId(FaultCode f) {
  // Align with websocket_server.MODULES ids:
  //  "battery", "ultrasonic", "actuator_[1..4]?" or the generic "actuator"
  //  "motor_1".."motor_4"
  switch (f) {
    case FaultCode::BatteryCritical:      return "battery";
    case FaultCode::UltrasonicPersistent: return "ultrasonic";
    case FaultCode::UltrasonicServoFault:  return "ultrasonic_servo";
    case FaultCode::ActuatorFault:        return "actuator_1"; // or "actuator" if you prefer generic
    case FaultCode::ClampUnclamped:       return "clamp"; 
    case FaultCode::Motor0Fault:          return "motor_1";
    case FaultCode::Motor1Fault:          return "motor_2";
    case FaultCode::Motor2Fault:          return "motor_3";
    case FaultCode::Motor3Fault:          return "motor_4";
    default:                              return "unknown";
  }
}

AndonManager::AndonManager(AndonLight& light,
                           MySerial& link,
                           Motors& motors,
                           ActuatorControl& actuator,
                           JogControl& jog,
                           BatteryMonitor& battery,
                           Ultrasonic& ultrasonic,
                           UltrasonicServo& ut_servo,
                           EStop& estop,
                           ClampSensor& clamp,
                           Config cfg)
: light_(light), link_(link), motors_(motors), actuator_(actuator),
  jog_(jog), battery_(battery), ultrasonic_(ultrasonic), ut_servo_(ut_servo), cfg_(cfg), estop_(estop), clamp_(clamp) {}


void AndonManager::setup() {
  last_tick_ms_   = millis();
  last_change_ms_ = last_tick_ms_;
  has_override_   = false;
  current_        = AndonLight::YELLOW;
  last_raw_       = AndonLight::YELLOW;
  startup_grace_until_ = last_tick_ms_ + CFG.andonMgr.comms_grace_ms;

  ut_verify_done_ = false;
  ut_verify_passed_ = false;
  ut_active_since_ms_ = 0;
  ut_good_consec_  = 0;
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
    if (isEStop() || hasFault() || isCommsLost()) {
      raw = compute(now);
    } else {
      raw = override_state_;
    }
  } else {
    raw = compute(now);
  }

  if (raw != last_raw_) { last_raw_ = raw; last_change_ms_ = now; }
  if (now - last_change_ms_ < CFG.andonMgr.stable_ms) return;  // debounce

  if (raw != current_) {
    current_ = raw;
    light_.setState(current_);

    ::std::vector<FaultCode> faults;
    collectFaults(faults);

    // Emit detailed diagnostics over the serial server
    link_.sendAndonDiagnostics(
        current_,
        now,
        has_override_,
        override_state_,
        isEStop(),
        !faults.empty(),               // hasFault
        isCommsLost(),
        isBlockedOrStarved(),
        isPausedOrJog(),
        isBatteryLow(),
        isRunning(),
        faults                         // NEW payload
    );
  }
}

// ----- Priority resolver -----
AndonLight::States AndonManager::compute(uint32_t /*now*/) {
  if (isEStop())               return AndonLight::RED;  
  if (hasFault())              return AndonLight::BLINK_RED;
  if (isCommsLost())           return AndonLight::YELLOW;
  if (isBlockedOrStarved())    return AndonLight::BLINK_YELLOW;
  if (isPausedOrJog())         return AndonLight::BLUE;
  if (isBatteryLow())          return AndonLight::BLINK_YELLOW;
  if (isRunning())             return AndonLight::BLINK_GREEN;
  return AndonLight::GREEN;
}


bool AndonManager::isEStop() const {
  return estop_.isActive();
}


bool AndonManager::hasFault() const {
  ::std::vector<FaultCode> tmp;
  collectFaults(tmp);
  return !tmp.empty();
}

void AndonManager::collectFaults(std::vector<AndonManager::FaultCode>& out) const {
  out.clear();

  //if (!clamp_.isClamped()) {
  //  out.push_back(FaultCode::ClampUnclamped);
  //}

  /*
  
  // --- Ultrasonic Servo jam check: verify only during first few seconds of ACTIVE ---
  {
    // Tunables
    const uint32_t kVerifyWindowMs   = CFG.andonMgr.ut_verify_window_ms;
    const float    kMinOkDistance_mm = CFG.andonMgr.ut_ok_min_distance;
    const uint16_t kConsecGoodNeeded = CFG.andonMgr.ut_good_consec;

    const uint32_t now = millis();
    const bool isActive = (ut_servo_.getState() == UltrasonicServo::ACTIVE);


    // Handle state transitions into/out of ACTIVE
    if (isActive) {
      if (ut_active_since_ms_ == 0) {
        // Transitioned to ACTIVE -> start verification window
        ut_active_since_ms_ = now;
        ut_verify_done_     = false;
        ut_verify_passed_   = false;
        ut_good_consec_     = 0;
      }
    } else {
      // Not ACTIVE -> reset for next cycle
      ut_active_since_ms_ = 0;
      ut_verify_done_     = false;
      ut_verify_passed_   = false;
      ut_good_consec_     = 0;
    }

    // Only evaluate during ACTIVE
    if (isActive) {
      if (!ut_verify_done_) {
        const uint32_t elapsed = now - ut_active_since_ms_;
        // Choose one canonical distance source and validate; assume mm
        float d = ultrasonic_.measuredDistance;

        auto isValid = [](float x) { return std::isfinite(x) && x > 1.0f && x < 10000.0f; }; // 1..10000 mm plausible

        if (isValid(d)) {
          // Good sample if not "too close"
          if (d >= kMinOkDistance_mm) {
            if (ut_good_consec_ < 0xFFFF) ++ut_good_consec_;
            if (ut_good_consec_ >= kConsecGoodNeeded) {
              ut_verify_done_   = true;
              ut_verify_passed_ = true;
            }
          } else {
            // Too close -> break the streak (but keep trying until window ends)
            ut_good_consec_ = 0;
          }
        }
        // else invalid sample; ignore and keep trying

        // If window ended and we never passed, flag a jam fault
        if (!ut_verify_done_ && elapsed >= kVerifyWindowMs) {
          ut_verify_done_   = true;
          ut_verify_passed_ = false;
        }
      }

      // After the window:
      // - If we passed -> no fault for the remainder of this ACTIVE cycle.
      // - If we failed -> raise fault once per tick (or you can latch it elsewhere).
      if (ut_verify_done_ && !ut_verify_passed_) {
        out.push_back(FaultCode::UltrasonicServoFault);
      }
    }
  }


  // Ultrasonic persistent failure => fault
  if (ultrasonic_.badReadStreak >= CFG.andonMgr.ultrasonic_bad_streak_threshold) {
    out.push_back(FaultCode::UltrasonicPersistent);
  }

  */
  
  if (battery_.isCritical(CFG.battery.critical_pct)) {
    out.push_back(FaultCode::BatteryCritical);
  }

  // Actuator PCB/jam faults (enable when actuator_.hasFault() is implemented)
  // if (actuator_.hasFault()) {
  //   out.push_back(FaultCode::ActuatorFault);
  // }

  // Optional: per-motor driver/encoder faults if/when you add them:
  // if (motors_.driverFault[0]) out.push_back(FaultCode::Motor0Fault);
  // if (motors_.driverFault[1]) out.push_back(FaultCode::Motor1Fault);
  // if (motors_.driverFault[2]) out.push_back(FaultCode::Motor2Fault);
  // if (motors_.driverFault[3]) out.push_back(FaultCode::Motor3Fault);
}


bool AndonManager::isCommsLost() const {
  // Prefer a helper on MySerial to avoid using its internals here
  //if (millis() < startup_grace_until_) return false;
  return !link_.commsAlive();
}

bool AndonManager::isBlockedOrStarved() const {
  /*
  // Tunables
  const float    kCmdThresh_ms       = 0.0040f;  // m/s; treat commands below as idle/noise
  const float    kPosDeltaEps_rev    = 0.0015f;  // rev; min delta that counts as movement
  const uint32_t kCheckPeriodMs      = 100;      // sampling interval
  const uint32_t kStallTimeoutMs     = 1500;     // time without motion while commanding => blocked

  static bool     initialized       = false;
  static uint32_t lastCheckMs       = 0;
  static float    lastPos[4]        = {0, 0, 0, 0};
  static uint32_t noMotionSinceMs   = 0;

  const uint32_t now = millis();

  if (!initialized) {
    for (int i = 0; i < 4; ++i) {
      lastPos[i] = motors_.positions[i];
    }
    lastCheckMs     = now;
    noMotionSinceMs = now;
    initialized     = true;
  }

  // Is there a meaningful motion command?
  bool commanding = false;
  for (int i = 0; i < 4; ++i) {
    if (fabsf(motors_.speeds[i]) > kCmdThresh_ms) {
      commanding = true;
      break;
    }
  }

  // Periodically check encoder movement
  if (now - lastCheckMs >= kCheckPeriodMs) {
    bool anyMove = false;
    for (int i = 0; i < 4; ++i) {
      float d = fabsf(motors_.positions[i] - lastPos[i]);
      if (d > kPosDeltaEps_rev) {
        anyMove = true;
      }
      lastPos[i] = motors_.positions[i];
    }
    if (anyMove) {
      noMotionSinceMs = now;  // reset if anything moved
    }
    lastCheckMs = now;
  }

  // If we’re commanding but nothing moved for too long → blocked/starved
  if (commanding) {
    if (now - noMotionSinceMs >= kStallTimeoutMs) {
      return true;
    }
  } else {
    // Idle commands → reset the baseline so we don't trigger on inactivity
    noMotionSinceMs = now;
  }
*/

  return false;
  
}

bool AndonManager::isPausedOrJog() const {
  // Consider the system "paused/jog" (BLUE) while jog is active.
  // If you later add a "paused" state in motors/serial, OR it in here:
  //   return jog_.isActive() || motors_.isPaused();
  //return jog_.isActive();
  return false;
}

bool AndonManager::isBatteryLow() const {  
  // Low battery warning threshold (20%)
  // Note: critical battery (<= 10%) is handled in hasFault() above.
  return battery_.isLow(20.0f);
}

bool AndonManager::isRunning() const {

  const uint32_t kDwellMs = 300;  // small smoothing buffer

  static bool initialized      = false;
  static bool latched          = false;
  static uint32_t runningUntil = 0;

  const uint32_t now = millis();

  // Primary signal: process state
  bool processActive = ultrasonicEnabled;

  // Optional: include actuator / clamp states if they matter
  // bool processActive = ultrasonicEnabled || actuator.isBusy() || clamp.isActive();

  // Latch logic to prevent flicker during transitions
  if (processActive) {
    latched      = true;
    runningUntil = now + kDwellMs;
  }
  else if (latched && now > runningUntil) {
    latched = false;
  }

  return latched;
}
