#pragma once
#include <Arduino.h>
#include <vector>
#include "AndonLight.hpp"

class MySerial;
class Motors;
class ActuatorControl;
class JogControl;
class BatteryMonitor;
class Ultrasonic;
class UltrasonicServo;
class EStop;
class ClampSensor;
class ContactorMonitor;

class AndonManager {
public:
  struct Config {
    uint16_t tick_period_ms;
    uint16_t stable_ms;
    uint16_t comms_grace_ms;
  };

  static constexpr Config kDefaultCfg{100, 200, 3000};

  enum class FaultCode : uint8_t {
    EStopActive,
    ContactorPowerLost,
    BatteryCritical,
    UltrasonicPersistent,
    UltrasonicServoFault,
    ActuatorFault,
    ClampUnclamped,
    Motor0Fault,
    Motor1Fault,
    Motor2Fault,
    Motor3Fault
  };

  static const char* faultToKey(FaultCode f);
  static const char* faultToModuleId(FaultCode f);

  AndonManager(AndonLight& light,
               MySerial& link,
               Motors& motors,
               ActuatorControl& actuator,
               JogControl& jog,
               BatteryMonitor& battery,
               Ultrasonic& ultrasonic,
               UltrasonicServo& ut_servo,
               EStop& estop,
               ClampSensor& clamp,
               ContactorMonitor& contactor,
               bool& hmiConnected,
               Config cfg = kDefaultCfg);

  void setup();
  void setOverride(AndonLight::States s);
  void clearOverride();
  void tick();
  void enforceMotionSafety();
  bool motionStopRequired() const;
  AndonLight::States currentState() const;

private:
  AndonLight::States compute(uint32_t now);
  bool isEStop() const;
  bool isContactorPowerLost() const;
  bool hasFault() const;
  bool isCommsLost() const;
  bool isHmiDisconnected() const;
  bool isBlockedOrStarved() const;
  bool isJogging() const;
  bool isActuatorDisconnected() const;
  bool isPausedOrJog() const;
  bool isBatteryLow() const;
  bool isRunning() const;

  mutable bool     ut_verify_done_{false};
  mutable bool     ut_verify_passed_{false};
  mutable uint32_t ut_active_since_ms_{0};
  mutable uint16_t ut_good_consec_{0};

  void collectFaults(std::vector<FaultCode>& out) const;

  AndonLight&        light_;
  MySerial&          link_;
  Motors&            motors_;
  ActuatorControl&   actuator_;
  JogControl&        jog_;
  BatteryMonitor&    battery_;
  Ultrasonic&        ultrasonic_;
  UltrasonicServo&   ut_servo_;
  Config             cfg_;
  EStop&             estop_;
  ClampSensor&       clamp_;
  ContactorMonitor&  contactor_;
  bool&               hmi_connected_;

  uint32_t           last_tick_ms_{0};
  uint32_t           last_change_ms_{0};
  uint32_t           startup_grace_until_{0};
  bool               has_override_{false};
  AndonLight::States override_state_{};
  AndonLight::States current_{};
  AndonLight::States last_raw_{};
};
