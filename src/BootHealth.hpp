#pragma once
#include <Arduino.h>

class AndonLight;
class Motors;
class ActuatorControl;
class Ultrasonic;
class UltrasonicServo;
class BatteryMonitor;
class EStop;

namespace BootHealth {

struct Report {
  bool ok = false;

  // Subsystem health flags
  bool andon_ok = false;
  bool estop_active = false;

  bool encoders_ok = false;
  uint8_t encoders_present_mask = 0;

  // Motor system health (encoder-driven)
  bool motors_ok = false;

  // Ultrasonic
  bool ultrasonic_ok = false;
  int   ultrasonic_adc = -1;
  float ultrasonic_voltage = NAN;
  float ultrasonic_distance = NAN;

  // Ultrasonic servo
  bool ultrasonic_servo_ok = false;

  // Actuator board/motion checks
  bool actuator_ok = false;

  // Battery system
  bool battery_ok = false;
  float battery_voltage = NAN;
  float battery_pct = NAN;

  // The probe timeout used for encoder liveness checks
  uint32_t timeout_ms_used = 0;
};


bool probeEncoders(Motors& motors, uint32_t timeoutMs, uint8_t& goodCount);


Report run(AndonLight& light,
           Motors& motors,
           ActuatorControl& actuator,
           Ultrasonic& ultrasonic,
           UltrasonicServo& us_servo,
           BatteryMonitor* battery /* optional */,
           EStop& estop,
           uint32_t timeout_ms = 500);


void sendReport(const Report& r, Stream& out);

} 