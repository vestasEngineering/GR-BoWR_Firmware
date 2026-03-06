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

  bool andon_ok = false;

  bool estop_active = false;

  bool can_ok = false;
  uint8_t can_axes_mask = 0;   // bit i = axis i (0..3) responded

  bool motors_ok = false;

  
  bool clamp_ok = false;
  bool clamp_state = false;


  bool ultrasonic_ok = false;
  int   ultrasonic_adc = -1;
  float ultrasonic_voltage = NAN;
  float ultrasonic_distance = NAN;

  bool ultrasonic_servo_ok = false;

  bool actuator_ok = false;

  bool battery_ok = false;
  float battery_voltage = NAN;
  float battery_pct = NAN;

  uint32_t timeout_ms_used = 0;   // CAN probe timeout used
};

/// Run all health checks. `battery` may be nullptr if not used.
/// Returns a filled Report.
Report run(AndonLight& light,
           Motors& motors,
           ActuatorControl& actuator,
           Ultrasonic& ultrasonic,
           UltrasonicServo& us_servo,
           BatteryMonitor* battery /* optional */,
           EStop& estop,
           uint32_t can_timeout_ms = 500);

/// Serialize and send the report as a single JSON line over Serial.
void sendReport(const Report& r);

}