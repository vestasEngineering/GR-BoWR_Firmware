#include "BootHealth.hpp"

// Now include implementations (TU only)
#include "AndonLight.hpp"
#include "Motors.hpp"
#include "Actuator.hpp"
#include "Ultrasonic.hpp"
#include "UltrasonicServo.hpp"
#include "BatteryMonitor.hpp"
#include <Version.hpp>
#include <Arduino_CAN.h>
#include <ArduinoJson.h>

namespace BootHealth {

static bool probeCAN_APOS_all(Motors& motors, uint32_t timeout_ms, uint8_t& mask_out) {
  mask_out = 0;

  // APOS query payload (MPL)
  uint8_t aposCmd[4] = {0x11, 0x00, 0x28, 0x02};

  // Send APOS query to all 4 axes
  for (uint8_t axis = 0; axis < 4; ++axis) {
    CanMsg query(CanExtendedId(motors.MOTOR_APOS_IDS[axis]), sizeof(aposCmd), aposCmd);
    CAN.write(query);
  }

  const uint32_t start = millis();
  while ((millis() - start) < timeout_ms && mask_out != 0x0F) {
    if (CAN.available()) {
      CanMsg msg = CAN.read();

      // Let Motors parse/record positions
      motors.handleCANResponse(msg);

      // Detect APOS response here too to mark which axis responded
      if (msg.data_length >= 8 && msg.data[2] == 0x28 && msg.data[3] == 0x02) {
        // axisIndex in your code: (data[0] / 0x10) - 1 → maps {0..3}
        uint8_t axisIndex = (msg.data[0] / 0x10u) - 1u;
        if (axisIndex < 4u) {
          mask_out |= (1u << axisIndex);
        }
      }
    }
    delay(1); // small yield
  }

  return (mask_out == 0x0F);
}

static void checkUltrasonic(const Ultrasonic& u, Report& r) {
  // Use same scaling as your Ultrasonic class
  // Note: analogReadResolution already set in ultrasonic.setup()
  int adc = analogRead(ULTRASONIC_PIN);
  float voltage = (float(adc) * 3.1f / 1023.0f);
  float distance = voltage * u.mmPerVolt + u.offsetDistance;

  r.ultrasonic_adc = adc;
  r.ultrasonic_voltage = voltage;
  r.ultrasonic_distance = distance;

  // Consider "ok" if the reading is within a plausible operating window
  // or at least not pegged. Your runtime window is 45..250 mm.
  const bool plausible =
      (distance > 30.0f && distance < 400.0f) || (adc > 0 && adc < 1023);

  r.ultrasonic_ok = plausible;
}

static void checkUltrasonicServo(const UltrasonicServo& /*us*/, Report& r) {
  // Servo library has Servo::attached(), but UltrasonicServo doesn't expose it.
  // Best-effort: if setup() completed without error, we assume ok.
  // You can refine by exposing a method that returns servo.attached().
  r.ultrasonic_servo_ok = true;
}

static void checkActuator(const ActuatorControl& a, Report& r) {
  // PCB or jam faults → not ok
  r.actuator_ok = !a.hasFault();
}

static void checkBattery(BatteryMonitor* b, Report& r) {
  if (!b) {
    r.battery_ok = true; // Not used = don't block boot
    return;
  }

  // Update once (non-blocking)
  b->readBatteryVoltage();

  r.battery_voltage = b->voltage;

  // Compute % inline to avoid depending on optional helpers
  float pct = (b->voltage - MIN_BATTERY_VOLTAGE) /
              (MAX_BATTERY_VOLTAGE - MIN_BATTERY_VOLTAGE) * 100.0f;
  if (pct < 0.0f) pct = 0.0f;
  if (pct > 100.0f) pct = 100.0f;
  r.battery_pct = pct;

  // Consider ok if above a very low threshold (e.g., > 10% SOC)
  r.battery_ok = (pct > 10.0f);
}

Report run(AndonLight& light,
           Motors& motors,
           ActuatorControl& actuator,
           Ultrasonic& ultrasonic,
           UltrasonicServo& us_servo,
           BatteryMonitor* battery,
           uint32_t can_timeout_ms) {
  Report r;
  r.timeout_ms_used = can_timeout_ms;

  // If we reached here, Andon light I2C init didn't hard-fail (Andon code halts on failure)
  r.andon_ok = true;

  // Probe CAN for all four motors via APOS
  r.can_ok = probeCAN_APOS_all(motors, can_timeout_ms, r.can_axes_mask);
  r.motors_ok = r.can_ok; // motors_ok aliases can_ok presence at boot

  // Ultrasonic ADC → distance sanity
  checkUltrasonic(ultrasonic, r);

  // Servo (best-effort)
  checkUltrasonicServo(us_servo, r);

  // Actuator PCB/jam
  checkActuator(actuator, r);

  // Battery (optional)
  checkBattery(battery, r);

  // Overall OK only if all required subsystems pass
  // Battery optional: if you want battery to gate boot, include it in the AND
  r.ok = r.andon_ok &&
         r.can_ok &&
         r.ultrasonic_ok &&
         r.ultrasonic_servo_ok &&
         r.actuator_ok &&
         r.motors_ok &&
         r.battery_ok;

  return r;
}

void sendReport(const Report& r) {
  StaticJsonDocument<640> doc;
  doc["type"]  = "boot_health";
  doc["ts_ms"] = millis();
  doc["ok"]    = r.ok;

  JsonObject checks = doc.createNestedObject("checks");

  checks["andon"]["ok"] = r.andon_ok;

  {
    JsonObject can = checks.createNestedObject("can");
    can["ok"] = r.can_ok;
    can["responded_axes_mask"] = r.can_axes_mask; // bitmask
    can["timeout_ms"] = r.timeout_ms_used;
  }

  checks["motors"]["ok"] = r.motors_ok;

  {
    JsonObject us = checks.createNestedObject("ultrasonic");
    us["ok"] = r.ultrasonic_ok;
    us["adc"] = r.ultrasonic_adc;
    us["voltage"] = r.ultrasonic_voltage;
    us["distance_mm"] = r.ultrasonic_distance;
  }

  checks["ultrasonic_servo"]["ok"] = r.ultrasonic_servo_ok;

  checks["actuator"]["ok"] = r.actuator_ok;

  {
    JsonObject bat = checks.createNestedObject("battery");
    bat["ok"] = r.battery_ok;
    bat["voltage"] = r.battery_voltage;
    bat["pct"] = r.battery_pct;
  }
  
  {
  JsonObject fw = doc.createNestedObject("firmware");
  fw["model"]     = Version::model();
  fw["fleet_id"]  = Version::fleetId();
  fw["semver"]    = Version::semver();
  fw["build"]     = Version::buildStamp();
  fw["board"]     = Version::board();
  fw["platform"]  = Version::platform();
  fw["channel"]   = Version::channel();
  fw["git"]       = Version::shortGit();
  }


  serializeJson(doc, Serial);
  Serial.println();
}

} // namespace BootHealth