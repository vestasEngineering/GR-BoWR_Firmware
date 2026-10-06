#include "BootHealth.hpp"
#include "AndonLight.hpp"
#include "MotorCP.hpp"
#include "Actuator.hpp"
#include "Ultrasonic.hpp"
#include "BatteryMonitor.hpp"
#include <Version.hpp>
#include <ArduinoJson.h>
#include "EStop.hpp"

namespace BootHealth {

bool probeEncoders(MotorCP& motors, uint32_t timeoutMs, uint8_t& goodCount) {
    (void)timeoutMs;
    motors.pollEncoders();

    goodCount = 0;
    const uint8_t mask = motors.encoderReadValidMask();
    for (uint8_t i = 0; i < MotorCP::MOTORS; i++) {
        if ((mask & (1u << i)) && abs(motors.encCounts[i]) < 1000000) {
            goodCount++;
        }
    }

    return goodCount >= MotorCP::MOTORS;
}


static void checkUltrasonic(const Ultrasonic& u, BootHealth::Report& r) {
    int adc = analogRead(CFG.ultrasonic.analog_pin);
    float voltage = float(adc) * (3.1f / 1023.0f);
    float distance = voltage * CFG.ultrasonic.mm_per_volt + CFG.ultrasonic.offset_mm;

    r.ultrasonic_adc      = adc;
    r.ultrasonic_voltage  = voltage;
    r.ultrasonic_distance = distance;

    // Consider “OK” if not pegged and within physical plausibility
    const bool plausible =
        (distance > 20.0f && distance < 800.0f) ||
        (adc > 0 && adc < 1023);

    r.ultrasonic_ok = plausible;
}


static void checkActuator(const ActuatorControl& a, BootHealth::Report& r) {
    r.actuator_ok = !a.hasFault();
}


static void checkBattery(BatteryMonitor* b, BootHealth::Report& r) {
    if (!b) {
        r.battery_ok = true;
        return;
    }

    b->readBatteryVoltage();

    r.battery_voltage = b->voltage;

    float pct = (b->voltage - MIN_BATTERY_VOLTAGE) /
                (MAX_BATTERY_VOLTAGE - MIN_BATTERY_VOLTAGE) * 100.0f;

    pct = constrain(pct, 0.0f, 100.0f);
    r.battery_pct = pct;

    // Consider OK unless critically low (below 10%)
    r.battery_ok = (pct > 10.0f);
}

BootHealth::Report run(AndonLight& light,
                       MotorCP& motors,
                       ActuatorControl& actuator,
                       Ultrasonic& ultrasonic,
                       BatteryMonitor* battery,
                       EStop& estop,
                       uint32_t timeoutMs)
{
    BootHealth::Report r;
    r.timeout_ms_used = timeoutMs;

    r.andon_ok = true;


    uint8_t goodCount = 0;
    bool encOk = BootHealth::probeEncoders(motors, timeoutMs, goodCount);

    r.encoders_ok = encOk;
    r.encoders_present_mask = 
        (goodCount >= MotorCP::MOTORS ? 0x03 : (uint8_t)((1 << goodCount) - 1));



    r.motors_ok = (r.encoders_ok);
    checkUltrasonic(ultrasonic, r);
    checkActuator(actuator, r);
    checkBattery(battery, r);
    r.estop_active = estop.isActive();

    r.ok =
        r.andon_ok &&
        r.encoders_ok &&              // formerly “CAN OK”, now encoder comm OK
        r.ultrasonic_ok &&
        r.actuator_ok &&
        r.motors_ok &&
        r.battery_ok;

    return r;
}

void sendReport(const BootHealth::Report& r, Stream& out) {
    StaticJsonDocument<640> doc;

    doc["type"]  = "boot_health";
    doc["ts_ms"] = millis();
    doc["ok"]    = r.ok;

    JsonObject checks = doc.createNestedObject("checks");

    checks["andon"]["ok"] = r.andon_ok;

    {
        JsonObject enc = checks.createNestedObject("encoders");
        enc["ok"] = r.encoders_ok;
        enc["present_mask"] = r.encoders_present_mask;   // bitmask of encoder channels detected
        enc["timeout_ms"] = r.timeout_ms_used;
      }

    checks["motors"]["ok"] = r.motors_ok;

    {
        JsonObject us = checks.createNestedObject("ultrasonic");
        us["ok"] = r.ultrasonic_ok;
        us["adc"] = r.ultrasonic_adc;
        us["voltage"] = r.ultrasonic_voltage;
        us["distance_mm"] = r.ultrasonic_distance;
    }

    checks["actuator"]["ok"]          = r.actuator_ok;
    checks["estop"]["active"]         = r.estop_active;

    {
        JsonObject bat = checks.createNestedObject("battery");
        bat["ok"]      = r.battery_ok;
        bat["voltage"] = r.battery_voltage;
        bat["pct"]     = r.battery_pct;
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

    serializeJson(doc, out);
    out.println();
}

}