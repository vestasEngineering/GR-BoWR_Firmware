#ifndef MY_ULTRASONIC_CLASS
#define MY_ULTRASONIC_CLASS

#include <Arduino.h>
#include <ArduinoJson.h>
#include "PID.hpp"
#include "Motors.hpp"
#include "Config.hpp"

#define ULTRASONIC_PIN A2

class Ultrasonic {
public:
    volatile int delay = 0;
    float distance[3] = {0.0, 0.0, 0.0}; // [n, n-1, n-2]
    float voltage = 0.0;
    float measuredDistance = 0.0;
    float mmPerVolt = (300.0 - 30.0) / (2.0 - 0.0);
    float offsetDistance = 40.0;

    float currentSpeed = 0.0;
    float processSpeed = 0.0;

    float setPoint = 180.0;
    float tolerance = 2.0;
    float lowerLimit = setPoint - tolerance;
    float upperLimit = setPoint + tolerance;

    uint16_t badReadStreak = 0;

    Motors* motors;

    PID pid = PID(3.5, 0.3, 0.08, setPoint);

    Ultrasonic() {}

    void attachMotors(Motors& m) {
        motors = &m;
    }

    void setup() {
        analogReadResolution(10);
        pid.setOutputLimits(CFG.ultrasonic.pid_out_min_ms, CFG.ultrasonic.pid_out_max_ms);
        pid.setSampleTime(CFG.ultrasonic.pid_sample_time_s);
    }

    void updateHeight(float height) {
        pid.setSetpoint(height);
        lowerLimit = height - tolerance;
        upperLimit = height + tolerance;
    }

    bool isValidMeasurement(float d) {
        return (d > CFG.ultrasonic.valid_min_mm && d < CFG.ultrasonic.valid_max_mm);
    }

    void stateMachine() {
        if (!delay) {
            voltage = (float(analogRead(CFG.ultrasonic.analog_pin)) * CFG.battery.vref / CFG.battery.adc_max_counts);
            measuredDistance = voltage * CFG.ultrasonic.mm_per_volt + CFG.ultrasonic.offset_mm;

            const bool isBad40 = fabsf(measuredDistance - CFG.ultrasonic.bad40_center_mm) <= CFG.ultrasonic.bad40_tol_mm;

            if (!isValidMeasurement(measuredDistance)) {
                if (isBad40) { if (badReadStreak < 0xFFFF) badReadStreak++; }
                delay = 10; return;
            } else {
                badReadStreak = 0;
            }

            // control band
            lowerLimit = setPoint - CFG.ultrasonic.tolerance_mm;
            upperLimit = setPoint + CFG.ultrasonic.tolerance_mm;

            if (distance[0] < CFG.ultrasonic.safe_stop_mm) {
                processSpeed = 0.0f; currentSpeed = 0.0f;
            } else {
                float u = 0.0f;
                if (distance[0] > upperLimit || distance[0] < lowerLimit) {
                    u = pid.compute(distance[0]);
                }
                processSpeed = roundf(max(currentSpeed + u, 0.0f) * 10000.0f) / 10000.0f;
                currentSpeed = processSpeed;
            }

            // Apply motor speeds
            if (motors) {
                motors->speeds[0] = -processSpeed;
                motors->speeds[1] = -processSpeed;
                motors->speeds[2] =  processSpeed;
                motors->speeds[3] =  processSpeed;
            }

            delay = 10; // 10ms
        }
    }
};

#endif