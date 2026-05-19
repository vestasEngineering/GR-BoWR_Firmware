#ifndef MY_ULTRASONIC_CLASS
#define MY_ULTRASONIC_CLASS

#include <Arduino.h>
#include <ArduinoJson.h>
#include "PID.hpp"
#include "Motors.hpp"
#include "Config.hpp"
#include <math.h>

// Legacy macro used by diagnostics / tests
#define ULTRASONIC_PIN A2

class Ultrasonic {
public:
    // ---- LEGACY PUBLIC STATE (required by other subsystems) ----
    volatile int delay = 0;                  // used by timer ISR
    float processSpeed = 0.0f;               // exposed target speed (m/s)
    float currentSpeed = 0.0f;               // mirrors processSpeed (compat)
    float distance[3]   = {0,0,0};            // filtered distance history
    float mmPerVolt     = CFG.ultrasonic.mm_per_volt;
    float offsetDistance= CFG.ultrasonic.offset_mm;
    volatile uint16_t servoSettleDelay = 0; // ultrasonic servo delay in 10 ms ticks


    // ---- INTERNAL ----
    float measuredDistance = 0.0f;
    uint16_t badReadStreak = 0;
    
    float maxAccelMps2 = CFG.ultrasonic.motion_accel_mps2;
    float maxDecelMps2 = CFG.ultrasonic.motion_decel_mps2;

    Motors* motors = nullptr;

    float setPoint  = 175.0f;
    float tolerance = 2.0f;

    PID pid = PID(
        CFG.ultrasonic.pid_kp,
        CFG.ultrasonic.pid_ki,
        CFG.ultrasonic.pid_kd,
        setPoint
    );

    Ultrasonic() {}

    void attachMotors(Motors& m) {
        motors = &m;
    }

    void notifyServoMoved(uint16_t settleTicks) {
        servoSettleDelay = settleTicks;
    }

    void setup() {
        analogReadResolution(10);
        pid.setOutputLimits(
            CFG.ultrasonic.pid_out_min_ms,
            CFG.ultrasonic.pid_out_max_ms
        );
        pid.setSampleTime(CFG.ultrasonic.pid_sample_time_s);
    }

    bool isValidMeasurement(float d) {
        return (d > CFG.ultrasonic.valid_min_mm &&
                d < CFG.ultrasonic.valid_max_mm);
    }

    static float median3(float a, float b, float c) {
        if (a > b) { float t = a; a = b; b = t; }
        if (b > c) { float t = b; b = c; c = t; }
        if (a > b) { float t = a; a = b; b = t; }
        return b;
    }

    void stateMachine() {

        if (servoSettleDelay > 0) {
            return;  // Servo still moving — DO NOTHING
        }

        if (delay) return;
        delay = 10;  // 10 ms pacing (legacy behavior preserved)

        const float voltage =
            analogRead(CFG.ultrasonic.analog_pin) *
            CFG.battery.vref / CFG.battery.adc_max_counts;

        measuredDistance =
            voltage * mmPerVolt + offsetDistance;

        if (!isValidMeasurement(measuredDistance)) {
            if (++badReadStreak >
                CFG.andonMgr.ultrasonic_bad_streak_threshold) {
                if (motors) motors->STOP();
            }
            return;
        }
        badReadStreak = 0;

        // shift history
        distance[2] = distance[1];
        distance[1] = distance[0];
        distance[0] = measuredDistance;

        const float filteredDistance = median3(distance[0], distance[1], distance[2]);

        // Stop when at or below target
        float targetSpeed = 0.0f;

        // Stop condition: command target speed to zero,
        // but allow the decel slew limiter to ramp processSpeed down.
        if (measuredDistance >= (setPoint + tolerance)) {
            targetSpeed = 0.0f;
        } else {
            targetSpeed = pid.compute(filteredDistance);
        }

        // Slew limit processSpeed
        const float dt = CFG.ultrasonic.pid_sample_time_s;

        float delta = targetSpeed - processSpeed;

        if (delta > 0.0f) {
            // Speeding up
            const float maxStepUp = maxAccelMps2 * dt;
            if (delta > maxStepUp) {
                delta = maxStepUp;
            }
        } else if (delta < 0.0f) {
            // Slowing down
            const float maxStepDown = maxDecelMps2 * dt;
            if (delta < -maxStepDown) {
                delta = -maxStepDown;
            }
        }

        processSpeed += delta;
        currentSpeed = processSpeed;

        if (motors) {
            if (fabsf(processSpeed) < Motors::ZERO_SPEED_THRESHOLD_MS) {
                processSpeed = 0.0f;
                currentSpeed = 0.0f;

                // Controlled closed-loop stop instead of raw duty zero.
                motors->BRAKE_STOP();
            } else {
                motors->setSpeeds(
                    processSpeed,
                    processSpeed,
                    -processSpeed,
                    -processSpeed
                );
            }
        }

    }
};

#endif