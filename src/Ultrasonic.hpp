#ifndef MY_ULTRASONIC_CLASS
#define MY_ULTRASONIC_CLASS

#include <Arduino.h>
#include <ArduinoJson.h>
#include "Motors.hpp"
#include "Config.hpp"
#include <math.h>

extern UART myUART0;

class Ultrasonic {
public:
    volatile int delay = 0;                  // used by timer ISR
    float processSpeed = 0.0f;               // exposed target speed m/s
    float currentSpeed = 0.0f;               // mirrors processSpeed compat
    float distance[3] = {0, 0, 0};            // filtered distance history
    volatile uint16_t servoSettleDelay = 0;  // ultrasonic servo delay in 10 ms ticks

    // -----------------------------
    // Internal state
    // -----------------------------
    float measuredDistance = 0.0f;
    uint16_t badReadStreak = 0;

    Motors* motors = nullptr;

    // -----------------------------
    // Tracking behavior
    // -----------------------------
    enum GlueTrackState {
        WAIT_FOR_GLUE,
        TRACKING
    };

    GlueTrackState glueState = WAIT_FOR_GLUE;

    bool distanceFilterInitialized = false;
    uint8_t goodGlueCounter = 0;

    // -----------------------------
    // Telemetry
    // -----------------------------
    uint32_t lastTelemetryMs = 0;

    Ultrasonic() {}

    void attachMotors(Motors& m) {
        motors = &m;
    }

    void notifyServoMoved(uint16_t settleTicks) {
        servoSettleDelay = settleTicks;
    }

    void setup() {
        analogReadResolution(10);
    }

    bool isValidMeasurement(float d) {
        return (
            d > CFG.ultrasonic.valid_min_mm &&
            d < CFG.ultrasonic.valid_max_mm
        );
    }

    bool isIgnoredCloseReading(float d) {
        return (
            d >= CFG.ultrasonic.ignored_close_min_mm &&
            d <= CFG.ultrasonic.ignored_close_max_mm
        );
    }

    void stopAndBrake() {
        processSpeed = 0.0f;
        currentSpeed = 0.0f;

        if (motors) {
            motors->BRAKE_STOP();
        }
    }

    void emitTelemetry(
        float filteredDistance,
        float targetSpeed,
        float processSpeedValue,
        float error
    ) {
        if (!CFG.ultrasonic.telemetry_enabled) return;

        uint32_t now = millis();

        if (now - lastTelemetryMs < CFG.ultrasonic.telemetry_period_ms) return;

        lastTelemetryMs = now;

        StaticJsonDocument<384> doc;

        doc["type"] = "ultrasonic_dbg";
        doc["t_ms"] = now;

        doc["meas_mm"] = measuredDistance;
        doc["filt_mm"] = filteredDistance;
        doc["sp_mm"] = CFG.ultrasonic.setpoint_mm;
        doc["err_mm"] = error;

        doc["target_ms"] = targetSpeed;
        doc["process_ms"] = processSpeedValue;

        doc["state"] = (glueState == WAIT_FOR_GLUE) ? "WAIT" : "TRACK";

        doc["bad"] = badReadStreak;

        if (motors) {
            JsonArray qppsArr = doc.createNestedArray("qpps");
            qppsArr.add(motors->qpps[0]);
            qppsArr.add(motors->qpps[1]);
            qppsArr.add(motors->qpps[2]);
            qppsArr.add(motors->qpps[3]);
        }

        serializeJson(doc, myUART0);
        myUART0.println();
    }

    static float median3(float a, float b, float c) {
        if (a > b) {
            float t = a;
            a = b;
            b = t;
        }

        if (b > c) {
            float t = b;
            b = c;
            c = t;
        }

        if (a > b) {
            float t = a;
            a = b;
            b = t;
        }

        return b;
    }

    void stateMachine() {

        if (servoSettleDelay > 0) return;

        if (delay) return;
        delay = 10;

        // -----------------------------
        // Read sensor
        // -----------------------------
        const float voltage =
            analogRead(CFG.ultrasonic.analog_pin) *
            CFG.battery.vref /
            CFG.battery.adc_max_counts;

        measuredDistance =
            voltage * CFG.ultrasonic.mm_per_volt +
            CFG.ultrasonic.offset_mm;

        // -----------------------------
        // Ignore known erroneous close readings
        // -----------------------------
        if (isIgnoredCloseReading(measuredDistance)) {
            return;
        }

        float filteredDistance = measuredDistance;
        float targetSpeed = 0.0f;
        float error = CFG.ultrasonic.setpoint_mm - filteredDistance;

        // -----------------------------
        // Validate measurement
        // -----------------------------
        if (!isValidMeasurement(measuredDistance)) {
            badReadStreak++;

            // Temporary invalid data: soft decel.
            if (badReadStreak > 3) {
                processSpeed -=
                    CFG.ultrasonic.motion_decel_mps2 *
                    CFG.ultrasonic.control_loop_dt_s;

                if (processSpeed < 0.0f) {
                    processSpeed = 0.0f;
                }

                currentSpeed = processSpeed;

                if (motors && processSpeed > Motors::ZERO_SPEED_THRESHOLD_MS) {
                    motors->setSpeeds(
                        processSpeed,
                        processSpeed,
                        -processSpeed,
                        -processSpeed
                    );
                }
            }

            // Too many invalid readings: full stop.
            if (
                badReadStreak >
                CFG.andonMgr.ultrasonic_bad_streak_threshold
            ) {
                glueState = WAIT_FOR_GLUE;
                goodGlueCounter = 0;
                targetSpeed = 0.0f;
                stopAndBrake();
            }

            emitTelemetry(
                measuredDistance,
                targetSpeed,
                processSpeed,
                CFG.ultrasonic.setpoint_mm - measuredDistance
            );

            return;
        }

        badReadStreak = 0;

        // -----------------------------
        // Median filter
        // -----------------------------
        if (!distanceFilterInitialized) {
            distance[0] = measuredDistance;
            distance[1] = measuredDistance;
            distance[2] = measuredDistance;
            distanceFilterInitialized = true;
        } else {
            distance[2] = distance[1];
            distance[1] = distance[0];
            distance[0] = measuredDistance;
        }

        filteredDistance = median3(distance[0], distance[1], distance[2]);

        error = CFG.ultrasonic.setpoint_mm - filteredDistance;

        // -----------------------------
        // State machine
        // -----------------------------
        switch (glueState) {

            // -------------------------
            // WAIT: glue not ready yet
            // -------------------------
            case WAIT_FOR_GLUE:
            {
                targetSpeed = 0.0f;

                // Fully stopped while waiting.
                stopAndBrake();

                // Require consecutive detections before tracking.
                if (filteredDistance <= CFG.ultrasonic.setpoint_mm) {
                    goodGlueCounter++;
                } else {
                    goodGlueCounter = 0;
                }

                if (goodGlueCounter >= CFG.ultrasonic.good_glue_required) {
                    glueState = TRACKING;
                }

                emitTelemetry(
                    filteredDistance,
                    targetSpeed,
                    processSpeed,
                    error
                );

                return;
            }

            // -------------------------
            // TRACKING: follow glue
            // -------------------------
            case TRACKING:
            {
                // -------------------------------------------------
                // Glue-lost condition that causes stop.
                // -------------------------------------------------
                if (
                    filteredDistance >
                    CFG.ultrasonic.setpoint_mm +
                    CFG.ultrasonic.wait_range_mm
                ) {
                    targetSpeed = 0.0f;

                    glueState = WAIT_FOR_GLUE;
                    goodGlueCounter = 0;

                    stopAndBrake();

                    emitTelemetry(
                        filteredDistance,
                        targetSpeed,
                        processSpeed,
                        error
                    );

                    return;
                }

                // -------------------------------------------------
                // Normal tracking control:
                //
                // filteredDistance < setpoint -> error positive -> speed up
                // filteredDistance > setpoint -> error negative -> slow down
                // -------------------------------------------------
                float effectiveError = error;

                if (fabsf(error) < CFG.ultrasonic.tracking_deadband_mm) {
                    effectiveError = 0.0f;
                }

                float speedCmd = CFG.ultrasonic.base_speed_ms;

                if (effectiveError > 0.0f) {
                    // Distance is below setpoint, speed up gently.
                    speedCmd =
                        CFG.ultrasonic.base_speed_ms +
                        CFG.ultrasonic.kp_speed_up_track *
                        effectiveError;
                } 
                else if (effectiveError < 0.0f) {
                    // Distance is above setpoint, slow down more aggressively.
                    speedCmd =
                        CFG.ultrasonic.base_speed_ms +
                        CFG.ultrasonic.kp_slow_down_track *
                        effectiveError;
                }

                // Do not allow zero speed during normal tracking.
                // Full stop only happens when glue is lost.
                if (speedCmd < CFG.ultrasonic.min_track_speed_ms) {
                    speedCmd = CFG.ultrasonic.min_track_speed_ms;
                }

                if (speedCmd > CFG.ultrasonic.max_track_speed_ms) {
                    speedCmd = CFG.ultrasonic.max_track_speed_ms;
                }

                targetSpeed = speedCmd;

                break;
            }
        }

        // -----------------------------
        // Slew-rate limiting
        // -----------------------------
        const float dt = CFG.ultrasonic.control_loop_dt_s;

        float delta = targetSpeed - processSpeed;

        float accelLimit = CFG.ultrasonic.motion_accel_mps2;
        float decelLimit = CFG.ultrasonic.motion_decel_mps2;

        // Use faster response while tracking glue.
        if (glueState == TRACKING) {
            accelLimit = CFG.ultrasonic.track_accel_mps2;
            decelLimit = CFG.ultrasonic.track_decel_mps2;
        }

        if (delta > 0.0f) {
            float maxStepUp = accelLimit * dt;

            if (delta > maxStepUp) {
                delta = maxStepUp;
            }
        } else if (delta < 0.0f) {
            float maxStepDown = decelLimit * dt;

            if (delta < -maxStepDown) {
                delta = -maxStepDown;
            }
        }

        processSpeed += delta;
        currentSpeed = processSpeed;

        // Safety clamp.
        if (processSpeed < 0.0f) {
            processSpeed = 0.0f;
            currentSpeed = 0.0f;
        }

        if (processSpeed > CFG.ultrasonic.max_speed_ms) {
            processSpeed = CFG.ultrasonic.max_speed_ms;
            currentSpeed = CFG.ultrasonic.max_speed_ms;
        }

        // -----------------------------
        // Send to motors
        // -----------------------------
        if (motors) {

            if (glueState == WAIT_FOR_GLUE) {

                // WAIT state means full stop.
                if (fabsf(processSpeed) < Motors::ZERO_SPEED_THRESHOLD_MS) {
                    stopAndBrake();
                } else {
                    motors->setSpeeds(
                        processSpeed,
                        processSpeed,
                        -processSpeed,
                        -processSpeed
                    );
                }

            } else {

                // TRACKING:
                motors->setSpeeds(
                    processSpeed,
                    processSpeed,
                    -processSpeed,
                    -processSpeed
                );
            }
        }

        emitTelemetry(
            filteredDistance,
            targetSpeed,
            processSpeed,
            error
        );
    }
};

#endif