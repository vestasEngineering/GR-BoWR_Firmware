#ifndef MY_ULTRASONIC_CLASS
#define MY_ULTRASONIC_CLASS

#include <Arduino.h>
#include <ArduinoJson.h>
#include "PID.hpp"
#include "Motors.hpp"

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

    float setPoint = 130.0;
    float tolerance = 2.0;
    float lowerLimit = setPoint - tolerance;
    float upperLimit = setPoint + tolerance;

    Motors* motors;

    PID pid = PID(3.5, 0.3, 0.08, setPoint);

    Ultrasonic() {}

    void attachMotors(Motors& m) {
        motors = &m;
    }

    void setup() {
        analogReadResolution(10);
        pid.setOutputLimits(-0.005, 0.005);
        pid.setSampleTime(0.05);
    }

    void updateHeight(float height) {
        pid.setSetpoint(height);
        lowerLimit = height - tolerance;
        upperLimit = height + tolerance;
    }

    bool isValidMeasurement(float d) {
        return (d > 45.0 && d < 250.0);
    }

    void stateMachine() {
        if (!delay) {
            // Read and smooth distance
            voltage = (float(analogRead(ULTRASONIC_PIN)) * 3.1 / 1023.0);
            measuredDistance = voltage * mmPerVolt + offsetDistance;

            if (!isValidMeasurement(measuredDistance)) {
                delay = 50;
                return;
            }

            distance[0] = distance[0] + (measuredDistance - distance[0]) * 0.8;

            // PID control
            if (distance[0] < 70.0) {
                processSpeed = 0.0;
                currentSpeed = 0.0;
            } else {
                float u = 0.0;
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

            delay = 50; // 50ms
        }
    }
};

#endif