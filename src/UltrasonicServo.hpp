#ifndef ULTRASONIC_SERVO_HPP
#define ULTRASONIC_SERVO_HPP

#include <Arduino.h>
#include <Servo.h>
#include "Config.hpp"

class Ultrasonic;

class UltrasonicServo {
public:
    enum ServoState { INACTIVE, ACTIVE };

    UltrasonicServo() : currentState(INACTIVE), activeAngle(10), inactiveAngle(180) {}

    void setup() {
        servo.attach(CFG.ut_servo.pwm_pin); // Always use PWN pin
        deactivate();     // Start in inactive state
    }

    void attachUltrasonic(Ultrasonic& u) {
        ultrasonic = &u;
    }

    void activate() {
        if (currentState != ACTIVE) {
            servo.write(CFG.ut_servo.active_angle_deg);
            currentState = ACTIVE;

            if (ultrasonic) {
                ultrasonic->notifyServoMoved(125); // 150 × 10 ms = 1.5 s
            }
        }
    }

    void deactivate() {
        if (currentState != INACTIVE) {
            servo.write(CFG.ut_servo.inactive_angle_deg);
            currentState = INACTIVE;
        }
    }


    ServoState getState() const {
        return currentState;
    }

    void setAngles(int active, int inactive) {
        activeAngle = active;
        inactiveAngle = inactive;
    }

private:
    Servo servo;
    Ultrasonic* ultrasonic = nullptr;
    ServoState currentState;
    int activeAngle;
    int inactiveAngle;
};

#endif