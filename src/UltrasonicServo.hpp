#ifndef ULTRASONIC_SERVO_HPP
#define ULTRASONIC_SERVO_HPP

#include <Arduino.h>
#include <Servo.h>
#include "Config.hpp"

class UltrasonicServo {
public:
    enum ServoState { INACTIVE, ACTIVE };

    UltrasonicServo() : currentState(INACTIVE), activeAngle(10), inactiveAngle(180) {}

    void setup() {
        servo.attach(CFG.ut_servo.pwm_pin); // Always use PWN pin
        deactivate();     // Start in inactive state
    }

    void activate()   { servo.write(CFG.ut_servo.active_angle_deg); currentState = ACTIVE; }
    void deactivate() { servo.write(CFG.ut_servo.inactive_angle_deg); currentState = INACTIVE; }


    ServoState getState() const {
        return currentState;
    }

    void setAngles(int active, int inactive) {
        activeAngle = active;
        inactiveAngle = inactive;
    }

private:
    Servo servo;
    ServoState currentState;
    int activeAngle;
    int inactiveAngle;
};

#endif