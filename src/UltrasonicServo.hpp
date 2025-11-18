#ifndef ULTRASONIC_SERVO_HPP
#define ULTRASONIC_SERVO_HPP

#include <Arduino.h>
#include <Servo.h>

class UltrasonicServo {
public:
    enum ServoState { INACTIVE, ACTIVE };

    UltrasonicServo() : currentState(INACTIVE), activeAngle(0), inactiveAngle(180) {}

    void setup() {
        servo.attach(D4); // Always use PWN pin
        deactivate();     // Start in inactive state
    }

    void activate() {
        servo.write(activeAngle);
        currentState = ACTIVE;
    }

    void deactivate() {
        servo.write(inactiveAngle);
        currentState = INACTIVE;
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
    ServoState currentState;
    int activeAngle;
    int inactiveAngle;
};

#endif