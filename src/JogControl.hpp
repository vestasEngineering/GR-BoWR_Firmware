#ifndef JOG_CONTROL_HPP
#define JOG_CONTROL_HPP

#include <Arduino.h>
#include "Motors.hpp"

class MySerial;

class JogControl {
public:
    // Pins & constants
    static constexpr int FORWARD_JOG_PIN  = D1;
    static constexpr int BACKWARD_JOG_PIN = D10;

    static constexpr float jogSpeedIncrement = 0.02f; // m/s per loop (unused in current code)
    static constexpr float jogSpeedMax       = 0.02f; // max jog speed
    static constexpr unsigned long jogDurationMs = 1000;

    explicit JogControl(Motors& motorsRef, MySerial& serialRef)
        : motors(motorsRef), mySerial(serialRef) {}

    void setup() {
        pinMode(FORWARD_JOG_PIN,  INPUT_PULLUP);
        pinMode(BACKWARD_JOG_PIN, INPUT_PULLUP);
    }

    void update() {
        // Optional: only jog when disconnected from Linux
        // if (mySerial.state != MySerial::LinkState::DISCONNECTED) {
        //     return;
        // }

        const bool forwardPressed  = !digitalRead(FORWARD_JOG_PIN);
        const bool backwardPressed = !digitalRead(BACKWARD_JOG_PIN);

        // Start jog only on new press
        if ((forwardPressed || backwardPressed) && !jogActive) {
            jogActive    = true;
            jogStartTime = millis();
            jogDirection = forwardPressed ? 1 : -1;
        }

        // If jog is active, check duration
        if (jogActive) {
            if (millis() - jogStartTime < jogDurationMs) {
                const float v = jogDirection * jogSpeedMax;
                motors.speeds[0] = -v;
                motors.speeds[1] = -v;
                motors.speeds[2] =  v;
                motors.speeds[3] =  v;
            } else {
                // Stop motors and reset jog
                motors.speeds[0] = 0.0f;
                motors.speeds[1] = 0.0f;
                motors.speeds[2] = 0.0f;
                motors.speeds[3] = 0.0f;
                jogActive    = false;
                jogDirection = 0;
            }
        }
    }

    bool isActive() const { return jogActive; }

private:
    Motors&    motors;
    MySerial&  mySerial;

    bool            jogActive     = false;
    int             jogDirection  = 0; // +1 forward, -1 backward
    unsigned long   jogStartTime  = 0;
};

#endif