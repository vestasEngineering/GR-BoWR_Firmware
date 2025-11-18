#ifndef JOG_CONTROL_HPP
#define JOG_CONTROL_HPP

#include <Arduino.h>
#include "Motors.hpp"
#include "MySerial.hpp"

class JogControl {
public:
    const int FORWARD_JOG_PIN = D1;
    const int BACKWARD_JOG_PIN = D10;

    float jogSpeed = 0.0;
    const float jogSpeedIncrement = 0.02; // m/s per loop
    const float jogSpeedMax = 0.02;       // max jog speed

    bool jogActive = false;
    int jogDirection = 0; // +1 for forward, -1 for backward
    unsigned long jogStartTime = 0;
    const unsigned long jogDuration = 1000;

    Motors* motors;
    MySerial* mySerial;

    JogControl(Motors& motorsRef, MySerial& serialRef)
        : motors(&motorsRef), mySerial(&serialRef) {}

    void setup() {
        pinMode(FORWARD_JOG_PIN, INPUT_PULLUP);
        pinMode(BACKWARD_JOG_PIN, INPUT_PULLUP);
    }

    void update() {
        // Optional: only jog when disconnected from Linux
        // if (mySerial->state != MySerial::DISCONNECTED) {
        //     return;
        // }

        bool forwardPressed = !digitalRead(FORWARD_JOG_PIN);
        bool backwardPressed = !digitalRead(BACKWARD_JOG_PIN);

        // Start jog only on new press
        if ((forwardPressed || backwardPressed) && !jogActive) {
            jogActive = true;
            jogStartTime = millis();
            jogDirection = forwardPressed ? 1 : -1;
        }

        // If jog is active, check duration
        if (jogActive) {
            if (millis() - jogStartTime < jogDuration) {
                float jogSpeed = jogDirection * jogSpeedMax;
                motors->speeds[0] = -jogSpeed;
                motors->speeds[1] = -jogSpeed;
                motors->speeds[2] =  jogSpeed;
                motors->speeds[3] =  jogSpeed;
            } else {
                // Stop motors and reset jog
                motors->speeds[0] = 0.0;
                motors->speeds[1] = 0.0;
                motors->speeds[2] = 0.0;
                motors->speeds[3] = 0.0;
                jogActive = false;
                jogDirection = 0;
            }
        }
    }
};

#endif