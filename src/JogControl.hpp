#ifndef JOG_CONTROL_HPP
#define JOG_CONTROL_HPP
#include <Arduino.h>
#include "Motors.hpp"
#include "MySerial.hpp"

class JogControl {
public:
    const int FORWARD_JOG_PIN = D1;
    const int BACKWARD_JOG_PIN = D7;

    float jogSpeed = 0.0;
    const float jogSpeedIncrement = 0.01; // m/s per loop
    const float jogSpeedMax = 0.2;        // max jog speed

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
        //if (mySerial->state != MySerial::DISCONNECTED) {
        //    return;
        //}

        bool forwardPressed = !digitalRead(FORWARD_JOG_PIN);
        bool backwardPressed = !digitalRead(BACKWARD_JOG_PIN);

        // Print button status
        //Serial.print("Forward Button: ");
        //Serial.println(forwardPressed ? "Pressed" : "Released");

        //Serial.print("Backward Button: ");
        //Serial.println(backwardPressed ? "Pressed" : "Released");

        if (forwardPressed && !backwardPressed) {
            jogSpeed += jogSpeedIncrement;
            if (jogSpeed > jogSpeedMax) jogSpeed = jogSpeedMax;
        } else if (backwardPressed && !forwardPressed) {
            jogSpeed -= jogSpeedIncrement;
            if (jogSpeed < -jogSpeedMax) jogSpeed = -jogSpeedMax;
        } else {
            jogSpeed = 0.0;
        }

        if (motors) {
            motors->speeds[0] = -jogSpeed;
            motors->speeds[1] = -jogSpeed;
            motors->speeds[2] =  jogSpeed;
            motors->speeds[3] =  jogSpeed;
        }

    }
};

#endif