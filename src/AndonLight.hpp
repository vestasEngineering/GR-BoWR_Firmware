#ifndef ANDON_LIGHT
#define ANDON_LIGHT

#include <Arduino.h>
#include <Adafruit_NeoPixel.h>

#define LED_PIN 1
#define NUM_LEDS 60
#define BLINK_INTERVAL 500 // Blink interval in milliseconds

Adafruit_NeoPixel strip(NUM_LEDS, LED_PIN, NEO_GRB + NEO_KHZ800);

class AndonLight {
    public:
        enum States {
            GREEN,
            YELLOW,
            BLUE,
            RED,
            OFF,
            BLINK_GREEN,
            BLINK_YELLOW,
            BLINK_BLUE,
            BLINK_RED
        };

        States state;
        bool blinkStatus = false;
        unsigned long lastBlinkTime = 0;

        void setup() {
            pinMode(LED_PIN, OUTPUT);
            strip.begin();
            strip.show(); // Initialize all pixels to 'off'
            setState(GREEN);
        }

        void setState(States newState) {
            state = newState;
            blinkStatus = false; // Reset blink status when changing state

            if (state == GREEN) {
                setColor(strip.Color(0, 255, 0)); // Green
            } else if (state == YELLOW) {
                setColor(strip.Color(255, 255, 0)); // Yellow
            } else if (state == BLUE) {
                setColor(strip.Color(0, 0, 255)); // Blue
            } else if (state == RED) {
                setColor(strip.Color(255, 0, 0)); // Red
            } else if (state == OFF) {
                setColor(strip.Color(0, 0, 0)); // Off
            } else if (state == BLINK_GREEN || state == BLINK_YELLOW || state == BLINK_BLUE || state == BLINK_RED) {
                blinkStatus = true; // Enable blinking for these states
            }
        }

        void setColor(uint32_t color) {
            for (int i = 0; i < NUM_LEDS; i++) {
                strip.setPixelColor(i, color);
            }
            strip.show();
        }

        void stateMachine() {
            unsigned long currentTime = millis();

            switch (state) {
                case OFF:
                    // All lights off
                    setState(OFF);
                    break;
                case RED:
                    // Red light on
                    setState(RED);
                    break;
                case YELLOW:
                    // Yellow light on
                    setState(YELLOW);
                    break;
                case GREEN:
                    // Green light on
                    setState(GREEN);
                    break;
                case BLUE:
                    // Blue light on
                    setState(BLUE);
                    break;
                case BLINK_RED:
                    if (currentTime - lastBlinkTime >= 500) {
                        blinkStatus = !blinkStatus; // Toggle blink status
                        lastBlinkTime = currentTime;
                        setColor(blinkStatus ? strip.Color(255, 0, 0) : strip.Color(0, 0, 0)); // Red blink
                    }
                    break;
                case BLINK_YELLOW:
                    if (currentTime - lastBlinkTime >= 500) {
                        blinkStatus = !blinkStatus; // Toggle blink status
                        lastBlinkTime = currentTime;
                        setColor(blinkStatus ? strip.Color(255, 255, 0) : strip.Color(0, 0, 0)); // Yellow blink
                    }
                    break;
                case BLINK_GREEN:
                    if (currentTime - lastBlinkTime >= 500) {
                        blinkStatus = !blinkStatus; // Toggle blink status
                        lastBlinkTime = currentTime;
                        setColor(blinkStatus ? strip.Color(0, 255, 0) : strip.Color(0, 0, 0)); // Green blink
                    }
                    break;
                case BLINK_BLUE:
                    if (currentTime - lastBlinkTime >= 500) {
                        blinkStatus = !blinkStatus; // Toggle blink status
                        lastBlinkTime = currentTime;
                        setColor(blinkStatus ? strip.Color(0, 0, 255) : strip.Color(0, 0, 0)); // Blue blink
                    }
                    break;

            }
        }
};

#endif