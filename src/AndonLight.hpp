#ifndef ANDON_LIGHT
#define ANDON_LIGHT

#include <Arduino.h>
#include <Adafruit_NeoPixel.h>

#define LED_PIN 5
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
            strip.begin();
            strip.show(); // Initialize all pixels to 'off'
            setState(GREEN);
        }

        void setState(States newState) {
            state = newState;
            blinkStatus = (state == BLINK_GREEN || state == BLINK_YELLOW || state == BLINK_BLUE || state == BLINK_RED);
            updateLEDs();
        }

        void updateLEDs() {
            switch (state) {
                case GREEN: setColor(strip.Color(0, 255, 0)); break;
                case YELLOW: setColor(strip.Color(255, 255, 0)); break;
                case BLUE: setColor(strip.Color(0, 0, 255)); break;
                case RED: setColor(strip.Color(255, 0, 0)); break;
                case OFF: setColor(strip.Color(0, 0, 0)); break;
                default: break;
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

            if (blinkStatus && currentTime - lastBlinkTime >= BLINK_INTERVAL) {
                lastBlinkTime = currentTime;
                blinkStatus = !blinkStatus;
            }
            
            uint32_t blinkColor;
            switch (state) {
                case BLINK_RED: blinkColor = strip.Color(255, 0, 0); break;
                case BLINK_YELLOW: blinkColor = strip.Color(255, 255, 0); break;
                case BLINK_GREEN: blinkColor = strip.Color(0, 255, 0); break;
                case BLINK_BLUE: blinkColor = strip.Color(0, 0, 255); break;

            }

            setColor(blinkStatus ? blinkColor : strip.Color(0,0,0));
        }
};

#endif