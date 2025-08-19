#ifndef ANDON_LIGHT
#define ANDON_LIGHT

#include <Arduino.h>
#include <seesaw_neopixel.h>
#include <math.h>

#define NEODRIVER_ADDR 0x62
#define NEOPIXEL_PIN 15
#define NUM_LEDS 36
#define BLINK_INTERVAL 500 // milliseconds
#define BOOT_DURATION 5000 // milliseconds


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

    seesaw_NeoPixel strip = seesaw_NeoPixel(NUM_LEDS, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);

    // Boot animation state
    bool booting = true;
    unsigned long bootStartTime = 0;
    unsigned long lastBootStepTime = 0;
    uint16_t bootStep = 0;

    void setup() {
        Serial.println("{\"status\": \"Initializing AndonLight...\"}");
        if (!strip.begin(NEODRIVER_ADDR)) {
            Serial.println("Seesaw not found!");
            while (1) delay(10);
        }
        strip.show(); // Initialize all pixels
        startBootAnimation(); // Run boot-up animation
        Serial.println("{\"status\": \"AndonLight initialized successfully.\"}");
        setState(GREEN);
    }

    void loop() {
        bootAnimationStep();
        stateMachine();
    }

    void setState(States newState) {
        Serial.println(newState);
        state = newState;
        blinkStatus = (state >= BLINK_GREEN && state <= BLINK_RED);
        updateLEDs(true); // Use fade for solid states
    }

    void updateLEDs(bool fade) {
        uint8_t r = 0, g = 0, b = 0;
        switch (state) {
            case GREEN: r = 0; g = 255; b = 0; break;
            case YELLOW: r = 255; g = 255; b = 0; break;
            case BLUE: r = 0; g = 0; b = 255; break;
            case RED: r = 255; g = 0; b = 0; break;
            case OFF: r = g = b = 0; break;
            default: return;
        }
        setColor(r, g, b, fade);
    }

    void setColor(uint8_t r, uint8_t g, uint8_t b, bool fade = true) {
        if (fade) {
            const int fadeStep = 15;

            for (int fadeVal = 0; fadeVal <= 255; fadeVal += fadeStep) {
                for (int i = 0; i < NUM_LEDS; i++) {
                    uint8_t fr = r * fadeVal / 255;
                    uint8_t fg = g * fadeVal / 255;
                    uint8_t fb = b * fadeVal / 255;
                    strip.setPixelColor(i, strip.Color(fr, fg, fb));
                }
                strip.show();
            }
        } else {
            for (int i = 0; i < NUM_LEDS; i++) {
                strip.setPixelColor(i, strip.Color(r, g, b));
            }
            strip.show();
        }
    }

    void stateMachine() {
        unsigned long currentTime = millis();
        if ((state >= BLINK_GREEN && state <= BLINK_RED) && currentTime - lastBlinkTime >= BLINK_INTERVAL) {
            lastBlinkTime = currentTime;
            blinkStatus = !blinkStatus;

            uint8_t r = 0, g = 0, b = 0;
            switch (state) {
                case BLINK_GREEN: r = 0; g = 255; b = 0; break;
                case BLINK_YELLOW: r = 255; g = 255; b = 0; break;
                case BLINK_BLUE: r = 0; g = 0; b = 255; break;
                case BLINK_RED: r = 255; g = 0; b = 0; break;
                default: break;
            }

            if (blinkStatus) {
                setColor(r, g, b, false); // No fade during blink
            } else {
                setColor(0, 0, 0, false);
            }
        }
    }

    void startBootAnimation() {
        booting = true;
        bootStartTime = millis();
        lastBootStepTime = 0;
        bootStep = 0;
    }

    void bootAnimationStep() {
        if (!booting) return;

        unsigned long currentTime = millis();
        if (currentTime - bootStartTime >= BOOT_DURATION) {
            booting = false;
            strip.clear();
            strip.show();
            return;
        }

        if (currentTime - lastBootStepTime >= 30) {
            lastBootStepTime = currentTime;

            for (int i = 0; i < NUM_LEDS; i++) {
                int trailPos = (bootStep - i + NUM_LEDS) % NUM_LEDS;
                float brightness = pow(0.6, trailPos);
                uint8_t hue = (i * 256 / NUM_LEDS + bootStep * 5) % 256;
                uint32_t color = Wheel(hue);
                uint8_t r = ((color >> 16) & 0xFF) * brightness;
                uint8_t g = ((color >> 8) & 0xFF) * brightness;
                uint8_t b = (color & 0xFF) * brightness;
                strip.setPixelColor(i, strip.Color(r, g, b));
            }
            strip.show();
            bootStep++;
        }
    }


private:
    uint32_t Wheel(byte WheelPos) {
        WheelPos = 255 - WheelPos;
        if (WheelPos < 85) {
            return strip.Color(255 - WheelPos * 3, 0, WheelPos * 3);
        }
        if (WheelPos < 170) {
            WheelPos -= 85;
            return strip.Color(0, WheelPos * 3, 255 - WheelPos * 3);
        }
        WheelPos -= 170;
        return strip.Color(WheelPos * 3, 255 - WheelPos * 3, 0);
    }
};

#endif
