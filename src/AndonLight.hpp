#ifndef ANDON_LIGHT
#define ANDON_LIGHT

#include <Arduino.h>
#include <seesaw_neopixel.h>
#include <math.h>
#include "Config.hpp"   // <-- central configuration

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

    // Frame cap to avoid I2C/Seesaw shimmer
    unsigned long lastFrameMs = 0;

    // Time-based fade
    bool     fading = false;
    unsigned long fadeStartMs = 0;
    uint16_t fadeDurMs   = 0;          // will be set from CFG.andon.fade_ms at setup and on fades
    uint8_t  curR=0, curG=0, curB=0;  // current applied color
    uint8_t  tgtR=0, tgtG=0, tgtB=0;  // fade target color
    uint8_t  srcR=0, srcG=0, srcB=0;  // fade source color

    // Construct with placeholder length/pin; configure real values at setup()
    seesaw_NeoPixel strip = seesaw_NeoPixel(1 /*placeholder len*/,
                                            15 /*placeholder pin*/,
                                            NEO_GRB + NEO_KHZ800,
                                            &Wire1);

    // Boot animation state
    bool booting = true;
    unsigned long bootStartTime = 0;
    unsigned long lastBootStepTime = 0;
    uint16_t bootStep = 0;

    void setup() {
        Serial.println("{\"status\": \"Initializing AndonLight...\"}");

        // Initialize seesaw NeoPixel driver at configured I2C address
        if (!strip.begin(CFG.andon.neo_addr)) {
            Serial.println("{\"error\":\"seesaw_not_found\"}");
            while (1) delay(10);
        }

        // Apply configured pixel type and count
        strip.updateType(NEO_GRB + NEO_KHZ800);
        strip.updateLength(CFG.andon.num_leds);

        // Initialize fade duration from config
        fadeDurMs = CFG.andon.fade_ms;

        strip.show(); // clear
        startBootAnimation(); // boot animation starts immediately
        Serial.println("{\"status\": \"AndonLight initialized successfully.\"}");

        // Default to YELLOW after boot finishes
        //setState(YELLOW);
    }

    void loop() {
        bootAnimationStep();
        stateMachine();
        frame();
    }

    void setState(States newState) {
        state = newState;
        blinkStatus = (state >= BLINK_GREEN && state <= BLINK_RED);
        updateLEDs(true); // Use fade for solid states (ignored for blink states)
    }

    void updateLEDs(bool fade) {
        uint8_t r = 0, g = 0, b = 0;
        switch (state) {
            case GREEN:  r = 0;   g = 255; b = 0;   break;
            case YELLOW: r = 255; g = 255; b = 0;   break;
            case BLUE:   r = 0;   g = 0;   b = 255; break;
            case RED:    r = 255; g = 0;   b = 0;   break;
            case OFF:    r = g = b = 0;              break;
            default: return; // blink states handled elsewhere
        }
        setColor(r, g, b, fade);
    }

    void setColor(uint8_t r, uint8_t g, uint8_t b, bool fade = true) {
        // If currently blinking, ignore fades; blink path is authoritative
        if (state >= BLINK_GREEN && state <= BLINK_RED) {
            applyColor(r, g, b);
            fading = false;
            curR = r; curG = g; curB = b;
            return;
        }

        if (!fade) {
            fading = false;
            curR = r; curG = g; curB = b;
            applyColor(curR, curG, curB);
            return;
        }

        srcR = curR; srcG = curG; srcB = curB;
        tgtR = r;    tgtG = g;    tgtB = b;
        fadeStartMs = millis();
        fadeDurMs   = CFG.andon.fade_ms;   // <-- use configured fade duration
        fading = true;
    }
    
    void frame() {
        unsigned long now = millis();

        // Frame cap (use configured frame dt)
        if (now - lastFrameMs < CFG.andon.frame_dt_ms) return;
        lastFrameMs = now;

        // While booting, only boot draws frames
        if (booting) {
            bootAnimationDraw(); // split out drawing from bootAnimationStep()
            strip.show();
            return;
        }

        bool wrote = false;

        // Blink handling (no fade during blink)
        if (state >= BLINK_GREEN && state <= BLINK_RED) {
            if (now - lastBlinkTime >= CFG.andon.blink_interval_ms) {
                lastBlinkTime = now;
                blinkStatus = !blinkStatus;

                uint8_t r=0, g=0, b=0;
                if (blinkStatus) {
                    switch (state) {
                        case BLINK_GREEN:  r=0;   g=255; b=0;   break;
                        case BLINK_YELLOW: r=255; g=255; b=0;   break;
                        case BLINK_BLUE:   r=0;   g=0;   b=255; break;
                        case BLINK_RED:    r=255; g=0;   b=0;   break;
                        default: break;
                    }
                } else {
                    r = g = b = 0;
                }

                curR = r; curG = g; curB = b;
                applyColor(curR, curG, curB);
                fading = false;
                wrote = true;
            }
        }

        // Fade step (only if not blinking this frame)
        if (!wrote && fading) {
            // Normalize time using configured fade duration
            float t = float(now - fadeStartMs) / float(CFG.andon.fade_ms);
            if (t >= 1.0f) t = 1.0f;

            uint8_t nr = (uint8_t)(srcR + (int)((tgtR - srcR) * t));
            uint8_t ng = (uint8_t)(srcG + (int)((tgtG - srcG) * t));
            uint8_t nb = (uint8_t)(srcB + (int)((tgtB - srcB) * t));

            if (nr != curR || ng != curG || nb != curB) {
                curR = nr; curG = ng; curB = nb;
                applyColor(curR, curG, curB);
                wrote = true;
            }

            if (t >= 1.0f) {
                fading = false;
            }
        }
    }

    void applyColor(uint8_t r, uint8_t g, uint8_t b) {
        for (int i = 0; i < (int)CFG.andon.num_leds; i++) {
            strip.setPixelColor(i, strip.Color(r, g, b));
        }
        strip.show();
    }

    void stateMachine() {
        unsigned long currentTime = millis();

        // Blink toggling when in blink states
        if ((state >= BLINK_GREEN && state <= BLINK_RED) &&
            (currentTime - lastBlinkTime >= CFG.andon.blink_interval_ms)) {
            lastBlinkTime = currentTime;
            blinkStatus = !blinkStatus;

            uint8_t r = 0, g = 0, b = 0;
            switch (state) {
                case BLINK_GREEN:  r = 0;   g = 255; b = 0;   break;
                case BLINK_YELLOW: r = 255; g = 255; b = 0;   break;
                case BLINK_BLUE:   r = 0;   g = 0;   b = 255; break;
                case BLINK_RED:    r = 255; g = 0;   b = 0;   break;
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
        if (currentTime - bootStartTime >= CFG.andon.boot_duration_ms) {
            booting = false;

            // Important: snap to the post-boot state immediately (no fade)
            updateLEDs(false);
            // Align fade baselines
            srcR = curR; srcG = curG; srcB = curB;
            tgtR = curR; tgtG = curG; tgtB = curB;
            fading = false;
            return;
        }
        // Manage timing here; no strip.show() in this function
        if (currentTime - lastBootStepTime >= 30) {
            lastBootStepTime = currentTime;
            bootStep++;
        }
    }

    void bootAnimationDraw() {
        // Draw one boot frame (no show here—frame() will call show once)
        for (int i = 0; i < (int)CFG.andon.num_leds; i++) {
            int trailPos = (bootStep - i + (int)CFG.andon.num_leds) % (int)CFG.andon.num_leds;
            float brightness = pow(0.6f, trailPos);
            uint8_t hue = (uint8_t)((i * 256 / (int)CFG.andon.num_leds + bootStep * 5) % 256);
            uint32_t color = Wheel(hue);
            uint8_t r = (uint8_t)(((color >> 16) & 0xFF) * brightness);
            uint8_t g = (uint8_t)(((color >> 8)  & 0xFF) * brightness);
            uint8_t b = (uint8_t)(( color        & 0xFF) * brightness);
            strip.setPixelColor(i, strip.Color(r, g, b));
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