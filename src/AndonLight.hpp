#ifndef ANDON_LIGHT
#define ANDON_LIGHT

#include <ArduinoJson.h>
#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include <math.h>
#include "Config.hpp"

class AndonLight {
public:
    enum States : uint8_t {
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

    explicit AndonLight(Stream& ioRef) : io(&ioRef) {}

    Stream* io = nullptr;

    // Public status retained for compatibility with existing code.
    States state = OFF;
    bool blinkStatus = false;
    bool booting = true;
    bool hardwareOk = false;

    // Placeholders: CFG may not be constructed yet when this object is.
    // setup() applies the configured pin, length and brightness.
    Adafruit_NeoPixel strip = Adafruit_NeoPixel(1, 4, NEO_GRB + NEO_KHZ800);

    void setup() {
        io->println(
            F("{\"type\":\"status\",\"module\":\"andon\",\"msg\":\"initializing\"}")
        );

        strip.setPin(CFG.andon.pin);
        strip.updateLength(CFG.andon.num_leds);
        strip.begin();
        strip.setBrightness(CFG.andon.brightness);
        strip.clear();
        strip.show();

        hardwareOk = true;
        curR_ = curG_ = curB_ = 0;

        // The boot animation intentionally owns the LEDs until it completes.
        // State changes received during boot are remembered and shown afterward.
        state = YELLOW;
        stateInitialized_ = true;
        startBootAnimation();

        io->println(
            F("{\"type\":\"status\",\"module\":\"andon\",\"msg\":\"initialized\"}")
        );
    }

    void loop() {
        const uint32_t now = millis();

        // This is intentional: while booting, the boot animation is the only
        // content drawn. Requested states are retained and applied afterward.
        if (booting) {
            serviceBootAnimation_(now);
            return;
        }

        if (isBlinkState_(state)) {
            serviceBlink_(now);
            return;
        }

        if (fading_) {
            serviceFade_(now);
        }
    }

    // Retained for compatibility. There is only one authoritative update path.
    void stateMachine() {
        loop();
    }

    void setState(States newState) {
        if (stateInitialized_ && state == newState) {
            return;
        }

        state = newState;
        stateInitialized_ = true;
        fading_ = false;

        // During boot, remember the requested state but leave animation output
        // untouched. It will be applied when the configured boot time expires.
        if (booting) {
            return;
        }

        applyCurrentState_(true);
    }

    void updateLEDs(bool fade) {
        if (!hardwareOk || booting) {
            return;
        }

        if (isBlinkState_(state)) {
            applyBlinkOnImmediately_();
            return;
        }

        uint8_t r = 0;
        uint8_t g = 0;
        uint8_t b = 0;
        getStateColor_(state, r, g, b);
        setColor(r, g, b, fade);
    }

    void setColor(uint8_t r, uint8_t g, uint8_t b, bool fade = true) {
        if (!hardwareOk || booting) {
            return;
        }

        if (isBlinkState_(state)) {
            // Blink timing owns output while a blinking state is active.
            return;
        }

        if (!fade || motionActive_ || CFG.andon.fade_ms == 0) {
            fading_ = false;
            curR_ = r;
            curG_ = g;
            curB_ = b;
            writeColor_(curR_, curG_, curB_);
            return;
        }

        srcR_ = curR_;
        srcG_ = curG_;
        srcB_ = curB_;
        tgtR_ = r;
        tgtG_ = g;
        tgtB_ = b;
        fadeStartMs_ = millis();
        fadeDurationMs_ = CFG.andon.fade_ms;
        fading_ = true;
    }

    void startBootAnimation() {
        booting = true;
        bootStartMs_ = millis();
        lastBootStepMs_ = bootStartMs_;
        lastFrameMs_ = 0;
        bootStep_ = 0;
        bootFrameDirty_ = true;
        fading_ = false;
    }

    // strip.show() masks interrupts for ~1.5 ms, which would stall the motor
    // step generator. While the motors are stepping, color changes snap
    // instead of fading so each state change costs a single show().
    void setMotionActive(bool active) { motionActive_ = active; }

private:
    bool motionActive_ = false;
    static constexpr uint32_t BOOT_STEP_INTERVAL_MS_ = 30;

    bool stateInitialized_ = false;

    uint32_t lastFrameMs_ = 0;
    uint32_t lastBlinkMs_ = 0;

    bool fading_ = false;
    uint32_t fadeStartMs_ = 0;
    uint32_t fadeDurationMs_ = 0;
    uint8_t curR_ = 0;
    uint8_t curG_ = 0;
    uint8_t curB_ = 0;
    uint8_t srcR_ = 0;
    uint8_t srcG_ = 0;
    uint8_t srcB_ = 0;
    uint8_t tgtR_ = 0;
    uint8_t tgtG_ = 0;
    uint8_t tgtB_ = 0;

    uint32_t bootStartMs_ = 0;
    uint32_t lastBootStepMs_ = 0;
    uint16_t bootStep_ = 0;
    bool bootFrameDirty_ = false;

        bool isBlinkState_(States value) const {
        return value >= BLINK_GREEN && value <= BLINK_RED;
    }

    void getStateColor_(States value, uint8_t& r, uint8_t& g, uint8_t& b) const {
        r = 0;
        g = 0;
        b = 0;

        switch (value) {
            case GREEN:
            case BLINK_GREEN:
                g = 255;
                break;

            case YELLOW:
            case BLINK_YELLOW:
                r = 255;
                g = 255;
                break;

            case BLUE:
            case BLINK_BLUE:
                b = 255;
                break;

            case RED:
            case BLINK_RED:
                r = 255;
                break;

            case OFF:
            default:
                break;
        }
    }

    void applyCurrentState_(bool fadeSolid) {
        if (!hardwareOk || booting) {
            return;
        }

        if (isBlinkState_(state)) {
            applyBlinkOnImmediately_();
            return;
        }

        blinkStatus = false;

        uint8_t r = 0;
        uint8_t g = 0;
        uint8_t b = 0;
        getStateColor_(state, r, g, b);
        setColor(r, g, b, fadeSolid);
    }

    void applyBlinkOnImmediately_() {
        uint8_t r = 0;
        uint8_t g = 0;
        uint8_t b = 0;
        getStateColor_(state, r, g, b);

        blinkStatus = true;
        lastBlinkMs_ = millis();
        fading_ = false;
        curR_ = r;
        curG_ = g;
        curB_ = b;
        writeColor_(r, g, b);
    }

    void serviceBlink_(uint32_t now) {
        if (
            static_cast<uint32_t>(now - lastBlinkMs_) <
            CFG.andon.blink_interval_ms
        ) {
            return;
        }

        lastBlinkMs_ = now;
        blinkStatus = !blinkStatus;

        uint8_t r = 0;
        uint8_t g = 0;
        uint8_t b = 0;

        if (blinkStatus) {
            getStateColor_(state, r, g, b);
        }

        curR_ = r;
        curG_ = g;
        curB_ = b;
        writeColor_(r, g, b);
    }

    void serviceFade_(uint32_t now) {
        if (
            static_cast<uint32_t>(now - lastFrameMs_) <
            CFG.andon.frame_dt_ms
        ) {
            return;
        }

        lastFrameMs_ = now;

        if (fadeDurationMs_ == 0) {
            curR_ = tgtR_;
            curG_ = tgtG_;
            curB_ = tgtB_;
            fading_ = false;
            writeColor_(curR_, curG_, curB_);
            return;
        }

        float t = static_cast<float>(now - fadeStartMs_) /
                  static_cast<float>(fadeDurationMs_);

        if (t >= 1.0f) {
            t = 1.0f;
        }

        const uint8_t nextR = interpolate_(srcR_, tgtR_, t);
        const uint8_t nextG = interpolate_(srcG_, tgtG_, t);
        const uint8_t nextB = interpolate_(srcB_, tgtB_, t);

        if (nextR != curR_ || nextG != curG_ || nextB != curB_) {
            curR_ = nextR;
            curG_ = nextG;
            curB_ = nextB;
            writeColor_(curR_, curG_, curB_);
        }

        if (t >= 1.0f) {
            fading_ = false;
        }
    }

    uint8_t interpolate_(uint8_t from, uint8_t to, float t) const {
        const float value =
            static_cast<float>(from) +
            (static_cast<float>(to) - static_cast<float>(from)) * t;

        return static_cast<uint8_t>(constrain(value, 0.0f, 255.0f));
    }

    void serviceBootAnimation_(uint32_t now) {
        if (
            static_cast<uint32_t>(now - bootStartMs_) >=
            CFG.andon.boot_duration_ms
        ) {
            booting = false;
            bootFrameDirty_ = false;

            // Snap directly to the latest requested post-boot state. This
            // preserves the intentional behavior that boot animation owns the
            // output for its entire configured duration.
            applyCurrentState_(false);
            return;
        }

        if (
            static_cast<uint32_t>(now - lastBootStepMs_) >=
            BOOT_STEP_INTERVAL_MS_
        ) {
            lastBootStepMs_ = now;
            ++bootStep_;
            bootFrameDirty_ = true;
        }

        if (!bootFrameDirty_) {
            return;
        }

        if (
            static_cast<uint32_t>(now - lastFrameMs_) <
            CFG.andon.frame_dt_ms
        ) {
            return;
        }

        lastFrameMs_ = now;
        bootFrameDirty_ = false;
        drawBootFrame_();
        strip.show();
    }

    void drawBootFrame_() {
        const int ledCount = static_cast<int>(CFG.andon.num_leds);
        if (ledCount <= 0) {
            return;
        }

        for (int i = 0; i < ledCount; ++i) {
            const int trailPosition =
                (static_cast<int>(bootStep_) - i + ledCount) % ledCount;

            const float brightness = powf(0.6f, trailPosition);
            const uint8_t hue = static_cast<uint8_t>(
                (i * 256 / ledCount + static_cast<int>(bootStep_) * 5) % 256
            );

            const uint32_t color = wheel_(hue);
            const uint8_t r = static_cast<uint8_t>(
                ((color >> 16) & 0xFFu) * brightness
            );
            const uint8_t g = static_cast<uint8_t>(
                ((color >> 8) & 0xFFu) * brightness
            );
            const uint8_t b = static_cast<uint8_t>(
                (color & 0xFFu) * brightness
            );

            strip.setPixelColor(i, strip.Color(r, g, b));
        }
    }

    void writeColor_(uint8_t r, uint8_t g, uint8_t b) {
        for (uint16_t i = 0; i < CFG.andon.num_leds; ++i) {
            strip.setPixelColor(i, strip.Color(r, g, b));
        }

        strip.show();
    }

    uint32_t wheel_(uint8_t wheelPosition) {
        wheelPosition = 255 - wheelPosition;

        if (wheelPosition < 85) {
            return strip.Color(
                255 - wheelPosition * 3,
                0,
                wheelPosition * 3
            );
        }

        if (wheelPosition < 170) {
            wheelPosition -= 85;
            return strip.Color(
                0,
                wheelPosition * 3,
                255 - wheelPosition * 3
            );
        }

        wheelPosition -= 170;
        return strip.Color(
            wheelPosition * 3,
            255 - wheelPosition * 3,
            0
        );
    }
};

#endif
