#ifndef JOG_CONTROL_HPP
#define JOG_CONTROL_HPP

#include <Arduino.h>
#include "Motors.hpp"
#include "Config.hpp"

class MySerial;

class JogControl {
public:
    // Fallback/default pins if CFG is not used
    static constexpr int FORWARD_JOG_PIN  = D1;
    static constexpr int BACKWARD_JOG_PIN = D10;

    // Conservative hard limits for remote wireless jog
    static constexpr float REMOTE_JOG_MAX_SPEED_MS = 0.02f;
    static constexpr unsigned long REMOTE_JOG_MAX_LEASE_MS = 500;
    static constexpr unsigned long REMOTE_JOG_DEFAULT_LEASE_MS = 250;

    explicit JogControl(Motors& motorsRef, MySerial& serialRef)
        : motors(motorsRef), mySerial(serialRef) {}

    void setup() {
        pinMode(CFG.jog.pin_forward,  INPUT_PULLUP);
        pinMode(CFG.jog.pin_backward, INPUT_PULLUP);
    }

    void update() {
        updateRemoteJog();
        updatePhysicalJog();
        applyJogOutput();
    }

    bool isActive() const {
        return activeSource != Source::NONE;
    }

    bool isRemoteActive() const {
        return activeSource == Source::REMOTE;
    }

    bool isPhysicalActive() const {
        return activeSource == Source::PHYSICAL;
    }

    // ------------------------------------------------------------
    // Remote HMI jog API
    // ------------------------------------------------------------

    bool startOrRefreshRemoteJog(int direction,
                                 float requestedSpeedMs,
                                 unsigned long requestedLeaseMs,
                                 uint32_t seq) {
        if (direction != 1 && direction != -1) {
            return false;
        }

        // Reject stale or duplicate sequence numbers.
        // Handles wraparound poorly after very long sessions, but fine for practical use.
        if (remoteSeqInitialized && seq <= lastRemoteSeq) {
            return false;
        }

        remoteSeqInitialized = true;
        lastRemoteSeq = seq;

        // Do not allow remote jog to take over while physical jog is active.
        if (activeSource == Source::PHYSICAL) {
            return false;
        }

        const float safeSpeed = clampFloat(
            fabsf(requestedSpeedMs),
            0.0f,
            getRemoteMaxSpeed()
        );

        const unsigned long safeLease = clampUL(
            requestedLeaseMs == 0 ? REMOTE_JOG_DEFAULT_LEASE_MS : requestedLeaseMs,
            1,
            REMOTE_JOG_MAX_LEASE_MS
        );

        remoteDirection = direction;
        remoteSpeedMs = safeSpeed;
        remoteExpireMs = millis() + safeLease;
        activeSource = Source::REMOTE;

        return true;
    }

    void stopRemoteJog(uint32_t seq = 0) {
        // If seq is provided, ignore stale stop messages.
        if (seq != 0) {
            if (remoteSeqInitialized && seq <= lastRemoteSeq) {
                return;
            }
            remoteSeqInitialized = true;
            lastRemoteSeq = seq;
        }

        if (activeSource == Source::REMOTE) {
            stopAllJog();
        }
    }

private:
    enum class Source {
        NONE,
        PHYSICAL,
        REMOTE
    };

    Motors&   motors;
    MySerial& mySerial;

    Source activeSource = Source::NONE;

    // Physical jog state
    bool physicalJogActive = false;
    int physicalDirection = 0;
    unsigned long physicalStartMs = 0;

    // Remote jog state
    int remoteDirection = 0;
    float remoteSpeedMs = 0.0f;
    unsigned long remoteExpireMs = 0;
    uint32_t lastRemoteSeq = 0;
    bool remoteSeqInitialized = false;

    static float clampFloat(float v, float lo, float hi) {
        if (v < lo) return lo;
        if (v > hi) return hi;
        return v;
    }

    static unsigned long clampUL(unsigned long v,
                                 unsigned long lo,
                                 unsigned long hi) {
        if (v < lo) return lo;
        if (v > hi) return hi;
        return v;
    }

    float getRemoteMaxSpeed() const {
        // Firmware config remains the authority.
        const float cfgMax = CFG.jog.jog_speed_max_ms;

        if (cfgMax <= 0.0f) {
            return REMOTE_JOG_MAX_SPEED_MS;
        }

        return min(cfgMax, REMOTE_JOG_MAX_SPEED_MS);
    }

    void stopAllJog() {
        activeSource = Source::NONE;

        physicalJogActive = false;
        physicalDirection = 0;

        remoteDirection = 0;
        remoteSpeedMs = 0.0f;
        remoteExpireMs = 0;

        motors.setSpeeds(0.0f, 0.0f, 0.0f, 0.0f);
        motors.STOP();
    }

    void updateRemoteJog() {
        if (activeSource != Source::REMOTE) {
            return;
        }

        const unsigned long now = millis();

        // Lease expired: wireless command stream stopped, so stop.
        if ((long)(now - remoteExpireMs) >= 0) {
            stopAllJog();
        }
    }

    void updatePhysicalJog() {
        const bool forwardPressed  = !digitalRead(CFG.jog.pin_forward);
        const bool backwardPressed = !digitalRead(CFG.jog.pin_backward);

        // If remote jog is active, ignore physical jog until remote releases/expires.
        if (activeSource == Source::REMOTE) {
            return;
        }

        // If both are pressed, treat as invalid and stop physical jog.
        if (forwardPressed && backwardPressed) {
            if (activeSource == Source::PHYSICAL) {
                stopAllJog();
            }
            return;
        }

        if ((forwardPressed || backwardPressed) && !physicalJogActive) {
            physicalJogActive = true;
            physicalStartMs = millis();
            physicalDirection = forwardPressed ? -1 : 1;
            activeSource = Source::PHYSICAL;
        }

        if (activeSource == Source::PHYSICAL && physicalJogActive) {
            const unsigned long now = millis();

            // Current behavior: short duration jog for hardwired buttons.
            if (now - physicalStartMs >= CFG.jog.jog_duration_ms) {
                stopAllJog();
            }
        }
    }

    void applyJogOutput() {
        if (activeSource == Source::NONE) {
            return;
        }

        int direction = 0;
        float speed = 0.0f;

        if (activeSource == Source::REMOTE) {
            direction = remoteDirection;
            speed = remoteSpeedMs;
        } else if (activeSource == Source::PHYSICAL) {
            direction = physicalDirection;
            speed = CFG.jog.jog_speed_max_ms;
        }

        if (direction == 0 || speed <= 0.0f) {
            stopAllJog();
            return;
        }

        const float v = -direction * speed;

        // Match your existing physical jog wheel mapping:
        // forward direction = -v, -v, +v, +v
        motors.setSpeeds(
            -v,
            -v,
             v,
             v
        );
    }
};

#endif