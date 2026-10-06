#ifndef JOG_CONTROL_HPP
#define JOG_CONTROL_HPP

#include <Arduino.h>
#include <string.h>
#include "MotorCP.hpp"
#include "Config.hpp"

class MySerial;

class JogControl {
public:
    static constexpr int FORWARD_JOG_PIN = D1;
    static constexpr int BACKWARD_JOG_PIN = D10;
    static constexpr float REMOTE_JOG_MAX_SPEED_MS = 0.02f;
    static constexpr unsigned long REMOTE_JOG_MAX_LEASE_MS = 500;
    static constexpr unsigned long REMOTE_JOG_DEFAULT_LEASE_MS = 250;
    static constexpr size_t REMOTE_SESSION_ID_MAX_CHARS = 48;

    enum class RemoteJogResult {
        ACCEPTED,
        INVALID_DIRECTION,
        INVALID_SESSION,
        STALE_SEQUENCE,
        PHYSICAL_JOG_ACTIVE,
        MOTION_INHIBITED,
    };

    explicit JogControl(MotorCP& motorsRef, MySerial& serialRef)
        : motors(motorsRef), mySerial(serialRef) {
        remoteSessionId[0] = '\0';
    }

    void setup() {
        pinMode(CFG.jog.pin_forward, INPUT_PULLUP);
        pinMode(CFG.jog.pin_backward, INPUT_PULLUP);
    }

    void update() {
        updateRemoteJog();
        updatePhysicalJog();
        applyJogOutput();
    }

    bool isActive() const { return activeSource != Source::NONE; }
    bool isRemoteActive() const { return activeSource == Source::REMOTE; }
    bool isPhysicalActive() const { return activeSource == Source::PHYSICAL; }

    const char* currentRemoteSessionId() const { return remoteSessionId; }
    uint32_t lastAcceptedRemoteSequence() const { return lastRemoteSeq; }

    static const char* remoteJogResultToString(RemoteJogResult result) {
        switch (result) {
            case RemoteJogResult::ACCEPTED: return "accepted";
            case RemoteJogResult::INVALID_DIRECTION: return "invalid_direction";
            case RemoteJogResult::INVALID_SESSION: return "invalid_session";
            case RemoteJogResult::STALE_SEQUENCE: return "stale_sequence";
            case RemoteJogResult::PHYSICAL_JOG_ACTIVE: return "physical_jog_active";
            case RemoteJogResult::MOTION_INHIBITED: return "motion_inhibited";
            default: return "jog_rejected";
        }
    }

    RemoteJogResult startOrRefreshRemoteJog(
        const char* sessionId,
        int direction,
        float requestedSpeedMs,
        unsigned long requestedLeaseMs,
        uint32_t seq
    ) {
        if (direction != 1 && direction != -1) {
            return RemoteJogResult::INVALID_DIRECTION;
        }
        if (!isValidSessionId(sessionId)) {
            return RemoteJogResult::INVALID_SESSION;
        }
        if (activeSource == Source::PHYSICAL) {
            return RemoteJogResult::PHYSICAL_JOG_ACTIVE;
        }
        if (motors.isMotionInhibited()) {
            return RemoteJogResult::MOTION_INHIBITED;
        }

        const bool newSession = strcmp(remoteSessionId, sessionId) != 0;
        if (newSession) {
            if (activeSource == Source::REMOTE) {
                stopAllJog();
            }
            copySessionId(sessionId);
            remoteSeqInitialized = false;
            lastRemoteSeq = 0;
        }

        if (remoteSeqInitialized && !isSequenceNewer(seq, lastRemoteSeq)) {
            return RemoteJogResult::STALE_SEQUENCE;
        }

        remoteSeqInitialized = true;
        lastRemoteSeq = seq;
        remoteDirection = direction;
        remoteSpeedMs = clampFloat(
            fabsf(requestedSpeedMs),
            0.0f,
            getRemoteMaxSpeed()
        );
        remoteExpireMs = millis() + clampUL(
            requestedLeaseMs == 0
                ? REMOTE_JOG_DEFAULT_LEASE_MS
                : requestedLeaseMs,
            1,
            REMOTE_JOG_MAX_LEASE_MS
        );
        activeSource = Source::REMOTE;
        return RemoteJogResult::ACCEPTED;
    }

    bool stopRemoteJog(const char* sessionId, uint32_t seq = 0) {
        if (!isValidSessionId(sessionId)) return false;
        if (remoteSessionId[0] == '\0' || strcmp(remoteSessionId, sessionId) != 0) {
            return false;
        }
        if (seq != 0) {
            if (remoteSeqInitialized && !isSequenceNewer(seq, lastRemoteSeq)) {
                return false;
            }
            remoteSeqInitialized = true;
            lastRemoteSeq = seq;
        }
        if (activeSource == Source::REMOTE) stopAllJog();
        return true;
    }

    void invalidateRemoteSession() {
        if (activeSource == Source::REMOTE) stopAllJog();
        remoteSessionId[0] = '\0';
        remoteSeqInitialized = false;
        lastRemoteSeq = 0;
    }

private:
    enum class Source { NONE, PHYSICAL, REMOTE };

    MotorCP& motors;
    MySerial& mySerial;
    Source activeSource = Source::NONE;

    bool physicalJogActive = false;
    int physicalDirection = 0;
    unsigned long physicalStartMs = 0;

    int remoteDirection = 0;
    float remoteSpeedMs = 0.0f;
    unsigned long remoteExpireMs = 0;
    uint32_t lastRemoteSeq = 0;
    bool remoteSeqInitialized = false;
    char remoteSessionId[REMOTE_SESSION_ID_MAX_CHARS + 1];

    static bool isSequenceNewer(uint32_t candidate, uint32_t previous) {
        return static_cast<int32_t>(candidate - previous) > 0;
    }

    static bool isValidSessionId(const char* sessionId) {
        if (sessionId == nullptr || sessionId[0] == '\0') return false;
        const size_t length = strnlen(sessionId, REMOTE_SESSION_ID_MAX_CHARS + 1);
        return length > 0 && length <= REMOTE_SESSION_ID_MAX_CHARS;
    }

    void copySessionId(const char* sessionId) {
        strncpy(remoteSessionId, sessionId, REMOTE_SESSION_ID_MAX_CHARS);
        remoteSessionId[REMOTE_SESSION_ID_MAX_CHARS] = '\0';
    }

    static float clampFloat(float value, float low, float high) {
        if (value < low) return low;
        if (value > high) return high;
        return value;
    }

    static unsigned long clampUL(unsigned long value, unsigned long low, unsigned long high) {
        if (value < low) return low;
        if (value > high) return high;
        return value;
    }

    float getRemoteMaxSpeed() const {
        const float configuredMax = CFG.jog.jog_speed_max_ms;
        if (configuredMax <= 0.0f) return REMOTE_JOG_MAX_SPEED_MS;
        return min(configuredMax, REMOTE_JOG_MAX_SPEED_MS);
    }

    void stopAllJog() {
        activeSource = Source::NONE;
        physicalJogActive = false;
        physicalDirection = 0;
        remoteDirection = 0;
        remoteSpeedMs = 0.0f;
        remoteExpireMs = 0;
        motors.setSpeeds(0.0f, 0.0f);
        motors.BRAKE_STOP();
    }

    void updateRemoteJog() {
        if (activeSource != Source::REMOTE) return;
        if (static_cast<long>(millis() - remoteExpireMs) >= 0) {
            stopAllJog();
        }
    }

    void updatePhysicalJog() {
        const bool forwardPressed = !digitalRead(CFG.jog.pin_forward);
        const bool backwardPressed = !digitalRead(CFG.jog.pin_backward);

        if (activeSource == Source::REMOTE) return;

        if (forwardPressed && backwardPressed) {
            if (activeSource == Source::PHYSICAL) stopAllJog();
            return;
        }

        if (activeSource == Source::PHYSICAL &&
            !forwardPressed && !backwardPressed) {
            stopAllJog();
            return;
        }

        if ((forwardPressed || backwardPressed) && !physicalJogActive) {
            physicalJogActive = true;
            physicalStartMs = millis();

            // Robot-frame convention: forward is +1 and backward is -1.
            physicalDirection = forwardPressed ? 1 : -1;
            activeSource = Source::PHYSICAL;
        }

        if (activeSource == Source::PHYSICAL && physicalJogActive &&
            millis() - physicalStartMs >= CFG.jog.jog_duration_ms) {
            stopAllJog();
        }
    }

    void applyJogOutput() {
        if (activeSource == Source::NONE) {
            return;
        }

        float robotVelocity = 0.0f;

        if (activeSource == Source::REMOTE) {
            /*
            * Remote convention:
            *
            *   +1 = robot forward
            *   -1 = robot backward
            */
            if (
                remoteDirection == 0 ||
                remoteSpeedMs <= 0.0f
            ) {
                stopAllJog();
                return;
            }

            robotVelocity =
                static_cast<float>(
                    remoteDirection
                ) *
                remoteSpeedMs;
        }
        else if (
            activeSource == Source::PHYSICAL
        ) {
            /*
            * Physical-button wiring convention:
            *
            *   forward button  = -1
            *   backward button = +1
            *
            * Invert it here to convert it into the
            * common robot-frame convention:
            *
            *   positive = robot forward
            *   negative = robot backward
            */
            if (
                physicalDirection == 0 ||
                CFG.jog.jog_speed_max_ms <= 0.0f
            ) {
                stopAllJog();
                return;
            }

            robotVelocity =
                -static_cast<float>(
                    physicalDirection
                ) *
                CFG.jog.jog_speed_max_ms;
        }

        if (
            motors.isMotionInhibited() ||
            fabsf(robotVelocity) <
                MotorCP::ZERO_SPEED_THRESHOLD_MS
        ) {
            stopAllJog();
            return;
        }

        /*
        * Straight-line robot-frame command.
        *
        * Both tracks receive the same logical direction.
        * MotorCP::motorDirection[] performs the individual
        * installation-polarity correction.
        */
        motors.setSpeeds(
            robotVelocity,
            robotVelocity
        );
    }
};

#endif
