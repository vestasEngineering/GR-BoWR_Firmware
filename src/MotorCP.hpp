#ifndef MOTORCP_HPP
#define MOTORCP_HPP

#include <Arduino.h>
#include <mbed.h>
#include "Config.hpp"
#include "SteeringControl.hpp"

// Two Teknic ClearPath motors in step & direction mode.
//
//  * STEP pulses come from stepIsr(), which must be called from a periodic
//    timer interrupt at ISR_HZ. A phase accumulator spreads pulses evenly in
//    time, and a trapezoidal slew limiter (run every 1 ms inside the ISR)
//    keeps the commanded step rate smooth so the motors do not jitter.
//  * HLFB is configured as a 16 PPR output and is counted on rising edges.
//    It carries no direction, so the sign comes from the commanded direction.
//  * Steering (laser-line angle -> speed difference between the tracks) is
//    layered on top of whatever speed the active controller commands.
//
// Axis 0 / axis 1 are the "motor 1" / "motor 2" connectors. Arrays are four
// wide only to keep the MySerial schema unchanged; axes 2 and 3 are ignored.
class MotorCP {
public:
    static constexpr uint8_t  AXES = 4;
    static constexpr uint8_t  MOTORS = 2;
    static constexpr uint32_t ISR_HZ = 100000;
    static constexpr float    ZERO_SPEED_THRESHOLD_MS = 0.0002f;

    float maxSpeedMs;
    float accelMps2;
    float decelMps2;
    float brakeDecelMps2;

    int8_t motorDirection[AXES]   = {1, -1, 1, 1};
    int8_t encoderDirection[AXES] = {1, 1, 1, 1};

    float   speeds[AXES]    = {0, 0, 0, 0};   // commanded wheel speed, m/s
    int32_t encCounts[AXES] = {0, 0, 0, 0};   // raw HLFB pulse counts
    int32_t qpps[AXES]      = {0, 0, 0, 0};   // commanded step rate, steps/s

    SteeringControl steering;

    MotorCP();

    void begin();
    void update();

    // Called from the timer ISR at ISR_HZ.
    static void stepIsr();

    void setSpeeds(float s0, float s1, float s2 = 0.0f, float s3 = 0.0f);
    void STOP();         // immediate: step pulses cease this instant
    void BRAKE_STOP();   // controlled stop at brakeDecelMps2
    void setMotionInhibited(bool inhibited);
    bool isMotionInhibited() const { return motionInhibited_; }
    bool isEnabled() const { return enabled_; }
    bool isReadyToStep() const { return stepAllowed_; }
    bool isStepping() const { return hw_[0].curSps != 0.0f || hw_[1].curSps != 0.0f; }

    void setMotorDirection(uint8_t axis, int8_t dir);
    void setMotorDirections(int8_t d0, int8_t d1, int8_t d2, int8_t d3);
    void setEncoderDirection(uint8_t axis, int8_t dir);
    void setEncoderDirections(int8_t d0, int8_t d1, int8_t d2, int8_t d3);

    void setRampLimits(float maxMs, float accel, float decel, float brakeDecel);

    void resetEncoders();
    void pollEncoders();
    uint8_t encoderReadValidMask() const { return begun_ ? 0x03 : 0x00; }
    bool allEncoderReadsValid() const { return begun_; }

    bool setRobotRearDistanceMMVerified(float mm, float toleranceMm = 2.0f);
    uint8_t lastEncoderRestoreValidMask() const { return lastEncoderRestoreValidMask_; }
    uint8_t lastEncoderRestoreWriteAttempts() const { return lastEncoderRestoreWriteAttempts_; }
    float lastEncoderRestoreMaximumErrorMM() const { return lastEncoderRestoreMaximumErrorMM_; }

    int32_t getNormalizedCounts(uint8_t axis) const;
    float getWheelMM(uint8_t axis) const;
    float getRobotRearDistanceMM() const;
    float getRobotRearDistanceM() const { return getRobotRearDistanceMM() * 0.001f; }
    float getRearWheelDisagreementMM() const;
    int32_t mm_to_distance_counts(float mm) const;
    void setRobotRearDistanceMM(float mm);
    void setRobotRearDistanceM(float meters) { setRobotRearDistanceMM(meters * 1000.0f); }

    bool readMotorMotionFeedback(
        uint8_t axis,
        int32_t& encoderCount,
        int32_t& speedQpps,
        uint8_t& encoderStatus,
        uint8_t& speedStatus
    );

    // Measured (HLFB-derived) wheel speed, m/s, forward positive.
    float measuredSpeedMs(uint8_t axis) const;
    int leftAxis() const { return CFG.motors.axis0_is_left ? 0 : 1; }
    int rightAxis() const { return CFG.motors.axis0_is_left ? 1 : 0; }

private:
    struct Axis {
        GPIO_TypeDef* stepPort = nullptr;
        GPIO_TypeDef* dirPort = nullptr;
        GPIO_TypeDef* enablePort = nullptr;
        uint32_t stepMask = 0;
        uint32_t dirMask = 0;
        uint32_t enableMask = 0;

        // Written by update(), read by the ISR.
        volatile float targetSps = 0.0f;
        volatile int8_t motorDir = 1;

        // Owned by the ISR (main context may touch them only with IRQs off).
        volatile float curSps = 0.0f;
        volatile uint32_t phaseInc = 0;
        uint32_t phase = 0;
        uint8_t pulseTicks = 0;
        bool dirChanged = false;
        int8_t dirLatched = 0;
        volatile int8_t lastSign = 1;

        volatile int32_t hlfbCount = 0;
        volatile uint32_t hlfbLastUs = 0;
    };

    Axis hw_[MOTORS];
    static MotorCP* instance_;

    volatile bool brake_ = false;
    volatile bool stepAllowed_ = false;
    volatile bool motionInhibited_ = true;
    bool enabled_ = false;
    bool begun_ = false;
    uint32_t settleStartMs_ = 0;

    uint8_t rampDiv_ = 0;
    float stepsPerMeter_ = 0.0f;
    float accelSpsPerMs_ = 0.0f;
    float decelSpsPerMs_ = 0.0f;
    float brakeSpsPerMs_ = 0.0f;
    float maxSps_ = 0.0f;

    int32_t lastCounts_[MOTORS] = {0, 0};
    uint32_t lastSpeedMs_ = 0;
    float measuredMs_[MOTORS] = {0.0f, 0.0f};
    float measuredPps_[MOTORS] = {0.0f, 0.0f};

    uint8_t lastEncoderRestoreValidMask_ = 0;
    uint8_t lastEncoderRestoreWriteAttempts_ = 0;
    float lastEncoderRestoreMaximumErrorMM_ = 0.0f;

    void refreshDerived();
    void bindPins();
    void writeEnable(bool asserted);
    void hardZero();
    void applyTargets();
    void isrTick();
    void rampAxes();

    static void hlfbIsr0();
    static void hlfbIsr1();
    static void hlfbEdge(uint8_t axis);
};

#endif
