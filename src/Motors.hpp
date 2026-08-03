#ifndef MOTORS_HPP
#define MOTORS_HPP

#include <Arduino.h>
#include <Basicmicro.h>

extern UART roboclaw_uart_a;
extern UART roboclaw_uart_b;

class Motors {
public:
    // ------------------------------------------------------------
    // RoboClaw configuration
    // ------------------------------------------------------------
    static constexpr uint8_t  ADDR_A = 0x80;
    static constexpr uint8_t  ADDR_B = 0x80;
    static constexpr uint32_t BAUD   = 38400;

    // Motor command update rate.
    uint16_t commandHz = 200;


    static constexpr float   WHEEL_DIAMETER_M = 0.048f;
    static constexpr int32_t ENCODER_CPR      = 4096;

    // Any commanded speed with magnitude below this threshold is treated as zero.
    // 0.0002 m/s = 0.2 mm/s.
    static constexpr float ZERO_SPEED_THRESHOLD_MS = 0.0002f;

    // Safe motor command limits.
    int32_t maxCommandQpps = 2500; //10000 original
    int32_t accelQppsPerSec = 4250;

    uint8_t encoderReadValidMask() const;
    bool allEncoderReadsValid() const;
    bool setRobotRearDistanceMMVerified(
        float mm,
        float toleranceMm = 2.0f,
        uint32_t timeoutMs = 4000,
        uint32_t retryDelayMs = 100
    );

    uint8_t lastEncoderRestoreValidMask() const {
        return lastEncoderRestoreValidMask_;
    }

    uint8_t lastEncoderRestoreWriteAttempts() const {
        return lastEncoderRestoreWriteAttempts_;
    }

    float lastEncoderRestoreMaximumErrorMM() const {
        return lastEncoderRestoreMaximumErrorMM_;
    }

    int32_t brakeDecelQppsPerSec = 8500;

    bool closedLoopStopActive = false;

    static constexpr float DISTANCE_MM_PER_COUNT = 0.0245793145f;

    static const int8_t AXIS_SIGN[4];
    
    int8_t motorDirection[4] = { 1, -1, -1, 1 };

    void setMotorDirection(uint8_t axis, int8_t dir) {
        if (axis >= 4) return;
        motorDirection[axis] = (dir < 0) ? -1 : 1;
    }

    void setMotorDirections(int8_t d0, int8_t d1, int8_t d2, int8_t d3) {
        motorDirection[0] = (d0 < 0) ? -1 : 1;
        motorDirection[1] = (d1 < 0) ? -1 : 1;
        motorDirection[2] = (d2 < 0) ? -1 : 1;
        motorDirection[3] = (d3 < 0) ? -1 : 1;
    }

    float   speeds[4]    = {0, 0, 0, 0};
    int32_t encCounts[4] = {0, 0, 0, 0};
    int32_t qpps[4]      = {0, 0, 0, 0};

    // Encoder/speed polling rate.
    uint16_t pollHz = 50;

    // ------------------------------------------------------------
    // Motion safety inhibit
    // ------------------------------------------------------------
    // When asserted:
    //   - Existing commands are cleared.
    //   - A zero-speed command is sent immediately.
    //   - Future nonzero commands are rejected/gated.
    //
    // When released:
    //   - Speeds remain zero.
    //   - A fresh command is required before motion resumes.
    void setMotionInhibited(bool inhibited);

    bool isMotionInhibited() const {
        return motionInhibited_;
    }

    // ------------------------------------------------------------
    // Construction
    // ------------------------------------------------------------
    Motors();
    Motors(UART* ua, UART* ub);

    // ------------------------------------------------------------
    // Lifecycle / loop
    // ------------------------------------------------------------
    void begin();
    void update();

    // ------------------------------------------------------------
    // Motor control
    // ------------------------------------------------------------
    void setSpeeds(float s0, float s1, float s2, float s3);
    void STOP();
    void BRAKE_STOP();

    // ------------------------------------------------------------
    // Encoder maintenance / polling
    // ------------------------------------------------------------
    void resetEncoders();
    void pollEncoders();

    // ------------------------------------------------------------
    // Calibrated distance helpers
    // ------------------------------------------------------------


    int32_t getNormalizedCounts(uint8_t axis) const;

    // Returns calibrated wheel travel in millimeters for one axis.
    float getWheelMM(uint8_t axis) const;

    // Returns calibrated rear robot travel/radius in millimeters.
    float getRobotRearDistanceMM() const;

    // Returns calibrated rear robot travel/radius in meters.
    float getRobotRearDistanceM() const;

    // Converts calibrated millimeters to normalized encoder counts.
    int32_t mm_to_distance_counts(float mm) const;

    // Physically writes RoboClaw encoder counters so the current robot
    void setRobotRearDistanceMM(float mm);

    // Same as above, but input is meters.
    void setRobotRearDistanceM(float meters);

private:
    // ------------------------------------------------------------
    // RoboClaw instances
    // ------------------------------------------------------------
    Basicmicro rcA;
    Basicmicro rcB;

    // ------------------------------------------------------------
    // Timing
    // ------------------------------------------------------------
    uint32_t lastPollMs = 0;
    uint32_t lastSendMs = 0;

    // ------------------------------------------------------------
    // Internal status/debug bookkeeping
    // ------------------------------------------------------------
    uint8_t encStatus_[4]   = {0, 0, 0, 0};
    uint8_t speedStatus_[4] = {0, 0, 0, 0};

    bool encValid_[4]   = {false, false, false, false};
    bool speedValid_[4] = {false, false, false, false};

    // ------------------------------------------------------------
    // Internal helpers
    // ------------------------------------------------------------
    static uint32_t u32bits(int32_t v);
    volatile bool motionInhibited_ = false;

    uint8_t lastEncoderRestoreValidMask_ = 0;
    uint8_t lastEncoderRestoreWriteAttempts_ = 0;
    float lastEncoderRestoreMaximumErrorMM_ = 0.0f;

    int32_t ms_to_qpps(float ms) const;

    void sendSpeeds();
};

#endif