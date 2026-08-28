#ifndef MOTORS_HPP
#define MOTORS_HPP

#include <Arduino.h>
#include <Basicmicro.h>

extern UART roboclaw_uart_a;
extern UART roboclaw_uart_b;

class Motors {
public:
    static constexpr uint8_t ADDR_A = 0x80;
    static constexpr uint8_t ADDR_B = 0x80;
    static constexpr uint32_t BAUD = 19200;
    static constexpr float WHEEL_DIAMETER_M = 0.048f;
    static constexpr int32_t ENCODER_CPR = 4096;
    static constexpr float ZERO_SPEED_THRESHOLD_MS = 0.0002f;
    static constexpr float DISTANCE_MM_PER_COUNT = 0.0245793145f;

    uint16_t commandHz = 200;
    uint16_t pollHz = 50;
    int32_t maxCommandQpps = 2500;
    int32_t accelQppsPerSec = 4250;
    int32_t brakeDecelQppsPerSec = 8500;
    bool closedLoopStopActive = false;

    int8_t motorDirection[4] = {1, -1, -1, 1};
    int8_t encoderDirection[4] = {1, -1, 1, -1};

    float speeds[4] = {0, 0, 0, 0};
    int32_t encCounts[4] = {0, 0, 0, 0};
    int32_t qpps[4] = {0, 0, 0, 0};

    struct MotorDiagnosticSnapshot {
        int32_t encoderCount = 0;
        int32_t speedQpps = 0;
        int16_t pwm = 0;
        int16_t currentMa = 0;
        uint8_t encoderStatus = 0;
        uint8_t speedStatus = 0;
        uint32_t errorFlags = 0;
        bool encoderValid = false;
        bool speedValid = false;
        bool pwmValid = false;
        bool currentValid = false;
        bool errorValid = false;
    };

    Motors();
    Motors(UART* ua, UART* ub);

    void begin();
    void update();
    void setSpeeds(float s0, float s1, float s2, float s3);
    void STOP();
    void BRAKE_STOP();
    void setMotionInhibited(bool inhibited);
    bool isMotionInhibited() const { return motionInhibited_; }

    void setMotorDirection(uint8_t axis, int8_t dir);
    void setMotorDirections(int8_t d0, int8_t d1, int8_t d2, int8_t d3);
    void setEncoderDirection(uint8_t axis, int8_t dir);
    void setEncoderDirections(int8_t d0, int8_t d1, int8_t d2, int8_t d3);

    void resetEncoders();
    void pollEncoders();
    uint8_t encoderReadValidMask() const;
    bool allEncoderReadsValid() const;

    bool setRobotRearDistanceMMVerified(
        float mm,
        float toleranceMm = 2.0f,
        uint32_t timeoutMs = 4000,
        uint32_t retryDelayMs = 100
    );

    uint8_t lastEncoderRestoreValidMask() const { return lastEncoderRestoreValidMask_; }
    uint8_t lastEncoderRestoreWriteAttempts() const { return lastEncoderRestoreWriteAttempts_; }
    float lastEncoderRestoreMaximumErrorMM() const { return lastEncoderRestoreMaximumErrorMM_; }

    int32_t getNormalizedCounts(uint8_t axis) const;
    float getWheelMM(uint8_t axis) const;
    float getRobotRearDistanceMM() const;
    float getRobotRearDistanceM() const;
    float getRearWheelDisagreementMM() const;
    int32_t mm_to_distance_counts(float mm) const;
    void setRobotRearDistanceMM(float mm);
    void setRobotRearDistanceM(float meters);

    bool readMotorDiagnostic(uint8_t axis, MotorDiagnosticSnapshot& snapshot);
    int8_t expectedEncoderDirection(uint8_t axis, float speedMps) const;
    bool readMotorMotionFeedback(
        uint8_t axis,
        int32_t& encoderCount,
        int32_t& speedQpps,
        uint8_t& encoderStatus,
        uint8_t& speedStatus
    );
    bool readControllerErrorFlags(uint8_t axis, uint32_t& errorFlags);

private:
    Basicmicro rcA;
    Basicmicro rcB;
    uint32_t lastPollMs = 0;
    uint32_t lastSendMs = 0;
    uint8_t encStatus_[4] = {0, 0, 0, 0};
    uint8_t speedStatus_[4] = {0, 0, 0, 0};
    bool encValid_[4] = {false, false, false, false};
    bool speedValid_[4] = {false, false, false, false};
    volatile bool motionInhibited_ = false;
    uint8_t lastEncoderRestoreValidMask_ = 0;
    uint8_t lastEncoderRestoreWriteAttempts_ = 0;
    float lastEncoderRestoreMaximumErrorMM_ = 0.0f;

    static uint32_t u32bits(int32_t v);
    int32_t ms_to_qpps(float ms) const;
    void sendSpeeds();
};

#endif
