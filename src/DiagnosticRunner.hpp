#pragma once

#include <Arduino.h>
#include <ArduinoJson.h>
#include "Motors.hpp"
#include "Actuator.hpp"
#include "BatteryMonitor.hpp"
#include "Ultrasonic.hpp"
#include "UltrasonicServo.hpp"
#include "AndonManager.hpp"
#include "EStop.hpp"
#include "ClampSensor.hpp"
#include "JogControl.hpp"

class DiagnosticRunner {
public:
    enum class State : uint8_t {
        Idle,
        MotorForwardRunning,
        MotorForwardSettling,
        MotorReverseRunning,
        MotorReverseSettling,
        ActuatorSettling,
        SensorSampling,
        AndonBlueFirst,
        AndonOff,
        AndonBlueSecond,
        Aborting
    };

    DiagnosticRunner(
        Stream& io,
        Motors& motors,
        ActuatorControl& actuator,
        BatteryMonitor& battery,
        Ultrasonic& ultrasonic,
        UltrasonicServo& servo,
        AndonManager& andon,
        EStop& estop,
        ClampSensor& clamp,
        JogControl& jog,
        bool& processEnabled
    );

    void update();
    bool start(const JsonDocument& command);
    bool abort(const char* runId);
    bool active() const { return state_ != State::Idle; }
    const char* activeRunId() const { return runId_; }

private:
    static constexpr size_t kIdBytes = 48;

    // Simple service motor test settings.
    static constexpr float kMotorTestSpeedMps = 0.02f;
    static constexpr uint32_t kMotorRunMs = 2000;
    static constexpr uint32_t kMotorSettleTimeoutMs = 1200;
    static constexpr uint32_t kMotorSamplePeriodMs = 75;
    static constexpr int32_t kMotorMinimumMovementCounts = 40;
    static constexpr int32_t kMotorReturnToleranceCounts = 150;
    static constexpr int32_t kMotorStoppedQpps = 25;

    static constexpr float kActuatorMaxCommandV = 3.0f;
    static constexpr uint32_t kActuatorMaxSettleMs = 15000;

    Stream& io_;
    Motors& motors_;
    ActuatorControl& actuator_;
    BatteryMonitor& battery_;
    Ultrasonic& ultrasonic_;
    UltrasonicServo& servo_;
    AndonManager& andon_;
    EStop& estop_;
    ClampSensor& clamp_;
    JogControl& jog_;
    bool& processEnabled_;

    State state_ = State::Idle;
    char runId_[kIdBytes] = {0};
    char transactionId_[kIdBytes] = {0};
    char moduleId_[32] = {0};
    char category_[20] = {0};

    uint32_t startedAtMs_ = 0;
    uint32_t phaseStartedAtMs_ = 0;
    uint32_t lastMotorSampleMs_ = 0;
    uint8_t axisOrChannel_ = 0;
    float commandValue_ = 0.0f;
    float tolerance_ = 0.0f;
    uint32_t durationMs_ = 0;
    bool terminalSent_ = false;

    int32_t initialCount_ = 0;
    int32_t forwardCount_ = 0;
    int32_t finalCount_ = 0;
    int32_t latestEncoderCount_ = 0;
    int32_t latestSpeedQpps_ = 0;
    bool latestFeedbackValid_ = false;
    uint16_t feedbackSamples_ = 0;
    uint16_t feedbackFailures_ = 0;
    uint32_t controllerFlags_ = 0;
    bool controllerFlagsValid_ = false;

    bool preflight(bool motionProducing, const char*& reason) const;
    bool copyIdentity(const JsonDocument& command);
    bool startMotor(const JsonDocument& command);
    bool startActuator(const JsonDocument& command);
    bool startSensor(const JsonDocument& command);
    bool startAndon(const JsonDocument& command);

    void commandSelectedMotor(float speedMps);
    bool sampleMotorFeedback(uint32_t now, bool force = false);
    void updateMotor(uint32_t now);
    void finishMotorDiagnostic(const char* forcedReason = nullptr);
    const char* evaluateMotor(bool& passed, const char*& recommendedAction) const;
    void addControllerInformation(JsonObject measurements) const;

    void updateActuator(uint32_t now);
    void updateSensor();
    void updateAndon(uint32_t now);
    void stopOwnedOutputs();
    void clearRun();

    void emitRejected(const char* reason);
    void emitRejectedCommand(const JsonDocument& command, const char* reason);
    void emitTerminal(bool passed, const char* reason, JsonObjectConst measurements);
    void emitSimpleTerminal(bool passed, const char* reason);
};
