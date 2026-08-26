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
        ActuatorExtending,
        ActuatorExtendedHold,
        ActuatorAwaitingExtendedConfirmation,
        ActuatorRetracting,
        ActuatorRetractedHold,
        SensorSampling,
        ClampAwaitingOpenConfirmation,
        ClampAwaitingClosedConfirmation,
        AndonAwaitingGreenConfirmation,
        AndonAwaitingYellowConfirmation,
        AndonAwaitingBlueConfirmation,
        AndonAwaitingRedConfirmation,
        Aborting
    };

    DiagnosticRunner(Stream& io, Motors& motors, ActuatorControl& actuator,
        BatteryMonitor& battery, Ultrasonic& ultrasonic, UltrasonicServo& servo,
        AndonManager& andon, EStop& estop, ClampSensor& clamp, JogControl& jog,
        bool& processEnabled);

    void update();
    bool start(const JsonDocument& command);
    bool abort(const char* runId);
    bool active() const { return state_ != State::Idle; }
    const char* activeRunId() const { return runId_; }
    bool ownsActuatorOutputs() const;
    bool confirmActuatorExtension(const JsonDocument& command);
    bool confirmClampState(const JsonDocument& command);
    bool confirmAndonColor(const JsonDocument& command);


private:
    static constexpr size_t kIdBytes = 48;
    static constexpr float kMotorTestSpeedMps = 0.02f;
    static constexpr uint32_t kMotorRunMs = 2000;
    static constexpr uint32_t kMotorSettleTimeoutMs = 1200;
    static constexpr uint32_t kMotorSamplePeriodMs = 75;
    static constexpr int32_t kMotorMinimumMovementCounts = 40;
    static constexpr int32_t kMotorReturnToleranceCounts = 150;
    static constexpr int32_t kMotorStoppedQpps = 25;

    static constexpr float kActuatorMaxCommandV = 3.0f;
    static constexpr uint32_t kActuatorPhaseTimeoutMs = 15000;
    static constexpr uint32_t kActuatorMinimumCalibrationDriveMs = 7000;
    static constexpr uint32_t kActuatorStableMs = 600;
    static constexpr uint32_t kActuatorHoldMs = 300;
    static constexpr float kCalibrationPlateauDeltaV = 0.035f;
    static constexpr float kMinimumFeedbackSpanV = 0.50f;

    Stream& io_; Motors& motors_; ActuatorControl& actuator_;
    BatteryMonitor& battery_; Ultrasonic& ultrasonic_; UltrasonicServo& servo_;
    AndonManager& andon_; EStop& estop_; ClampSensor& clamp_; JogControl& jog_;
    bool& processEnabled_;

    State state_ = State::Idle;
    char runId_[kIdBytes] = {0}; char transactionId_[kIdBytes] = {0};
    char moduleId_[32] = {0}; char category_[20] = {0};
    uint32_t startedAtMs_ = 0, phaseStartedAtMs_ = 0, lastMotorSampleMs_ = 0;
    uint8_t axisOrChannel_ = 0; float commandValue_ = 0.0f, tolerance_ = 0.0f;
    uint32_t durationMs_ = 0; bool terminalSent_ = false;

    int32_t initialCount_ = 0, forwardCount_ = 0, finalCount_ = 0;
    int32_t latestEncoderCount_ = 0, latestSpeedQpps_ = 0;
    bool latestFeedbackValid_ = false; uint16_t feedbackSamples_ = 0, feedbackFailures_ = 0;
    uint32_t controllerFlags_ = 0; bool controllerFlagsValid_ = false;

    bool actuatorCalibration_ = false;
    float extendedTargetV_ = 0.0f, retractedTargetV_ = 0.0f;
    float extendedToleranceV_ = 0.0f, retractedToleranceV_ = 0.0f;
    float lastActuatorFeedbackV_ = 0.0f, extendedFeedbackV_ = 0.0f, retractedFeedbackV_ = 0.0f;
    float extendedMinV_ = 99.0f, extendedMaxV_ = -99.0f, retractedMinV_ = 99.0f, retractedMaxV_ = -99.0f;
    uint32_t stableSinceMs_ = 0, extensionTimeMs_ = 0, retractionTimeMs_ = 0;
    uint16_t actuatorSamples_ = 0;

    static constexpr uint32_t kActuatorConfirmationTimeoutMs = 15000;
    static constexpr size_t kActuatorFailureReasonBytes = 48;
    uint32_t confirmationStartedAtMs_ = 0;
    char pendingActuatorFailureReason_[kActuatorFailureReasonBytes] = {0};
    void emitActuatorProgress(const char* phase);

    void emitProgress(
        const char* phase,
        const char* reason = nullptr
    );

    bool preflight(bool motionProducing, const char*& reason) const;
    bool copyIdentity(const JsonDocument& command);
    bool startMotor(const JsonDocument& command); bool startActuator(const JsonDocument& command);
    bool startSensor(const JsonDocument& command); bool startAndon(const JsonDocument& command);
    void commandSelectedMotor(float speedMps); bool sampleMotorFeedback(uint32_t now, bool force=false);
    void updateMotor(uint32_t now); void finishMotorDiagnostic(const char* forcedReason=nullptr);
    const char* evaluateMotor(bool& passed, const char*& recommendedAction) const;
    void addControllerInformation(JsonObject measurements) const;
    void updateActuator(uint32_t now); void commandActuator(float voltage);
    bool actuatorFeedbackStable(uint32_t now, float target, float tolerance, bool calibration);
    void finishActuatorDiagnostic(bool passed, const char* reason);
    void updateSensor(); void updateAndon(uint32_t now); void stopOwnedOutputs(); void clearRun();
    void emitRejected(const char* reason); void emitRejectedCommand(const JsonDocument& command,const char* reason);
    void emitTerminal(bool passed,const char* reason,JsonObjectConst measurements);
    void emitSimpleTerminal(bool passed,const char* reason);

    static constexpr uint32_t kGuidedConfirmationTimeoutMs = 30000;
    static constexpr uint8_t kAndonGreenBit = 1U << 0;
    static constexpr uint8_t kAndonYellowBit = 1U << 1;
    static constexpr uint8_t kAndonBlueBit = 1U << 2;
    static constexpr uint8_t kAndonRedBit = 1U << 3;
    uint32_t guidedConfirmationStartedAtMs_ = 0;
    bool clampOpenReading_ = false;
    bool clampClosedReading_ = false;
    bool clampOpenRaw_ = false;
    bool clampClosedRaw_ = false;
    uint8_t andonConfirmedMask_ = 0;
    uint8_t andonFailedMask_ = 0;
    void emitClampProgress(const char* phase, bool expectedClamped);
    void showAndonStep(State state, AndonLight::States color, const char* colorName,
                    uint8_t step);
    void finishClampDiagnostic();
    void finishAndonDiagnostic();
};
