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
        MotorRunning,
        MotorSettling,
        ActuatorCommanded,
        ActuatorSettling,
        SensorSampling,
        ServoActivating,
        ServoSampling,
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
    static constexpr float kMotorMaxSpeedMps = 0.02f;
    static constexpr uint32_t kMotorMaxDurationMs = 1500;
    static constexpr uint32_t kMotorSettleMs = 150;
    static constexpr int32_t kMotorMinimumCounts = 40;
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
    uint8_t axisOrChannel_ = 0;
    float commandValue_ = 0.0f;
    float tolerance_ = 0.0f;
    uint32_t durationMs_ = 0;
    int32_t initialCount_ = 0;
    bool terminalSent_ = false;

    bool preflight(bool motionProducing, const char*& reason) const;
    bool copyIdentity(const JsonDocument& command);
    bool startMotor(const JsonDocument& command);
    bool startActuator(const JsonDocument& command);
    bool startSensor(const JsonDocument& command);
    bool startServo(const JsonDocument& command);
    void updateMotor(uint32_t now);
    void updateActuator(uint32_t now);
    void updateSensor();
    void updateServo(uint32_t now);
    void stopOwnedOutputs();
    void clearRun();

    void emitRejected(const char* reason);
    void emitRejectedCommand(const JsonDocument& command, const char* reason);
    void emitTerminal(bool passed, const char* reason, JsonObjectConst measurements);
    void emitSimpleTerminal(bool passed, const char* reason);
};
