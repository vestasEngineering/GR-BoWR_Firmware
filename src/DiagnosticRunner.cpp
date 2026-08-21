#include "DiagnosticRunner.hpp"
#include <math.h>
#include <string.h>

DiagnosticRunner::DiagnosticRunner(
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
) : io_(io), motors_(motors), actuator_(actuator), battery_(battery),
    ultrasonic_(ultrasonic), servo_(servo), andon_(andon), estop_(estop),
    clamp_(clamp), jog_(jog), processEnabled_(processEnabled) {}

bool DiagnosticRunner::copyIdentity(const JsonDocument& command) {
    const char* run = command["run_id"] | "";
    const char* tx = command["transaction_id"] | "";
    const char* module = command["id"] | "";
    if (!run[0] || !tx[0] || !module[0]) return false;
    strlcpy(runId_, run, sizeof(runId_));
    strlcpy(transactionId_, tx, sizeof(transactionId_));
    strlcpy(moduleId_, module, sizeof(moduleId_));
    return true;
}

bool DiagnosticRunner::preflight(bool motionProducing, const char*& reason) const {
    if (active()) { reason = "diagnostic_already_active"; return false; }
    if (processEnabled_) { reason = "process_must_be_stopped"; return false; }
    if (jog_.isActive()) { reason = "jog_must_be_stopped"; return false; }
    if (motionProducing) {
        if (estop_.isActive()) { reason = "estop_active"; return false; }
        if (!clamp_.isClamped()) { reason = "clamp_not_confirmed"; return false; }
        if (motors_.isMotionInhibited()) { reason = "motion_inhibited"; return false; }
    }
    return true;
}

bool DiagnosticRunner::start(const JsonDocument& command) {
    const char* action = command["action"] | "";
    const bool motion = strcasecmp(action, "test_motor") == 0 ||
                        strcasecmp(action, "test_actuator") == 0 ||
                        strcasecmp(action, "test_servo") == 0;
    if (active()) {
        emitRejectedCommand(command, "diagnostic_already_active");
        return false;
    }
    if (!copyIdentity(command)) {
        emitRejectedCommand(command, "missing_diagnostic_identity");
        clearRun();
        return false;
    }
    const char* reason = nullptr;
    if (!preflight(motion, reason)) {
        emitRejected(reason);
        clearRun();
        return false;
    }
    startedAtMs_ = millis();
    phaseStartedAtMs_ = startedAtMs_;
    terminalSent_ = false;

    if (strcasecmp(action, "test_motor") == 0) return startMotor(command);
    if (strcasecmp(action, "test_actuator") == 0) return startActuator(command);
    if (strcasecmp(action, "test_sensor") == 0) return startSensor(command);
    if (strcasecmp(action, "test_servo") == 0) return startServo(command);

    emitRejected("unsupported_diagnostic_action");
    clearRun();
    return false;
}

bool DiagnosticRunner::startMotor(const JsonDocument& command) {
    const int axis = command["index"] | -1;
    float speed = command["speed"] | kMotorMaxSpeedMps;
    uint32_t duration = command["duration_ms"] | 600;
    if (axis < 0 || axis > 3) { emitRejected("invalid_motor_axis"); clearRun(); return false; }
    if (!isfinite(speed) || speed == 0.0f) { emitRejected("invalid_motor_speed"); clearRun(); return false; }
    speed = constrain(speed, -kMotorMaxSpeedMps, kMotorMaxSpeedMps);
    duration = constrain(duration, static_cast<uint32_t>(100), kMotorMaxDurationMs);
    axisOrChannel_ = static_cast<uint8_t>(axis);
    commandValue_ = speed;
    durationMs_ = duration;
    initialCount_ = motors_.encCounts[axisOrChannel_];
    strlcpy(category_, "motor", sizeof(category_));

    float commands[4] = {0, 0, 0, 0};
    commands[axisOrChannel_] = commandValue_;
    motors_.setSpeeds(commands[0], commands[1], commands[2], commands[3]);
    state_ = State::MotorRunning;
    return true;
}

bool DiagnosticRunner::startActuator(const JsonDocument& command) {
    const int channel = command["channel"] | -1;
    float voltage = command["voltage"] | 3.0f;
    float tolerance = command["tolerance"] | 0.8f;
    uint32_t settle = command["settle_ms"] | 1000;
    if (channel < 0 || channel >= NUM_ACTUATORS) { emitRejected("invalid_actuator_channel"); clearRun(); return false; }
    if (actuator_.hasPCBFault()) { emitRejected("actuator_pcb_fault"); clearRun(); return false; }
    if (!isfinite(voltage) || voltage < 0.0f || voltage > kActuatorMaxCommandV) { emitRejected("invalid_actuator_voltage"); clearRun(); return false; }
    if (!isfinite(tolerance) || tolerance < 0.0f) { emitRejected("invalid_actuator_tolerance"); clearRun(); return false; }
    axisOrChannel_ = static_cast<uint8_t>(channel);
    commandValue_ = voltage;
    tolerance_ = tolerance;
    durationMs_ = constrain(settle, static_cast<uint32_t>(100), kActuatorMaxSettleMs);
    strlcpy(category_, "actuator", sizeof(category_));
    actuator_.actuatorPositions[axisOrChannel_] = commandValue_;
    actuator_.writeDAC(axisOrChannel_, commandValue_);
    state_ = State::ActuatorSettling;
    return true;
}

bool DiagnosticRunner::startSensor(const JsonDocument& command) {
    const char* sensor = command["sensor"] | "";
    strlcpy(category_, "sensor", sizeof(category_));
    if (strcasecmp(sensor, "battery") == 0 || strcasecmp(sensor, "ultrasonic") == 0) {
        strlcpy(category_, sensor, sizeof(category_));
        state_ = State::SensorSampling;
        return true;
    }
    emitRejected("unsupported_sensor_test");
    clearRun();
    return false;
}

bool DiagnosticRunner::startServo(const JsonDocument&) {
    strlcpy(category_, "servo", sizeof(category_));
    servo_.activate();
    state_ = State::ServoActivating;
    return true;
}

void DiagnosticRunner::update() {
    if (!active()) return;
    if (estop_.isActive() || motors_.isMotionInhibited()) {
        stopOwnedOutputs();
        emitSimpleTerminal(false, "safety_inhibit_activated");
        clearRun();
        return;
    }
    const uint32_t now = millis();
    switch (state_) {
        case State::MotorRunning: updateMotor(now); break;
        case State::MotorSettling: updateMotor(now); break;
        case State::ActuatorSettling: updateActuator(now); break;
        case State::SensorSampling: updateSensor(); break;
        case State::ServoActivating:
        case State::ServoSampling: updateServo(now); break;
        case State::Aborting:
            stopOwnedOutputs(); emitSimpleTerminal(false, "operator_aborted"); clearRun(); break;
        default: break;
    }
}

void DiagnosticRunner::updateMotor(uint32_t now) {
    if (state_ == State::MotorRunning && now - phaseStartedAtMs_ >= durationMs_) {
        motors_.BRAKE_STOP();
        phaseStartedAtMs_ = now;
        state_ = State::MotorSettling;
        return;
    }
    if (state_ != State::MotorSettling || now - phaseStartedAtMs_ < kMotorSettleMs) return;
    const int32_t finalCount = motors_.encCounts[axisOrChannel_];
    const int32_t delta = finalCount - initialCount_;
    StaticJsonDocument<224> measurements;
    measurements["axis"] = axisOrChannel_;
    measurements["initial_count"] = initialCount_;
    measurements["final_count"] = finalCount;
    measurements["delta_count"] = delta;
    measurements["qpps"] = motors_.qpps[axisOrChannel_];
    measurements["speed_command_mps"] = commandValue_;
    measurements["duration_ms"] = durationMs_;
    const bool passed = abs(delta) >= kMotorMinimumCounts;
    emitTerminal(passed, passed ? "axis_encoder_movement_detected" : "axis_encoder_movement_not_detected", measurements.as<JsonObjectConst>());
    clearRun();
}

void DiagnosticRunner::updateActuator(uint32_t now) {
    if (now - phaseStartedAtMs_ < durationMs_) return;
    const float feedback = actuator_.readADC_Avg(axisOrChannel_, 5, 3);
    actuator_.feedbackSignals[axisOrChannel_] = feedback;
    const float expected = actuator_.expectedFeedbackMapped(axisOrChannel_, commandValue_);
    const float error = feedback - expected;
    StaticJsonDocument<224> measurements;
    measurements["channel"] = axisOrChannel_;
    measurements["command_V"] = commandValue_;
    measurements["expected_V"] = expected;
    measurements["feedback_V"] = feedback;
    measurements["error_V"] = error;
    measurements["settle_ms"] = durationMs_;
    const bool passed = !actuator_.hasPCBFault() && fabsf(error) <= tolerance_;
    actuator_.actuatorPositions[axisOrChannel_] = 0.0f;
    actuator_.writeDAC(axisOrChannel_, 0.0f);
    emitTerminal(passed, passed ? "within_tolerance" : "feedback_out_of_tolerance", measurements.as<JsonObjectConst>());
    clearRun();
}

void DiagnosticRunner::updateSensor() {
    StaticJsonDocument<224> measurements;
    bool passed = false;
    const char* reason = "sensor_out_of_range";
    if (strcasecmp(category_, "battery") == 0) {
        battery_.readBatteryVoltage();
        const float voltage = battery_.voltage;
        float pct = (voltage - MIN_BATTERY_VOLTAGE) /
                    (MAX_BATTERY_VOLTAGE - MIN_BATTERY_VOLTAGE) * 100.0f;
        pct = constrain(pct, 0.0f, 100.0f);
        measurements["voltage_V"] = voltage;
        measurements["pct"] = pct;
        passed = voltage >= MIN_BATTERY_VOLTAGE && voltage <= MAX_BATTERY_VOLTAGE;
        reason = passed ? "voltage_in_range" : "voltage_out_of_range";
    } else {
        const int adc = analogRead(CFG.ultrasonic.analog_pin);
        const float voltage = static_cast<float>(adc) * 3.1f / 1023.0f;
        const float distance = voltage * CFG.ultrasonic.mm_per_volt + CFG.ultrasonic.offset_mm;
        measurements["adc"] = adc;
        measurements["voltage_V"] = voltage;
        measurements["distance_mm"] = distance;
        passed = distance >= CFG.ultrasonic.valid_min_mm && distance <= CFG.ultrasonic.valid_max_mm;
        reason = passed ? "plausible_reading" : "distance_out_of_range";
    }
    emitTerminal(passed, reason, measurements.as<JsonObjectConst>());
    clearRun();
}

void DiagnosticRunner::updateServo(uint32_t now) {
    if (state_ == State::ServoActivating) {
        if (now - phaseStartedAtMs_ < 1000) return;
        phaseStartedAtMs_ = now;
        state_ = State::ServoSampling;
        return;
    }
    ultrasonic_.stateMachine();
    if (now - phaseStartedAtMs_ < 1000) return;
    const float distance = ultrasonic_.distance[0] > 0.0f ? ultrasonic_.distance[0] : ultrasonic_.measuredDistance;
    StaticJsonDocument<128> measurements;
    measurements["distance_mm"] = distance;
    const bool passed = isfinite(distance) && distance >= CFG.andonMgr.ut_ok_min_distance;
    servo_.deactivate();
    emitTerminal(passed, passed ? "distance_ok" : "distance_too_close_or_invalid", measurements.as<JsonObjectConst>());
    clearRun();
}

bool DiagnosticRunner::abort(const char* runId) {
    if (!active() || !runId || strcmp(runId, runId_) != 0) return false;
    state_ = State::Aborting;
    return true;
}

void DiagnosticRunner::stopOwnedOutputs() {
    motors_.BRAKE_STOP();
    if (axisOrChannel_ < NUM_ACTUATORS && strcasecmp(category_, "actuator") == 0) {
        actuator_.actuatorPositions[axisOrChannel_] = 0.0f;
        actuator_.writeDAC(axisOrChannel_, 0.0f);
    }
    servo_.deactivate();
}

void DiagnosticRunner::emitRejectedCommand(const JsonDocument& command, const char* reason) {
    StaticJsonDocument<320> doc;
    doc["type"] = "test_result";
    doc["id"] = command["id"] | "";
    doc["category"] = "diagnostic";
    doc["run_id"] = command["run_id"] | "";
    doc["transaction_id"] = command["transaction_id"] | "";
    doc["pass"] = false;
    doc["reason"] = reason;
    doc.createNestedObject("measurements");
    serializeJson(doc, io_);
    io_.println();
}

void DiagnosticRunner::emitRejected(const char* reason) {
    StaticJsonDocument<256> measurements;
    emitTerminal(false, reason, measurements.as<JsonObjectConst>());
}

void DiagnosticRunner::emitSimpleTerminal(bool passed, const char* reason) {
    StaticJsonDocument<128> measurements;
    emitTerminal(passed, reason, measurements.as<JsonObjectConst>());
}

void DiagnosticRunner::emitTerminal(bool passed, const char* reason, JsonObjectConst measurements) {
    if (terminalSent_) return;
    terminalSent_ = true;
    StaticJsonDocument<512> doc;
    doc["type"] = "test_result";
    doc["id"] = moduleId_;
    doc["category"] = category_;
    doc["run_id"] = runId_;
    doc["transaction_id"] = transactionId_;
    doc["pass"] = passed;
    doc["reason"] = reason;
    doc["started_at_ms"] = startedAtMs_;
    doc["completed_at_ms"] = millis();
    JsonObject out = doc.createNestedObject("measurements");
    for (JsonPairConst pair : measurements) out[pair.key()] = pair.value();
    serializeJson(doc, io_);
    io_.println();
}

void DiagnosticRunner::clearRun() {
    stopOwnedOutputs();
    state_ = State::Idle;
    runId_[0] = '\0';
    transactionId_[0] = '\0';
    moduleId_[0] = '\0';
    category_[0] = '\0';
    axisOrChannel_ = 0;
    commandValue_ = 0.0f;
    tolerance_ = 0.0f;
    durationMs_ = 0;
    initialCount_ = 0;
    terminalSent_ = false;
}
