#include "DiagnosticRunner.hpp"
#include <math.h>
#include <string.h>

static int32_t safeAbs32(int32_t value) {
    return value == INT32_MIN ? INT32_MAX : abs(value);
}

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
) : io_(io),
    motors_(motors),
    actuator_(actuator),
    battery_(battery),
    ultrasonic_(ultrasonic),
    servo_(servo),
    andon_(andon),
    estop_(estop),
    clamp_(clamp),
    jog_(jog),
    processEnabled_(processEnabled) {}

bool DiagnosticRunner::copyIdentity(const JsonDocument& command) {
    const char* run = command["run_id"] | "";
    const char* transaction = command["transaction_id"] | "";
    const char* module = command["id"] | "";

    if (!run[0] || !transaction[0] || !module[0]) {
        return false;
    }

    strlcpy(runId_, run, sizeof(runId_));
    strlcpy(transactionId_, transaction, sizeof(transactionId_));
    strlcpy(moduleId_, module, sizeof(moduleId_));
    return true;
}

bool DiagnosticRunner::preflight(
    bool motionProducing,
    const char*& reason
) const {
    if (active()) {
        reason = "diagnostic_already_active";
        return false;
    }
    if (processEnabled_) {
        reason = "process_must_be_stopped";
        return false;
    }
    if (jog_.isActive()) {
        reason = "jog_must_be_stopped";
        return false;
    }

    if (motionProducing) {
        if (estop_.isActive()) {
            reason = "estop_active";
            return false;
        }
        if (!clamp_.isClamped()) {
            reason = "clamp_not_confirmed";
            return false;
        }
        if (motors_.isMotionInhibited()) {
            reason = "motion_inhibited";
            return false;
        }
    }

    return true;
}

bool DiagnosticRunner::start(const JsonDocument& command) {
    const char* action = command["action"] | "";
    const bool motionProducing =
        strcasecmp(action, "test_motor") == 0 ||
        strcasecmp(action, "test_actuator") == 0;

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
    if (!preflight(motionProducing, reason)) {
        emitRejected(reason);
        clearRun();
        return false;
    }

    startedAtMs_ = millis();
    phaseStartedAtMs_ = startedAtMs_;
    terminalSent_ = false;

    if (strcasecmp(action, "test_motor") == 0) {
        return startMotor(command);
    }
    if (strcasecmp(action, "test_actuator") == 0) {
        return startActuator(command);
    }
    if (strcasecmp(action, "test_sensor") == 0) {
        return startSensor(command);
    }
    if (strcasecmp(action, "test_andon") == 0) {
        return startAndon(command);
    }

    emitRejected("unsupported_diagnostic_action");
    clearRun();
    return false;
}

bool DiagnosticRunner::startMotor(const JsonDocument& command) {
    const int axis = command["index"] | -1;

    if (axis < 0 || axis > 3) {
        emitRejected("invalid_motor_axis");
        clearRun();
        return false;
    }

    axisOrChannel_ = static_cast<uint8_t>(axis);
    commandValue_ = kMotorTestSpeedMps;
    strlcpy(category_, "motor", sizeof(category_));

    motors_.BRAKE_STOP();

    initialCount_ = 0;
    forwardCount_ = 0;
    finalCount_ = 0;
    latestEncoderCount_ = 0;
    latestSpeedQpps_ = 0;
    latestFeedbackValid_ = false;
    feedbackSamples_ = 0;
    feedbackFailures_ = 0;
    controllerFlags_ = 0;
    controllerFlagsValid_ = false;
    lastMotorSampleMs_ = 0;

    state_ = State::MotorForwardRunning;

    // The initial position must be a fresh, direct controller reading. Do not
    // use Motors::encCounts because the normal poll cache may lag the test.
    if (!sampleMotorFeedback(millis(), true)) {
        finishMotorDiagnostic("motor_feedback_invalid");
        return false;
    }

    initialCount_ = latestEncoderCount_;
    forwardCount_ = initialCount_;
    finalCount_ = initialCount_;

    commandSelectedMotor(kMotorTestSpeedMps);
    phaseStartedAtMs_ = millis();
    return true;
}

void DiagnosticRunner::commandSelectedMotor(float speedMps) {
    float commands[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    commands[axisOrChannel_] = speedMps;
    motors_.setSpeeds(commands[0], commands[1], commands[2], commands[3]);
}

bool DiagnosticRunner::sampleMotorFeedback(uint32_t now, bool force) {
    if (!force && now - lastMotorSampleMs_ < kMotorSamplePeriodMs) {
        return latestFeedbackValid_;
    }

    lastMotorSampleMs_ = now;
    ++feedbackSamples_;

    int32_t encoderCount = 0;
    int32_t speedQpps = 0;
    uint8_t encoderStatus = 0;
    uint8_t speedStatus = 0;

    const bool valid = motors_.readMotorMotionFeedback(
        axisOrChannel_,
        encoderCount,
        speedQpps,
        encoderStatus,
        speedStatus
    );

    latestFeedbackValid_ = valid;
    if (!valid) {
        ++feedbackFailures_;
        return false;
    }

    latestSpeedQpps_ = speedQpps;
    latestEncoderCount_ = encoderCount;

    uint32_t flags = 0;
    if (motors_.readControllerErrorFlags(axisOrChannel_, flags)) {
        controllerFlags_ |= flags;
        controllerFlagsValid_ = true;
    }

    return true;
}

void DiagnosticRunner::updateMotor(uint32_t now) {
    sampleMotorFeedback(now);

    if (state_ == State::MotorForwardRunning) {
        if (now - phaseStartedAtMs_ < kMotorRunMs) {
            return;
        }

        motors_.BRAKE_STOP();
        if (latestFeedbackValid_) {
            forwardCount_ = latestEncoderCount_;
        }
        phaseStartedAtMs_ = now;
        state_ = State::MotorForwardSettling;
        return;
    }

    if (state_ == State::MotorForwardSettling) {
        const bool stopped =
            latestFeedbackValid_ &&
            safeAbs32(latestSpeedQpps_) <= kMotorStoppedQpps;

        if (stopped && now - phaseStartedAtMs_ >= 150) {
            forwardCount_ = latestEncoderCount_;
            commandSelectedMotor(-kMotorTestSpeedMps);
            phaseStartedAtMs_ = now;
            state_ = State::MotorReverseRunning;
            return;
        }

        if (now - phaseStartedAtMs_ >= kMotorSettleTimeoutMs) {
            finishMotorDiagnostic("motor_did_not_stop");
        }
        return;
    }

    if (state_ == State::MotorReverseRunning) {
        if (now - phaseStartedAtMs_ < kMotorRunMs) {
            return;
        }

        motors_.BRAKE_STOP();
        if (latestFeedbackValid_) {
            finalCount_ = latestEncoderCount_;
        }
        phaseStartedAtMs_ = now;
        state_ = State::MotorReverseSettling;
        return;
    }

    if (state_ == State::MotorReverseSettling) {
        const bool stopped =
            latestFeedbackValid_ &&
            safeAbs32(latestSpeedQpps_) <= kMotorStoppedQpps;

        if (stopped && now - phaseStartedAtMs_ >= 150) {
            finalCount_ = latestEncoderCount_;
            finishMotorDiagnostic();
            return;
        }

        if (now - phaseStartedAtMs_ >= kMotorSettleTimeoutMs) {
            finishMotorDiagnostic("motor_did_not_stop");
        }
    }
}

const char* DiagnosticRunner::evaluateMotor(
    bool& passed,
    const char*& recommendedAction
) const {
    passed = false;

    if (!latestFeedbackValid_ || feedbackFailures_ > feedbackSamples_ / 2) {
        recommendedAction =
            "Check RoboClaw communication and the encoder connection for this motor.";
        return "motor_feedback_invalid";
    }

    const int32_t forwardDelta = forwardCount_ - initialCount_;
    const int32_t reverseDelta = finalCount_ - forwardCount_;
    const int32_t returnError = finalCount_ - initialCount_;

    if (safeAbs32(forwardDelta) < kMotorMinimumMovementCounts) {
        recommendedAction =
            "Check the motor connector, encoder connector, and mechanical load.";
        return "no_forward_movement";
    }

    if (safeAbs32(reverseDelta) < kMotorMinimumMovementCounts) {
        recommendedAction =
            "Check reverse-direction operation, motor wiring, encoder wiring, and mechanical load.";
        return "no_reverse_movement";
    }

    if ((forwardDelta > 0) == (reverseDelta > 0)) {
        recommendedAction =
            "Check the configured motor direction and encoder direction for this axis.";
        return "motor_direction_invalid";
    }

    if (safeAbs32(returnError) > kMotorReturnToleranceCounts) {
        recommendedAction =
            "Check for wheel or coupling slip, excessive backlash, brake drag, or an encoder mounting issue.";
        return "return_position_out_of_tolerance";
    }

    passed = true;
    recommendedAction =
        "No service action is required. The motor moved in both directions and returned near its starting position.";
    return "motor_test_passed";
}

void DiagnosticRunner::addControllerInformation(JsonObject measurements) const {
    measurements["controller_error_flags"] = controllerFlags_;
    measurements["controller_error_valid"] = controllerFlagsValid_;

    JsonArray conditions = measurements.createNestedArray("controller_conditions");
    const uint32_t flags = controllerFlags_;

    if (flags & Basicmicro::ERROR_ESTOP) conditions.add("estop_active");
    if (flags & Basicmicro::ERROR_TEMP) conditions.add("temperature_1_error");
    if (flags & Basicmicro::ERROR_TEMP2) conditions.add("temperature_2_error");
    if (flags & Basicmicro::ERROR_LBATHIGH) conditions.add("logic_battery_high");
    if (flags & Basicmicro::ERROR_LBATLOW) conditions.add("logic_battery_low");
    if (flags & Basicmicro::ERROR_SPEED1) conditions.add("motor_1_speed_error");
    if (flags & Basicmicro::ERROR_SPEED2) conditions.add("motor_2_speed_error");
    if (flags & Basicmicro::ERROR_POS1) conditions.add("motor_1_position_error");
    if (flags & Basicmicro::ERROR_POS2) conditions.add("motor_2_position_error");
    if (flags & Basicmicro::ERROR_CURRENTM1) conditions.add("motor_1_current_limited");
    if (flags & Basicmicro::ERROR_CURRENTM2) conditions.add("motor_2_current_limited");
    if (flags & Basicmicro::WARN_OVERCURRENTM1) conditions.add("motor_1_overcurrent_warning");
    if (flags & Basicmicro::WARN_OVERCURRENTM2) conditions.add("motor_2_overcurrent_warning");
    if (flags & Basicmicro::WARN_MBATHIGH) conditions.add("main_battery_high_warning");
    if (flags & Basicmicro::WARN_MBATLOW) conditions.add("main_battery_low_warning");
    if (flags & Basicmicro::WARN_TEMP) conditions.add("temperature_1_warning");
    if (flags & Basicmicro::WARN_TEMP2) conditions.add("temperature_2_warning");
    if (flags & Basicmicro::WARN_BOOT) conditions.add("boot_warning");
    if (flags & Basicmicro::WARN_OVERREGENM1) conditions.add("motor_1_regen_warning");
    if (flags & Basicmicro::WARN_OVERREGENM2) conditions.add("motor_2_regen_warning");

    // S4 and S5 are unconnected. They are trace data only and never affect pass/fail.
    measurements["s4_ignored_unconnected"] =
        (flags & Basicmicro::WARN_S4) != 0;
    measurements["s5_ignored_unconnected"] =
        (flags & Basicmicro::WARN_S5) != 0;
}

void DiagnosticRunner::finishMotorDiagnostic(const char* forcedReason) {
    motors_.BRAKE_STOP();
    sampleMotorFeedback(millis(), true);

    if (state_ == State::MotorForwardRunning ||
        state_ == State::MotorForwardSettling) {
        if (latestFeedbackValid_) {
            forwardCount_ = latestEncoderCount_;
        }
        finalCount_ = forwardCount_;
    } else if (latestFeedbackValid_) {
        finalCount_ = latestEncoderCount_;
    }

    bool passed = false;
    const char* recommendedAction = "Inspect the motor feedback and try again.";
    const char* reason = forcedReason
        ? forcedReason
        : evaluateMotor(passed, recommendedAction);

    if (forcedReason && strcmp(forcedReason, "motor_did_not_stop") == 0) {
        recommendedAction =
            "Keep the robot isolated and inspect stopping behavior, brake drag, and controller response.";
    } else if (forcedReason && strcmp(forcedReason, "motor_feedback_invalid") == 0) {
        recommendedAction =
            "Check RoboClaw communication and the encoder connection for this motor.";
    }

    const int32_t forwardDelta = forwardCount_ - initialCount_;
    const int32_t reverseDelta = finalCount_ - forwardCount_;
    const int32_t returnError = finalCount_ - initialCount_;

    StaticJsonDocument<1024> document;
    JsonObject measurements = document.to<JsonObject>();
    measurements["axis"] = axisOrChannel_;
    measurements["speed_command_mps"] = kMotorTestSpeedMps;
    measurements["run_time_each_direction_ms"] = kMotorRunMs;
    measurements["initial_count"] = initialCount_;
    measurements["forward_count"] = forwardCount_;
    measurements["final_count"] = finalCount_;
    measurements["forward_delta_count"] = forwardDelta;
    measurements["reverse_delta_count"] = reverseDelta;
    measurements["return_error_count"] = returnError;
    measurements["minimum_movement_counts"] = kMotorMinimumMovementCounts;
    measurements["return_tolerance_counts"] = kMotorReturnToleranceCounts;
    measurements["final_speed_qpps"] = latestSpeedQpps_;
    measurements["feedback_samples"] = feedbackSamples_;
    measurements["feedback_failures"] = feedbackFailures_;
    measurements["recommended_action"] = recommendedAction;
    addControllerInformation(measurements);

    emitTerminal(passed, reason, measurements);
    clearRun();
}

bool DiagnosticRunner::startActuator(const JsonDocument& command) {
    const int channel = command["channel"] | -1;
    const float voltage = command["voltage"] | 3.0f;
    const float tolerance = command["tolerance"] | 0.8f;
    const uint32_t settle = command["settle_ms"] | 1000;

    if (channel < 0 || channel >= NUM_ACTUATORS) {
        emitRejected("invalid_actuator_channel");
        clearRun();
        return false;
    }
    if (actuator_.hasPCBFault()) {
        emitRejected("actuator_pcb_fault");
        clearRun();
        return false;
    }
    if (!isfinite(voltage) || voltage < 0.0f || voltage > kActuatorMaxCommandV ||
        !isfinite(tolerance) || tolerance < 0.0f) {
        emitRejected("invalid_actuator_parameters");
        clearRun();
        return false;
    }

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
    strlcpy(category_, sensor, sizeof(category_));

    if (strcasecmp(sensor, "battery") == 0 ||
        strcasecmp(sensor, "ultrasonic") == 0) {
        state_ = State::SensorSampling;
        return true;
    }

    emitRejected("unsupported_sensor_test");
    clearRun();
    return false;
}

bool DiagnosticRunner::startAndon(const JsonDocument&) {
    strlcpy(category_, "andon", sizeof(category_));
    andon_.setOverride(AndonLight::BLUE);
    phaseStartedAtMs_ = millis();
    state_ = State::AndonBlueFirst;
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
        case State::MotorForwardRunning:
        case State::MotorForwardSettling:
        case State::MotorReverseRunning:
        case State::MotorReverseSettling:
            updateMotor(now);
            break;
        case State::ActuatorSettling:
            updateActuator(now);
            break;
        case State::SensorSampling:
            updateSensor();
            break;
        case State::AndonBlueFirst:
        case State::AndonOff:
        case State::AndonBlueSecond:
            updateAndon(now);
            break;
        case State::Aborting:
            stopOwnedOutputs();
            emitSimpleTerminal(false, "operator_aborted");
            clearRun();
            break;
        default:
            break;
    }
}

void DiagnosticRunner::updateActuator(uint32_t now) {
    if (now - phaseStartedAtMs_ < durationMs_) return;

    const float feedback = actuator_.readADC_Avg(axisOrChannel_, 5, 3);
    const float expected = actuator_.expectedFeedbackMapped(axisOrChannel_, commandValue_);
    const float error = feedback - expected;

    StaticJsonDocument<224> measurements;
    measurements["channel"] = axisOrChannel_;
    measurements["command_V"] = commandValue_;
    measurements["expected_V"] = expected;
    measurements["feedback_V"] = feedback;
    measurements["error_V"] = error;

    const bool passed = !actuator_.hasPCBFault() && fabsf(error) <= tolerance_;
    actuator_.actuatorPositions[axisOrChannel_] = 0.0f;
    actuator_.writeDAC(axisOrChannel_, 0.0f);
    emitTerminal(
        passed,
        passed ? "within_tolerance" : "feedback_out_of_tolerance",
        measurements.as<JsonObjectConst>()
    );
    clearRun();
}

void DiagnosticRunner::updateSensor() {
    StaticJsonDocument<224> measurements;
    bool passed = false;
    const char* reason = "sensor_out_of_range";

    if (strcasecmp(category_, "battery") == 0) {
        battery_.readBatteryVoltage();
        const float voltage = battery_.voltage;
        measurements["voltage_V"] = voltage;
        passed = voltage >= MIN_BATTERY_VOLTAGE && voltage <= MAX_BATTERY_VOLTAGE;
        reason = passed ? "voltage_in_range" : "voltage_out_of_range";
    } else {
        const int adc = analogRead(CFG.ultrasonic.analog_pin);
        const float voltage = static_cast<float>(adc) * 3.1f / 1023.0f;
        const float distance =
            voltage * CFG.ultrasonic.mm_per_volt + CFG.ultrasonic.offset_mm;
        measurements["adc"] = adc;
        measurements["distance_mm"] = distance;
        passed =
            distance >= CFG.ultrasonic.valid_min_mm &&
            distance <= CFG.ultrasonic.valid_max_mm;
        reason = passed ? "plausible_reading" : "distance_out_of_range";
    }

    emitTerminal(passed, reason, measurements.as<JsonObjectConst>());
    clearRun();
}

void DiagnosticRunner::updateAndon(uint32_t now) {
    static constexpr uint32_t kBlueDurationMs = 700;
    static constexpr uint32_t kOffDurationMs = 400;

    if (state_ == State::AndonBlueFirst) {
        if (now - phaseStartedAtMs_ < kBlueDurationMs) return;
        andon_.setOverride(AndonLight::OFF);
        phaseStartedAtMs_ = now;
        state_ = State::AndonOff;
        return;
    }

    if (state_ == State::AndonOff) {
        if (now - phaseStartedAtMs_ < kOffDurationMs) return;
        andon_.setOverride(AndonLight::BLUE);
        phaseStartedAtMs_ = now;
        state_ = State::AndonBlueSecond;
        return;
    }

    if (state_ == State::AndonBlueSecond) {
        if (now - phaseStartedAtMs_ < kBlueDurationMs) return;

        StaticJsonDocument<192> measurements;
        measurements["pattern"] = "BLUE-OFF-BLUE";
        measurements["blue_duration_ms"] = kBlueDurationMs;
        measurements["off_duration_ms"] = kOffDurationMs;
        measurements["visual_confirmation_required"] = true;

        andon_.clearOverride();
        emitTerminal(
            true,
            "pattern_completed_visual_confirmation_required",
            measurements.as<JsonObjectConst>()
        );
        clearRun();
    }
}

bool DiagnosticRunner::abort(const char* runId) {
    if (!active() || !runId || strcmp(runId, runId_) != 0) {
        return false;
    }
    state_ = State::Aborting;
    return true;
}

void DiagnosticRunner::stopOwnedOutputs() {
    motors_.BRAKE_STOP();

    if (axisOrChannel_ < NUM_ACTUATORS &&
        strcasecmp(category_, "actuator") == 0) {
        actuator_.actuatorPositions[axisOrChannel_] = 0.0f;
        actuator_.writeDAC(axisOrChannel_, 0.0f);
    }

    andon_.clearOverride();
}

void DiagnosticRunner::emitRejectedCommand(
    const JsonDocument& command,
    const char* reason
) {
    StaticJsonDocument<320> document;
    document["type"] = "test_result";
    document["id"] = command["id"] | "";
    document["category"] = "diagnostic";
    document["run_id"] = command["run_id"] | "";
    document["transaction_id"] = command["transaction_id"] | "";
    document["pass"] = false;
    document["reason"] = reason;
    document.createNestedObject("measurements");
    serializeJson(document, io_);
    io_.println();
}

void DiagnosticRunner::emitRejected(const char* reason) {
    StaticJsonDocument<8> measurements;
    emitTerminal(false, reason, measurements.as<JsonObjectConst>());
}

void DiagnosticRunner::emitSimpleTerminal(bool passed, const char* reason) {
    StaticJsonDocument<8> measurements;
    emitTerminal(passed, reason, measurements.as<JsonObjectConst>());
}

void DiagnosticRunner::emitTerminal(
    bool passed,
    const char* reason,
    JsonObjectConst measurements
) {
    if (terminalSent_) return;
    terminalSent_ = true;

    StaticJsonDocument<1536> document;
    document["type"] = "test_result";
    document["id"] = moduleId_;
    document["category"] = category_;
    document["run_id"] = runId_;
    document["transaction_id"] = transactionId_;
    document["pass"] = passed;
    document["reason"] = reason;
    document["started_at_ms"] = startedAtMs_;
    document["completed_at_ms"] = millis();

    JsonObject output = document.createNestedObject("measurements");
    for (JsonPairConst pair : measurements) {
        output[pair.key()] = pair.value();
    }

    serializeJson(document, io_);
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
    terminalSent_ = false;
}
