#include "DiagnosticRunner.hpp"
#include <math.h>
#include <string.h>

static int32_t safeAbs32(int32_t v){ return v==INT32_MIN ? INT32_MAX : abs(v); }

DiagnosticRunner::DiagnosticRunner(Stream& io, Motors& motors, ActuatorControl& actuator,
 BatteryMonitor& battery, Ultrasonic& ultrasonic, UltrasonicServo& servo,
 AndonManager& andon, EStop& estop, ClampSensor& clamp, JogControl& jog, bool& processEnabled)
 : io_(io), motors_(motors), actuator_(actuator), battery_(battery), ultrasonic_(ultrasonic),
 servo_(servo), andon_(andon), estop_(estop), clamp_(clamp), jog_(jog), processEnabled_(processEnabled) {}

bool DiagnosticRunner::copyIdentity(const JsonDocument& command)
{
    const char* runId = command["run_id"] | "";
    const char* transactionId = command["transaction_id"] | "";
    const char* moduleId = command["id"] | "";

    if (!runId[0] || !transactionId[0] || !moduleId[0]) {
        return false;
    }

    strlcpy(runId_, runId, sizeof(runId_));
    strlcpy(
        transactionId_,
        transactionId,
        sizeof(transactionId_)
    );
    strlcpy(moduleId_, moduleId, sizeof(moduleId_));

    return true;
}

bool DiagnosticRunner::preflight(
    bool motionDiagnostic,
    const char*& reason
) const
{
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

    if (motionDiagnostic && estop_.isActive()) {
        reason = "estop_active";
        return false;
    }

    if (motionDiagnostic && !clamp_.isClamped()) {
        reason = "clamp_not_confirmed";
        return false;
    }

    if (motionDiagnostic && motors_.isMotionInhibited()) {
        reason = "motion_inhibited";
        return false;
    }

    return true;
}

bool DiagnosticRunner::start(const JsonDocument& command)
{
    const char* action = command["action"] | "";

    const bool motionDiagnostic =
        !strcasecmp(action, "test_motor") ||
        !strcasecmp(action, "test_actuator") ||
        !strcasecmp(action, "calibrate_actuator");

    if (active()) {
        emitRejectedCommand(
            command,
            "diagnostic_already_active"
        );
        return false;
    }

    if (!copyIdentity(command)) {
        emitRejectedCommand(
            command,
            "missing_diagnostic_identity"
        );
        clearRun();
        return false;
    }

    emitProgress(
        "command_received"
    );

    const char* reason = nullptr;

    if (!preflight(motionDiagnostic, reason)) {
        emitProgress(
            "preflight_rejected",
            reason
        );

        emitRejected(reason);
        clearRun();
        return false;
    }

    emitProgress(
        "preflight_accepted"
    );

    startedAtMs_ = millis();
    phaseStartedAtMs_ = startedAtMs_;
    terminalSent_ = false;

    if (!strcasecmp(action, "test_motor")) {
        return startMotor(command);
    }

    if (!strcasecmp(action, "test_actuator") ||
        !strcasecmp(action, "calibrate_actuator")) {
        return startActuator(command);
    }

    if (!strcasecmp(action, "test_sensor")) {
        return startSensor(command);
    }

    if (!strcasecmp(action, "test_andon")) {
        return startAndon(command);
    }

    emitRejected("unsupported_diagnostic_action");
    clearRun();
    return false;
}

bool DiagnosticRunner::startMotor(const JsonDocument& command)
{
    const int axis = command["index"] | -1;

    if (axis < 0 || axis > 3) {
        emitRejected("invalid_motor_axis");
        clearRun();
        return false;
    }

    axisOrChannel_ = axis;
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

    if (!sampleMotorFeedback(millis(), true)) {
        finishMotorDiagnostic("motor_feedback_invalid");
        return false;
    }

    initialCount_ = latestEncoderCount_;
    forwardCount_ = latestEncoderCount_;
    finalCount_ = latestEncoderCount_;

    commandSelectedMotor(kMotorTestSpeedMps);
    phaseStartedAtMs_ = millis();

    return true;
}

void DiagnosticRunner::commandSelectedMotor(float speedMps)
{
    float speeds[4] = {
        0.0f,
        0.0f,
        0.0f,
        0.0f
    };

    speeds[axisOrChannel_] = speedMps;

    motors_.setSpeeds(
        speeds[0],
        speeds[1],
        speeds[2],
        speeds[3]
    );
}

bool DiagnosticRunner::sampleMotorFeedback(
    uint32_t now,
    bool force
)
{
    const uint32_t elapsed =
        static_cast<uint32_t>(now - lastMotorSampleMs_);

    if (!force && elapsed < kMotorSamplePeriodMs) {
        return latestFeedbackValid_;
    }

    lastMotorSampleMs_ = now;
    ++feedbackSamples_;

    int32_t encoderCount = 0;
    int32_t speedQpps = 0;
    uint8_t encoderStatus = 0;
    uint8_t speedStatus = 0;

    const bool feedbackValid =
        motors_.readMotorMotionFeedback(
            axisOrChannel_,
            encoderCount,
            speedQpps,
            encoderStatus,
            speedStatus
        );

    latestFeedbackValid_ = feedbackValid;

    if (!feedbackValid) {
        ++feedbackFailures_;
        return false;
    }

    latestEncoderCount_ = encoderCount;
    latestSpeedQpps_ = speedQpps;

    uint32_t controllerFlags = 0;
    if (motors_.readControllerErrorFlags(
            axisOrChannel_,
            controllerFlags
        )) {
        controllerFlags_ |= controllerFlags;
        controllerFlagsValid_ = true;
    }

    return true;
}

void DiagnosticRunner::updateMotor(uint32_t now)
{
    sampleMotorFeedback(now);

    if (state_ == State::MotorForwardRunning) {
        if (static_cast<uint32_t>(now - phaseStartedAtMs_) >=
            kMotorRunMs) {
            motors_.BRAKE_STOP();

            if (latestFeedbackValid_) {
                forwardCount_ = latestEncoderCount_;
            }

            phaseStartedAtMs_ = now;
            state_ = State::MotorForwardSettling;
        }

        return;
    }

    if (state_ == State::MotorForwardSettling) {
        const bool stopped =
            latestFeedbackValid_ &&
            safeAbs32(latestSpeedQpps_) <=
                kMotorStoppedQpps;

        const bool minimumSettleElapsed =
            static_cast<uint32_t>(now - phaseStartedAtMs_) >=
            150U;

        if (stopped && minimumSettleElapsed) {
            forwardCount_ = latestEncoderCount_;

            commandSelectedMotor(-kMotorTestSpeedMps);

            phaseStartedAtMs_ = now;
            state_ = State::MotorReverseRunning;
            return;
        }

        if (static_cast<uint32_t>(now - phaseStartedAtMs_) >=
            kMotorSettleTimeoutMs) {
            finishMotorDiagnostic("motor_did_not_stop");
        }

        return;
    }

    if (state_ == State::MotorReverseRunning) {
        if (static_cast<uint32_t>(now - phaseStartedAtMs_) >=
            kMotorRunMs) {
            motors_.BRAKE_STOP();

            if (latestFeedbackValid_) {
                finalCount_ = latestEncoderCount_;
            }

            phaseStartedAtMs_ = now;
            state_ = State::MotorReverseSettling;
        }

        return;
    }

    if (state_ == State::MotorReverseSettling) {
        const bool stopped =
            latestFeedbackValid_ &&
            safeAbs32(latestSpeedQpps_) <=
                kMotorStoppedQpps;

        const bool minimumSettleElapsed =
            static_cast<uint32_t>(now - phaseStartedAtMs_) >=
            150U;

        if (stopped && minimumSettleElapsed) {
            finalCount_ = latestEncoderCount_;
            finishMotorDiagnostic();
            return;
        }

        if (static_cast<uint32_t>(now - phaseStartedAtMs_) >=
            kMotorSettleTimeoutMs) {
            finishMotorDiagnostic("motor_did_not_stop");
        }
    }
}

const char* DiagnosticRunner::evaluateMotor(bool& pass,const char*& action) const{pass=false;if(!latestFeedbackValid_||feedbackFailures_>feedbackSamples_/2){action="Check RoboClaw communication and encoder.";return "motor_feedback_invalid";}int32_t f=forwardCount_-initialCount_,r=finalCount_-forwardCount_,e=finalCount_-initialCount_;if(safeAbs32(f)<kMotorMinimumMovementCounts){action="Check motor, encoder, and load.";return "no_forward_movement";}if(safeAbs32(r)<kMotorMinimumMovementCounts){action="Check reverse operation and load.";return "no_reverse_movement";}if((f>0)==(r>0)){action="Check motor and encoder directions.";return "motor_direction_invalid";}if(safeAbs32(e)>kMotorReturnToleranceCounts){action="Check slip, backlash, or encoder mounting.";return "return_position_out_of_tolerance";}pass=true;action="No service action required.";return "motor_test_passed";}
void DiagnosticRunner::addControllerInformation(JsonObject m) const{m["controller_error_flags"]=controllerFlags_;m["controller_error_valid"]=controllerFlagsValid_;}
void DiagnosticRunner::finishMotorDiagnostic(const char* forced){motors_.BRAKE_STOP();sampleMotorFeedback(millis(),true);if(latestFeedbackValid_)finalCount_=latestEncoderCount_;bool pass=false;const char* action="Inspect motor feedback.";const char* reason=forced?forced:evaluateMotor(pass,action);StaticJsonDocument<768>d;JsonObject m=d.to<JsonObject>();m["axis"]=axisOrChannel_;m["initial_count"]=initialCount_;m["forward_count"]=forwardCount_;m["final_count"]=finalCount_;m["recommended_action"]=action;addControllerInformation(m);emitTerminal(pass,reason,m);clearRun();}

void DiagnosticRunner::emitProgress(
    const char* phase,
    const char* reason
) {
    StaticJsonDocument<448> doc;

    doc["type"] = "diagnostic_progress";
    doc["run_id"] = runId_;
    doc["transaction_id"] = transactionId_;
    doc["id"] = moduleId_;
    doc["category"] = category_;
    doc["phase"] = phase;
    doc["state"] = static_cast<uint8_t>(state_);
    doc["channel"] = axisOrChannel_;
    doc["ts_ms"] = millis();

    if (reason && reason[0] != '\0') {
        doc["reason"] = reason;
    }

    doc["process_enabled"] = processEnabled_;
    doc["jog_active"] = jog_.isActive();
    doc["estop_active"] = estop_.isActive();
    doc["clamp_confirmed"] = clamp_.isClamped();
    doc["motion_inhibited"] =
        motors_.isMotionInhibited();
    doc["actuator_pcb_fault"] =
        actuator_.hasPCBFault();
    doc["actuator_jam_mask"] =
        actuator_.jamMask();

    serializeJson(doc, io_);
    io_.println();
}

bool DiagnosticRunner::startActuator(const JsonDocument& c) {
    const int ch = c["channel"] | -1;
    if (ch < 0 || ch >= NUM_ACTUATORS) {
        emitRejected("invalid_actuator_channel");
        clearRun();
        return false;
    }

    if (actuator_.hasPCBFault()) {
        emitRejected("actuator_pcb_fault");
        clearRun();
        return false;
    }

    actuatorCalibration_ = !strcasecmp(c["action"] | "", "calibrate_actuator");
    axisOrChannel_ = static_cast<uint8_t>(ch);
    strlcpy(
        category_,
        actuatorCalibration_ ? "actuator_calibration" : "actuator",
        sizeof(category_)
    );

    commandValue_ = c["extend_voltage"] | kActuatorMaxCommandV;
    if (!isfinite(commandValue_) ||
        commandValue_ < 0.0f ||
        commandValue_ > kActuatorMaxCommandV) {
        emitRejected("invalid_actuator_parameters");
        clearRun();
        return false;
    }

    bool hasCalibrationParameters = false;

    if (
        c.containsKey("limits") &&
        c["limits"].is<JsonArrayConst>()
    ) {
        JsonArrayConst limits =
            c["limits"].as<JsonArrayConst>();

        if (limits.size() != 4) {
            emitRejected(
                "invalid_actuator_limits"
            );

            clearRun();
            return false;
        }

        extendedTargetV_ =
            limits[0].as<float>();

        retractedTargetV_ =
            limits[1].as<float>();

        extendedToleranceV_ =
            limits[2].as<float>();

        retractedToleranceV_ =
            limits[3].as<float>();

        hasCalibrationParameters = true;
    }
    else if (
        c.containsKey(
            "extended_target_v"
        ) &&
        c.containsKey(
            "retracted_target_v"
        ) &&
        c.containsKey(
            "extended_tolerance_v"
        ) &&
        c.containsKey(
            "retracted_tolerance_v"
        )
    ) {
        extendedTargetV_ =
            c["extended_target_v"].as<float>();

        retractedTargetV_ =
            c["retracted_target_v"].as<float>();

        extendedToleranceV_ =
            c["extended_tolerance_v"].as<float>();

        retractedToleranceV_ =
            c["retracted_tolerance_v"].as<float>();

        hasCalibrationParameters = true;
    }
    else {
        extendedTargetV_ = 0.0f;
        retractedTargetV_ = 0.0f;
        extendedToleranceV_ = 0.15f;
        retractedToleranceV_ = 0.15f;
    }

    if (
        !actuatorCalibration_ &&
        !hasCalibrationParameters
    ) {
        emitRejected(
            "actuator_calibration_required"
        );

        clearRun();
        return false;
    }

    if (
        !isfinite(extendedTargetV_) ||
        !isfinite(retractedTargetV_) ||
        !isfinite(extendedToleranceV_) ||
        !isfinite(retractedToleranceV_) ||
        extendedTargetV_ < 0.0f ||
        retractedTargetV_ < 0.0f ||
        extendedToleranceV_ <= 0.0f ||
        retractedToleranceV_ <= 0.0f
    ) {
        emitRejected(
            "invalid_actuator_calibration"
        );

        clearRun();
        return false;
    }

    pendingActuatorFailureReason_[0] = '\0';
    confirmationStartedAtMs_ = 0;
    extensionTimeMs_ = 0;
    retractionTimeMs_ = 0;
    extendedFeedbackV_ = 0.0f;
    retractedFeedbackV_ = 0.0f;
    lastActuatorFeedbackV_ = actuator_.feedbackSignals[ch];
    stableSinceMs_ = 0;
    actuatorSamples_ = 0;
    extendedMinV_ = retractedMinV_ = 99.0f;
    extendedMaxV_ = retractedMaxV_ = -99.0f;

    commandActuator(commandValue_);    
    emitProgress("extension_command_applied");
    phaseStartedAtMs_ = millis();
    state_ = State::ActuatorExtending;
    return true;
}

void DiagnosticRunner::commandActuator(float v){actuator_.actuatorPositions[axisOrChannel_]=v;actuator_.writeDAC(axisOrChannel_,v);}
bool DiagnosticRunner::actuatorFeedbackStable(uint32_t now,float target,float tol,bool calibration){
 float v=actuator_.feedbackSignals[axisOrChannel_]; if(!isfinite(v))return false; ++actuatorSamples_;
 bool qualifying=calibration ? fabsf(v-lastActuatorFeedbackV_)<=kCalibrationPlateauDeltaV : fabsf(v-target)<=tol;
 lastActuatorFeedbackV_=v; if(!qualifying){stableSinceMs_=0;return false;} if(stableSinceMs_==0)stableSinceMs_=now; return now-stableSinceMs_>=kActuatorStableMs;
}

bool DiagnosticRunner::confirmActuatorExtension(const JsonDocument& command) {
    if (state_ != State::ActuatorAwaitingExtendedConfirmation ||
        !actuatorCalibration_) {
        return false;
    }

    const char* run = command["run_id"] | "";
    const char* transaction = command["transaction_id"] | "";
    const char* module = command["id"] | "";

    if (strcmp(run, runId_) != 0 ||
        strcmp(transaction, transactionId_) != 0 ||
        strcmp(module, moduleId_) != 0) {
        return false;
    }

    const bool confirmed = command["confirmed"] | false;

    if (confirmed) {
        extendedFeedbackV_ = actuator_.readADC_Avg(axisOrChannel_, 5, 3);
        if (!isfinite(extendedFeedbackV_)) {
            strlcpy(
                pendingActuatorFailureReason_,
                "extended_feedback_invalid",
                sizeof(pendingActuatorFailureReason_)
            );
        } else {
            pendingActuatorFailureReason_[0] = '\0';
        }
    } else {
        strlcpy(
            pendingActuatorFailureReason_,
            "physical_extension_not_confirmed",
            sizeof(pendingActuatorFailureReason_)
        );
    }

    // Remove extension command only after accepting a correctly correlated
    // confirmation. Terminal output is deferred until safe retraction settles.
    commandActuator(0.0f);
    phaseStartedAtMs_ = millis();
    stableSinceMs_ = 0;
    lastActuatorFeedbackV_ = actuator_.feedbackSignals[axisOrChannel_];
    state_ = State::ActuatorRetracting;
    return true;
}

void DiagnosticRunner::emitActuatorProgress(const char* phase) {
    StaticJsonDocument<320> doc;
    doc["type"] = "diagnostic_progress";
    doc["category"] = "actuator_calibration";
    doc["run_id"] = runId_;
    doc["transaction_id"] = transactionId_;
    doc["id"] = moduleId_;
    doc["phase"] = phase;
    doc["channel"] = axisOrChannel_;
    doc["feedback_v"] = actuator_.feedbackSignals[axisOrChannel_];
    doc["confirmation_timeout_ms"] = kActuatorConfirmationTimeoutMs;
    serializeJson(doc, io_);
    io_.println();
}

void DiagnosticRunner::updateActuator(uint32_t now) {
    if (actuator_.hasPCBFault()) {
        commandActuator(0.0f);
        finishActuatorDiagnostic(false, "actuator_pcb_fault");
        return;
    }

    const float v = actuator_.feedbackSignals[axisOrChannel_];

    if (state_ == State::ActuatorExtending) {
        extendedMinV_ = min(extendedMinV_, v);
        extendedMaxV_ = max(extendedMaxV_, v);

        const bool minimumDriveComplete =
            !actuatorCalibration_ ||
            now - phaseStartedAtMs_ >= kActuatorMinimumCalibrationDriveMs;

        if (minimumDriveComplete &&
            actuatorFeedbackStable(
                now,
                extendedTargetV_,
                extendedToleranceV_,
                actuatorCalibration_
            )) {
            extensionTimeMs_ = now - phaseStartedAtMs_;
            extendedFeedbackV_ = v;
            phaseStartedAtMs_ = now;
            state_ = State::ActuatorExtendedHold;
            return;
        }

        if (now - phaseStartedAtMs_ >= kActuatorPhaseTimeoutMs) {
            strlcpy(
                pendingActuatorFailureReason_,
                "extension_timeout",
                sizeof(pendingActuatorFailureReason_)
            );
            commandActuator(0.0f);
            phaseStartedAtMs_ = now;
            stableSinceMs_ = 0;
            lastActuatorFeedbackV_ = v;
            state_ = State::ActuatorRetracting;
        }
        return;
    }

    if (state_ == State::ActuatorExtendedHold) {
        if (now - phaseStartedAtMs_ < kActuatorHoldMs) {
            return;
        }

        extendedFeedbackV_ = actuator_.readADC_Avg(axisOrChannel_, 5, 3);

        if (actuatorCalibration_) {
            // Keep the extension command active. Only a correctly correlated
            // operator response or timeout may release it.
            confirmationStartedAtMs_ = millis();
            state_ = State::ActuatorAwaitingExtendedConfirmation;
            emitActuatorProgress("awaiting_extended_confirmation");
            return;
        }

        commandActuator(0.0f);
        phaseStartedAtMs_ = millis();
        stableSinceMs_ = 0;
        lastActuatorFeedbackV_ = extendedFeedbackV_;
        state_ = State::ActuatorRetracting;
        return;
    }

    if (state_ == State::ActuatorAwaitingExtendedConfirmation) {
        if (now - confirmationStartedAtMs_ >=
            kActuatorConfirmationTimeoutMs) {
            commandActuator(0.0f);
            finishActuatorDiagnostic(false, "operator_confirmation_timeout");
        }
        return;
    }

    if (state_ == State::ActuatorRetracting) {
        retractedMinV_ = min(retractedMinV_, v);
        retractedMaxV_ = max(retractedMaxV_, v);

        const bool minimumDriveComplete =
            !actuatorCalibration_ ||
            now - phaseStartedAtMs_ >= kActuatorMinimumCalibrationDriveMs;

        if (minimumDriveComplete &&
            actuatorFeedbackStable(
                now,
                retractedTargetV_,
                retractedToleranceV_,
                actuatorCalibration_
            )) {
            retractionTimeMs_ = now - phaseStartedAtMs_;
            retractedFeedbackV_ = v;
            phaseStartedAtMs_ = now;
            state_ = State::ActuatorRetractedHold;
            return;
        }

        if (now - phaseStartedAtMs_ >= kActuatorPhaseTimeoutMs) {
            const char* reason = pendingActuatorFailureReason_[0] != '\0'
                ? pendingActuatorFailureReason_
                : "retraction_timeout";
            finishActuatorDiagnostic(false, reason);
        }
        return;
    }

    if (state_ == State::ActuatorRetractedHold &&
        now - phaseStartedAtMs_ >= kActuatorHoldMs) {
        retractedFeedbackV_ = actuator_.readADC_Avg(axisOrChannel_, 5, 3);

        if (pendingActuatorFailureReason_[0] != '\0') {
            finishActuatorDiagnostic(
                false,
                pendingActuatorFailureReason_
            );
            return;
        }

        const float span = fabsf(retractedFeedbackV_ - extendedFeedbackV_);
        const bool pass = span >= kMinimumFeedbackSpanV;
        finishActuatorDiagnostic(
            pass,
            pass
                ? (actuatorCalibration_
                    ? "calibration_complete"
                    : "full_cycle_passed")
                : "feedback_span_too_small"
        );
    }
}
void DiagnosticRunner::finishActuatorDiagnostic(bool pass,const char* reason){commandActuator(0);StaticJsonDocument<768>d;JsonObject m=d.to<JsonObject>();m["channel"]=axisOrChannel_;m["calibration"]=actuatorCalibration_;m["extended_feedback_v"]=extendedFeedbackV_;m["retracted_feedback_v"]=retractedFeedbackV_;m["feedback_span_v"]=fabsf(retractedFeedbackV_-extendedFeedbackV_);m["extension_time_ms"]=extensionTimeMs_;m["retraction_time_ms"]=retractionTimeMs_;m["extended_sample_min_v"]=extendedMinV_;m["extended_sample_max_v"]=extendedMaxV_;m["retracted_sample_min_v"]=retractedMinV_;m["retracted_sample_max_v"]=retractedMaxV_;m["sample_count"]=actuatorSamples_;m["final_command_v"]=0.0f;m["pcb_fault"]=actuator_.hasPCBFault();m["jam"]=actuator_.jammed[axisOrChannel_];emitTerminal(pass,reason,m);clearRun();}

bool DiagnosticRunner::ownsActuatorOutputs() const {
    return state_ == State::ActuatorExtending ||
        state_ == State::ActuatorExtendedHold ||
        state_ == State::ActuatorAwaitingExtendedConfirmation ||
        state_ == State::ActuatorRetracting ||
        state_ == State::ActuatorRetractedHold;
}

bool DiagnosticRunner::startSensor(const JsonDocument& command) {
    const char* sensor = command["sensor"] | "";
    strlcpy(category_, sensor, sizeof(category_));

    if (!strcasecmp(sensor, "battery") || !strcasecmp(sensor, "ultrasonic")) {
        state_ = State::SensorSampling;
        return true;
    }

    if (!strcasecmp(sensor, "clamp")) {
        clampOpenReading_ = false;
        clampClosedReading_ = false;
        clampOpenRaw_ = false;
        clampClosedRaw_ = false;
        guidedConfirmationStartedAtMs_ = millis();
        state_ = State::ClampAwaitingOpenConfirmation;
        emitClampProgress("awaiting_clamp_open_confirmation", false);
        return true;
    }

    emitRejected("unsupported_sensor_test");
    clearRun();
    return false;
}

void DiagnosticRunner::emitClampProgress(const char* phase, bool expectedClamped) {
    StaticJsonDocument<384> doc;
    doc["type"] = "diagnostic_progress";
    doc["category"] = "clamp";
    doc["run_id"] = runId_;
    doc["transaction_id"] = transactionId_;
    doc["id"] = moduleId_;
    doc["phase"] = phase;
    doc["expected_clamped"] = expectedClamped;
    doc["confirmation_timeout_ms"] = kGuidedConfirmationTimeoutMs;
    doc["ts_ms"] = millis();
    serializeJson(doc, io_);
    io_.println();
}

bool DiagnosticRunner::confirmClampState(const JsonDocument& command) {
    const char* run = command["run_id"] | "";
    const char* transaction = command["transaction_id"] | "";
    const char* module = command["id"] | "";
    const bool expected = command["expected_clamped"] | false;

    if (strcmp(run, runId_) || strcmp(transaction, transactionId_) ||
        strcmp(module, moduleId_)) return false;

    if (state_ == State::ClampAwaitingOpenConfirmation && !expected) {
        clampOpenReading_ = clamp_.isClamped();
        clampOpenRaw_ = clamp_.rawLevel();
        guidedConfirmationStartedAtMs_ = millis();
        state_ = State::ClampAwaitingClosedConfirmation;
        emitClampProgress("awaiting_clamp_closed_confirmation", true);
        return true;
    }

    if (state_ == State::ClampAwaitingClosedConfirmation && expected) {
        clampClosedReading_ = clamp_.isClamped();
        clampClosedRaw_ = clamp_.rawLevel();
        finishClampDiagnostic();
        return true;
    }

    return false;
}

void DiagnosticRunner::finishClampDiagnostic() {
    const bool changed = clampOpenReading_ != clampClosedReading_;
    const bool pass = !clampOpenReading_ && clampClosedReading_ && changed;
    const char* reason = pass ? "clamp_sensor_passed" :
        (clampOpenReading_ ? "clamp_indicates_closed_when_open" :
         (!clampClosedReading_ ? "clamp_indicates_open_when_closed" :
          "clamp_sensor_state_did_not_change"));

    StaticJsonDocument<384> measurementsDoc;
    JsonObject measurements = measurementsDoc.to<JsonObject>();
    measurements["open_clamped"] = clampOpenReading_;
    measurements["closed_clamped"] = clampClosedReading_;
    measurements["open_raw_level"] = clampOpenRaw_;
    measurements["closed_raw_level"] = clampClosedRaw_;
    measurements["state_changed"] = changed;
    measurements["debounce_ms"] = clamp_.debounceMs();
    emitTerminal(pass, reason, measurements);
    clearRun();
}

bool DiagnosticRunner::startAndon(const JsonDocument&) {
    strlcpy(category_, "andon", sizeof(category_));
    andonConfirmedMask_ = 0;
    andonFailedMask_ = 0;
    showAndonStep(State::AndonAwaitingGreenConfirmation,
                  AndonLight::GREEN, "GREEN", 1);
    return true;
}

void DiagnosticRunner::showAndonStep(State nextState, AndonLight::States color,
                                     const char* colorName, uint8_t step) {
    andon_.setOverride(color);
    state_ = nextState;
    guidedConfirmationStartedAtMs_ = millis();

    StaticJsonDocument<384> doc;
    doc["type"] = "diagnostic_progress";
    doc["category"] = "andon";
    doc["run_id"] = runId_;
    doc["transaction_id"] = transactionId_;
    doc["id"] = moduleId_;
    doc["phase"] = "awaiting_andon_color_confirmation";
    doc["color"] = colorName;
    doc["step"] = step;
    doc["total_steps"] = 4;
    doc["confirmation_timeout_ms"] = kGuidedConfirmationTimeoutMs;
    doc["ts_ms"] = millis();
    serializeJson(doc, io_);
    io_.println();
}

bool DiagnosticRunner::confirmAndonColor(const JsonDocument& command) {
    const char* run = command["run_id"] | "";
    const char* transaction = command["transaction_id"] | "";
    const char* module = command["id"] | "";
    const char* color = command["color"] | "";
    const bool confirmed = command["confirmed"] | false;

    if (strcmp(run, runId_) || strcmp(transaction, transactionId_) ||
        strcmp(module, moduleId_)) return false;

    uint8_t bit = 0;
    State nextState = state_;
    if (state_ == State::AndonAwaitingGreenConfirmation && !strcasecmp(color, "GREEN")) bit = kAndonGreenBit;
    else if (state_ == State::AndonAwaitingYellowConfirmation && !strcasecmp(color, "YELLOW")) bit = kAndonYellowBit;
    else if (state_ == State::AndonAwaitingBlueConfirmation && !strcasecmp(color, "BLUE")) bit = kAndonBlueBit;
    else if (state_ == State::AndonAwaitingRedConfirmation && !strcasecmp(color, "RED")) bit = kAndonRedBit;
    else return false;

    if (confirmed) andonConfirmedMask_ |= bit;
    else andonFailedMask_ |= bit;

    if (bit == kAndonGreenBit) showAndonStep(State::AndonAwaitingYellowConfirmation, AndonLight::YELLOW, "YELLOW", 2);
    else if (bit == kAndonYellowBit) showAndonStep(State::AndonAwaitingBlueConfirmation, AndonLight::BLUE, "BLUE", 3);
    else if (bit == kAndonBlueBit) showAndonStep(State::AndonAwaitingRedConfirmation, AndonLight::RED, "RED", 4);
    else finishAndonDiagnostic();
    return true;
}

void DiagnosticRunner::finishAndonDiagnostic() {
    andon_.clearOverride();
    const bool pass = andonFailedMask_ == 0 &&
        andonConfirmedMask_ == (kAndonGreenBit | kAndonYellowBit | kAndonBlueBit | kAndonRedBit);
    StaticJsonDocument<384> mdoc;
    JsonObject m = mdoc.to<JsonObject>();
    m["green_confirmed"] = (andonConfirmedMask_ & kAndonGreenBit) != 0;
    m["yellow_confirmed"] = (andonConfirmedMask_ & kAndonYellowBit) != 0;
    m["blue_confirmed"] = (andonConfirmedMask_ & kAndonBlueBit) != 0;
    m["red_confirmed"] = (andonConfirmedMask_ & kAndonRedBit) != 0;
    m["confirmed_mask"] = andonConfirmedMask_;
    m["failed_mask"] = andonFailedMask_;
    m["colors_tested"] = 4;
    emitTerminal(pass, pass ? "all_andon_colors_confirmed" :
                            "one_or_more_andon_colors_failed", m);
    clearRun();
}

void DiagnosticRunner::update() {
    if (!active()) {
        return;
    }

    const bool clampObservationState =
        state_ == State::ClampAwaitingOpenConfirmation ||
        state_ == State::ClampAwaitingClosedConfirmation;

    if (estop_.isActive()) {
        stopOwnedOutputs();
        emitSimpleTerminal(false, "safety_inhibit_activated");
        clearRun();
        return;
    }

    if (motors_.isMotionInhibited() && !clampObservationState) {
        stopOwnedOutputs();
        emitSimpleTerminal(false, "safety_inhibit_activated");
        clearRun();
        return;
    }

    const uint32_t now = millis();

    const bool guidedWaiting =
        clampObservationState ||
        state_ == State::AndonAwaitingGreenConfirmation ||
        state_ == State::AndonAwaitingYellowConfirmation ||
        state_ == State::AndonAwaitingBlueConfirmation ||
        state_ == State::AndonAwaitingRedConfirmation;

    if (guidedWaiting &&
        static_cast<uint32_t>(now - guidedConfirmationStartedAtMs_) >=
            kGuidedConfirmationTimeoutMs) {
        stopOwnedOutputs();
        emitSimpleTerminal(false, "operator_confirmation_timeout");
        clearRun();
        return;
    }

    switch (state_) {
        case State::MotorForwardRunning:
        case State::MotorForwardSettling:
        case State::MotorReverseRunning:
        case State::MotorReverseSettling:
            updateMotor(now);
            break;

        case State::ActuatorExtending:
        case State::ActuatorExtendedHold:
        case State::ActuatorAwaitingExtendedConfirmation:
        case State::ActuatorRetracting:
        case State::ActuatorRetractedHold:
            updateActuator(now);
            break;

        case State::SensorSampling:
            updateSensor();
            break;

        case State::ClampAwaitingOpenConfirmation:
        case State::ClampAwaitingClosedConfirmation:
        case State::AndonAwaitingGreenConfirmation:
        case State::AndonAwaitingYellowConfirmation:
        case State::AndonAwaitingBlueConfirmation:
        case State::AndonAwaitingRedConfirmation:
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

void DiagnosticRunner::updateSensor(){StaticJsonDocument<224>m;bool p=false;const char*r="sensor_out_of_range";if(!strcasecmp(category_,"battery")){battery_.readBatteryVoltage();float v=battery_.voltage;m["voltage_V"]=v;p=v>=MIN_BATTERY_VOLTAGE&&v<=MAX_BATTERY_VOLTAGE;r=p?"voltage_in_range":"voltage_out_of_range";}else{int a=analogRead(CFG.ultrasonic.analog_pin);float d=(float)a*3.1f/1023.0f*CFG.ultrasonic.mm_per_volt+CFG.ultrasonic.offset_mm;m["adc"]=a;m["distance_mm"]=d;p=d>=CFG.ultrasonic.valid_min_mm&&d<=CFG.ultrasonic.valid_max_mm;r=p?"plausible_reading":"distance_out_of_range";}emitTerminal(p,r,m.as<JsonObjectConst>());clearRun();}

bool DiagnosticRunner::abort(const char*r){if(!active()||!r||strcmp(r,runId_))return false;state_=State::Aborting;return true;}

void DiagnosticRunner::stopOwnedOutputs(){motors_.BRAKE_STOP();if(ownsActuatorOutputs()||(axisOrChannel_<NUM_ACTUATORS&&!strcasecmp(category_,"actuator"))||(axisOrChannel_<NUM_ACTUATORS&&!strcasecmp(category_,"actuator_calibration")))commandActuator(0);andon_.clearOverride();}

void DiagnosticRunner::emitRejectedCommand(const JsonDocument&c,const char*r){StaticJsonDocument<320>d;d["type"]="test_result";d["id"]=c["id"]|"";d["category"]="diagnostic";d["run_id"]=c["run_id"]|"";d["transaction_id"]=c["transaction_id"]|"";d["pass"]=false;d["reason"]=r;d.createNestedObject("measurements");serializeJson(d,io_);io_.println();}

void DiagnosticRunner::emitRejected(const char*r){StaticJsonDocument<8>m;emitTerminal(false,r,m.as<JsonObjectConst>());}void DiagnosticRunner::emitSimpleTerminal(bool p,const char*r){StaticJsonDocument<8>m;emitTerminal(p,r,m.as<JsonObjectConst>());}

void DiagnosticRunner::emitTerminal(bool p,const char*r,JsonObjectConst m){if(terminalSent_)return;terminalSent_=true;StaticJsonDocument<1536>d;d["type"]="test_result";d["id"]=moduleId_;d["category"]=category_;d["run_id"]=runId_;d["transaction_id"]=transactionId_;d["pass"]=p;d["reason"]=r;d["started_at_ms"]=startedAtMs_;d["completed_at_ms"]=millis();JsonObject o=d.createNestedObject("measurements");for(JsonPairConst x:m)o[x.key()]=x.value();serializeJson(d,io_);io_.println();}

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
    actuatorCalibration_ = false;
    confirmationStartedAtMs_ = 0;
    pendingActuatorFailureReason_[0] = '\0';
}