#ifndef MY_SERIAL_CLASS
#define MY_SERIAL_CLASS

#include <Arduino.h>
#include <ArduinoJson.h>
#include <vector>
#include "Config.hpp"
#include "AndonLight.hpp"
#include "Motors.hpp"
#include "AndonManager.hpp"
#include "Ultrasonic.hpp"
#include "Actuator.hpp"
#include "BatteryMonitor.hpp"
#include "UltrasonicServo.hpp"
#include "JogControl.hpp"
#include "Version.hpp"
#include "EncoderSession.hpp"
#include "DiagnosticRunner.hpp"
#include "FeedforwardConfiguration.hpp"
#include <limits.h>
#include <string.h>


class AndonManager;
class Ultrasonic;

#ifndef RED_LED
#define RED_LED LEDR   // Portenta H7 red LED alias
#endif

class MySerial
{
public:
    // Bump buffers for real payloads
    static constexpr size_t kJsonCap   = 640;
    static constexpr size_t kBufSize   = 512;

    StaticJsonDocument<kJsonCap> jsonPacket;
    static const size_t numChars = kBufSize;
    char receivedChars[numChars];
    bool newData = false;

    bool serialStarted = false;
    bool serialEnded   = false;
    bool LED_STATE     = false; // optional latch if you want a heartbeat LED
    bool* hmiConnected_ = nullptr;
    bool triggerEvaluationSuspended = false;

    float transitionActuatorVoltage_ = CFG.actuator.transitionActiveVoltage;

    struct ActuatorPreviewLease {
        bool active = false;
        uint8_t channel = 0;
        uint32_t expiresAtMs = 0;
        uint32_t lastSequence = 0;
        char sessionId[64] = {0};
    };
    ActuatorPreviewLease actuatorPreview_;

    unsigned long lastEncoderEmitMs = 0;
    static constexpr unsigned long encoderEmitPeriodMs = 500;
    
    unsigned long lastActuatorEmitMs = 0;
    static constexpr unsigned long actuatorEmitPeriodMs = 250;

    unsigned long lastActuatorSnapshotMs = 0;
    static constexpr unsigned long actuatorSnapshotPeriodMs = 2000;

    uint8_t lastActuatorCommandMask = 0xFF;
    uint8_t lastActuatorFeedbackMask = 0xFF;
    uint8_t lastActuatorJamMask = 0xFF;
    bool lastActuatorPcbFault = false;


    struct Trigger {
        int threshold;
        int activate_channel;
        int deactivate_channel;
        float delay_seconds;
        bool hold_active = false;
        bool triggered = false;
        unsigned long triggerTime = 0;
        bool waitingToDeactivate = false;
    };
    std::vector<Trigger> triggerBuffer;
    std::vector<Trigger> pendingTriggerBuffer;
    bool triggerLoadInProgress = false;
    
    struct DirectionTestState {
        bool active = false;
        uint8_t axis = 0;
        uint32_t expiresAtMs = 0;
        int32_t startCount = 0;
        int8_t testedMotorDirection = 1;
        int8_t testedEncoderDirection = 1;
        char transactionId[80] = {0};
    };
    DirectionTestState directionTest_;

    // Wiring to other subsystems
    AndonManager* andonMgr = nullptr;
    AndonLight* andonLight;
    Motors*     motors;
    ActuatorControl* actuator;
    UltrasonicServo* ultrasonicServo_ = nullptr;    
    Stream* io = nullptr;
    JogControl* jogControl = nullptr;
    EncoderSession* encoderSession_ = nullptr;
    DiagnosticRunner* diagnosticRunner_ = nullptr;


    
    MySerial(Stream& ioRef, ActuatorControl& actuatorRef, AndonLight& lightRef, Motors& motorsRef)
    : io(&ioRef)
    , andonMgr(nullptr)
    , andonLight(&lightRef)
    , motors(&motorsRef)
    , actuator(&actuatorRef) {}


    void attachAndonManager(AndonManager& mgr) { andonMgr = &mgr; }

    void attachUltrasonic(Ultrasonic& u, bool& enabledFlag) {
        ultrasonic_        = &u;
        ultrasonicEnabled_ = &enabledFlag;
    }
    
    void attachUltrasonicServo(UltrasonicServo& s) {
        ultrasonicServo_ = &s;
    }

    void attachJogControl(JogControl& jog) {
            jogControl = &jog;
        }

    void attachEncoderSession(EncoderSession& session) { encoderSession_ = &session; }

    void attachDiagnosticRunner(
        DiagnosticRunner& runner
    ) {
        diagnosticRunner_ = &runner;
    }

    // Helper for AndonManager: "is comms alive recently?"
    bool commsAlive() const {
        return timeout > 0;
    }

    void attachHmiConnectionState(bool& connectedFlag) {
        hmiConnected_ = &connectedFlag;
    }

    //Helper: emit test result over serial for HMI
    void emitTestResult(const char* id,
                        const char* category,
                        bool pass,
                        const JsonObjectConst meas,
                        const char* reason = nullptr) {
        StaticJsonDocument<320> doc;
        doc["type"] = "test_result";
        doc["id"] = id;
        doc["category"] = category;
        doc["pass"] = pass;
        if (reason && reason[0] != '\0') doc["reason"] = reason;
        JsonObject m = doc.createNestedObject("measurements");
        for (JsonPairConst kv : meas) {
            m[kv.key()] = kv.value();
        }
        serializeJson(doc, *io); io->println();
    }

    // Helper: convert AndonLight::States to string for diagnostics
    static const char* stateToString(AndonLight::States s) {
        switch (s) {
            case AndonLight::GREEN:        return "GREEN";
            case AndonLight::YELLOW:       return "YELLOW";
            case AndonLight::BLUE:         return "BLUE";
            case AndonLight::RED:          return "RED";
            case AndonLight::OFF:          return "OFF";
            case AndonLight::BLINK_GREEN:  return "BLINK_GREEN";
            case AndonLight::BLINK_YELLOW: return "BLINK_YELLOW";
            case AndonLight::BLINK_BLUE:   return "BLINK_BLUE";
            case AndonLight::BLINK_RED:    return "BLINK_RED";
            default:                       return "UNKNOWN";
        }
    }

    void emitActuatorStatus(bool force = false) {
        if (!actuator) {
            return;
        }

        const uint8_t commandMask =
            actuator->commandedActiveMask();

        const uint8_t feedbackMask =
            actuator->feedbackActiveMask();

        const uint8_t jamMask =
            actuator->jamMask();

        const bool pcbFault =
            actuator->hasPCBFault();

        const bool changed =
            commandMask != lastActuatorCommandMask ||
            feedbackMask != lastActuatorFeedbackMask ||
            jamMask != lastActuatorJamMask ||
            pcbFault != lastActuatorPcbFault;

        if (!force && !changed) {
            return;
        }

        lastActuatorCommandMask = commandMask;
        lastActuatorFeedbackMask = feedbackMask;
        lastActuatorJamMask = jamMask;
        lastActuatorPcbFault = pcbFault;

        StaticJsonDocument<768> doc;

        doc["type"] = "actuator_status";
        doc["commanded_mask"] = commandMask;
        doc["feedback_mask"] = feedbackMask;
        doc["jam_mask"] = jamMask;
        doc["pcb_fault"] = pcbFault;
        doc["ts_ms"] = millis();

        doc["activate_cmd_min_v"] =
            CFG.actuator.activate_cmd_min_v;

        doc["deactivate_cmd_max_v"] =
            CFG.actuator.deactivate_cmd_max_v;

        doc["active_fb_max_v"] =
            CFG.actuator.active_fb_max_v;

        doc["inactive_fb_min_v"] =
            CFG.actuator.inactive_fb_min_v;

        JsonArray commandVolts =
            doc.createNestedArray("command_v");

        JsonArray feedbackVolts =
            doc.createNestedArray("feedback_v");

        for (uint8_t i = 0; i < NUM_ACTUATORS; ++i) {
            commandVolts.add(
                actuator->actuatorPositions[i]
            );

            feedbackVolts.add(
                actuator->feedbackSignals[i]
            );
        }

        serializeJson(doc, *io);
        io->println();
    }

    void sendAndonDiagnostics(AndonLight::States state,
                              uint32_t ms,
                              bool overrideActive,
                              AndonLight::States overrideState,
                              bool isEStop,
                              bool hasFault,
                              bool commsLost,
                              bool blockedOrStarved,
                              bool pausedOrJog,
                              bool batteryLow,
                              bool running,
                              const std::vector<AndonManager::FaultCode>& faults)
    {
        StaticJsonDocument<512> doc;
        JsonObject root = doc.createNestedObject("andon_diag");
        root["code"]    = static_cast<int>(state);
        root["state"]   = stateToString(state);
        root["ms"]      = (unsigned long)ms;
        root["override"]= overrideActive;
        if (overrideActive) {
            root["override_state"] = stateToString(overrideState);
        }

        JsonObject reasons = root.createNestedObject("reasons");
        reasons["isEStop"]            = isEStop;
        reasons["hasFault"]           = hasFault;
        reasons["isCommsLost"]        = commsLost;
        reasons["isBlockedOrStarved"] = blockedOrStarved;
        reasons["isPausedOrJog"]      = pausedOrJog;
        reasons["isBatteryLow"]       = batteryLow;
        reasons["isRunning"]          = running;

        JsonArray faultsArr  = root.createNestedArray("faults");
        JsonArray modulesArr = root.createNestedArray("fault_modules");
        for (auto f : faults) {
            faultsArr.add(AndonManager::faultToKey(f));
            modulesArr.add(AndonManager::faultToModuleId(f));
        }

        serializeJson(doc, *io); io->println();
    }

    // Old overload for backward compatibility
    void sendAndonDiagnostics(AndonLight::States state,
                              uint32_t ms,
                              bool overrideActive,
                              AndonLight::States overrideState,
                              bool isEStop,
                              bool hasFault,
                              bool commsLost,
                              bool blockedOrStarved,
                              bool pausedOrJog,
                              bool batteryLow,
                              bool running)
    {
        static std::vector<AndonManager::FaultCode> empty;
        sendAndonDiagnostics(state, ms, overrideActive, overrideState,
                             isEStop, hasFault, commsLost, blockedOrStarved,
                             pausedOrJog, batteryLow, running, empty);
    }

    void updateDirectionTest() {
        if (!directionTest_.active) return;

        if (static_cast<int32_t>(millis() - directionTest_.expiresAtMs) >= 0) {
            stopDirectionTest("lease_expired");
            return;
        }

        float command[4] = {0.0f, 0.0f, 0.0f, 0.0f};
        command[directionTest_.axis] = 0.025f;
        motors->setSpeeds(command[0], command[1], command[2], command[3]);
    }

    enum class LinkState { CONNECTED, DISCONNECTED };
    LinkState state;

    // Touched by ISR, keep volatile
    volatile int thisDelay     = 0;     // ~ms scale (decremented at 1 kHz)
    volatile int timeout       = 0;     // ms left before link considered dead (1 kHz)
    volatile int receiveDelay  = 0;     // LED flash counter (see ISR decrement rate)

    void setup() {
        state = LinkState::DISCONNECTED;

        pinMode(RED_LED, OUTPUT);
        digitalWrite(RED_LED, LOW);

        io->println("{\"status\":\"serial_started\"}");
    }

    void stateMachine(void) {
        receiveLinux(); // refresh timeout if traffic arrives
        updateDirectionTest();
        updateActuatorPreviewLease();  
        checkTriggers(); // encoder-driven triggers

        unsigned long now = millis();
        if (now - lastEncoderEmitMs >= encoderEmitPeriodMs) {
            lastEncoderEmitMs = now;
            emitEncoderStatus();
        }

        if (now - lastActuatorEmitMs >= actuatorEmitPeriodMs) {
            lastActuatorEmitMs = now;

            // Normally emit only when one of the actuator masks changes.
            // Every 2 seconds, force a full snapshot even if nothing changed.
            const bool forceSnapshot =
                now - lastActuatorSnapshotMs >=
                actuatorSnapshotPeriodMs;

            emitActuatorStatus(forceSnapshot);

            if (forceSnapshot) {
                lastActuatorSnapshotMs = now;
            }
        }

        switch (state) {
        case LinkState::CONNECTED:
            // If no traffic for too long, consider disconnected
            if (!thisDelay && !timeout) {
                thisDelay = 500;
                state = LinkState::DISCONNECTED;

                if (actuatorPreview_.active) {
                    stopActuatorPreview("transport_lost");
                }

                motors->STOP();
            }
            if (!thisDelay) {
                thisDelay = 500;
                // io->println("connected");
            }
            break;

        case LinkState::DISCONNECTED:
            // If we saw traffic recently, go connected
            if (!thisDelay && timeout > 0) {
                thisDelay = 500;
                state = LinkState::CONNECTED;
            }
            if (!thisDelay) {
                thisDelay = 500;
                // io->println("disconnected");
            }
            break;

        default:
            break;
        }
    }

    void receiveLinux(void) {
        if (!receiveDelay) digitalWrite(RED_LED, HIGH);

        while (io->available() > 0) {
            recvLineJson();
            if (!newData) {
                break;  
            }
            processMessage();
        }
    }


    void recvLineJson() {
        static size_t ndx = 0;
        static bool inFrame = false;
        static bool overflow = false;

        while (io->available() > 0 && !newData) {
            char c = io->read();
            timeout = CFG.serial.rx_keepalive_ms;

            if (!inFrame) {
                if (c == '{') {
                    inFrame = true;
                    ndx = 0;
                    overflow = false;
                    receivedChars[ndx++] = c;
                }
                continue;
            }

            if (c == '\n' || c == '\r') {
                if (ndx > 0 && !overflow) {
                    receivedChars[ndx] = '\0';
                    newData = true;
                    receiveDelay = CFG.serial.rx_led_flash_ms;
                }
                ndx = 0;
                inFrame = false;
                overflow = false;
                return;
            }

            if (ndx < numChars - 1) {
                receivedChars[ndx++] = c;
            } else {
                overflow = true;
            }
        }
    }


    void recvWithStartEndMarkers(void) {
        static bool  recvInProcess = false;
        static size_t ndx = 0;
        const char startMarker = '<';
        const char endMarker   = '>';
        char rc;

        while (io->available() > 0 && newData == false) {
            rc = io->read();
            
            timeout = CFG.serial.rx_keepalive_ms;;


            if (recvInProcess) {
                if (rc != endMarker) {
                    if (ndx < numChars - 1) {
                        receivedChars[ndx++] = rc;
                    } else {
                        // truncate gracefully
                        receivedChars[numChars - 2] = '\0';
                    }
                } else {
                    receivedChars[ndx] = '\0';
                    recvInProcess = false;
                    ndx = 0;
                    newData = true;
                }
            } else if (rc == startMarker) {
                recvInProcess = true;
                ndx = 0;

                timeout = CFG.serial.rx_keepalive_ms;
                receiveDelay = CFG.serial.rx_led_flash_ms;
            }
        }
    }

    void processMessage() {
        if (!newData) return;

        depackage();
        newData = false;

        // 2 s comms keep-alive; timer ISR decrements at 1 kHz.
        timeout = 2000;

        // Brief LED flash to indicate RX
        // If your ISR decrements at 1 kHz, set ~50 for 50 ms
        // If at 10 kHz, set ~500 for 50 ms, or move decrement to 1 kHz.
        receiveDelay = 50;

        // Optional: toggle LED state on message
        digitalWrite(RED_LED, LED_STATE);
    }

    void depackage(void) {
        jsonPacket.clear();

        const size_t inputLength = strnlen(
            receivedChars,
            numChars
        );

        DeserializationError error =
            deserializeJson(
                jsonPacket,
                receivedChars
            );

        if (error) {
            StaticJsonDocument<384> response;

            response["type"] =
                "serial_rx_error";

            response["error"] =
                "json_parse_failed";

            response["code"] =
                error.c_str();

            response["input_length"] =
                inputLength;

            response["buffer_capacity"] =
                numChars;

            response["json_capacity"] =
                jsonPacket.capacity();

            response["ts_ms"] =
                millis();

            serializeJson(
                response,
                *io
            );

            io->println();
            return;
        }

        if (jsonPacket.overflowed()) {
            StaticJsonDocument<320> response;

            response["type"] =
                "serial_rx_error";

            response["error"] =
                "json_document_overflow";

            response["input_length"] =
                inputLength;

            response["json_capacity"] =
                jsonPacket.capacity();

            response["ts_ms"] =
                millis();

            serializeJson(
                response,
                *io
            );

            io->println();
            return;
        }

        updateParameters();
    }


    // Helper: parse Andon state string → enum
    static bool parseAndonState(const String& s, AndonLight::States& out) {
        if      (s.equalsIgnoreCase("GREEN"))         out = AndonLight::GREEN;
        else if (s.equalsIgnoreCase("YELLOW"))        out = AndonLight::YELLOW;
        else if (s.equalsIgnoreCase("BLUE"))          out = AndonLight::BLUE;
        else if (s.equalsIgnoreCase("RED"))           out = AndonLight::RED;
        else if (s.equalsIgnoreCase("OFF"))           out = AndonLight::OFF;
        else if (s.equalsIgnoreCase("BLINK_GREEN"))   out = AndonLight::BLINK_GREEN;
        else if (s.equalsIgnoreCase("BLINK_YELLOW"))  out = AndonLight::BLINK_YELLOW;
        else if (s.equalsIgnoreCase("BLINK_BLUE"))    out = AndonLight::BLINK_BLUE;
        else if (s.equalsIgnoreCase("BLINK_RED"))     out = AndonLight::BLINK_RED;
        else return false;
        return true;
    }

    void stopDirectionTest(const char* reason) {
        if (!directionTest_.active) return;

        const uint8_t axis = directionTest_.axis;
        const int32_t startCount = directionTest_.startCount;
        const int8_t testedMotorDirection = directionTest_.testedMotorDirection;
        const int8_t testedEncoderDirection = directionTest_.testedEncoderDirection;
        char transactionId[sizeof(directionTest_.transactionId)];
        strlcpy(transactionId, directionTest_.transactionId, sizeof(transactionId));

        motors->BRAKE_STOP();
        delay(30);

        int32_t endCount = startCount;
        int32_t speedQpps = 0;
        uint8_t encoderStatus = 0;
        uint8_t speedStatus = 0;
        const bool feedbackValid = motors->readMotorMotionFeedback(
            axis,
            endCount,
            speedQpps,
            encoderStatus,
            speedStatus
        );

        const int64_t delta64 =
            static_cast<int64_t>(endCount) - static_cast<int64_t>(startCount);
        const bool deltaFitsI32 =
            delta64 >= INT32_MIN && delta64 <= INT32_MAX;
        const int32_t rawDelta = deltaFitsI32
            ? static_cast<int32_t>(delta64)
            : 0;
        const int64_t normalizedDelta64 =
            delta64 * static_cast<int64_t>(testedEncoderDirection);

        directionTest_.active = false;
        directionTest_.expiresAtMs = 0;
        directionTest_.transactionId[0] = '\0';

        StaticJsonDocument<512> response;
        response["type"] = "drive_direction_test_result";
        response["transaction_id"] = transactionId;
        response["axis"] = axis;
        response["ok"] = feedbackValid && deltaFitsI32;
        response["reason"] = reason;
        response["start_raw_count"] = startCount;
        response["end_raw_count"] = endCount;
        response["delta_raw_count"] = rawDelta;
        response["normalized_delta_count"] = normalizedDelta64;
        response["tested_motor_direction"] = testedMotorDirection;
        response["tested_encoder_direction"] = testedEncoderDirection;
        response["speed_qpps"] = speedQpps;
        response["encoder_status"] = encoderStatus;
        response["speed_status"] = speedStatus;
        response["ts_ms"] = millis();

        if (!feedbackValid) {
            response["error"] = "encoder_feedback_invalid";
        } else if (!deltaFitsI32) {
            response["error"] = "encoder_delta_overflow";
        }

        serializeJson(response, *io);
        io->println();
    }

    void updateParameters() {
        // Update motor speeds if present: speed0..speed3
        float requestedSpeeds[4] = {
            motors->speeds[0],
            motors->speeds[1],
            motors->speeds[2],
            motors->speeds[3]
        };

        String action = jsonPacket["action"];
        bool hasSpeedUpdate = false;

        for (int i = 0; i < 4; ++i) {
            String speedKey = "speed" + String(i);

            if (jsonPacket.containsKey(speedKey)) {
                float requested = jsonPacket[speedKey] | 0.0f;

                if (!isfinite(requested)) {
                    requested = 0.0f;
                }

                requestedSpeeds[i] = requested;
                hasSpeedUpdate = true;
            }
        }

        if (hasSpeedUpdate) {
            motors->setSpeeds(
                requestedSpeeds[0],
                requestedSpeeds[1],
                requestedSpeeds[2],
                requestedSpeeds[3]
            );
        }

        if (jsonPacket.containsKey("hb")) {
            return; 
        }

        if (!jsonPacket.containsKey("action")) {
            return;
        }

        else if (action.equalsIgnoreCase("confirm_clamp_state")) {
            StaticJsonDocument<320> response;
            response["type"] = "ack";
            response["id"] = "confirm_clamp_state";
            response["run_id"] = jsonPacket["run_id"] | "";
            response["transaction_id"] = jsonPacket["transaction_id"] | "";
            response["module_id"] = jsonPacket["id"] | "";
            response["ts_ms"] = millis();
            const bool accepted = diagnosticRunner_ &&
                diagnosticRunner_->confirmClampState(jsonPacket);
            response["ok"] = accepted;
            response["accepted"] = accepted;
            if (!accepted) response["error"] = "clamp_confirmation_rejected";
            serializeJson(response, *io); io->println();
        }
        else if (action.equalsIgnoreCase("confirm_andon_color")) {
            StaticJsonDocument<320> response;
            response["type"] = "ack";
            response["id"] = "confirm_andon_color";
            response["run_id"] = jsonPacket["run_id"] | "";
            response["transaction_id"] = jsonPacket["transaction_id"] | "";
            response["module_id"] = jsonPacket["id"] | "";
            response["color"] = jsonPacket["color"] | "";
            response["ts_ms"] = millis();
            const bool accepted = diagnosticRunner_ &&
                diagnosticRunner_->confirmAndonColor(jsonPacket);
            response["ok"] = accepted;
            response["accepted"] = accepted;
            if (!accepted) response["error"] = "andon_confirmation_rejected";
            serializeJson(response, *io); io->println();
        }


        if (
            action.equalsIgnoreCase("test_motor") ||
            action.equalsIgnoreCase("test_actuator") ||
            action.equalsIgnoreCase("calibrate_actuator") ||
            action.equalsIgnoreCase("test_sensor") ||
            action.equalsIgnoreCase("test_andon") ||
            action.equalsIgnoreCase("abort_diagnostic")
        ) {
            StaticJsonDocument<384> received;

            received["type"] =
                "diagnostic_command_received";

            received["action"] = action;

            received["run_id"] =
                jsonPacket["run_id"] | "";

            received["transaction_id"] =
                jsonPacket["transaction_id"] | "";

            received["id"] =
                jsonPacket["id"] | "";

            received["channel"] =
                jsonPacket["channel"] | -1;

            received["diagnostic_active"] =
                diagnosticRunner_ &&
                diagnosticRunner_->active();

            received["active_run_id"] =
                diagnosticRunner_
                    ? diagnosticRunner_->activeRunId()
                    : "";

            received["ts_ms"] = millis();

            serializeJson(received, *io);
            io->println();
        }

        if (action.equalsIgnoreCase("set_hmi_connected")) {
            if (!hmiConnected_) {
                io->println(
                    "{\"type\":\"error\","
                    "\"error\":\"hmi_connection_state_not_attached\"}"
                );
                return;
            }

            const bool connected = jsonPacket["connected"] | false;
            *hmiConnected_ = connected;

            StaticJsonDocument<128> response;
            response["type"] = "hmi_connection_status";
            response["connected"] = connected;
            response["ts_ms"] = millis();

            serializeJson(response, *io);
            io->println();
        }

        else if (action.equalsIgnoreCase("set_transition_actuator_voltage")) {
            const char* transactionId = jsonPacket["transaction_id"] | "";
            const float voltage = jsonPacket["voltage"] | NAN;
            StaticJsonDocument<256> response;
            response["type"] = "transition_actuator_voltage_ack";
            response["transaction_id"] = transactionId;
            response["ts_ms"] = millis();
            if (transactionId[0] == '\0') {
                response["ok"] = false;
                response["error"] =
                    "missing_transaction_id";
            } else if (
                !isfinite(voltage) ||
                voltage < 0.0f ||
                voltage > CFG.actuator.maxCommandVoltage
            ) {
                response["ok"] = false;
                response["error"] =
                    "invalid_actuator_voltage";
            } else if (actuatorPreview_.active) {
                response["ok"] = false;
                response["error"] =
                    "actuator_preview_active";
            } else if (
                ultrasonicEnabled_ &&
                *ultrasonicEnabled_
            ) {
                response["ok"] = false;
                response["error"] =
                    "process_active";
            } else {
                transitionActuatorVoltage_ = voltage;

                response["ok"] = true;
                response["voltage"] =
                    transitionActuatorVoltage_;
            }
            serializeJson(response, *io); io->println();
        }
        else if (action.equalsIgnoreCase("preview_actuator_voltage")) {
            handleActuatorPreview_();
        }
        else if (action.equalsIgnoreCase("stop_actuator_preview")) {
            const char* sessionId =
                jsonPacket["preview_session_id"] | "";

            if (
                actuatorPreview_.active &&
                sessionId[0] != '\0' &&
                strcmp(
                    sessionId,
                    actuatorPreview_.sessionId
                ) == 0
            ) {
                stopActuatorPreview("operator_stop");
            } else {
                StaticJsonDocument<224> response;

                response["type"] =
                    "actuator_preview_status";

                response["preview_session_id"] =
                    sessionId;

                response["active"] =
                    actuatorPreview_.active;

                response["ok"] = false;
                response["ts_ms"] = millis();

                if (sessionId[0] == '\0') {
                    response["error"] =
                        "missing_preview_session_id";
                } else if (!actuatorPreview_.active) {
                    response["error"] =
                        "preview_not_active";
                } else {
                    response["error"] =
                        "preview_session_mismatch";
                }

                serializeJson(response, *io);
                io->println();
            }
        }
        else if (action.equalsIgnoreCase("set_voltage")) {
            const int channel = jsonPacket["channel"] | -1;
            const float requestedVoltage = jsonPacket["voltage"] | NAN;

            StaticJsonDocument<224> response;
            response["type"] = "actuator_command_status";
            response["action"] = "set_voltage";
            response["channel"] = channel;
            response["ts_ms"] = millis();

            if (!actuator) {
                response["ok"] = false;
                response["error"] =
                    "actuator_not_attached";
            } else if (
                channel < 0 ||
                channel >= NUM_ACTUATORS
            ) {
                response["ok"] = false;
                response["error"] =
                    "invalid_actuator_channel";
            } else if (
                diagnosticRunner_ &&
                diagnosticRunner_->ownsActuatorOutputs()
            ) {
                response["ok"] = false;
                response["error"] =
                    "actuator_owned_by_diagnostic";
            } else if (
                actuatorPreview_.active &&
                channel == actuatorPreview_.channel
            ) {
                response["ok"] = false;
                response["error"] =
                    "actuator_owned_by_preview";
            } else if (
                ultrasonicEnabled_ &&
                *ultrasonicEnabled_
            ) {
                response["ok"] = false;
                response["error"] =
                    "process_active";
            } else if (!isfinite(requestedVoltage)) {
                response["ok"] = false;
                response["error"] =
                    "invalid_actuator_voltage";
            } else {
                const float appliedVoltage = constrain(
                    requestedVoltage,
                    0.0f,
                    CFG.actuator.maxCommandVoltage
                );

                actuator->actuatorPositions[channel] =
                    appliedVoltage;

                actuator->writeDAC(
                    static_cast<uint8_t>(channel),
                    appliedVoltage
                );

                response["ok"] = true;
                response["status"] = "OK";

                response["requested_voltage"] =
                    requestedVoltage;

                response["voltage"] =
                    appliedVoltage;

                response["clamped"] =
                    appliedVoltage != requestedVoltage;
            }

            serializeJson(response, *io);
            io->println();
        }
        else if (action.equalsIgnoreCase("read_feedback")) {
            const int channel = jsonPacket["channel"] | -1;

            StaticJsonDocument<192> response;
            response["type"] = "actuator_feedback";
            response["channel"] = channel;
            response["ts_ms"] = millis();

            if (!actuator) {
                response["ok"] = false;
                response["error"] = "actuator_not_attached";
            } else if (channel < 0 || channel >= NUM_ACTUATORS) {
                response["ok"] = false;
                response["error"] = "invalid_actuator_channel";
            } else {
                const float feedback = actuator->readADC(
                    static_cast<uint8_t>(channel)
                );
                actuator->feedbackSignals[channel] = feedback;

                response["ok"] = isfinite(feedback);
                response["feedback"] = feedback;
                if (!isfinite(feedback)) {
                    response["error"] = "invalid_actuator_feedback";
                }
            }

            serializeJson(response, *io);
            io->println();
        }
        else if (action.equalsIgnoreCase("get_actuator_status")) {
            emitActuatorStatus(true);
        }
        else if (action.equalsIgnoreCase("reset_encoder") || action.equalsIgnoreCase("set_encoder")) {
            const bool isReset = action.equalsIgnoreCase("reset_encoder");
            const char* transactionId = jsonPacket["transaction_id"] | "";
            const char* source = jsonPacket["source"] | (isReset ? "operator_reset" : "operator_set");
            const uint32_t expectedSession = jsonPacket["expected_session_id"] |
                (encoderSession_ ? encoderSession_->sessionId() : 0);

            StaticJsonDocument<768> response;
            response["type"] = isReset ? "encoder_reset" : "encoder_set";
            response["transaction_id"] = transactionId;
            response["source"] = source;
            response["encoder_session_id"] = encoderSession_ ? encoderSession_->sessionId() : 0;
            response["ts_ms"] = millis();

            if (!encoderSession_) {
                response["ok"] = false;
                response["valid"] = false;
                response["error"] = "encoder_session_not_attached";
                serializeJson(response, *io); io->println();
                return;
            }

            if (!encoderSession_->beginTransaction(expectedSession)) {
                response["ok"] = false;
                response["valid"] = false;
                response["error"] = encoderSession_->motorPowerPresent()
                    ? "encoder_session_mismatch" : "motor_power_absent";
                serializeJson(response, *io); io->println();
                return;
            }

            float requestedMm = 0.0f;
            if (!isReset) {
                if (jsonPacket.containsKey("position_mm")) {
                    requestedMm = jsonPacket["position_mm"] | 0.0f;
                } else {
                    const float radiusM = jsonPacket["radius_m"] | 0.0f;
                    requestedMm = radiusM * 1000.0f;
                }
            }

            if (!isfinite(requestedMm)) {
                encoderSession_->transactionFailed("invalid_position", transactionId);
                response["ok"] = false;
                response["valid"] = false;
                response["error"] = "invalid_position";
                serializeJson(response, *io); io->println();
                return;
            }

            motors->BRAKE_STOP();
            const bool verified = motors->setRobotRearDistanceMMVerified(requestedMm, 2.0f);
            if (!verified) {
                encoderSession_->transactionFailed(
                    "readback_failed",
                    transactionId
                );

             response["ok"] = false;
                response["valid"] = false;
                response["error"] = "readback_failed";

                response["encoder_read_mask"] =
                    motors->lastEncoderRestoreValidMask();

                response["restore_attempts"] =
                    motors->lastEncoderRestoreWriteAttempts();

                response["maximum_error_mm"] =
                    motors->lastEncoderRestoreMaximumErrorMM();

                response["requested_distance_mm"] =
                    requestedMm;
                response["motor_power_present"] =
                    encoderSession_->motorPowerPresent();

                JsonArray observedCounts =
                    response.createNestedArray(
                        "observed_counts"
                    );

                for (
                   uint8_t axis = 0;
                    axis < 4;
                    ++axis
                ) {
                observedCounts.add(
                        motors->encCounts[axis]
                    );
                }

                serializeJson(response, *io);
                io->println();

                return;
            }

            encoderSession_->transactionSucceeded(source, transactionId);
            const float confirmedMm = motors->getRobotRearDistanceMM();

            StaticJsonDocument<768> triggerResult;
            const bool triggersReconciled = reconcileTriggersToPosition_(
                static_cast<int>(confirmedMm),
                triggerResult
            );

            if (!triggersReconciled) {
                encoderSession_->transactionFailed(
                    "trigger_reconciliation_failed",
                    transactionId
                );

                response["ok"] = false;
                response["valid"] = false;
                response["error"] = "trigger_reconciliation_failed";
                response["rear_distance_mm"] = confirmedMm;
                response["radius_m"] = confirmedMm / 1000.0f;
                response["trigger_error"] = triggerResult["error"] | "unknown";
                response["trigger_count"] = triggerBuffer.size();
                serializeJson(response, *io);
                io->println();
                emitEncoderStatus();
                return;
            }

            response["ok"] = true;
            response["valid"] = true;
            response["rear_distance_mm"] = confirmedMm;
            response["radius_m"] = confirmedMm / 1000.0f;
            response["encoder_read_mask"] = motors->encoderReadValidMask();
            response["triggers_reconciled"] = true;
            response["trigger_count"] = triggerBuffer.size();
            response["trigger_reached_count"] = triggerResult["reached_count"] | 0;
            response["trigger_pending_count"] = triggerResult["pending_count"] | 0;
            response["commanded_mask"] = triggerResult["commanded_mask"] | 0;

            JsonArray counts = response.createNestedArray("counts");
            for (int i = 0; i < 4; ++i) {
                counts.add(motors->encCounts[i]);
            }

            serializeJson(response, *io);
            io->println();
            emitEncoderStatus();
        }
        else if (action.equalsIgnoreCase("STOP")) {
            if (actuatorPreview_.active) {
                stopActuatorPreview("operator_stop");
            }

            motors->STOP();
        }
        else if (action.equalsIgnoreCase("set_light")) {
            if (!jsonPacket.containsKey("state")) {
                io->println("{\"error\":\"Missing state for set_light\"}");
                return;
            }
            String stateStr = jsonPacket["state"];
            AndonLight::States newState;
            if (!parseAndonState(stateStr, newState)) {
                io->println("{\"error\":\"Invalid light state\"}");
                return;
            }

            // Recommended: route through AndonManager override if attached:
            if (andonMgr) {
                andonMgr->setOverride(newState);
            } else if (andonLight) {
                // Direct (bypasses priority logic)
                andonLight->setState(newState);
            }

            StaticJsonDocument<64> response;
            response["status"] = "light_updated";
            response["state"]  = stateStr;
            serializeJson(response, *io); io->println();
        }
        else if (action.equalsIgnoreCase("light_override")) {
            if (!jsonPacket.containsKey("state") || !andonMgr) {
                io->println("{\"error\":\"Missing state/manager\"}");
                return;
            }
            String s = jsonPacket["state"];
            AndonLight::States st;
            if (!parseAndonState(s, st)) {
                io->println("{\"error\":\"Invalid light state\"}");
                return;
            }
            andonMgr->setOverride(st);
            io->println("{\"status\":\"override_set\"}");
        }
        else if (action.equalsIgnoreCase("light_clear_override")) {
            if (andonMgr) {
                andonMgr->clearOverride();
                io->println("{\"status\":\"override_cleared\"}");
            }
        }
        else if (action.equalsIgnoreCase("set_triggers")) {
            const bool shouldClear = jsonPacket["clear"] | false;
            const bool begin = jsonPacket["begin"] | false;
            const bool commit = jsonPacket["commit"] | false;
            std::vector<Trigger> parsedTriggers;

            if (begin) {
                triggerEvaluationSuspended = true;
            }

            bool valid = true;
            const char* validationError = nullptr;

            const bool hasSingleTrigger =
                jsonPacket.containsKey("trigger");

            const bool hasTriggerArray =
                jsonPacket.containsKey("triggers");

            if (
                !hasSingleTrigger &&
                !hasTriggerArray &&
                !shouldClear &&
                !commit
            ) {
                valid = false;
                validationError =
                    "missing_trigger_payload";
            }

            if (valid && hasSingleTrigger) {
                JsonObject t = jsonPacket["trigger"].as<JsonObject>();

                if (
                    t.containsKey("threshold") &&
                    t.containsKey("activate") &&
                    t.containsKey("deactivate") &&
                    t.containsKey("delay")
                ) {
                    Trigger trig;
                    trig.threshold = t["threshold"];
                    trig.activate_channel = t["activate"];
                    trig.deactivate_channel = t["deactivate"];
                    trig.delay_seconds = t["delay"];
                    trig.hold_active = t["hold_active"] | false;
                    trig.triggered = false;
                    trig.triggerTime = 0;
                    trig.waitingToDeactivate = false;

                    if (
                        trig.threshold < 0 ||
                        trig.activate_channel < 0 ||
                        trig.activate_channel >= NUM_ACTUATORS ||
                        trig.deactivate_channel < 0 ||
                        trig.deactivate_channel >= NUM_ACTUATORS ||
                        !isfinite(trig.delay_seconds) ||
                        trig.delay_seconds < 0.0f
                    ) {
                        valid = false;
                        validationError = "invalid_trigger_values";
                    } else {
                        parsedTriggers.push_back(trig);
                    }
                } else {
                    valid = false;
                    validationError = "invalid_single_trigger_format";
                }
            }

            if (valid && hasTriggerArray) {
                JsonArray arr = jsonPacket["triggers"].as<JsonArray>();

                for (JsonObject t : arr) {
                    if (
                        !t.containsKey("threshold") ||
                        !t.containsKey("activate") ||
                        !t.containsKey("deactivate") ||
                        !t.containsKey("delay")
                    ) {
                        valid = false;
                        validationError = "invalid_trigger_format";
                        break;
                    }

                    Trigger trig;
                    trig.threshold = t["threshold"];
                    trig.activate_channel = t["activate"];
                    trig.deactivate_channel = t["deactivate"];
                    trig.delay_seconds = t["delay"];
                    trig.hold_active = t["hold_active"] | false;
                    trig.triggered = false;
                    trig.triggerTime = 0;
                    trig.waitingToDeactivate = false;

                    if (
                        trig.threshold < 0 ||
                        trig.activate_channel < 0 ||
                        trig.activate_channel >= NUM_ACTUATORS ||
                        trig.deactivate_channel < 0 ||
                        trig.deactivate_channel >= NUM_ACTUATORS ||
                        !isfinite(trig.delay_seconds) ||
                        trig.delay_seconds < 0.0f
                    ) {
                        valid = false;
                        validationError = "invalid_trigger_values";
                        break;
                    }
                    parsedTriggers.push_back(trig);
                }
            }

            if (!valid) {
                StaticJsonDocument<192> response;
                response["status"] = "trigger_load_failed";
                response["ok"] = false;
                response["error"] = validationError;
                response["count"] = triggerBuffer.size();
                response["received_count"] = parsedTriggers.size();
                response["ts_ms"] = millis();
                serializeJson(response, *io);
                io->println();
                return;
            }

            if (shouldClear) {
                triggerBuffer.clear();
            }

            triggerBuffer.insert(
                triggerBuffer.end(),
                parsedTriggers.begin(),
                parsedTriggers.end()
            );

            if (commit) {
                StaticJsonDocument<768> response;

                const int currentPositionMm = motors
                    ? static_cast<int>(
                        motors->getRobotRearDistanceMM()
                    )
                    : 0;

                const bool reconciled =
                    reconcileTriggersToPosition_(
                        currentPositionMm,
                        response
                    );

                /*
                * Resume trigger evaluation only after the complete validated table
                * has been reconciled successfully.
                *
                * On failure, evaluation remains suspended so a partial or invalid
                * table cannot affect actuator outputs.
                */
                if (reconciled) {
                    triggerEvaluationSuspended = false;
                }

                /*
                * The commit acknowledgement is the authoritative result of this
                * operation and must be transmitted before supplementary telemetry.
                */
                const size_t bytesWritten =
                    serializeJson(
                        response,
                        *io
                    );

                io->println();

                if (bytesWritten == 0) {
                    io->println(
                        "{\"status\":\"trigger_load_failed\","
                        "\"ok\":false,"
                        "\"error\":\"commit_ack_serialization_failed\"}"
                    );

                    triggerEvaluationSuspended = true;
                    return;
                }

                /*
                * Publish actuator telemetry only after the commit acknowledgement.
                * The acknowledgement already contains the masks needed to verify
                * the reconciliation result, so this message is supplementary.
                */
                emitActuatorStatus(true);
                return;
            }

            StaticJsonDocument<128> response;
            response["status"] = "triggers_loaded";
            response["ok"] = true;
            response["count"] = triggerBuffer.size();
            response["loading"] = triggerEvaluationSuspended;
            response["ts_ms"] = millis();
            serializeJson(response, *io);
            io->println();
        }
        else if (
            action.equalsIgnoreCase(
                "set_feedforward_configuration"
            )
        ) {
            const char* transactionId =
                jsonPacket[
                    "transaction_id"
                ] | "";

            StaticJsonDocument<384>
                response;

            response["type"] =
                "feedforward_configuration_ack";

            response["transaction_id"] =
                transactionId;

            response["ts_ms"] =
                millis();

            if (
                transactionId[0] == '\0'
            ) {
                response["ok"] = false;

                response["error"] =
                    "missing_transaction_id";
            }
            else if (
                !ultrasonicEnabled_
            ) {
                response["ok"] = false;

                response["error"] =
                    "ultrasonic_process_not_attached";
            }
            else if (
                *ultrasonicEnabled_
            ) {
                response["ok"] = false;

                response["error"] =
                    "process_active";
            }
            else if (
                !jsonPacket["ff"]
                    .is<JsonArray>()
            ) {
                response["ok"] = false;

                response["error"] =
                    "missing_ff_array";
            }
            else {
                FeedforwardConfiguration::Values
                    candidate;

                const char*
                    validationError = nullptr;

                const bool valid =
                    FeedforwardConfiguration
                    ::parseAndValidate(
                        jsonPacket["ff"]
                            .as<JsonArrayConst>(),
                        candidate,
                        validationError
                    );

                if (!valid) {
                    response["ok"] = false;

                    response["error"] =
                        validationError
                            ? validationError
                            : "invalid_ff_values";
                }
                else {
                    FeedforwardConfiguration
                        ::apply(
                            candidate
                        );

                    if (ultrasonic_) {
                        ultrasonic_->
                            processSpeed = 0.0f;

                        ultrasonic_->
                            currentSpeed = 0.0f;

                        ultrasonic_->
                            glueState =
                                Ultrasonic
                                ::WAIT_FOR_GLUE;

                        ultrasonic_->
                            goodGlueCounter = 0;

                        ultrasonic_->
                            badReadStreak = 0;

                        ultrasonic_->
                            distanceFilterInitialized =
                                false;
                    }

                    motors->BRAKE_STOP();

                    response["ok"] = true;

                    JsonArray appliedValues =
                        response
                        .createNestedArray(
                            "ff"
                        );

                    FeedforwardConfiguration
                        ::writeJson(
                            appliedValues,
                            candidate
                        );
                }
            }

            if (response.overflowed()) {
                response.clear();

                response["type"] =
                    "feedforward_configuration_ack";

                response["transaction_id"] =
                    transactionId;

                response["ok"] = false;

                response["error"] =
                    "acknowledgement_json_overflow";

                response["ts_ms"] =
                    millis();
            }

            const size_t bytesWritten =
                serializeJson(
                    response,
                    *io
                );

            io->println();

            if (bytesWritten == 0) {
                StaticJsonDocument<192>
                    fallback;

                fallback["type"] =
                    "feedforward_configuration_ack";

                fallback["transaction_id"] =
                    transactionId;

                fallback["ok"] = false;

                fallback["error"] =
                    "acknowledgement_serialization_failed";

                fallback["ts_ms"] =
                    millis();

                serializeJson(
                    fallback,
                    *io
                );

                io->println();
            }
        }
        else if (action.equalsIgnoreCase("set_motor_tuning")) {
            int32_t maxSpeed = jsonPacket["max_speed"] | 2500;
            int32_t accel    = jsonPacket["accel"] | 4250;
            int32_t decel    = jsonPacket["decel"] | 8500;

            motors->maxCommandQpps = maxSpeed;
            motors->accelQppsPerSec = (accel > decel) ? accel : decel;
            motors->brakeDecelQppsPerSec = decel;

            // These are sent in physical units for the PID/process slew limiter.
            float accelMps2 = jsonPacket["accel_mps2"] | CFG.ultrasonic.motion_accel_mps2;
            float decelMps2 = jsonPacket["decel_mps2"] | CFG.ultrasonic.motion_decel_mps2;

            if (accelMps2 < 0.0f) accelMps2 = CFG.ultrasonic.motion_accel_mps2;
            if (decelMps2 < 0.0f) decelMps2 = CFG.ultrasonic.motion_decel_mps2;

            //if (ultrasonic_) {
            //    ultrasonic_->maxAccelMps2 = accelMps2;
            //    ultrasonic_->maxDecelMps2 = decelMps2;
            //}

            StaticJsonDocument<192> response;
            response["type"] = "ack";
            response["ok"] = true;
            response["info"] = "motor_tuning_set";
            response["max_speed_qpps"] = maxSpeed;
            response["accel_qpps_s"] = accel;
            response["decel_qpps_s"] = decel;
            response["roboclaw_accel_qpps_s"] = motors->accelQppsPerSec;
            response["accel_mps2"] = accelMps2;
            response["decel_mps2"] = decelMps2;
            serializeJson(response, *io); 
            io->println();
        }
        else if (
            action.equalsIgnoreCase(
                "set_motor_direction"
            )
        ) {
            const char* transactionId =
                jsonPacket[
                    "transaction_id"
                ] | "";

            StaticJsonDocument<256>
                response;

            response["type"] =
                "motor_direction_ack";

            response["transaction_id"] =
                transactionId;

            response["ts_ms"] =
                millis();

            if (
                transactionId[0] == '\0'
            ) {
                response["ok"] = false;

                response["error"] =
                    "missing_transaction_id";

                serializeJson(
                    response,
                    *io
                );

                io->println();

                return;
            }

            if (
                !jsonPacket["directions"]
                    .is<JsonArray>()
            ) {
                response["ok"] = false;

                response["error"] =
                    "missing_directions_array";

                serializeJson(
                    response,
                    *io
                );

                io->println();

                return;
            }

            JsonArrayConst directions =
                jsonPacket["directions"]
                    .as<JsonArrayConst>();

            if (directions.size() != 4) {
                response["ok"] = false;

                response["error"] =
                    "invalid_direction_count";

                serializeJson(
                    response,
                    *io
                );

                io->println();

                return;
            }

            int requested[4];

            for (
                uint8_t index = 0;
                index < 4;
                ++index
            ) {
                const int value =
                    directions[index]
                        .as<int>();

                if (
                    value != 1
                    && value != -1
                ) {
                    response["ok"] = false;

                    response["error"] =
                        "invalid_direction_value";

                    response["invalid_index"] =
                        index;

                    serializeJson(
                        response,
                        *io
                    );

                    io->println();

                    return;
                }

                requested[index] =
                    value;
            }

            if (
                ultrasonicEnabled_
                && *ultrasonicEnabled_
            ) {
                response["ok"] = false;

                response["error"] =
                    "process_active";

                serializeJson(
                    response,
                    *io
                );

                io->println();

                return;
            }

            motors->BRAKE_STOP();

            motors->setMotorDirections(
                requested[0],
                requested[1],
                requested[2],
                requested[3]
            );

            response["ok"] = true;

            JsonArray applied =
                response.createNestedArray(
                    "directions"
                );

            applied.add(
                motors->motorDirection[0]
            );

            applied.add(
                motors->motorDirection[1]
            );

            applied.add(
                motors->motorDirection[2]
            );

            applied.add(
                motors->motorDirection[3]
            );

            serializeJson(
                response,
                *io
            );

            io->println();
        }
        else if (action.equalsIgnoreCase("get_firmware")) {
            Version::printJson(*io);
        }
        else if (action.equalsIgnoreCase("get_firmware_features")) {
            emitFirmwareFeatures();
        }
        else if (
            action.equalsIgnoreCase(
                "start_process"
            )
        ) {
            const char* transactionId =
                jsonPacket["transaction_id"] | "";

            if (transactionId[0] == '\0') {
                emitProcessStartAck(
                    transactionId,
                    false,
                    "rejected",
                    nullptr,
                    "missing_transaction_id"
                );

                emitProcessStatus(
                    false,
                    "missing_transaction_id"
                );

                return;
            }

            if (
                !ultrasonicEnabled_
                || !ultrasonic_
            ) {
                emitProcessStartAck(
                    transactionId,
                    false,
                    "rejected",
                    nullptr,
                    "ultrasonic_process_not_attached"
                );

                emitProcessStatus(
                    false,
                    "ultrasonic_process_not_attached"
                );

                return;
            }

            if (
                diagnosticRunner_
                && diagnosticRunner_->active()
            ) {
                emitProcessStartAck(
                    transactionId,
                    false,
                    "blocked",
                    nullptr,
                    "diagnostic_active"
                );

                emitProcessStatus(
                    false,
                    "diagnostic_active"
                );

                return;
            }

            if (
                actuator
                && actuator->hasPCBFault()
            ) {
                *ultrasonicEnabled_ = false;
                motors->BRAKE_STOP();

                emitProcessStartAck(
                    transactionId,
                    false,
                    "blocked",
                    nullptr,
                    "actuator_dac_adc_disconnected"
                );

                emitProcessStatus(
                    false,
                    "actuator_dac_adc_disconnected"
                );

                return;
            }

            if (
                motors->isMotionInhibited()
            ) {
                *ultrasonicEnabled_ = false;
                motors->BRAKE_STOP();

                emitProcessStartAck(
                    transactionId,
                    false,
                    "blocked",
                    nullptr,
                    "motion_inhibited"
                );

                emitProcessStatus(
                    false,
                    "motion_inhibited"
                );

                return;
            }

            if (actuatorPreview_.active) {
                stopActuatorPreview("process_start");
            }

            *ultrasonicEnabled_ = true;

            ultrasonic_->processSpeed = 0.0f;
            ultrasonic_->currentSpeed = 0.0f;

            emitProcessStartAck(
                transactionId,
                true,
                "running",
                "operator_start",
                nullptr
            );

            emitProcessStatus(
                true,
                "operator_start"
            );
        }
        else if (
            action.equalsIgnoreCase(
                "get_process_status"
            )
        ) {
            const bool active =
                ultrasonicEnabled_
                && *ultrasonicEnabled_;

            emitProcessStatus(
                active,
                "status_request"
            );
        }
        else if (
            action.equalsIgnoreCase(
                "stop_process"
            )
        ) {
            if (ultrasonicEnabled_) {
                *ultrasonicEnabled_ = false;
            }

            motors->BRAKE_STOP();

            if (ultrasonicServo_) {
                ultrasonicServo_->deactivate();
            }

            emitProcessStatus(
                false,
                "operator_stop"
            );
        }
        else if (action.equalsIgnoreCase("set_speed")) {
            float speed = jsonPacket["speed"] | 0.0f;
            if (ultrasonic_) {
                ultrasonic_->processSpeed = speed;
                ultrasonic_->currentSpeed = speed;
            }

            StaticJsonDocument<64> response;
            response["status"] = "speed_set";
            response["speed"]  = speed;
            serializeJson(response, *io); io->println();
        }
        else if (action.equalsIgnoreCase("shutdown")) {
            if (actuatorPreview_.active) {
                stopActuatorPreview("shutdown");
            }

            motors->STOP();

            if (ultrasonicEnabled_) {
                *ultrasonicEnabled_ = false;
            }

            triggerEvaluationSuspended = true;
            triggerBuffer.clear();
            serialStarted = false;

            StaticJsonDocument<96> response;

            response["status"] =
                "shutdown_complete";

            response["actuator_preview_active"] =
                false;

            response["ts_ms"] = millis();

            serializeJson(response, *io);
            io->println();
        }
        else if (action.equalsIgnoreCase("confirm_actuator_extension")) {
            const char* runId = jsonPacket["run_id"] | "";
            const char* transactionId = jsonPacket["transaction_id"] | "";
            const char* moduleId = jsonPacket["id"] | "";

            StaticJsonDocument<320> response;
            response["type"] = "ack";
            response["id"] = "confirm_actuator_extension";
            response["run_id"] = runId;
            response["transaction_id"] = transactionId;
            response["module_id"] = moduleId;
            response["ts_ms"] = millis();

            if (!diagnosticRunner_) {
                response["type"] = "error";
                response["ok"] = false;
                response["error"] = "diagnostic_runner_not_attached";
                serializeJson(response, *io);
                io->println();
                return;
            }

            const bool accepted =
                diagnosticRunner_->confirmActuatorExtension(jsonPacket);

            response["type"] = accepted ? "ack" : "error";
            response["ok"] = accepted;
            response["accepted"] = accepted;
            if (!accepted) {
                response["error"] = "actuator_extension_confirmation_rejected";
            }

            serializeJson(response, *io);
            io->println();
        }
        else if (
            action.equalsIgnoreCase("test_motor") ||
            action.equalsIgnoreCase("test_actuator") ||
            action.equalsIgnoreCase("calibrate_actuator") ||
            action.equalsIgnoreCase("test_sensor") ||
            action.equalsIgnoreCase("test_andon")
        ) {
            if (!diagnosticRunner_) {
                StaticJsonDocument<320> response;

                response["type"] = "test_result";
                response["id"] = jsonPacket["id"] | "";
                response["category"] = "diagnostic";
                response["run_id"] = jsonPacket["run_id"] | "";
                response["transaction_id"] =
                    jsonPacket["transaction_id"] | "";
                response["pass"] = false;
                response["reason"] =
                    "diagnostic_runner_not_attached";

                response.createNestedObject(
                    "measurements"
                );

                serializeJson(response, *io);
                io->println();
                return;
            }

            if (actuatorPreview_.active) {
                stopActuatorPreview(
                    "diagnostic_start"
                );
            }

            diagnosticRunner_->start(
                jsonPacket
            );
        }
        else if (
            action.equalsIgnoreCase(
                "abort_diagnostic"
            )
        ) {
            const char* runId =
                jsonPacket["run_id"] | "";

            const bool accepted =
                diagnosticRunner_ &&
                diagnosticRunner_->abort(
                    runId
                );

            StaticJsonDocument<224> response;

            response["type"] =
                accepted ? "ack" : "error";

            response["id"] =
                "abort_diagnostic";

            response["ok"] = accepted;
            response["run_id"] = runId;

            if (!accepted) {
                response["error"] =
                    "diagnostic_abort_rejected";
            }

            serializeJson(response, *io);
            io->println();
        }
        else if (action.equalsIgnoreCase("ping")) {
            StaticJsonDocument<128> response;
            response["status"] = "pong";
            response["uptime_ms"] = millis();
            serializeJson(response, *io);
            io->println();
        }
        else if (
            action.equalsIgnoreCase(
                "set_drive_direction_configuration"
            )
        ) {
            const char* transactionId =
                jsonPacket["transaction_id"] | "";

            JsonArrayConst motorValues =
                jsonPacket["motor_directions"]
                    .as<JsonArrayConst>();

            JsonArrayConst encoderValues =
                jsonPacket["encoder_directions"]
                    .as<JsonArrayConst>();

            StaticJsonDocument<384> response;

            response["type"] =
                "drive_direction_configuration_ack";
            response["transaction_id"] =
                transactionId;
            response["ts_ms"] =
                millis();

            bool valid =
                transactionId[0] != '\0' &&
                motorValues.size() == 4 &&
                encoderValues.size() == 4;

            int8_t motor[4] = {};
            int8_t encoder[4] = {};

            for (
                uint8_t i = 0;
                valid && i < 4;
                ++i
            ) {
                motor[i] =
                    motorValues[i].as<int>();

                encoder[i] =
                    encoderValues[i].as<int>();

                valid =
                    (motor[i] == 1 || motor[i] == -1) &&
                    (encoder[i] == 1 || encoder[i] == -1);
            }

            const bool processActive =
                ultrasonicEnabled_ &&
                *ultrasonicEnabled_;

            if (!valid) {
                response["ok"] = false;
                response["error"] =
                    "invalid_drive_direction_configuration";
            } else if (!encoderSession_) {
                response["ok"] = false;
                response["error"] =
                    "encoder_session_not_attached";
            } else if (directionTest_.active) {
                response["ok"] = false;
                response["error"] =
                    "direction_test_active";
            } else if (processActive) {
                response["ok"] = false;
                response["error"] =
                    "process_active";
            } else if (
                diagnosticRunner_ &&
                diagnosticRunner_->active()
            ) {
                response["ok"] = false;
                response["error"] =
                    "diagnostic_active";
            } else {
                applyDriveDirectionConfiguration_(
                    motor,
                    encoder,
                    transactionId,
                    response
                );
            }

            serializeJson(response, *io);
            io->println();
        }
        else if (action.equalsIgnoreCase("stop_drive_direction_test")) {
            const char* transactionId = jsonPacket["transaction_id"] | "";
            const bool matches = directionTest_.active &&
                strcmp(transactionId, directionTest_.transactionId) == 0;

            if (matches) {
                stopDirectionTest("operator_release");
            } else {
                motors->BRAKE_STOP();
                StaticJsonDocument<224> response;
                response["type"] = "drive_direction_test_result";
                response["transaction_id"] = transactionId;
                response["ok"] = false;
                response["error"] = "direction_test_transaction_mismatch";
                response["ts_ms"] = millis();
                serializeJson(response, *io);
                io->println();
            }
        }
        else if (action.equalsIgnoreCase("start_drive_direction_test")) {
            handleStartDriveDirectionTest_();
        }
        else if (action.equalsIgnoreCase("jog")) {
            if (!jogControl) {
                StaticJsonDocument<192> response;
                response["type"] = "jog_status";
                response["id"] = "jog";
                response["ok"] = false;
                response["accepted"] = false;
                response["active"] = false;
                response["reason"] = "jog_not_attached";
                serializeJson(response, *io);
                io->println();
                return;
            }

            const char* sessionId = jsonPacket["jog_session_id"] | "";
            const char* dirStr = jsonPacket["dir"] | "";
            int direction = 0;

            if (strcasecmp(dirStr, "forward") == 0) {
                direction = 1;
            } else if (strcasecmp(dirStr, "backward") == 0) {
                direction = -1;
            }

            const float speed = jsonPacket["speed"] | CFG.jog.jog_speed_max_ms;
            const unsigned long leaseMs = jsonPacket["lease_ms"] | 250;
            const uint32_t seq = jsonPacket["seq"] | 0;

            const JogControl::RemoteJogResult result =
                jogControl->startOrRefreshRemoteJog(
                    sessionId,
                    direction,
                    speed,
                    leaseMs,
                    seq
                );

            const bool accepted =
                result == JogControl::RemoteJogResult::ACCEPTED;

            StaticJsonDocument<256> response;
            response["type"] = "jog_status";
            response["id"] = "jog";
            response["ok"] = accepted;
            response["accepted"] = accepted;
            response["active"] = accepted && jogControl->isRemoteActive();
            response["jog_session_id"] = sessionId;
            response["seq"] = seq;
            response["direction"] = dirStr;
            response["reason"] = JogControl::remoteJogResultToString(result);
            response["motion_inhibited"] = motors->isMotionInhibited();
            response["last_accepted_seq"] =
                jogControl->lastAcceptedRemoteSequence();

            serializeJson(response, *io);
            io->println();
        }
        else if (action.equalsIgnoreCase("jog_stop")) {
            const char* sessionId = jsonPacket["jog_session_id"] | "";
            const uint32_t seq = jsonPacket["seq"] | 0;
            const bool stopped = jogControl &&
                jogControl->stopRemoteJog(sessionId, seq);

            StaticJsonDocument<224> response;
            response["type"] = "jog_status";
            response["id"] = "jog_stop";
            response["ok"] = stopped;
            response["accepted"] = stopped;
            response["active"] = jogControl && jogControl->isRemoteActive();
            response["jog_session_id"] = sessionId;
            response["seq"] = seq;
            response["reason"] = stopped ? "stopped" : "stop_rejected";
            response["motion_inhibited"] = motors->isMotionInhibited();

            serializeJson(response, *io);
            io->println();
        }
    }
    
    void checkTriggers() {
        if (
            triggerEvaluationSuspended ||
            actuatorPreview_.active ||
            !actuator ||
            !motors ||
            (diagnosticRunner_ && diagnosticRunner_->ownsActuatorOutputs())
        ) {
            return;
        }

        const int currentPos = motors->getRobotRearDistanceMM();
        const unsigned long now = millis();

        for (auto& trig : triggerBuffer) {
            if (!trig.triggered && currentPos >= trig.threshold) {
                const int activateChannel = trig.activate_channel;

                if (activateChannel < 0 || activateChannel >= NUM_ACTUATORS) {
                    StaticJsonDocument<192> error;
                    error["type"] = "error";
                    error["module"] = "actuator";
                    error["error"] = "invalid_trigger_activate_channel";
                    error["channel"] = activateChannel;
                    error["threshold"] = trig.threshold;
                    serializeJson(error, *io);
                    io->println();
                    trig.triggered = true;
                    trig.waitingToDeactivate = false;
                    continue;
                }

                const float activeVoltage = constrain(
                    transitionActuatorVoltage_, 0.0f, CFG.actuator.maxCommandVoltage
                );

                actuator->actuatorPositions[activateChannel] = activeVoltage;
                actuator->writeDAC(
                    static_cast<uint8_t>(activateChannel),
                    activeVoltage
                );

                trig.triggerTime = now;
                trig.waitingToDeactivate = !trig.hold_active;
                trig.triggered = true;
                emitActuatorStatus(true);

                StaticJsonDocument<256> response;
                response["type"] = "trigger_event";
                response["event"] = "activated";
                response["trigger_reached"] = true;
                response["channel"] = activateChannel;
                response["deactivate_channel"] = trig.deactivate_channel;
                response["threshold"] = trig.threshold;
                response["current_position_mm"] = currentPos;
                response["command_voltage"] = activeVoltage;
                response["delay_seconds"] = trig.delay_seconds;
                response["commanded_mask"] = actuator->commandedActiveMask();
                response["feedback_mask"] = actuator->feedbackActiveMask();
                response["pcb_fault"] = actuator->hasPCBFault();
                serializeJson(response, *io);
                io->println();
            }

            if (trig.waitingToDeactivate) {
                const float safeDelaySeconds = trig.delay_seconds < 0.0f
                    ? 0.0f
                    : trig.delay_seconds;
                const unsigned long delayMs = static_cast<unsigned long>(
                    safeDelaySeconds * 1000.0f
                );

                if ((unsigned long)(now - trig.triggerTime) >= delayMs) {
                    const int deactivateChannel = trig.deactivate_channel;

                    if (
                        deactivateChannel < 0 ||
                        deactivateChannel >= NUM_ACTUATORS
                    ) {
                        StaticJsonDocument<192> error;
                        error["type"] = "error";
                        error["module"] = "actuator";
                        error["error"] = "invalid_trigger_deactivate_channel";
                        error["channel"] = deactivateChannel;
                        error["threshold"] = trig.threshold;
                        serializeJson(error, *io);
                        io->println();
                        trig.waitingToDeactivate = false;
                        continue;
                    }

                    const float inactiveVoltage = 0.0f;
                    actuator->actuatorPositions[deactivateChannel] = inactiveVoltage;
                    actuator->writeDAC(
                        static_cast<uint8_t>(deactivateChannel),
                        inactiveVoltage
                    );
                    trig.waitingToDeactivate = false;
                    emitActuatorStatus(true);

                    StaticJsonDocument<256> response;
                    response["type"] = "trigger_event";
                    response["event"] = "deactivated";
                    response["trigger_deactivated"] = true;
                    response["channel"] = deactivateChannel;
                    response["activated_channel"] = trig.activate_channel;
                    response["threshold"] = trig.threshold;
                    response["current_position_mm"] = currentPos;
                    response["command_voltage"] = inactiveVoltage;
                    response["elapsed_ms"] = static_cast<unsigned long>(
                        now - trig.triggerTime
                    );
                    response["commanded_mask"] = actuator->commandedActiveMask();
                    response["feedback_mask"] = actuator->feedbackActiveMask();
                    response["pcb_fault"] = actuator->hasPCBFault();
                    serializeJson(response, *io);
                    io->println();
                }
            }
        }
    }

    void emitEncoderStatus() {
        StaticJsonDocument<1024> doc;
        const float rearMm = motors->getRobotRearDistanceMM();
        doc["type"] = "encoder";
        doc["valid"] = encoderSession_ ? encoderSession_->valid() : false;
        doc["encoder_session_id"] = encoderSession_ ? encoderSession_->sessionId() : 0;
        doc["motor_power_present"] = encoderSession_ ? encoderSession_->motorPowerPresent() : false;
        doc["restore_required"] = encoderSession_ ? encoderSession_->restoreRequired() : true;
        doc["encoder_read_mask"] = motors->encoderReadValidMask();
        doc["radius_m"] = rearMm / 1000.0f;
        doc["rear_distance_mm"] = rearMm;
        doc["ts_ms"] = millis();

        // Public counts are robot-frame normalized. Forward must be positive.
        JsonArray counts = doc.createNestedArray("counts");
        JsonArray rawCounts = doc.createNestedArray("raw_counts");
        JsonArray directions = doc.createNestedArray("encoder_directions");
        for (uint8_t axis = 0; axis < 4; ++axis) {
            counts.add(motors->getNormalizedCounts(axis));
            rawCounts.add(motors->encCounts[axis]);
            directions.add(motors->encoderDirection[axis]);
        }

        serializeJson(doc, *io);
        io->println();
    }

    void emitProcessStatus(
        bool active,
        const char* reason = nullptr
    ) {
        StaticJsonDocument<192> doc;

        doc["type"] = "process_status";
        doc["state"] = active ? "running" : "stopped";
        doc["active"] = active;
        doc["ts_ms"] = millis();

        if (reason && reason[0] != '\0') {
            doc["reason"] = reason;
        }

        serializeJson(doc, *io);
        io->println();
    }

    void emitFirmwareFeatures()
    {
        StaticJsonDocument<256> doc;

        doc["type"] = "firmware_features";
        doc["actuator_status"] = true;
        doc["encoder_status"] = true;
        doc["main_loop_heartbeat"] = true;
        doc["loop_checkpoints"] = true;
        doc["build_date"] = __DATE__;
        doc["build_time"] = __TIME__;
        doc["uptime_ms"] = millis();

        serializeJson(doc, *io);
        io->println();
    }
    
    void emitProcessStartAck(
        const char* transactionId,
        bool accepted,
        const char* state,
        const char* reason = nullptr,
        const char* error = nullptr
    ) {
        StaticJsonDocument<320> doc;

        doc["type"] = "process_start_ack";
        doc["transaction_id"] =
            transactionId ? transactionId : "";
        doc["ok"] = accepted;
        doc["accepted"] = accepted;
        doc["state"] =
            state ? state : "unknown";
        doc["motion_inhibited"] =
            motors
            ? motors->isMotionInhibited()
            : true;
        doc["ts_ms"] = millis();

        if (reason && reason[0] != '\0') {
            doc["reason"] = reason;
        }

        if (error && error[0] != '\0') {
            doc["error"] = error;
        }

        serializeJson(doc, *io);
        io->println();
    }

    void applyDriveDirectionConfiguration_(
        const int8_t motor[4],
        const int8_t encoder[4],
        const char* transactionId,
        StaticJsonDocument<384>& response
    ) {
        motors->BRAKE_STOP();
        motors->setMotorDirections(motor[0], motor[1], motor[2], motor[3]);
        motors->setEncoderDirections(encoder[0], encoder[1], encoder[2], encoder[3]);
        encoderSession_->configurationChanged(
            "drive_direction_configuration_changed",
            transactionId
        );

        response["ok"] = true;
        response["encoder_restore_required"] = true;
        response["encoder_session_id"] = encoderSession_->sessionId();
        response["motion_inhibited"] = motors->isMotionInhibited();

        JsonArray appliedMotor = response.createNestedArray("motor_directions");
        JsonArray appliedEncoder = response.createNestedArray("encoder_directions");
        for (uint8_t i = 0; i < 4; ++i) {
            appliedMotor.add(motors->motorDirection[i]);
            appliedEncoder.add(motors->encoderDirection[i]);
        }
    }

    void handleStartDriveDirectionTest_() {
        const char* transactionId = jsonPacket["transaction_id"] | "";
        const int axis = jsonPacket["axis"] | -1;
        uint32_t leaseMs = jsonPacket["lease_ms"] | 350;

        StaticJsonDocument<384> response;
        response["type"] = "drive_direction_test_ack";
        response["transaction_id"] = transactionId;
        response["axis"] = axis;
        response["ts_ms"] = millis();

        const bool processActive = ultrasonicEnabled_ && *ultrasonicEnabled_;

        if (transactionId[0] == '\0') {
            response["ok"] = false;
            response["error"] = "missing_transaction_id";
        } else if (strlen(transactionId) >= sizeof(directionTest_.transactionId)) {
            response["ok"] = false;
            response["error"] = "transaction_id_too_long";
        } else if (axis < 0 || axis >= 4) {
            response["ok"] = false;
            response["error"] = "invalid_axis";
        } else if (directionTest_.active) {
            response["ok"] = false;
            response["error"] = "direction_test_active";
        } else if (processActive) {
            response["ok"] = false;
            response["error"] = "process_active";
        } else if (diagnosticRunner_ && diagnosticRunner_->active()) {
            response["ok"] = false;
            response["error"] = "diagnostic_active";
        } else if (!encoderSession_ || !encoderSession_->motorPowerPresent()) {
            response["ok"] = false;
            response["error"] = "motor_power_absent";
        } else if (motors->isMotionInhibited()) {
            response["ok"] = false;
            response["error"] = "motion_inhibited";
        } else {
            leaseMs = constrain(leaseMs, 250UL, 2000UL);

            int32_t count = 0;
            int32_t speed = 0;
            uint8_t encoderStatus = 0;
            uint8_t speedStatus = 0;
            const bool valid = motors->readMotorMotionFeedback(
                static_cast<uint8_t>(axis),
                count,
                speed,
                encoderStatus,
                speedStatus
            );

            if (!valid) {
                response["ok"] = false;
                response["error"] = "encoder_feedback_invalid";
            } else {
                motors->BRAKE_STOP();
                directionTest_.active = true;
                directionTest_.axis = static_cast<uint8_t>(axis);
                directionTest_.expiresAtMs = millis() + leaseMs;
                directionTest_.startCount = count;
                directionTest_.testedMotorDirection = motors->motorDirection[axis];
                directionTest_.testedEncoderDirection = motors->encoderDirection[axis];
                strlcpy(
                    directionTest_.transactionId,
                    transactionId,
                    sizeof(directionTest_.transactionId)
                );

                response["ok"] = true;
                response["accepted"] = true;
                response["lease_ms"] = leaseMs;
                response["start_raw_count"] = count;
                response["tested_motor_direction"] =
                    directionTest_.testedMotorDirection;
                response["tested_encoder_direction"] =
                    directionTest_.testedEncoderDirection;
            }
        }

        serializeJson(response, *io);
        io->println();
    }

    void commandActuatorState_(uint8_t channel, bool active) {
        if (!actuator || channel >= NUM_ACTUATORS) {
            return;
        }

        const float voltage = active
            ? constrain(
                transitionActuatorVoltage_,
                0.0f,
                CFG.actuator.maxCommandVoltage
            )
            : 0.0f;

        actuator->actuatorPositions[channel] = voltage;
        actuator->writeDAC(channel, voltage);
    }

    bool reconcileTriggersToPosition_(
        int currentPositionMm,
        StaticJsonDocument<768>& result
    ) {
        result.clear();
        result["status"] = "triggers_reconciled";
        result["position_mm"] = currentPositionMm;
        result["count"] = triggerBuffer.size();
        result["ts_ms"] = millis();

        if (!actuator) {
            result["ok"] = false;
            result["error"] = "actuator_not_attached";
            return false;
        }

        if (diagnosticRunner_ && diagnosticRunner_->ownsActuatorOutputs()) {
            result["ok"] = false;
            result["error"] = "actuator_owned_by_diagnostic";
            return false;
        }

        if (actuatorPreview_.active) {
            result["ok"] = false;
            result["error"] = "actuator_owned_by_preview";
            return false;
        }

        if (actuator->hasPCBFault()) {
            result["ok"] = false;
            result["error"] = "actuator_pcb_fault_before_reconciliation";
            return false;
        }

        /*
        * Validate the complete trigger table before modifying any output.
        * This prevents a malformed entry later in the table from causing a
        * partially reconstructed actuator state.
        */
        for (const auto& trig : triggerBuffer) {
            if (
                trig.activate_channel < 0 ||
                trig.activate_channel >= NUM_ACTUATORS ||
                trig.deactivate_channel < 0 ||
                trig.deactivate_channel >= NUM_ACTUATORS ||
                trig.threshold < 0 ||
                !isfinite(trig.delay_seconds) ||
                trig.delay_seconds < 0.0f
            ) {
                result["ok"] = false;
                result["error"] = "invalid_trigger_configuration";
                result["threshold"] = trig.threshold;
                result["activate_channel"] = trig.activate_channel;
                result["deactivate_channel"] = trig.deactivate_channel;
                return false;
            }
        }

        /*
        * Start from an explicitly safe inactive state. Reconciliation then
        * replays the stable result of every trigger already reached at the
        * confirmed encoder position.
        */
        for (uint8_t channel = 0; channel < NUM_ACTUATORS; ++channel) {
            commandActuatorState_(channel, false);

            if (actuator->hasPCBFault()) {
                result["ok"] = false;
                result["error"] =
                    "actuator_pcb_fault_during_reconciliation_reset";
                result["failed_channel"] = channel;
                return false;
            }
        }

        size_t reachedCount = 0;
        size_t pendingCount = 0;

        for (auto& trig : triggerBuffer) {
            trig.triggerTime = 0;
            trig.waitingToDeactivate = false;

            if (currentPositionMm < trig.threshold) {
                trig.triggered = false;
                ++pendingCount;
                continue;
            }

            commandActuatorState_(
                static_cast<uint8_t>(trig.activate_channel),
                true
            );

            if (actuator->hasPCBFault()) {
                for (uint8_t channel = 0; channel < NUM_ACTUATORS; ++channel) {
                    commandActuatorState_(channel, false);
                }

                result["ok"] = false;
                result["error"] =
                    "actuator_pcb_fault_during_reconciliation";
                result["failed_channel"] = trig.activate_channel;
                result["threshold"] = trig.threshold;
                return false;
            }

            /*
            * Historical delay intervals are not replayed. A reached non-hold
            * trigger is reconstructed directly into its stable post-delay
            * state. A hold trigger leaves its activation output active.
            */
            if (!trig.hold_active) {
                commandActuatorState_(
                    static_cast<uint8_t>(trig.deactivate_channel),
                    false
                );

                if (actuator->hasPCBFault()) {
                    for (
                        uint8_t channel = 0;
                        channel < NUM_ACTUATORS;
                        ++channel
                    ) {
                        commandActuatorState_(channel, false);
                    }

                    result["ok"] = false;
                    result["error"] =
                        "actuator_pcb_fault_during_reconciliation";
                    result["failed_channel"] =
                        trig.deactivate_channel;
                    result["threshold"] = trig.threshold;
                    return false;
                }
            }

            trig.triggered = true;
            ++reachedCount;
        }

        result["ok"] = true;
        result["reached_count"] = reachedCount;
        result["pending_count"] = pendingCount;
        result["commanded_mask"] =
            actuator->commandedActiveMask();
        result["feedback_mask"] =
            actuator->feedbackActiveMask();
        result["jam_mask"] =
            actuator->jamMask();
        result["pcb_fault"] =
            actuator->hasPCBFault();

        return true;
    }

    //HMI Test Section
    void runTestMotor(const char* id, int index, float speed, unsigned long durationMs) {
    int apos0 = motors->getRobotRearDistanceMM();  // uses your averaged encoder mm
    if (ultrasonic_) {
        ultrasonic_->processSpeed = speed;
        ultrasonic_->currentSpeed = speed;
    }
    unsigned long t0 = millis();
    while (millis() - t0 < durationMs) { delay(10); }
    motors->STOP();
    int apos1 = motors->getRobotRearDistanceMM();

    int delta = apos1 - apos0; // mm (per your scaling)
    StaticJsonDocument<128> meas;
    meas["apos_delta_mm"] = delta;
    meas["speed_cmd"] = speed;

    bool pass = (abs(delta) > 5); // moved at least ~5 mm; tune as needed
    emitTestResult(id, "motor", pass, meas.as<JsonObject>(), pass ? "movement detected" : "no movement");
}

void runTestActuator(const char* id, int channel, float v, float tol, unsigned long settleMs) {
    // Command the actuator
    actuator->actuatorPositions[channel] = v;
    actuator->writeDAC(channel, v);

    // Guarantee enough time for worst-case travel (keep your 10s min)
    const unsigned long MIN_WAIT_MS = 10000; // 10 s covers your slowest case
    const unsigned long waitMs = max(settleMs, MIN_WAIT_MS);
    delay(waitMs);

    // Read averaged feedback for robustness
    float fb = actuator->readADC_Avg(channel, /*samples*/5, /*delay*/3);
    actuator->feedbackSignals[channel] = fb;

    // Use the mapping you measured
    const float expFb = actuator->expectedFeedbackMapped(channel, v);

    StaticJsonDocument<192> meas;
    meas["command_V"]   = v;
    meas["expected_V"]  = expFb;
    meas["feedback_V"]  = fb;
    meas["error_V"]     = fb - expFb;
    meas["wait_ms"]     = waitMs;

    bool pass = fabs(fb - expFb) <= tol;
    emitTestResult(id, "actuator", pass, meas.as<JsonObject>(),
                   pass ? "within tolerance" : "out of tolerance");

    // Return to safe 0V
    actuator->actuatorPositions[channel] = 0.0f;
    actuator->writeDAC(channel, 0.0f);
}

void runTestUltrasonic(const char* id) {
    int adc = analogRead(CFG.ultrasonic.analog_pin);
    float voltage = (float(adc) * 3.1f / 1023.0f);
    float distance = voltage * CFG.ultrasonic.mm_per_volt + CFG.ultrasonic.offset_mm;

    StaticJsonDocument<128> meas;
    meas["adc"] = adc;
    meas["v"] = voltage;
    meas["distance_mm"] = distance;

    bool pass = (distance > 30.0f && distance < 400.0f) || (adc > 0 && adc < 1023);
    emitTestResult(id, "sensor", pass, meas.as<JsonObject>(), pass ? "plausible reading" : "out of range");
}

void runTestDigitalPin(const char* id, int pin, bool expectHigh, unsigned long sampleMs) {
    pinMode(pin, INPUT_PULLUP);
    unsigned long t0 = millis(); int hi=0, lo=0;
    while (millis() - t0 < sampleMs) { int v = digitalRead(pin); v?hi++:lo++; delay(2); }
    bool isHigh = (hi >= lo);
    StaticJsonDocument<128> meas;
    meas["pin"] = pin;
    meas["high_pct"] = (hi * 100) / (hi + lo);
    bool pass = (isHigh == expectHigh);
    emitTestResult(id, "sensor", pass, meas.as<JsonObject>(), pass ? "state ok" : "unexpected state");
}

void runTestAndon(const char* id) {
    StaticJsonDocument<128> meas;

    if (andonMgr) {
        andonMgr->setOverride(AndonLight::BLUE);
        delay(400);
        andonMgr->setOverride(AndonLight::OFF);
        delay(300);
        andonMgr->setOverride(AndonLight::BLUE);
        delay(400);
        andonMgr->clearOverride();
    }

    meas["pattern"] = "BLUE-OFF-BLUE";

    // Do NOT mark pass — operator must confirm.
    emitTestResult(id, "andon", false, meas.as<JsonObject>(), "visual_confirmation_required");
}

void runTestBattery(const char* id) {
    
    const float VREF   = 3.1f;       // same as BatteryMonitor
    const float ADCMAX = 1023.0f;    // 10-bit
    const float R1     = 100000.0f;  // same as BatteryMonitor
    const float R2     = 10000.0f;

    int adcValue = analogRead(BATTERY_PIN);
    float measuredVoltage = (adcValue * VREF / ADCMAX);
    float actualVoltage = measuredVoltage * ((R1 + R2) / R2);

    float pct = (actualVoltage - MIN_BATTERY_VOLTAGE) /
                (MAX_BATTERY_VOLTAGE - MIN_BATTERY_VOLTAGE) * 100.0f;
    if (pct < 0.0f) pct = 0.0f;
    if (pct > 100.0f) pct = 100.0f;

    StaticJsonDocument<128> meas;
    meas["adc"] = adcValue;
    meas["voltage_V"] = actualVoltage;
    meas["pct"] = pct;

    // Consider PASS if within expected min/max rails
    bool pass = (actualVoltage >= MIN_BATTERY_VOLTAGE && actualVoltage <= MAX_BATTERY_VOLTAGE);
    emitTestResult(id, "sensor", pass, meas.as<JsonObject>(), pass ? "voltage in range" : "out of range");
}

void runTestUltrasonicServo(const char* id) {
    StaticJsonDocument<128> meas;

    if (!ultrasonicServo_ || !ultrasonic_) {
        emitTestResult(id, "servo", false, meas.as<JsonObject>(), "servo_not_attached");
        return;
    }

    // 1) Activate servo
    ultrasonicServo_->activate();
    delay(1000);

    // 2) Let ultrasonic update for ~1 second
    const unsigned long t_end = millis() + 1000;
    while (millis() < t_end) {
        ultrasonic_->stateMachine();
        delay(10);
    }

    // 2.5) Wait for the ultrasonic’s last measurement to finish
    delay(150);

    // 3) Read measurement
    float d = ultrasonic_->distance[0] > 0.0f
                ? ultrasonic_->distance[0]
                : ultrasonic_->measuredDistance;

    meas["distance_mm"] = d;

    // 4) Evaluate against fault criteria (< 60mm)
    bool pass = d >= 60.0f;

    // 5) Return servo to inactive
    ultrasonicServo_->deactivate();
    delay(300);

    emitTestResult(id, "servo", pass, meas.as<JsonObject>(), 
        pass ? "distance_ok" : "distance_too_close");
}

    void stopActuatorPreview(const char* reason) {
        if (!actuatorPreview_.active) {
            return;
        }

        const uint8_t channel =
            actuatorPreview_.channel;

        char sessionId[
            sizeof(actuatorPreview_.sessionId)
        ];

        strlcpy(
            sessionId,
            actuatorPreview_.sessionId,
            sizeof(sessionId)
        );

        actuatorPreview_.active = false;
        actuatorPreview_.expiresAtMs = 0;
        actuatorPreview_.lastSequence = 0;
        actuatorPreview_.sessionId[0] = '\0';

        commandActuatorState_(channel, false);

        StaticJsonDocument<224> response;

        response["type"] =
            "actuator_preview_status";

        response["preview_session_id"] =
            sessionId;

        response["ok"] = true;
        response["active"] = false;
        response["channel"] = channel;

        response["reason"] =
            reason ? reason : "stopped";

        response["ts_ms"] = millis();

        serializeJson(response, *io);
        io->println();
    }

    void updateActuatorPreviewLease() {
        if (!actuatorPreview_.active) {
            return;
        }

        if (!actuator) {
            stopActuatorPreview("actuator_not_attached");
            return;
        }

        if (actuator->hasPCBFault()) {
            stopActuatorPreview("actuator_pcb_fault");
            return;
        }

        if (
            diagnosticRunner_ &&
            diagnosticRunner_->ownsActuatorOutputs()
        ) {
            stopActuatorPreview("actuator_owned_by_diagnostic");
            return;
        }

        if (
            ultrasonicEnabled_ &&
            *ultrasonicEnabled_
        ) {
            stopActuatorPreview("process_started");
            return;
        }

        if (
            static_cast<int32_t>(
                millis() - actuatorPreview_.expiresAtMs
            ) >= 0
        ) {
            stopActuatorPreview("lease_expired");
        }
    }

    void handleActuatorPreview_() {
        const char* sessionId = jsonPacket["preview_session_id"] | "";
        const int channel = jsonPacket["channel"] | -1;
        const float voltage = jsonPacket["voltage"] | NAN;
        uint32_t leaseMs = jsonPacket["lease_ms"] | 500;
        const uint32_t sequence = jsonPacket["seq"] | 0;

        StaticJsonDocument<256> response;
        response["type"] = "actuator_preview_status";
        response["preview_session_id"] = sessionId;
        response["channel"] = channel;
        response["seq"] = sequence;
        response["ts_ms"] = millis();

        if (!actuator || actuator->hasPCBFault()) {
            response["ok"] = false; response["error"] = "actuator_pcb_fault";
        } else if (channel != 0) {
            response["ok"] = false; response["error"] = "preview_channel_must_be_zero";
        } else if (sessionId[0] == '\0' || strlen(sessionId) >= sizeof(actuatorPreview_.sessionId)) {
            response["ok"] = false; response["error"] = "invalid_preview_session_id";
        } else if (sequence == 0) {
            response["ok"] = false;
            response["error"] =
                "invalid_preview_sequence";        
        } else if (!isfinite(voltage) || voltage < 0.0f || voltage > CFG.actuator.maxCommandVoltage) {
            response["ok"] = false; response["error"] = "invalid_preview_voltage";
        } else if (diagnosticRunner_ && diagnosticRunner_->ownsActuatorOutputs()) {
            response["ok"] = false; response["error"] = "actuator_owned_by_diagnostic";
        } else if (ultrasonicEnabled_ && *ultrasonicEnabled_) {
            response["ok"] = false; response["error"] = "process_active";
        } else if (actuatorPreview_.active && strcmp(sessionId, actuatorPreview_.sessionId) != 0) {
            response["ok"] = false; response["error"] = "preview_owned_by_another_session";
        } else if (actuatorPreview_.active && sequence <= actuatorPreview_.lastSequence) {
            response["ok"] = false; response["error"] = "stale_preview_sequence";
        } else {
            leaseMs = constrain(leaseMs, 100UL, 1000UL);
            actuatorPreview_.active = true;
            actuatorPreview_.channel = 0;
            actuatorPreview_.expiresAtMs = millis() + leaseMs;
            actuatorPreview_.lastSequence = sequence;
            strlcpy(actuatorPreview_.sessionId, sessionId, sizeof(actuatorPreview_.sessionId));
            actuator->actuatorPositions[0] = voltage;
            actuator->writeDAC(0, voltage);
            response["ok"] = true;
            response["active"] = true;
            response["voltage"] = voltage;
            response["lease_ms"] = leaseMs;
        }
        serializeJson(response, *io); io->println();
    }

private:
    Ultrasonic* ultrasonic_        = nullptr;
    bool*       ultrasonicEnabled_ = nullptr;
};

#endif