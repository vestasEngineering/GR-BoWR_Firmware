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

    unsigned long lastEncoderEmitMs = 0;
    static constexpr unsigned long encoderEmitPeriodMs = 500;

    struct Trigger {
        int threshold;
        int activate_channel;
        int deactivate_channel;
        float delay_seconds;
        bool triggered = false;
        unsigned long triggerTime = 0;
        bool waitingToDeactivate = false;
    };
    std::vector<Trigger> triggerBuffer;

    // Wiring to other subsystems
    AndonManager* andonMgr = nullptr;
    AndonLight* andonLight;
    Motors*     motors;
    ActuatorControl* actuator;
    UltrasonicServo* ultrasonicServo_ = nullptr;    
    Stream* io = nullptr;
    JogControl* jogControl = nullptr;

    
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

    // Helper for AndonManager: "is comms alive recently?"
    bool commsAlive() const {
        return timeout > 0;
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
        receiveLinux();  // refresh timeout if traffic arrives
        checkTriggers(); // encoder-driven triggers

        unsigned long now = millis();
        if (now - lastEncoderEmitMs >= encoderEmitPeriodMs) {
            lastEncoderEmitMs = now;
            emitEncoderStatus();
        }

        switch (state) {
        case LinkState::CONNECTED:
            // If no traffic for too long, consider disconnected
            if (!thisDelay && !timeout) {
                thisDelay = 500;     // 500 ms heartbeat
                state = LinkState::DISCONNECTED;
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
        DeserializationError err = deserializeJson(jsonPacket, receivedChars);

        if (err) {
            StaticJsonDocument<256> doc;
            doc["error"] = "json_parse";
            doc["code"]  = err.c_str();
            doc["raw"]   = receivedChars;   // extremely useful
            serializeJson(doc, *io);
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

    void updateParameters() {
        // Update motor speeds if present: speed0..speed3
        for (int i = 0; i < 4; i++) {
            String speedKey = "speed" + String(i);
            if (jsonPacket.containsKey(speedKey)) {
                motors->speeds[i] = jsonPacket[speedKey];
            }
        }

        if (jsonPacket.containsKey("hb")) {
            return; 
        }

        if (!jsonPacket.containsKey("action")) return;
        String action = jsonPacket["action"];

        if (action.equalsIgnoreCase("set_voltage")) {
            int channel = jsonPacket["channel"] | -1;
            float voltage = jsonPacket["voltage"] | 0.0f;
            actuator->actuatorPositions[channel] = voltage;
            actuator->writeDAC(channel, voltage);

            StaticJsonDocument<128> response;
            response["status"]  = "OK";
            response["channel"] = channel;
            response["voltage"] = voltage;
            serializeJson(response, *io); io->println();
        }
        else if (action.equalsIgnoreCase("read_feedback")) {
            int channel = jsonPacket["channel"] | -1;
            float feedback = actuator->readADC(channel);
            actuator->feedbackSignals[channel] = feedback;

            StaticJsonDocument<128> response;
            response["channel"]  = channel;
            response["feedback"] = feedback;
            serializeJson(response, *io); io->println();
        }
        else if (action.equalsIgnoreCase("reset_encoder")) {
            motors->resetEncoders();

            StaticJsonDocument<96> response;
            response["status"] = "All encoders reset";
            response["type"] = "encoder_reset";
            response["radius_m"] = 0.0;
            serializeJson(response, *io);
            io->println();

            emitEncoderStatus();
        }
        else if (action.equalsIgnoreCase("set_encoder")) {
            float radiusM = jsonPacket["radius_m"] | 0.0f;

            // Keep operator input sane. Adjust upper bound if needed.
            if (!isfinite(radiusM)) {
                radiusM = 0.0f;
            }

            if (radiusM < 0.0f) {
                radiusM = 0.0f;
            }

            motors->STOP();
            motors->setRobotRearDistanceM(radiusM);

            StaticJsonDocument<128> response;
            response["status"] = "encoder_set";
            response["type"] = "encoder_set";
            response["radius_m"] = radiusM;
            response["rear_distance_mm"] = radiusM * 1000.0f;
            serializeJson(response, *io);
            io->println();

            emitEncoderStatus();
        }
        else if (action.equalsIgnoreCase("STOP")) {
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
            bool shouldClear = jsonPacket["clear"] | false;
            if (shouldClear) triggerBuffer.clear();

            // Prefer single trigger
            if (jsonPacket.containsKey("trigger")) {
                JsonObject t = jsonPacket["trigger"].as<JsonObject>();
                if (t.containsKey("threshold") && t.containsKey("activate") &&
                    t.containsKey("deactivate") && t.containsKey("delay")) {
                    Trigger trig;
                    trig.threshold         = t["threshold"];
                    trig.activate_channel  = t["activate"];
                    trig.deactivate_channel= t["deactivate"];
                    trig.delay_seconds     = t["delay"];
                    triggerBuffer.push_back(trig);
                } else {
                    io->println("{\"error\":\"Invalid single trigger format\"}");
                }
            }

            // Also support array
            if (jsonPacket.containsKey("triggers")) {
                JsonArray arr = jsonPacket["triggers"].as<JsonArray>();
                for (JsonObject t : arr) {
                    if (t.containsKey("threshold") && t.containsKey("activate") &&
                        t.containsKey("deactivate") && t.containsKey("delay")) {
                        Trigger trig;
                        trig.threshold          = t["threshold"];
                        trig.activate_channel   = t["activate"];
                        trig.deactivate_channel = t["deactivate"];
                        trig.delay_seconds      = t["delay"];
                        triggerBuffer.push_back(trig);
                    } else {
                        io->println("{\"error\":\"Invalid trigger format\"}");
                    }
                }
            }

            StaticJsonDocument<64> response;
            response["status"] = "triggers_loaded";
            response["count"]  = triggerBuffer.size();
            serializeJson(response, *io); io->println();
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

            if (ultrasonic_) {
                ultrasonic_->maxAccelMps2 = accelMps2;
                ultrasonic_->maxDecelMps2 = decelMps2;
            }

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
        else if (action.equalsIgnoreCase("set_motor_direction")) {
            int d0 = jsonPacket["directions"][0] | 1;
            int d1 = jsonPacket["directions"][1] | -1;
            int d2 = jsonPacket["directions"][2] | -1;
            int d3 = jsonPacket["directions"][3] | 1;

            motors->STOP();

            motors->setMotorDirections(
                d0 < 0 ? -1 : 1,
                d1 < 0 ? -1 : 1,
                d2 < 0 ? -1 : 1,
                d3 < 0 ? -1 : 1
            );

            StaticJsonDocument<160> response;
            response["type"] = "ack";
            response["ok"] = true;
            response["info"] = "motor_direction_set";

            JsonArray arr = response.createNestedArray("directions");
            arr.add(motors->motorDirection[0]);
            arr.add(motors->motorDirection[1]);
            arr.add(motors->motorDirection[2]);
            arr.add(motors->motorDirection[3]);

            serializeJson(response, *io);
            io->println();
        }
        else if (action.equalsIgnoreCase("get_firmware")) {
            Version::printJson(*io);
        }
        else if (action.equalsIgnoreCase("start_process")) {
            if (ultrasonicEnabled_) *ultrasonicEnabled_ = true;
            if (ultrasonic_) {
                ultrasonic_->processSpeed = 0.0f;
                ultrasonic_->currentSpeed = 0.0f;
            }

            StaticJsonDocument<64> response;
            response["status"] = "process_started";
            serializeJson(response, *io); io->println();
        }
        else if (action.equalsIgnoreCase("stop_process")) {
            if (ultrasonicEnabled_) *ultrasonicEnabled_ = false;
            motors->STOP();

            StaticJsonDocument<64> response;
            response["status"] = "process_stopped";
            serializeJson(response, *io); io->println();
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
            motors->STOP();
            if (ultrasonicEnabled_) *ultrasonicEnabled_ = false;
            triggerBuffer.clear();
            serialStarted = false;

            StaticJsonDocument<64> response;
            response["status"] = "shutdown_complete";
            serializeJson(response, *io); io->println();
        }
        else if (action.equalsIgnoreCase("test_motor")) {
            int index = jsonPacket["index"] | 0;
            float speed = jsonPacket["speed"] | 0.02f;       
            unsigned long dur = jsonPacket["duration_ms"] | 600;
            const char* id = jsonPacket["id"] | "motor_0";
           runTestMotor(id, index, speed, dur);
        }
        else if (action.equalsIgnoreCase("test_actuator")) {
            int channel = jsonPacket["channel"] | 0;
            float v = jsonPacket["voltage"] | 2.5f;
            float tol = jsonPacket["tolerance"] | 0.3f;
            unsigned long settle = jsonPacket["settle_ms"] | 300;
            const char* id = jsonPacket["id"] | "actuator_0";
            runTestActuator(id, channel, v, tol, settle);
        }
        else if (action.equalsIgnoreCase("test_sensor")) {
            const char* sensor = jsonPacket["sensor"] | "ultrasonic";
            const char* id = jsonPacket["id"] | sensor;
            if (strcasecmp(sensor, "ultrasonic") == 0) {
                runTestUltrasonic(id);
            } else if (strcasecmp(sensor, "battery") == 0) {
                runTestBattery(id);
            } else if (strcasecmp(sensor, "digital") == 0) {
                int pin = jsonPacket["pin"] | D1;
                bool expectHigh = jsonPacket["expect"] | true;
                unsigned long sample = jsonPacket["sample_ms"] | 300;
                runTestDigitalPin(id, pin, expectHigh, sample);
            } else {
                StaticJsonDocument<80> err;
                err["type"] = "test_result";
                err["id"] = id;
                err["pass"] = false;
                err["reason"] = "unknown_sensor";
                serializeJson(err, *io); io->println();
            }
        }
        else if (action.equalsIgnoreCase("test_light")) {
            const char* id = jsonPacket["id"] | "andon_ring";
            runTestAndon(id);
        }
        else if (action.equalsIgnoreCase("test_servo")) {
            const char* id = jsonPacket["id"] | "ultrasonic_servo";
            runTestUltrasonicServo(id);
        }
        else if (action.equalsIgnoreCase("ping")) {
            StaticJsonDocument<128> response;
            response["status"] = "pong";
            response["uptime_ms"] = millis();
            serializeJson(response, *io);
            io->println();
        }
        else if (action.equalsIgnoreCase("jog")) {
            if (!jogControl) {
                StaticJsonDocument<128> err;
                err["type"] = "error";
                err["id"] = "jog";
                err["error"] = "jog_not_attached";
                serializeJson(err, *io); io->println();
                return;
            }

            const char* dirStr = jsonPacket["dir"] | "";
            int dir = 0;

            if (strcasecmp(dirStr, "forward") == 0) {
                dir = 1;
            } else if (strcasecmp(dirStr, "backward") == 0) {
                dir = -1;
            } else {
                StaticJsonDocument<128> err;
                err["type"] = "error";
                err["id"] = "jog";
                err["error"] = "invalid_direction";
                serializeJson(err, *io); io->println();
                return;
            }

            float speed = jsonPacket["speed"] | CFG.jog.jog_speed_max_ms;
            unsigned long leaseMs = jsonPacket["lease_ms"] | 250;
            uint32_t seq = jsonPacket["seq"] | 0;

            bool ok = jogControl->startOrRefreshRemoteJog(
                dir,
                speed,
                leaseMs,
                seq
            );

            StaticJsonDocument<160> response;
            response["type"] = ok ? "ack" : "error";
            response["id"] = "jog";
            response["ok"] = ok;
            response["seq"] = seq;

            if (!ok) {
                response["error"] = "jog_rejected";
            }

            serializeJson(response, *io); io->println();
        }
        else if (action.equalsIgnoreCase("jog_stop")) {
            if (jogControl) {
                uint32_t seq = jsonPacket["seq"] | 0;
                jogControl->stopRemoteJog(seq);

                StaticJsonDocument<128> response;
                response["type"] = "ack";
                response["id"] = "jog_stop";
                response["ok"] = true;
                response["seq"] = seq;
                serializeJson(response, *io); io->println();
            }
        }
    }
    
    void checkTriggers() {
        int currentPos = motors->getRobotRearDistanceMM();
        unsigned long now = millis();

        for (auto& trig : triggerBuffer) {
            // Activation
            if (!trig.triggered && currentPos >= trig.threshold) {

                actuator->actuatorPositions[trig.activate_channel] = 4.4;
                trig.triggerTime        = now;
                trig.waitingToDeactivate= true;
                trig.triggered          = true;

                StaticJsonDocument<128> response;
                response["trigger_reached"] = true;
                response["channel"]         = trig.activate_channel;
                response["value"]           = trig.threshold;
                serializeJson(response, *io);
                io->println();
            }

            // Deactivation after delay
            if (trig.waitingToDeactivate &&
                (now - trig.triggerTime >= (unsigned long)(trig.delay_seconds * 1000))) {

                actuator->actuatorPositions[trig.deactivate_channel] = 0.0;
                trig.waitingToDeactivate = false;

                StaticJsonDocument<128> response;
                response["trigger_deactivated"] = true;
                response["channel"]             = trig.deactivate_channel;
                serializeJson(response, *io);
                io->println();
            }
        }
    }

    void emitEncoderStatus() {
        StaticJsonDocument<256> doc;

        const float rearMm = motors->getRobotRearDistanceMM();
        const float radiusM = rearMm / 1000.0f;

        doc["type"] = "encoder";
        doc["radius_m"] = radiusM;
        doc["rear_distance_mm"] = rearMm;

        JsonArray counts = doc.createNestedArray("counts");
        for (int i = 0; i < 4; i++) {
            counts.add(motors->encCounts[i]);
        }

        serializeJson(doc, *io);
        io->println();
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
    int adc = analogRead(ULTRASONIC_PIN);
    float voltage = (float(adc) * 3.1f / 1023.0f);
    float distance = voltage * ultrasonic_->mmPerVolt + ultrasonic_->offsetDistance;

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


private:
    Ultrasonic* ultrasonic_        = nullptr;
    bool*       ultrasonicEnabled_ = nullptr;
};

#endif