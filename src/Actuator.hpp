#ifndef ACTUATOR_CONTROL_H
#define ACTUATOR_CONTROL_H

#include <Arduino.h>
#include <ArduinoJson.h>
#include <Wire.h>
#include <Adafruit_MCP4728.h>
#include <Adafruit_ADS1X15.h>
#include <math.h>  // fabsf

#define NUM_ACTUATORS 4
#define MAX_VOLTAGE   5.0f
#define MAX_FEEDBACK  5.0f   // <-- updated per your measurement

class ActuatorControl {
public:
    Adafruit_MCP4728 dac;
    Adafruit_ADS1115 adc;
    
    float actuatorPositions[NUM_ACTUATORS] = {0}; // Desired positions (0..5V)
    float feedbackSignals [NUM_ACTUATORS] = {0};  // Feedback (0..5V)

    enum States { SET_POSITION, READ_FEEDBACK };
    States state;

    StaticJsonDocument<32> json;
    char packet[32];

    // ----- Fault flags -----
    bool pcbFault_ = false;                 // DAC or ADC init failure (PCB fault)
    bool jammed    [NUM_ACTUATORS] = {0};   // per-channel jam latch
    float lastErrV [NUM_ACTUATORS] = {0};   // last endpoint error, for debug

    // Endpoint jam detection configuration
    struct EndpointCfg {
        uint16_t settle_ms             = 15000; // 15 s after request edge
        float    activate_cmd_min_v    = 4.5f;  // >= this is an "activate request"
        float    deactivate_cmd_max_v  = 0.5f;  // <= this is a "deactivate request"
        float    active_fb_min_v       = 4.0f;  // expected min FB when activated
        float    inactive_fb_max_v     = 2.0f;  // expected max FB when deactivated
    } epCfg;

    // Internal jam detection state
    bool     lastRequestedActive[NUM_ACTUATORS] = {false, false, false, false};
    bool     checkArmed         [NUM_ACTUATORS] = {false, false, false, false};
    bool     expectedActive     [NUM_ACTUATORS] = {false, false, false, false};
    uint32_t checkAtMs          [NUM_ACTUATORS] = {0,0,0,0};

    // ----- Public status API for AndonManager -----
    bool hasPCBFault() const { return pcbFault_; }
    bool hasFault()   const {
        if (pcbFault_) return true;
        for (uint8_t i = 0; i < NUM_ACTUATORS; ++i) if (jammed[i]) return true;
        return false;
    }
    uint8_t jamMask() const {
        uint8_t m = 0;
        for (uint8_t i = 0; i < NUM_ACTUATORS; ++i) if (jammed[i]) m |= (1u << i);
        return m;
    }
    void clearJammed(uint8_t i) { if (i < NUM_ACTUATORS) jammed[i] = false; }
    void clearAllJammed()       { for (uint8_t i = 0; i < NUM_ACTUATORS; ++i) jammed[i] = false; }

    void setup() {
        Wire.begin();
        Serial.println("{\"status\": \"Initializing ActuatorControl...\"}");

        // DAC init
        if (!dac.begin()) {
            pcbFault_ = true;
            Serial.println("{\"status\":\"MCP4728 init FAILED\",\"fault\":\"pcb\"}");
        } else {
            Serial.println("{\"status\":\"MCP4728 init OK\"}");
        }
        
        // ADC init
        if (!adc.begin()) {
            pcbFault_ = true;
            Serial.println("{\"status\":\"ADS1115 init FAILED\",\"fault\":\"pcb\"}");
        } else {
            Serial.println("{\"status\":\"ADS1115 init OK\"}");
        }

        state = SET_POSITION;
    }

    void writeDAC(uint8_t channel, float voltage) {
        // Bounds
        voltage = constrain(voltage, 0.0f, MAX_VOLTAGE);

        // Scale to 12-bit
        uint16_t dacValue = (uint16_t)((voltage / MAX_VOLTAGE) * 4095.0f);

        MCP4728_channel_t channelEnum;
        switch (channel) {
            case 0: channelEnum = MCP4728_CHANNEL_A; break;
            case 1: channelEnum = MCP4728_CHANNEL_B; break;
            case 2: channelEnum = MCP4728_CHANNEL_C; break;
            case 3: channelEnum = MCP4728_CHANNEL_D; break;
            default:
                StaticJsonDocument<96> doc;
                doc["error"] = "invalid_dac_channel";
                doc["channel"] = channel;
                serializeJson(doc, Serial); Serial.println();
                return;
        }

        // If PCB fault, skip actual writes to avoid spurious I2C traffic
        if (pcbFault_) return;

        (void)dac.setChannelValue(channelEnum, dacValue, MCP4728_VREF_VDD, MCP4728_GAIN_1X, MCP4728_PD_MODE_NORMAL);
    }

    float readADC(uint8_t channel) {
        if (pcbFault_) return 0.0f; // no reliable read if PCB fault
        int16_t rawValue = adc.readADC_SingleEnded(channel);
        // Scale raw (-32768..32767) to 0..MAX_FEEDBACK; adjust if you change PGA
        float voltage = (rawValue / 32767.0f) * MAX_FEEDBACK;
        if (voltage < 0.0f) voltage = 0.0f;
        if (voltage > MAX_FEEDBACK) voltage = MAX_FEEDBACK;
        return voltage;
    }

    void stateMachine(){
        switch (state) {
            case SET_POSITION: {
                const uint32_t now = millis();
                for (uint8_t i = 0; i < NUM_ACTUATORS; i++) {
                    const float cmdV = actuatorPositions[i];
                    writeDAC(i, cmdV);

                    // Recognize endpoint requests
                    const bool reqActivate   = (cmdV >= epCfg.activate_cmd_min_v);
                    const bool reqDeactivate = (cmdV <= epCfg.deactivate_cmd_max_v);

                    // Arm a check when the requested endpoint state changes
                    if (reqActivate && (lastRequestedActive[i] != true)) {
                        lastRequestedActive[i] = true;
                        expectedActive[i]      = true;
                        checkAtMs[i]           = now + epCfg.settle_ms;
                        checkArmed[i]          = true;
                        jammed[i]              = false;  // clear old jam on new request

                        // Optional debug:
                        // Serial.printf("{\"act_arm\":%u,\"type\":\"activate\",\"ms\":%lu}\n", i, (unsigned long)checkAtMs[i]);
                    } else if (reqDeactivate && (lastRequestedActive[i] != false)) {
                        lastRequestedActive[i] = false;
                        expectedActive[i]      = false;
                        checkAtMs[i]           = now + epCfg.settle_ms;
                        checkArmed[i]          = true;
                        jammed[i]              = false;

                        // Optional debug:
                        // Serial.printf("{\"act_arm\":%u,\"type\":\"deactivate\",\"ms\":%lu}\n", i, (unsigned long)checkAtMs[i]);
                    }
                    // If command is mid-range, we don't arm a check.
                }
                state = READ_FEEDBACK;
            } break;

            case READ_FEEDBACK: {
                const uint32_t now = millis();
                for (uint8_t i = 0; i < NUM_ACTUATORS; i++) {
                    feedbackSignals[i] = readADC(i);

                    if (checkArmed[i] && ( (int32_t)(now - checkAtMs[i]) >= 0 )) {
                        // Time to evaluate endpoint success/failure
                        if (expectedActive[i]) {
                            // Expect high feedback
                            const float err = (epCfg.active_fb_min_v - feedbackSignals[i]); // negative if OK
                            lastErrV[i] = err;
                            if (feedbackSignals[i] < epCfg.active_fb_min_v) {
                                jammed[i] = true;
                                // Serial.printf("{\"act_jam\":%u,\"exp\":\">=%.2f\",\"fb\":%.2f}\n", i, epCfg.active_fb_min_v, feedbackSignals[i]);
                            }
                        } else {
                            // Expect low feedback
                            const float err = (feedbackSignals[i] - epCfg.inactive_fb_max_v); // negative if OK
                            lastErrV[i] = err;
                            if (feedbackSignals[i] > epCfg.inactive_fb_max_v) {
                                jammed[i] = true;
                                // Serial.printf("{\"act_jam\":%u,\"exp\":\"<=%.2f\",\"fb\":%.2f}\n", i, epCfg.inactive_fb_max_v, feedbackSignals[i]);
                            }
                        }
                        checkArmed[i] = false; // evaluate once per request
                    }
                }
                state = SET_POSITION; 
            } break;

            default:
                break;
        }
    }

    void debugOutput() {
        Serial.println("--------- Actuator Debug ---------");
        for (uint8_t i = 0; i < NUM_ACTUATORS; i++) {
            Serial.print("Ch "); Serial.print(i);
            Serial.print("  Cmd="); Serial.print(actuatorPositions[i], 3);
            Serial.print("V  FB="); Serial.print(feedbackSignals[i], 3);
            Serial.print("V  Jam="); Serial.print(jammed[i] ? "Y" : "N");
            Serial.print("  LastErr="); Serial.print(lastErrV[i], 3);
            Serial.println();
        }
        Serial.println("----------------------------------");
    }

    void runSelfTest() {
        Serial.println (">>> Starting self-test routine...");
        for (uint8_t i = 0; i < NUM_ACTUATORS; i++) {
            Serial.print("Testing actuator "); Serial.println(i);

            float testVoltage = 2.5f;
            actuatorPositions[i] = testVoltage;
            writeDAC(i, testVoltage);
            delay(500);

            float feedback = readADC(i % 4);
            feedbackSignals[i] = feedback;
            Serial.print(" -> Set: "); Serial.print(testVoltage);
            Serial.print(" V, Feedback: "); Serial.print(feedback); Serial.println(" V" );

            actuatorPositions[i] = 0.0f;
            writeDAC(i, 0.0f);
            delay(250);
        }
        Serial.println("<<< Self-test routine completed.");
    }
    
    // ---- Mapping-derived expected feedback for your measured system ----
    // Linear fit from your mapping: fb = m*cmd + b
    float expectedFeedbackMapped(uint8_t channel, float cmdV) const {
        (void)channel; // if you later have per-channel fits, use channel
        const float m = -0.529920101f;
        const float b =  2.705734968f;
        float expFb = m * cmdV + b;
        // Clamp to physical rails
        if (expFb < 0.0f)       expFb = 0.0f;
        if (expFb > MAX_FEEDBACK) expFb = MAX_FEEDBACK;
        return expFb;
    }

    // Small averaging to reduce ADC noise when testing
    float readADC_Avg(uint8_t channel, int samples = 5, int sampleDelayMs = 3) {
        if (pcbFault_) return 0.0f;
        if (samples <= 1) return readADC(channel);
        float acc = 0.0f;
        for (int i = 0; i < samples; ++i) {
            acc += readADC(channel);
            if (sampleDelayMs > 0) delay(sampleDelayMs);
        }
        float v = acc / samples;
        if (v < 0.0f) v = 0.0f;
        if (v > MAX_FEEDBACK) v = MAX_FEEDBACK;
        return v;
    }



};

#endif
