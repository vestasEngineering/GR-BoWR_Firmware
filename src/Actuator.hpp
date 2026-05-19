#ifndef ACTUATOR_CONTROL_H
#define ACTUATOR_CONTROL_H

#include "Config.hpp"
#include <Arduino.h>
#include <ArduinoJson.h>
#include <Wire.h>
#include <Adafruit_MCP4728.h>
#include <Adafruit_ADS1X15.h>
#include <math.h>  // fabsf, isfinite

#define NUM_ACTUATORS 4

class ActuatorControl {
public:
    Stream* io = nullptr;
    explicit ActuatorControl(Stream& ioRef) : io(&ioRef) {}

    // Hardware drivers
    Adafruit_MCP4728 dac;
    Adafruit_ADS1115 adc;

    // Commanded positions (0..maxCommandVoltage)
    float actuatorPositions[NUM_ACTUATORS] = {0, 0, 0, 0};

    // Feedback voltages (0..maxFeedbackVoltage)
    float feedbackSignals[NUM_ACTUATORS] = {0, 0, 0, 0};

    enum States { SET_POSITION, READ_FEEDBACK };
    States state = SET_POSITION;

    StaticJsonDocument<32> json;
    char packet[32];

    // ----- Fault flags -----
    // When true: do not touch I2C DAC/ADC (prevents the whole firmware from blocking)
    bool pcbFault_ = false;

    // Indicates we detected a runtime I2C disconnect/fault and latched it
    bool i2cFaultLatched_ = false;

    // per-channel jam latch
    bool  jammed[NUM_ACTUATORS] = {false, false, false, false};
    float lastErrV[NUM_ACTUATORS] = {0, 0, 0, 0};

    // Endpoint jam detection internal state
    bool     lastRequestedActive[NUM_ACTUATORS] = {false, false, false, false};
    bool     checkArmed[NUM_ACTUATORS]          = {false, false, false, false};
    bool     expectedActive[NUM_ACTUATORS]      = {false, false, false, false};
    uint32_t checkAtMs[NUM_ACTUATORS]           = {0, 0, 0, 0};

    // ---- Runtime I2C recovery ----
    uint32_t nextReinitAttemptMs_ = 0;
    static constexpr uint32_t REINIT_PERIOD_MS = 5000;

    // If your hardware uses different addresses, change these:
    static constexpr uint8_t DAC_ADDR = 0x60; // MCP4728 default
    static constexpr uint8_t ADC_ADDR = 0x48; // ADS1115 default

    // ----- Public status API -----
    bool hasPCBFault() const { return pcbFault_; }

    bool hasFault() const {
        if (pcbFault_) return true;
        for (uint8_t i = 0; i < NUM_ACTUATORS; ++i) {
            if (jammed[i]) return true;
        }
        return false;
    }

    uint8_t jamMask() const {
        uint8_t m = 0;
        for (uint8_t i = 0; i < NUM_ACTUATORS; ++i) {
            if (jammed[i]) m |= (1u << i);
        }
        return m;
    }

    void clearJammed(uint8_t i) { if (i < NUM_ACTUATORS) jammed[i] = false; }
    void clearAllJammed()       { for (uint8_t i = 0; i < NUM_ACTUATORS; ++i) jammed[i] = false; }

    // ------------------------------------------------------------
    // Setup
    // ------------------------------------------------------------
    void setup() {
        Wire.begin();

        // Prevent I2C lockups from freezing the entire firmware loop. [1](https://docs.arduino.cc/language-reference/en/functions/communication/wire/setWireTimeout/)
        #if defined(WIRE_HAS_TIMEOUT)
          Wire.setWireTimeout(25000 /*us*/, true /*reset_on_timeout*/);
        #endif

        io->println("{\"type\":\"status\",\"module\":\"actuator\",\"msg\":\"initializing\"}");

        bool dacOk = dac.begin();
        if (!dacOk) {
            pcbFault_ = true;
            io->println("{\"type\":\"status\",\"module\":\"actuator\",\"status\":\"mcp4728_init_failed\",\"fault\":\"pcb\"}");
        } else {
            io->println("{\"type\":\"status\",\"module\":\"actuator\",\"status\":\"mcp4728_init_ok\"}");
        }

        bool adcOk = adc.begin();
        if (!adcOk) {
            pcbFault_ = true;
            io->println("{\"type\":\"status\",\"module\":\"actuator\",\"status\":\"ads1115_init_failed\",\"fault\":\"pcb\"}");
        } else {
            io->println("{\"type\":\"status\",\"module\":\"actuator\",\"status\":\"ads1115_init_ok\"}");
        }

        state = SET_POSITION;
    }

    // ------------------------------------------------------------
    // PUBLIC: writeDAC / readADC (kept public because MySerial uses them)
    // These are SAFE: they latch faults and become no-ops when I2C is gone.
    // ------------------------------------------------------------
    void writeDAC(uint8_t channel, float voltage) {
        const float v = constrain(voltage, 0.0f, CFG.actuator.maxCommandVoltage);
        const uint16_t dacValue = (uint16_t)((v / CFG.actuator.maxCommandVoltage) * 4095.0f);

        MCP4728_channel_t channelEnum;
        switch (channel) {
            case 0: channelEnum = MCP4728_CHANNEL_A; break;
            case 1: channelEnum = MCP4728_CHANNEL_B; break;
            case 2: channelEnum = MCP4728_CHANNEL_C; break;
            case 3: channelEnum = MCP4728_CHANNEL_D; break;
            default: {
                StaticJsonDocument<96> doc;
                doc["type"]   = "error";
                doc["module"] = "actuator";
                doc["error"]  = "invalid_dac_channel";
                doc["channel"] = channel;
                serializeJson(doc, *io);
                io->println();
                return;
            }
        }

        // If we latched a fault, never touch I2C
        if (pcbFault_) return;

        // setChannelValue returns true/false depending on I2C success [2](https://adafruit.github.io/Adafruit_MCP4728/html/class_adafruit___m_c_p4728.html)
        const bool ok = dac.setChannelValue(
            channelEnum,
            dacValue,
            MCP4728_VREF_VDD,
            MCP4728_GAIN_1X,
            MCP4728_PD_MODE_NORMAL
        );

        if (!ok) {
            latchI2CFault_("dac_setChannelValue", channel);
        }
    }

    float readADC(uint8_t channel) {
        if (pcbFault_) return 0.0f;

        // If ADC is missing (runtime disconnect), latch and bail fast
        if (!i2cPing_(ADC_ADDR)) {
            latchI2CFault_("adc_ping_failed", channel);
            return 0.0f;
        }

        int16_t rawValue = adc.readADC_SingleEnded(channel);

        float voltage = (rawValue / 32767.0f) * CFG.actuator.maxFeedbackVoltage;
        if (voltage < 0.0f) voltage = 0.0f;
        if (voltage > CFG.actuator.maxFeedbackVoltage) voltage = CFG.actuator.maxFeedbackVoltage;
        return voltage;
    }

    // ------------------------------------------------------------
    // Core loop state machine
    // ------------------------------------------------------------
    void stateMachine() {
        // If cable is unplugged during runtime, latch fault to keep robot responsive.
        // Periodically attempt recovery if cable returns.
        tryReinitIfNeeded_();

        switch (state) {
            case SET_POSITION: {
                const uint32_t now = millis();

                for (uint8_t i = 0; i < NUM_ACTUATORS; i++) {
                    const float cmdV = actuatorPositions[i];
                    writeDAC(i, cmdV);

                    const bool reqActivate   = (cmdV >= CFG.actuator.activate_cmd_min_v);
                    const bool reqDeactivate = (cmdV <= CFG.actuator.deactivate_cmd_max_v);

                    if (reqActivate && (lastRequestedActive[i] != true)) {
                        lastRequestedActive[i] = true;
                        expectedActive[i]      = true;
                        checkAtMs[i]           = now + CFG.actuator.settle_ms;
                        checkArmed[i]          = true;
                        jammed[i]              = false;
                    } else if (reqDeactivate && (lastRequestedActive[i] != false)) {
                        lastRequestedActive[i] = false;
                        expectedActive[i]      = false;
                        checkAtMs[i]           = now + CFG.actuator.settle_ms;
                        checkArmed[i]          = true;
                        jammed[i]              = false;
                    }
                }

                state = READ_FEEDBACK;
            } break;

            case READ_FEEDBACK: {
                const uint32_t now = millis();

                for (uint8_t i = 0; i < NUM_ACTUATORS; i++) {
                    feedbackSignals[i] = readADC(i);

                    if (checkArmed[i] && ((int32_t)(now - checkAtMs[i]) >= 0)) {
                        if (expectedActive[i]) {
                            lastErrV[i] = (CFG.actuator.active_fb_min_v - feedbackSignals[i]);
                            if (feedbackSignals[i] < CFG.actuator.active_fb_min_v) {
                                jammed[i] = true;
                            }
                        } else {
                            lastErrV[i] = (feedbackSignals[i] - CFG.actuator.inactive_fb_max_v);
                            if (feedbackSignals[i] > CFG.actuator.inactive_fb_max_v) {
                                jammed[i] = true;
                            }
                        }
                        checkArmed[i] = false;
                    }
                }

                state = SET_POSITION;
            } break;
        }
    }

    // ------------------------------------------------------------
    // Debug output
    // ------------------------------------------------------------
    void debugOutput() {
        for (uint8_t i = 0; i < NUM_ACTUATORS; i++) {
            StaticJsonDocument<192> doc;
            doc["type"]       = "actuator_debug";
            doc["channel"]    = i;
            doc["cmd_V"]      = actuatorPositions[i];
            doc["fb_V"]       = feedbackSignals[i];
            doc["jam"]        = jammed[i];
            doc["last_err_V"] = lastErrV[i];
            doc["pcbFault"]   = pcbFault_;
            doc["i2cLatch"]   = i2cFaultLatched_;
            serializeJson(doc, *io);
            io->println();
        }
    }

    // ------------------------------------------------------------
    // Self test
    // ------------------------------------------------------------
    void runSelfTest() {
        {
            StaticJsonDocument<96> doc;
            doc["type"]  = "actuator_self_test";
            doc["phase"] = "start";
            serializeJson(doc, *io);
            io->println();
        }

        for (uint8_t i = 0; i < NUM_ACTUATORS; i++) {
            float testVoltage = 2.5f;
            actuatorPositions[i] = testVoltage;
            writeDAC(i, testVoltage);
            delay(500);

            float feedback = readADC(i);
            feedbackSignals[i] = feedback;

            StaticJsonDocument<160> doc;
            doc["type"]       = "actuator_self_test";
            doc["phase"]      = "sample";
            doc["channel"]    = i;
            doc["command_V"]  = testVoltage;
            doc["feedback_V"] = feedback;
            serializeJson(doc, *io);
            io->println();

            actuatorPositions[i] = 0.0f;
            writeDAC(i, 0.0f);
            delay(250);
        }

        {
            StaticJsonDocument<96> doc;
            doc["type"]  = "actuator_self_test";
            doc["phase"] = "complete";
            serializeJson(doc, *io);
            io->println();
        }
    }

    // ---- Mapping-derived expected feedback ----
    float expectedFeedbackMapped(uint8_t /*channel*/, float cmdV) const {
        float expFb = CFG.actuator.fb_map_m * cmdV + CFG.actuator.fb_map_b;
        if (expFb < 0.0f) expFb = 0.0f;
        if (expFb > CFG.actuator.maxFeedbackVoltage) expFb = CFG.actuator.maxFeedbackVoltage;
        return expFb;
    }

    // Averaging helper
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
        if (v > CFG.actuator.maxFeedbackVoltage) v = CFG.actuator.maxFeedbackVoltage;
        return v;
    }

private:
    // ------------------------------------------------------------
    // Runtime I2C fault / recovery helpers
    // ------------------------------------------------------------
    void latchI2CFault_(const char* where, int detail = -1) {
        pcbFault_ = true;       // disables all I2C interaction in writeDAC/readADC
        i2cFaultLatched_ = true;
        nextReinitAttemptMs_ = millis() + REINIT_PERIOD_MS;

        StaticJsonDocument<160> doc;
        doc["type"]   = "status";
        doc["module"] = "actuator";
        doc["fault"]  = "i2c_runtime_disconnect";
        doc["where"]  = where;
        if (detail >= 0) doc["detail"] = detail;
        serializeJson(doc, *io);
        io->println();
    }

    bool i2cPing_(uint8_t addr) {
        Wire.beginTransmission(addr);
        uint8_t err = Wire.endTransmission();
        return (err == 0);
    }

    void tryReinitIfNeeded_() {
        if (!pcbFault_) return;

        const int32_t dt = (int32_t)(millis() - nextReinitAttemptMs_);
        if (dt < 0) return;

        // Only attempt reinit if devices respond
        if (!i2cPing_(DAC_ADDR) || !i2cPing_(ADC_ADDR)) {
            nextReinitAttemptMs_ = millis() + REINIT_PERIOD_MS;
            return;
        }

        bool dacOk = dac.begin();
        bool adcOk = adc.begin();

        if (dacOk && adcOk) {
            pcbFault_ = false;
            i2cFaultLatched_ = false;
            io->println("{\"type\":\"status\",\"module\":\"actuator\",\"status\":\"i2c_recovered\"}");
        } else {
            pcbFault_ = true;
            nextReinitAttemptMs_ = millis() + REINIT_PERIOD_MS;
        }
    }
};

#endif