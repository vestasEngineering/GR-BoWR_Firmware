#ifndef ACTUATOR_CONTROL_H
#define ACTUATOR_CONTROL_H

#include "Config.hpp"
#include <Arduino.h>
#include <ArduinoJson.h>
#include <Wire.h>
#include <Adafruit_MCP4728.h>
#include <Adafruit_ADS1X15.h>
#include <math.h>

#define NUM_ACTUATORS 4
#ifndef ACTUATOR_I2C_DEBUG
#define ACTUATOR_I2C_DEBUG 0
#endif

class ActuatorControl {
public:
    Stream* io = nullptr;
    explicit ActuatorControl(Stream& ioRef) : io(&ioRef) {}

    Adafruit_MCP4728 dac;
    Adafruit_ADS1115 adc;

    float actuatorPositions[NUM_ACTUATORS] = {0, 0, 0, 0};
    float feedbackSignals[NUM_ACTUATORS] = {0, 0, 0, 0};

    enum States { SET_POSITION, READ_FEEDBACK };
    States state = SET_POSITION;

    StaticJsonDocument<32> json;
    char packet[32];

    bool pcbFault_ = false;
    bool i2cFaultLatched_ = false;

    bool  jammed[NUM_ACTUATORS] = {false, false, false, false};
    float lastErrV[NUM_ACTUATORS] = {0, 0, 0, 0};
    bool     lastRequestedActive[NUM_ACTUATORS] = {false, false, false, false};
    bool     checkArmed[NUM_ACTUATORS] = {false, false, false, false};
    bool     expectedActive[NUM_ACTUATORS] = {false, false, false, false};
    uint32_t checkAtMs[NUM_ACTUATORS] = {0, 0, 0, 0};

    bool commandedActive[NUM_ACTUATORS] = {false, false, false, false};
    bool feedbackActive[NUM_ACTUATORS] = {false, false, false, false};    

    static constexpr uint8_t DAC_ADDR = 0x60;
    static constexpr uint8_t ADC_ADDR = 0x48;
    static constexpr uint32_t REINIT_PERIOD_MS = 500;
    static constexpr uint32_t WIRE_TIMEOUT_US = 5000;
    static constexpr uint32_t ADC_CONVERSION_US = 9000; // ADS1115 at 128 SPS plus margin.

    uint8_t commandedActiveMask() const {
        uint8_t mask = 0;

        for (uint8_t i = 0; i < NUM_ACTUATORS; ++i) {
            if (commandedActive[i]) {
                mask |= (1u << i);
            }
        }

        return mask;
    }

    uint8_t feedbackActiveMask() const {
        uint8_t mask = 0;

        for (uint8_t i = 0; i < NUM_ACTUATORS; ++i) {
            if (feedbackActive[i]) {
                mask |= (1u << i);
            }
        }

        return mask;
    }

    bool hasPCBFault() const { return pcbFault_; }
    bool hasFault() const {
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
    void clearAllJammed() { for (uint8_t i = 0; i < NUM_ACTUATORS; ++i) jammed[i] = false; }

    void setup() {
        Wire.begin();
        configureWireTimeout_();

        emitStatus_("wire_configured");

        const bool dacOk = dac.begin(DAC_ADDR, &Wire);
        const bool adcOk = i2cPing_(ADC_ADDR); // ADS1115 requires no initialization sequence.

        if (!dacOk) emitStatus_("mcp4728_init_failed", "pcb");
        else emitStatus_("mcp4728_init_ok");
        if (!adcOk) emitStatus_("ads1115_init_failed", "pcb");
        else emitStatus_("ads1115_init_ok");

        pcbFault_ = !(dacOk && adcOk);
        i2cFaultLatched_ = pcbFault_;
        if (pcbFault_) nextReinitAttemptMs_ = millis() + REINIT_PERIOD_MS;
        resetAdcSequencer_();
        state = SET_POSITION;
    }

    void writeDAC(uint8_t channel, float voltage) {
        if (channel >= NUM_ACTUATORS) {
            emitError_("invalid_dac_channel", channel);
            return;
        }

        const float v = constrain(
            voltage,
            0.0f,
            CFG.actuator.maxCommandVoltage
        );

        if (v >= CFG.actuator.activate_cmd_min_v) {
            commandedActive[channel] = true;
        } else if (v <= CFG.actuator.deactivate_cmd_max_v) {
            commandedActive[channel] = false;
        }

        if (pcbFault_) {
            return;
        }

        MCP4728_channel_t c = MCP4728_CHANNEL_A;

        switch (channel) {
            case 0:
                c = MCP4728_CHANNEL_A;
                break;
            case 1:
                c = MCP4728_CHANNEL_B;
                break;
            case 2:
                c = MCP4728_CHANNEL_C;
                break;
            case 3:
                c = MCP4728_CHANNEL_D;
                break;
        }

        const uint16_t value = static_cast<uint16_t>(
            (v / CFG.actuator.maxCommandVoltage) * 4095.0f
        );

        clearWireTimeout_();
        emitI2CDebug_("before", "dac_write", channel);

        const bool ok = dac.setChannelValue(
            c,
            value,
            MCP4728_VREF_VDD,
            MCP4728_GAIN_1X,
            MCP4728_PD_MODE_NORMAL
        );

        emitI2CDebug_("after", "dac_write", channel);

        if (!ok || consumeWireTimeout_()) {
            latchI2CFault_("dac_write", channel);
        }
    }

    // Compatibility API. The normal state machine does NOT call this blocking wrapper.
    float readADC(uint8_t channel) {
        float voltage = 0.0f;
        if (!readADCBounded_(channel, voltage)) return 0.0f;
        return voltage;
    }

    void stateMachine() {
        if (pcbFault_) {
            tryReinitIfNeeded_();
            return;
        }

        if (state == SET_POSITION) {
            const uint32_t now = millis();
            for (uint8_t i = 0; i < NUM_ACTUATORS; ++i) {
                const float cmdV = actuatorPositions[i];
                writeDAC(i, cmdV);
                if (pcbFault_) return;
                armJamCheck_(i, cmdV, now);
            }
            resetAdcSequencer_();
            state = READ_FEEDBACK;
            return;
        }

        serviceAdcSequencer_(); 
    }

    void debugOutput() {
        for (uint8_t i = 0; i < NUM_ACTUATORS; ++i) {
            StaticJsonDocument<192> doc;
            doc["type"] = "actuator_debug";
            doc["channel"] = i;
            doc["cmd_V"] = actuatorPositions[i];
            doc["fb_V"] = feedbackSignals[i];
            doc["jam"] = jammed[i];
            doc["last_err_V"] = lastErrV[i];
            doc["pcbFault"] = pcbFault_;
            doc["i2cLatch"] = i2cFaultLatched_;
            serializeJson(doc, *io); io->println();
        }
    }

    void runSelfTest() {
        emitSelfTest_("start");
        for (uint8_t i = 0; i < NUM_ACTUATORS && !pcbFault_; ++i) {
            actuatorPositions[i] = 2.5f;
            writeDAC(i, 2.5f);
            delay(500);
            float fb = 0.0f;
            const bool ok = readADCBounded_(i, fb);
            if (ok) feedbackSignals[i] = fb;

            StaticJsonDocument<160> doc;
            doc["type"] = "actuator_self_test";
            doc["phase"] = "sample";
            doc["channel"] = i;
            doc["command_V"] = 2.5f;
            doc["feedback_V"] = fb;
            doc["ok"] = ok;
            serializeJson(doc, *io); io->println();

            actuatorPositions[i] = 0.0f;
            writeDAC(i, 0.0f);
            delay(250);
        }
        emitSelfTest_(pcbFault_ ? "aborted" : "complete");
    }

    float expectedFeedbackMapped(uint8_t, float cmdV) const {
        return constrain(CFG.actuator.fb_map_m * cmdV + CFG.actuator.fb_map_b,
                         0.0f, CFG.actuator.maxFeedbackVoltage);
    }

    float readADC_Avg(uint8_t channel, int samples = 5, int sampleDelayMs = 3) {
        if (pcbFault_ || samples <= 0) return 0.0f;
        float acc = 0.0f;
        int good = 0;
        for (int i = 0; i < samples && !pcbFault_; ++i) {
            float sample = 0.0f;
            if (!readADCBounded_(channel, sample)) break;
            acc += sample; ++good;
            if (sampleDelayMs > 0) delay(sampleDelayMs);
        }
        return good ? constrain(acc / good, 0.0f, CFG.actuator.maxFeedbackVoltage) : 0.0f;
    }

    void emitI2CDebug_(const char* phase, const char* operation, int channel = -1) {
#if ACTUATOR_I2C_DEBUG
        StaticJsonDocument<160> doc;
        doc["type"] = "debug"; doc["module"] = "actuator";
        doc["phase"] = phase; doc["operation"] = operation; doc["time_ms"] = millis();
        if (channel >= 0) doc["channel"] = channel;
        serializeJson(doc, *io); io->println();
#else
        (void)phase; (void)operation; (void)channel;
#endif
    }

private:
    enum class AdcPhase : uint8_t { START, WAIT, READ };
    AdcPhase adcPhase_ = AdcPhase::START;
    uint8_t adcChannel_ = 0;
    uint32_t adcReadyUs_ = 0;
    uint32_t nextReinitAttemptMs_ = 0;

    static constexpr uint8_t ADS_REG_CONVERSION = 0x00;
    static constexpr uint8_t ADS_REG_CONFIG = 0x01;
    static constexpr uint16_t ADS_OS_SINGLE = 0x8000;
    static constexpr uint16_t ADS_PGA_6_144V = 0x0000;
    static constexpr uint16_t ADS_MODE_SINGLE = 0x0100;
    static constexpr uint16_t ADS_DR_128SPS = 0x0080;
    static constexpr uint16_t ADS_COMP_DISABLE = 0x0003;

    void configureWireTimeout_() {
#if defined(WIRE_HAS_TIMEOUT)
        Wire.setWireTimeout(WIRE_TIMEOUT_US, true);
        Wire.clearWireTimeoutFlag();
#endif
    }
    void clearWireTimeout_() {
#if defined(WIRE_HAS_TIMEOUT)
        Wire.clearWireTimeoutFlag();
#endif
    }
    bool consumeWireTimeout_() {
#if defined(WIRE_HAS_TIMEOUT)
        if (Wire.getWireTimeoutFlag()) {
            Wire.clearWireTimeoutFlag();
            return true;
        }
#endif
        return false;
    }

    bool writeRegister16_(uint8_t reg, uint16_t value) {
        clearWireTimeout_();
        Wire.beginTransmission(ADC_ADDR);
        Wire.write(reg);
        Wire.write(static_cast<uint8_t>(value >> 8));
        Wire.write(static_cast<uint8_t>(value & 0xFF));
        const uint8_t err = Wire.endTransmission(true);
        return err == 0 && !consumeWireTimeout_();
    }

    bool readRegister16_(uint8_t reg, uint16_t& value) {
        clearWireTimeout_();
        Wire.beginTransmission(ADC_ADDR);
        Wire.write(reg);
        const uint8_t err = Wire.endTransmission(false);
        if (err != 0 || consumeWireTimeout_()) return false;

        clearWireTimeout_();
        const uint8_t count = Wire.requestFrom(ADC_ADDR, static_cast<uint8_t>(2), static_cast<uint8_t>(true));
        if (count != 2 || consumeWireTimeout_()) {
            while (Wire.available()) (void)Wire.read();
            return false;
        }
        const uint8_t msb = static_cast<uint8_t>(Wire.read());
        const uint8_t lsb = static_cast<uint8_t>(Wire.read());
        value = (static_cast<uint16_t>(msb) << 8) | lsb;
        return true;
    }

    uint16_t adcConfig_(uint8_t channel) const {
        const uint16_t mux = static_cast<uint16_t>(0x4000u + (static_cast<uint16_t>(channel) << 12));
        return ADS_OS_SINGLE | mux | ADS_PGA_6_144V | ADS_MODE_SINGLE |
               ADS_DR_128SPS | ADS_COMP_DISABLE;
    }

    void resetAdcSequencer_() {
        adcPhase_ = AdcPhase::START;
        adcChannel_ = 0;
        adcReadyUs_ = 0;
    }

    void serviceAdcSequencer_() {
        if (adcChannel_ >= NUM_ACTUATORS) {
            state = SET_POSITION;
            resetAdcSequencer_();
            return;
        }

        switch (adcPhase_) {
            case AdcPhase::START:
                emitI2CDebug_("before", "adc_start", adcChannel_);
                if (!writeRegister16_(ADS_REG_CONFIG, adcConfig_(adcChannel_))) {
                    latchI2CFault_("adc_start", adcChannel_);
                    return;
                }
                emitI2CDebug_("after", "adc_start", adcChannel_);
                adcReadyUs_ = micros() + ADC_CONVERSION_US;
                adcPhase_ = AdcPhase::WAIT;
                return;

            case AdcPhase::WAIT:
                if (static_cast<int32_t>(micros() - adcReadyUs_) >= 0) adcPhase_ = AdcPhase::READ;
                return;

            case AdcPhase::READ: {
                uint16_t rawBits = 0;
                emitI2CDebug_("before", "adc_result", adcChannel_);
                if (!readRegister16_(ADS_REG_CONVERSION, rawBits)) {
                    latchI2CFault_("adc_result", adcChannel_);
                    return;
                }
                emitI2CDebug_("after", "adc_result", adcChannel_);
                feedbackSignals[adcChannel_] =
                    rawToVoltage_(static_cast<int16_t>(rawBits));

                updateFeedbackActive_(
                    adcChannel_,
                    feedbackSignals[adcChannel_]
                );
                evaluateJam_(adcChannel_, millis());
                ++adcChannel_;
                adcPhase_ = AdcPhase::START;
                return;
            }
        }
    }

    bool readADCBounded_(uint8_t channel, float& voltage) {
        voltage = 0.0f;
        if (channel >= NUM_ACTUATORS || pcbFault_) return false;
        if (!writeRegister16_(ADS_REG_CONFIG, adcConfig_(channel))) {
            latchI2CFault_("adc_start_sync", channel); return false;
        }
        delayMicroseconds(ADC_CONVERSION_US);
        uint16_t rawBits = 0;
        if (!readRegister16_(ADS_REG_CONVERSION, rawBits)) {
            latchI2CFault_("adc_result_sync", channel); return false;
        }
        voltage = rawToVoltage_(
            static_cast<int16_t>(rawBits)
        );

        updateFeedbackActive_(
            channel,
            voltage
        );
        return true;
    }

    float rawToVoltage_(int16_t raw) const {
        static constexpr float ADS_FULL_SCALE_V =
            6.144f;

        const float voltage =
            static_cast<float>(raw) *
            (ADS_FULL_SCALE_V / 32768.0f);

        return constrain(
            voltage,
            0.0f,
            CFG.actuator.maxFeedbackVoltage
        );
    }

    void armJamCheck_(uint8_t i, float cmdV, uint32_t now) {
        const bool activate = cmdV >= CFG.actuator.activate_cmd_min_v;
        const bool deactivate = cmdV <= CFG.actuator.deactivate_cmd_max_v;
        if (activate && !lastRequestedActive[i]) {
            lastRequestedActive[i] = true; expectedActive[i] = true;
        } else if (deactivate && lastRequestedActive[i]) {
            lastRequestedActive[i] = false; expectedActive[i] = false;
        } else return;
        checkAtMs[i] = now + CFG.actuator.settle_ms;
        checkArmed[i] = true;
        jammed[i] = false;
    }

    void evaluateJam_(
        uint8_t i,
        uint32_t now
    ) {
        if (
            !checkArmed[i] ||
            static_cast<int32_t>(
                now - checkAtMs[i]
            ) < 0
        ) {
            return;
        }

        if (expectedActive[i]) {
            // An active actuator should have low feedback.
            lastErrV[i] =
                feedbackSignals[i] -
                CFG.actuator.active_fb_max_v;

            jammed[i] =
                feedbackSignals[i] >
                CFG.actuator.active_fb_max_v;
        } else {
            // An inactive actuator should have high feedback.
            lastErrV[i] =
                CFG.actuator.inactive_fb_min_v -
                feedbackSignals[i];

            jammed[i] =
                feedbackSignals[i] <
                CFG.actuator.inactive_fb_min_v;
        }

        checkArmed[i] = false;
    }

    bool i2cPing_(uint8_t addr) {
        clearWireTimeout_();
        Wire.beginTransmission(addr);
        const uint8_t err = Wire.endTransmission(true);
        return err == 0 && !consumeWireTimeout_();
    }

    void latchI2CFault_(const char* where, int detail = -1) {
        pcbFault_ = true;
        i2cFaultLatched_ = true;
        nextReinitAttemptMs_ = millis() + REINIT_PERIOD_MS;
        resetAdcSequencer_();
        StaticJsonDocument<160> doc;
        doc["type"] = "status"; doc["module"] = "actuator";
        doc["fault"] = "i2c_runtime_disconnect"; doc["where"] = where;
        if (detail >= 0) doc["detail"] = detail;
        serializeJson(doc, *io); io->println();
    }

    void tryReinitIfNeeded_() {
        const uint32_t now = millis();
        if (static_cast<int32_t>(now - nextReinitAttemptMs_) < 0) return;
        nextReinitAttemptMs_ = now + REINIT_PERIOD_MS;

        // Reapply timeout because reset-on-timeout may have reset the peripheral.
        Wire.begin();
        configureWireTimeout_();

        if (!i2cPing_(DAC_ADDR) || !i2cPing_(ADC_ADDR)) return;
        if (!dac.begin(DAC_ADDR, &Wire)) return;

        pcbFault_ = false;
        i2cFaultLatched_ = false;
        state = SET_POSITION;
        resetAdcSequencer_();
        emitStatus_("i2c_recovered");
    }

    void updateFeedbackActive_(uint8_t channel, float voltage) {
        if (channel >= NUM_ACTUATORS) {
            return;
        }

        // Feedback is active-low:
        // Low voltage confirms active.
        // High voltage confirms inactive.
        // Between the thresholds, retain the previous state.
        if (voltage <= CFG.actuator.active_fb_max_v) {
            feedbackActive[channel] = true;
        } else if (voltage >= CFG.actuator.inactive_fb_min_v) {
            feedbackActive[channel] = false;
        }
    }

    void emitStatus_(const char* status, const char* fault = nullptr) {
        StaticJsonDocument<160> doc;
        doc["type"] = "status"; doc["module"] = "actuator"; doc["status"] = status;
        if (fault) doc["fault"] = fault;
#if defined(WIRE_HAS_TIMEOUT)
        doc["wire_timeout_us"] = WIRE_TIMEOUT_US;
#else
        doc["wire_timeout"] = "unsupported";
#endif
        serializeJson(doc, *io); io->println();
    }
    void emitError_(const char* error, int detail) {
        StaticJsonDocument<96> doc;
        doc["type"] = "error"; doc["module"] = "actuator"; doc["error"] = error; doc["detail"] = detail;
        serializeJson(doc, *io); io->println();
    }
    void emitSelfTest_(const char* phase) {
        StaticJsonDocument<96> doc;
        doc["type"] = "actuator_self_test"; doc["phase"] = phase;
        serializeJson(doc, *io); io->println();
    }
};

#endif
