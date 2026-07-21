#ifndef BATTERY_MONITOR
#define BATTERY_MONITOR

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <math.h>
#include <ArduinoJson.h>
#include "Config.hpp"

#define BATTERY_PIN A0
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1

// Legacy compatibility for MySerial.hpp and BootHealth.cpp.
// Keep one source of truth by mapping the old names to CFG values.
#ifndef MIN_BATTERY_VOLTAGE
#define MIN_BATTERY_VOLTAGE (CFG.battery.min_voltage)
#endif

#ifndef MAX_BATTERY_VOLTAGE
#define MAX_BATTERY_VOLTAGE (CFG.battery.max_voltage)
#endif

extern Adafruit_SSD1306 display;

class BatteryMonitor {
public:
    explicit BatteryMonitor(Stream& ioRef) : io(&ioRef) {}

    Stream* io = nullptr;

    float voltage = 0.0f;
    float lastPercentage = NAN;
    bool displayOk = false;

    static constexpr int NUM_SAMPLES = 10;
    float voltageSamples[NUM_SAMPLES] = {0.0f};
    int sampleIndex = 0;
    bool bufferFilled = false;

    unsigned long lastEmitMs = 0;
    float lastEmittedPct = NAN;

    // Retained for source compatibility. Timing is now controlled by independent
    // sample/display gates rather than alternating on every loop iteration.
    enum States { READING, DISPLAYING };
    States state = READING;

    void setup() {
        analogReadResolution(10);
        Wire1.begin();

        displayOk = display.begin(
            SSD1306_SWITCHCAPVCC,
            CFG.battery.oled_i2c_addr
        );

        if (!displayOk) {
            io->println(F(
                "{\"type\":\"status\","
                "\"module\":\"battery\","
                "\"msg\":\"oled_not_found_running_headless\"}"
            ));
        } else {
            display.clearDisplay();
            display.setTextSize(3);
            display.setTextColor(SSD1306_WHITE);
            display.setCursor(0, 0);
            display.println(F("GR-LRR"));
            display.display();
            delay(2000);
        }

        const uint32_t now = millis();

        // Make the first sample and first display update occur on the next loop.
        lastSampleMs_ = now - SAMPLE_INTERVAL_MS;
        lastDisplayMs_ = now - DISPLAY_INTERVAL_MS;

        lastEmitMs = now;
        lastEmittedPct = NAN;
        newBatterySample_ = false;
        state = READING;
    }

    float computePercentage(float v) const {
        const float span = CFG.battery.max_voltage - CFG.battery.min_voltage;
        if (span <= 0.0f) return 0.0f;

        const float pct =
            ((v - CFG.battery.min_voltage) / span) * 100.0f;

        return constrain(pct, 0.0f, 100.0f);
    }

    bool isLow(float thresholdPercent = NAN) const {
        const float threshold = isnan(thresholdPercent)
            ? CFG.battery.low_pct
            : thresholdPercent;

        return getPercentage() <= threshold;
    }

    bool isCritical(float thresholdPercent = NAN) const {
        const float threshold = isnan(thresholdPercent)
            ? CFG.battery.critical_pct
            : thresholdPercent;

        return getPercentage() <= threshold;
    }

    float getPercentage() const {
        return isnan(lastPercentage)
            ? computePercentage(voltage)
            : lastPercentage;
    }

    void readBatteryVoltage() {
        const int adcValue = analogRead(BATTERY_PIN);

        const float measuredVoltage =
            static_cast<float>(adcValue) *
            CFG.battery.vref /
            CFG.battery.adc_max_counts;

        const float actualVoltage =
            measuredVoltage *
            (CFG.battery.r1_ohm + CFG.battery.r2_ohm) /
            CFG.battery.r2_ohm;

        voltageSamples[sampleIndex] = actualVoltage;
        sampleIndex = (sampleIndex + 1) % NUM_SAMPLES;

        if (sampleIndex == 0) {
            bufferFilled = true;
        }

        const int count = bufferFilled ? NUM_SAMPLES : sampleIndex;
        if (count <= 0) return;

        float sum = 0.0f;
        for (int i = 0; i < count; ++i) {
            sum += voltageSamples[i];
        }

        voltage = sum / static_cast<float>(count);
        lastPercentage = computePercentage(voltage);
    }

    void drawVoltage(float v) {
        if (!displayOk) return;

        display.setTextSize(2);
        display.setCursor(0, 0);
        display.print(F("V: "));
        display.print(v, 1);
    }

    void drawPercentage(float percentage) {
        if (!displayOk) return;

        display.setTextSize(2);
        display.setCursor(0, 20);
        display.print(F("Chg: "));
        display.print(static_cast<int>(percentage));
        display.print('%');
    }

    void drawBatteryIcon(float percentage) {
        if (!displayOk) return;

        constexpr int iconX = 100;
        constexpr int iconY = 0;
        constexpr int iconWidth = 20;
        constexpr int iconHeight = 12;

        const float bounded = constrain(percentage, 0.0f, 100.0f);
        const int innerWidth = iconWidth - 2;
        const int fillWidth = static_cast<int>(
            (bounded / 100.0f) * innerWidth
        );

        display.drawRect(
            iconX,
            iconY,
            iconWidth,
            iconHeight,
            SSD1306_WHITE
        );

        display.fillRect(
            iconX + iconWidth,
            iconY + 4,
            2,
            4,
            SSD1306_WHITE
        );

        if (fillWidth > 0) {
            display.fillRect(
                iconX + 1,
                iconY + 1,
                fillWidth,
                iconHeight - 2,
                SSD1306_WHITE
            );
        }
    }

    void drawGaugeBar(float percentage) {
        if (!displayOk) return;

        constexpr int gaugeX = 14;
        constexpr int gaugeY = 45;
        constexpr int gaugeHeight = 10;
        constexpr int gaugeWidth = 100;

        const float bounded = constrain(percentage, 0.0f, 100.0f);
        const int innerWidth = gaugeWidth - 2;
        const int fillWidth = static_cast<int>(
            (bounded / 100.0f) * innerWidth
        );

        display.drawRect(
            gaugeX,
            gaugeY,
            gaugeWidth,
            gaugeHeight,
            SSD1306_WHITE
        );

        if (fillWidth > 0) {
            display.fillRect(
                gaugeX + 1,
                gaugeY + 1,
                fillWidth,
                gaugeHeight - 2,
                SSD1306_WHITE
            );
        }
    }

    void displayVoltage() {
        const float batteryPercentage = computePercentage(voltage);
        lastPercentage = batteryPercentage;

        if (!displayOk) return;

        // These are the values actually visible on the OLED. Skip the full
        // framebuffer transfer if neither displayed value changed.
        const int displayedPct = static_cast<int>(batteryPercentage);
        const int displayedVoltageTenths = static_cast<int>(
            roundf(voltage * 10.0f)
        );

        if (
            displayedPct == lastDisplayedPct_ &&
            displayedVoltageTenths == lastDisplayedVoltageTenths_
        ) {
            return;
        }

        lastDisplayedPct_ = displayedPct;
        lastDisplayedVoltageTenths_ = displayedVoltageTenths;

        display.clearDisplay();
        drawVoltage(voltage);
        drawPercentage(batteryPercentage);
        drawBatteryIcon(batteryPercentage);
        drawGaugeBar(batteryPercentage);
        display.display();
    }

    void stateMachine() {
        const uint32_t now = millis();

        // Battery voltage changes slowly; 10 Hz is ample and keeps ADC work
        // out of the high-frequency robot loop.
        if (
            static_cast<uint32_t>(now - lastSampleMs_) >=
            SAMPLE_INTERVAL_MS
        ) {
            lastSampleMs_ = now;
            readBatteryVoltage();
            newBatterySample_ = true;
            emitBatteryStatusIfNeeded();
        }

        // Limit OLED traffic on Wire1 to 2 Hz. displayVoltage() additionally
        // suppresses the transfer when the visible values have not changed.
        if (
            displayOk &&
            newBatterySample_ &&
            static_cast<uint32_t>(now - lastDisplayMs_) >=
                DISPLAY_INTERVAL_MS
        ) {
            lastDisplayMs_ = now;
            newBatterySample_ = false;
            displayVoltage();
        }
    }

    void emitBatteryStatusIfNeeded() {
        const uint32_t now = millis();
        const float pct = getPercentage();

        const bool firstEmission = isnan(lastEmittedPct);

        const bool minimumIntervalPassed =
            static_cast<uint32_t>(now - lastEmitMs) >=
            MIN_EMIT_INTERVAL_MS;

        const bool heartbeatDue =
            static_cast<uint32_t>(now - lastEmitMs) >=
            CFG.battery.emit_ms;

        const bool meaningfulChange =
            firstEmission ||
            fabsf(pct - lastEmittedPct) >=
                CFG.battery.emit_pct_delta;

        if (
            !firstEmission &&
            !(heartbeatDue ||
              (minimumIntervalPassed && meaningfulChange))
        ) {
            return;
        }

        const bool pass =
            voltage >= CFG.battery.min_voltage &&
            voltage <= CFG.battery.max_voltage;

        StaticJsonDocument<192> doc;
        doc["type"] = "test_result";
        doc["id"] = "battery";
        doc["category"] = "sensor";
        doc["pass"] = pass;
        doc["reason"] = pass
            ? "voltage_in_range"
            : "out_of_range";

        JsonObject measurements =
            doc.createNestedObject("measurements");

        measurements["voltage_V"] = voltage;
        measurements["pct"] = pct;

        serializeJson(doc, *io);
        io->println();

        lastEmitMs = now;
        lastEmittedPct = pct;
    }

private:
    // Sampling and presentation are intentionally independent.
    static constexpr uint32_t SAMPLE_INTERVAL_MS = 100;   // 10 Hz
    static constexpr uint32_t DISPLAY_INTERVAL_MS = 500;  // 2 Hz
    static constexpr uint32_t MIN_EMIT_INTERVAL_MS = 1000;

    uint32_t lastSampleMs_ = 0;
    uint32_t lastDisplayMs_ = 0;
    bool newBatterySample_ = false;

    int lastDisplayedPct_ = -1;
    int lastDisplayedVoltageTenths_ = -1;
};

#endif
