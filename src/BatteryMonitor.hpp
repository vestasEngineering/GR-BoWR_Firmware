
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
#define OLED_RESET -1 // Reset pin (not used with I2C)
#define MAX_BATTERY_VOLTAGE 20.0
#define MIN_BATTERY_VOLTAGE 16.5

extern Adafruit_SSD1306 display;

class BatteryMonitor {
public:

    Stream* io = nullptr;
    BatteryMonitor(Stream& ioRef) : io(&ioRef) {}

    float voltage = 0.0;
    float lastPercentage = NAN;
    bool  displayOk = false;
    const float R1 = 100000.0;
    const float R2 = 10000.0;
    const float VREF = 3.1;
    const float ADC_MAX = 1023.0;
    static const int NUM_SAMPLES = 10;
    float voltageSamples[NUM_SAMPLES];
    int sampleIndex = 0;
    bool bufferFilled = false;
    unsigned long lastEmitMs = 0;
    float         lastEmittedPct = NAN;

    enum States { READING, DISPLAYING };
    States state;

    void setup(void) {
        analogReadResolution(10);
        Wire1.begin();

        if (display.begin(SSD1306_SWITCHCAPVCC, CFG.battery.oled_i2c_addr)) {
            displayOk = true;
        } else {
            displayOk = false;
            io->println(F("{\"type\":\"status\",\"module\":\"battery\",\"msg\":\"oled_not_found_running_headless\"}"));
        }

        if (displayOk) {
            display.clearDisplay();
            display.setTextSize(3);
            display.setTextColor(SSD1306_WHITE);
            display.setCursor(0, 0);
            display.println("GR-LRR");
            display.display();
            delay(2000);
       }

        state = READING;
    }

    // Compute battery % from current voltage
    float computePercentage(float v) const {
        float pct = (v - CFG.battery.min_voltage) /
                    (CFG.battery.max_voltage - CFG.battery.min_voltage) * 100.0f;
        if (pct < 0.0f) pct = 0.0f;
        if (pct > 100.0f) pct = 100.0f;
        return pct;
    }

    bool isLow(float thresholdPercent = NAN) const {
        float thresh = isnan(thresholdPercent) ? CFG.battery.low_pct : thresholdPercent;
        float pct = isnan(lastPercentage) ? computePercentage(voltage) : lastPercentage;
        return pct <= thresh;
    }
    bool isCritical(float thresholdPercent = NAN) const {
        float thresh = isnan(thresholdPercent) ? CFG.battery.critical_pct : thresholdPercent;
        float pct = isnan(lastPercentage) ? computePercentage(voltage) : lastPercentage;
        return pct <= thresh;
    }
    float getPercentage() const {
        return isnan(lastPercentage) ? computePercentage(voltage) : lastPercentage;
    }


    void readBatteryVoltage(void) {
        int adcValue = analogRead(BATTERY_PIN); // PIN can also be moved to config if needed
        float measuredVoltage = (adcValue * CFG.battery.vref / CFG.battery.adc_max_counts);
        float actualVoltage = measuredVoltage * (CFG.battery.r1_ohm + CFG.battery.r2_ohm) / CFG.battery.r2_ohm;

        voltageSamples[sampleIndex] = actualVoltage;
        sampleIndex = (sampleIndex + 1) % NUM_SAMPLES;
        if (sampleIndex == 0) bufferFilled = true;

        int count = bufferFilled ? NUM_SAMPLES : sampleIndex;
        float sum = 0.0;
        for (int i = 0; i < count; i++) {
            sum += voltageSamples[i];
        }
        voltage = sum / count;
        lastPercentage = computePercentage(voltage);
    }

    void drawVoltage(float v) {
        if (!displayOk) return;

        display.setTextSize(2);
        display.setCursor(0, 0);
        display.print("V: ");
        display.print(v, 1);
    }

    void drawPercentage(float percentage) {
        if (!displayOk) return;

        display.setTextSize(2);
        display.setCursor(0, 20);
        display.print("Chg: ");
        display.print(int(percentage));
        display.print("%");
    }

    void drawBatteryIcon(float percentage) {
        if (!displayOk) return;
        int iconX = 100;
        int iconY = 0;
        int iconWidth = 20;
        int iconHeight = 12;

        display.drawRect(iconX, iconY, iconWidth, iconHeight, SSD1306_WHITE);
        display.fillRect(iconX + iconWidth, iconY + 4, 2, 4, SSD1306_WHITE);

        int fillWidth = (percentage / 100.0) * (iconWidth - 2);
        display.fillRect(iconX + 1, iconY + 1, fillWidth, iconHeight - 2, SSD1306_WHITE);
    }

    void drawGaugeBar(float percentage) {
        if (!displayOk) return;
        int gaugeX = 14;
        int gaugeY = 45;
        int gaugeHeight = 10;
        int gaugeMaxWidth = 100;
        int gaugeFillWidth = (percentage / 100.0) * gaugeMaxWidth;

        display.drawRect(gaugeX, gaugeY, gaugeMaxWidth, gaugeHeight, SSD1306_WHITE);
        display.fillRect(gaugeX + 1, gaugeY + 1, gaugeFillWidth - 2, gaugeHeight - 2, SSD1306_WHITE);
    }

    void displayVoltage(void) {
        float batteryPercentage = computePercentage(voltage);
        lastPercentage = batteryPercentage;

        if (!displayOk) return;
        display.clearDisplay();

        drawVoltage(voltage);
        drawPercentage(batteryPercentage);
        drawBatteryIcon(batteryPercentage);
        drawGaugeBar(batteryPercentage);

        display.display();
    }

    void stateMachine() {
        switch (state) {
            case READING:    readBatteryVoltage(); state = DISPLAYING; break;
            case DISPLAYING: 
                if (displayOk) {
                    displayVoltage();
                }
                emitBatteryStatusIfNeeded();
                state = READING;    
                break;
            default: break;
        }
    }


    void emitBatteryStatusIfNeeded() {
        const unsigned long now = millis();
        const bool timeGate = (now - lastEmitMs) >= CFG.battery.emit_ms;
        const float pct = isnan(lastPercentage) ? computePercentage(voltage) : lastPercentage;
        const bool changeGate = isnan(lastEmittedPct) || fabs(pct - lastEmittedPct) >= CFG.battery.emit_pct_delta;

        if (!(timeGate || changeGate)) return;

        const float v = voltage;
        const bool pass = (v >= CFG.battery.min_voltage && v <= CFG.battery.max_voltage);

        StaticJsonDocument<192> doc;
        doc["type"] = "test_result";
        doc["id"] = "battery";
        doc["category"] = "sensor";
        doc["pass"] = pass;
        doc["reason"] = pass ? "voltage_in_range" : "out_of_range";
        auto m = doc.createNestedObject("measurements");
        m["voltage_V"] = v;
        m["pct"] = pct;

        // Emit one line JSON to the Pi over USB serial
        serializeJson(doc, *io); io->println();

        lastEmitMs = now;
        lastEmittedPct = pct;
    }
};

#endif