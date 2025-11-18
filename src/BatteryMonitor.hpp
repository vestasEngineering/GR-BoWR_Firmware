#ifndef BATTERY_MONITOR
#define BATTERY_MONITOR

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define BATTERY_PIN A0
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1 // Reset pin (not used with I2C)
#define MAX_BATTERY_VOLTAGE 20.0
#define MIN_BATTERY_VOLTAGE 16.5

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

class BatteryMonitor {
public:
    float voltage = 0.0;
    const float R1 = 100000.0;
    const float R2 = 10000.0;
    const float VREF = 3.1;
    const float ADC_MAX = 1023.0;
    static const int NUM_SAMPLES = 10;
    float voltageSamples[NUM_SAMPLES];
    int sampleIndex = 0;
    bool bufferFilled = false;

    enum States {
        READING,
        DISPLAYING,
    };
    States state;

    void setup(void) {
        analogReadResolution(10);

        if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
            Serial.println(F("SSD1306 allocation failed"));
            while (true); // Halt
        }

        display.clearDisplay();
        display.setTextSize(3);
        display.setTextColor(SSD1306_WHITE);
        display.setCursor(0, 0);
        display.println("GR-LRR");
        display.display();
        delay(2000);

        state = READING;
    }

    void readBatteryVoltage(void) {
        int adcValue = analogRead(BATTERY_PIN);
        float measuredVoltage = (adcValue * VREF / ADC_MAX);
        float actualVoltage = measuredVoltage * (R1 + R2) / R2;

        voltageSamples[sampleIndex] = actualVoltage;
        sampleIndex = (sampleIndex + 1) % NUM_SAMPLES;
        if (sampleIndex == 0) bufferFilled = true;

        int count = bufferFilled ? NUM_SAMPLES : sampleIndex;
        float sum = 0.0;
        for (int i = 0; i < count; i++) {
            sum += voltageSamples[i];
        }
        voltage = sum / count;
    }

    void drawVoltage(float voltage) {
        display.setTextSize(2);
        display.setCursor(0, 0);
        display.print("V: ");
        display.print(voltage, 1); // One decimal place
    }

    void drawPercentage(float percentage) {
        display.setTextSize(2);
        display.setCursor(0, 20);
        display.print("Chg: ");
        display.print(int(percentage));
        display.print("%");
    }

    void drawBatteryIcon(float percentage) {
        int iconX = 100;
        int iconY = 0;
        int iconWidth = 20;
        int iconHeight = 12;

        // Outline
        display.drawRect(iconX, iconY, iconWidth, iconHeight, SSD1306_WHITE);
        // Tip
        display.fillRect(iconX + iconWidth, iconY + 4, 2, 4, SSD1306_WHITE);

        // Fill level
        int fillWidth = (percentage / 100.0) * (iconWidth - 2);
        display.fillRect(iconX + 1, iconY + 1, fillWidth, iconHeight - 2, SSD1306_WHITE);
    }

    void drawGaugeBar(float percentage) {
        int gaugeX = 14;
        int gaugeY = 45;
        int gaugeHeight = 10;
        int gaugeMaxWidth = 100;
        int gaugeFillWidth = (percentage / 100.0) * gaugeMaxWidth;

        display.drawRect(gaugeX, gaugeY, gaugeMaxWidth, gaugeHeight, SSD1306_WHITE);
        display.fillRect(gaugeX + 1, gaugeY + 1, gaugeFillWidth - 2, gaugeHeight - 2, SSD1306_WHITE);
    }

    void displayVoltage(void) {
        float batteryPercentage = (voltage - MIN_BATTERY_VOLTAGE) / (MAX_BATTERY_VOLTAGE - MIN_BATTERY_VOLTAGE) * 100.0;
        batteryPercentage = constrain(batteryPercentage, 0.0, 100.0);

        display.clearDisplay();

        drawVoltage(voltage);
        drawPercentage(batteryPercentage);
        drawBatteryIcon(batteryPercentage);
        drawGaugeBar(batteryPercentage);

        display.display();
    }

    void stateMachine() {
        switch (state) {
            case READING:
                readBatteryVoltage();
                state = DISPLAYING;
                break;
            case DISPLAYING:
                displayVoltage();
                state = READING;
                break;
            default:
                break;
        }
    }
};

#endif