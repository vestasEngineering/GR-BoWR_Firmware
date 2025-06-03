#ifndef BATTERY_MONITOR
#define BATTERY_MONITOR

#include <Arduino.h>
#include <Wire.h>
#include <hd44780.h>
#include <hd44780ioClass/hd44780_I2Cexp.h>

#define BATTERY_PIN A0
#define LCD_ADDRESS 0x27
#define LCD_COLUMNS 16 
#define LCD_ROWS 2
#define MAX_BATTERY_VOLTAGE 20.5 // Maximum battery voltage for a fully charged LiPo cell
#define MIN_BATTERY_VOLTAGE 12.5 // Minimum battery voltage for a LiPo cell

hd44780_I2Cexp lcd;

class BatteryMonitor {
    public:
        float voltage = 0.0;
        const float R1 = 100000.0; // Resistor 1 value in ohms
        const float R2 = 10000.0; // Resistor 2 value in ohms
        const float VREF = 3.1; // Supply voltage in volts
        const float ADC_MAX = 1023.0; // Maximum ADC value for 10-bit resolution

        enum States {
            READING,
            DISPLAYING,
        };
        States state;

        void setup(void) {
            analogReadResolution(10);
            
            if (lcd.begin(16,2) == 0) {
                lcd.clear();
                lcd.print("Battery Monitor");
                delay(2000);
            } else {
                Serial.println("LCD initialization failed");
            }

            state = READING;
        }

        void readBatteryVoltage(void) {
            int adcValue = analogRead(BATTERY_PIN);
            float measuredVoltage = (adcValue * VREF / ADC_MAX);
            voltage = measuredVoltage * (R1 + R2) / R2; // Voltage divider formula
        }

        void displayVoltage(void) {

            float batteryPercentage = (voltage - MIN_BATTERY_VOLTAGE) / (MAX_BATTERY_VOLTAGE - MIN_BATTERY_VOLTAGE) * 100.0;

            batteryPercentage = constrain(batteryPercentage, 0.0, 100.0); // Ensure percentage is within 0-100%

            int gaugeValue = (batteryPercentage / 100.0) * 14; // Map percentage to LCD columns

            String guage = "|";
            for (int i = 0; i < gaugeValue; i++) {
                guage += '=';
            }
            for (int i = gaugeValue; i < 14; i++) {
                guage += ' ';
            }
            guage += "|";


            lcd.clear();
            lcd.setCursor(0, 0);
            lcd.print("Battery: ");
            lcd.print(int(batteryPercentage));
            lcd.print("%");

            lcd.setCursor(0, 1);
            lcd.print(guage);
        }

        void stateMachine() {
            switch (state) {
                case READING:
                    readBatteryVoltage();
                    state = DISPLAYING;
                    break;
                case DISPLAYING:
                    displayVoltage();
                    //delay(200); // Update display every 2 seconds
                    state = READING; 
                    break;
                
                default:
                    break;
            }
        }
};

#endif