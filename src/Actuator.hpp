#ifndef ACTUATOR_CONTROL_H
#define ACTUATOR_CONTROL_H

#include <Arduino.h>
#include <ArduinoJson.h>
#include <Wire.h>
#include <Adafruit_MCP4728.h>
#include <Adafruit_ADS1X15.h>

#define NUM_ACTUATORS 3
#define MAX_VOLTAGE 5.0
#define MAX_FEEDBACK 3.3

class ActuatorControl {
    public:
        Adafruit_MCP4728 dac;
        Adafruit_ADS1115 adc;
        
        float actuatorPositions[NUM_ACTUATORS] = {0}; // Desired positions (0-5V)
        float feedbackSignals[NUM_ACTUATORS] = {0}; //Feedback (0-3.3V)

        enum States { SET_POSITION, READ_FEEDBACK};
        States state;

        StaticJsonDocument<32> json;
        char packet[32];

        void setup() {
            Wire.begin(); // Initialize I2C on specified pins
            Serial.println("Initializing ActuatorControl...");

            if (!dac.begin()) {
                Serial.println("Failed to initialize MCP4728");
                while(1); // Stop if DAC initialization fails.
            } else {
                Serial.println("MCP4728 initialized successfully.");
            }
            
            // Initialize ADC
            if (!adc.begin()) {
                Serial.println("** Error: Failed to initialize ADS1115. Halting. **");
                while (1);
            } else {
                Serial.println("ADS1115 initialized successfully");
            }
            state = SET_POSITION;
        }

        void writeDAC(uint8_t channel, float voltage) {
            // Ensure voltage is within bounds
            voltage = constrain(voltage, 0.0, MAX_VOLTAGE);

            // Scale voltage to 12-bit value
            uint16_t dacValue = (uint16_t)((voltage / MAX_VOLTAGE) * 4095);

            // Map uint8_t channel to MCP4728_channel_t enum
            MCP4728_channel_t channelEnum;
            switch (channel) {
                case 0:
                    channelEnum = MCP4728_CHANNEL_A; break;
                case 1:
                    channelEnum = MCP4728_CHANNEL_B; break;
                case 2:
                    channelEnum = MCP4728_CHANNEL_C; break;
                case 3:
                    channelEnum = MCP4728_CHANNEL_D; break;

                default:
                    Serial.println("Invalid DAC channel.");
                    Serial.println(channel);
                    return; // Exit if an invalid channel is given
            }

            // Write the value to the specified DAC channel in JSON
            if (!dac.setChannelValue(channelEnum, dacValue, MCP4728_VREF_VDD, MCP4728_GAIN_1X, MCP4728_PD_MODE_NORMAL)) {
                Serial.println("** Error: Failed to set DAC channel value!");
                Serial.print(channel);
                Serial.println(" value! **");
            } else {
                //Serial.print("DAC channel ");
                //Serial.print(channel);
                //Serial.print(" successfully set to ");
                //Serial.print(voltage);
                //Serial.print(" V");
                //delay(250);
            }

        }

        float readADC(uint8_t channel) {
            //Read ADC values for given channel
            int16_t rawValue = adc.readADC_SingleEnded(channel);
            float voltage = (rawValue / 32767.0) * MAX_FEEDBACK; // Scale raw ADC value to feedback range
            return voltage;
        }


        void stateMachine(){
            switch (state) {
                case SET_POSITION:
                    for (uint8_t i = 0; i < NUM_ACTUATORS; i++) {
                        writeDAC(i, actuatorPositions[i]);
                    }
                    state = READ_FEEDBACK;
                    break;

                case READ_FEEDBACK:
                    for (uint8_t i = 0; i < NUM_ACTUATORS; i++) {
                        feedbackSignals[i] = readADC(i);
                    }
                    state = SET_POSITION; 
                    break;

                default:
                    break;

            }
        }

        void debugOutput() {
            Serial.println("--------- Debug Output ---------");
            for (uint8_t i = 0; i < NUM_ACTUATORS; i++) {
                Serial.print("Actuator ");
                Serial.print(i);
                Serial.print("Position Set = ");
                Serial.print(actuatorPositions[i]);
                Serial.print(" V, Feedback = ");
                Serial.print(feedbackSignals[i]);
                Serial.print(" V");
            }
            Serial.println("-------------------------------");

        }

        void runSelfTest() {
            Serial.println (">>> Starting self-test routine...");
            for (uint8_t i = 0; i < NUM_ACTUATORS; i++) {
                Serial.print("Testing actuator ");
                Serial.println(i);

                float testVoltage = 2.5;
                actuatorPositions[i] = testVoltage;
                writeDAC(i, testVoltage);
                delay(500);

                float feedback = readADC(i % 4);
                feedbackSignals[i] = feedback;
                Serial.print("Actuator ");
                Serial.print(i);
                Serial.print(" -> Set: ");
                Serial.print(testVoltage);
                Serial.print(" V, Feedback: ");
                Serial.print(feedback);
                Serial.print(" V" );

                // Reset actuator output before moving on
                actuatorPositions[i]= 0.0;
                writeDAC(i, 0.0);
                delay(250);
            }
            Serial.println("<<< Self-test routine completed.");
        }
};

#endif