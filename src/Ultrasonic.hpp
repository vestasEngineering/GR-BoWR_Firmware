#ifndef MY_ULTRASONIC_CLASS
#define MY_ULTRASONIC_CLASS

#include <Arduino.h>
#include <ArduinoJson.h>
#define ULTRASONIC_PIN A2

class Ultrasonic {
    public:

        volatile int delay = 0;
        float distance[3] = {0.0, 0.0, 0.0}; // timesteps [n, n-1, n-2] so distance[0] = current distance
        //float distance = 0.0;
        float voltage = 0.0;
        float measuredDistance = 0.0;
        float mmPerVolt = (300.0-30.0)/(2.0 - 0.0);
        float offsetDistance = 40.0;
        StaticJsonDocument<32> json;
        char packet[32];

        enum States
        {
            READING,
            WRITING,
        };
        States state;


        void setup(void){
            analogReadResolution(10);
            state = READING;
        }


        void writeDistance(void) {
            json["distance"] = round(distance[0]);
            serializeJson(json, packet);
            Serial.println(packet);

        }



        void stateMachine()
        {
            switch (state)
            {
            case READING:
                // read the sensor
                if (!delay)
                {
                    // 1. float() to change analogRead() units
                    // 2. times 3.1 to scale to max ADC voltage for Portenta
                    // 3. divide by 1023 to convert from the ADC precision
                    // leaky integrator over rolling ave, gain of 0.5
                    voltage =  (float(analogRead(ULTRASONIC_PIN)) * 3.1 / 1023.0);
                    measuredDistance =  voltage * mmPerVolt + offsetDistance;
                    //distance[0] += (measuredDistance - distance[0]) * 0.5; // distance[0] = distance[0] + (measuredDistance - distance[0]) * 0.5
                    distance[0] = distance[0] + (measuredDistance - distance[0]) * 0.8;
                    ////distance[0] = 0.70 * distance[1] + 0.30 * distance[2];
                    //distance[1] = distance[0];
                    //distance[2] = distance[1];
                    delay = 25; // 0.25 seconds
                    state = WRITING;
                }
                break;

            case WRITING:
                // write to serial
                if (!delay)
                {
                    writeDistance();
                    state = READING;
                }
                break;
            default:
                break;
            }
        }
    };  

#endif