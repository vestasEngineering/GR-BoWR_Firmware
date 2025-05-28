#ifndef MY_ULTRASONIC_CLASS
#define MY_ULTRASONIC_CLASS

#include <Arduino.h>
#include <ArduinoJson.h>
#include <MyPacket.hpp>

#define ULTRASONIC_PIN A2

class Ultrasonic{
    public:

            Ultrasonic(MyPacket& p) : packet(p) {} // pass a ref 


      volatile int delay = 0;
      float distance = 0.0;

      StaticJsonDocument<32> packet;
      char out[32];

      enum States {
        READING,
        WRITING,
      };
      States state;



      void serialStream(void){
        packet["distance"].set(distance);
        size_t len = serializeJson(packet, out);
        Serial.write(out, len);
        Serial.write('\n');
      }


      void stateMachine() {
            switch(state){
              case READING:
              // read the sensor
                if (!delay) {
                  // 1. float() to change analogRead() units
                  // 2. times 3.1 to scale to max ADC voltage for Portenta
                  // 3. divide by 4096 to convert from the ADC precision 
                  // leaky integrator over rolling ave, gain of 0.1
                  distance += ((float(analogRead(ULTRASONIC_PIN)) * 3.1 / 4096.0) - distance) * 0.1;
                  delay = 25; // 0.25 seconds
                  state = WRITING;
                }
              break;

              case WRITING:
              // write to serial
                if (!delay) { 
                  serialStream();
                  state = READING;  
                }   
              break;
              default:
              break;
            }

        }
    };

#endif