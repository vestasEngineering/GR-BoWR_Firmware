#ifndef EXTERNAL_ENCODER
#define EXTERNAL_ENCODER

#include <Arduino.h>
#include <ArduinoJson.h>

#define pinA A5
#define pinB A6
#define PULSES_PER_CM 37.6 //Calculated from encoder wheel circumference and encoder resolution.
#define ENCODER_UPDATE_DELAY 1250 //Delay in milliseconds.

class ExtEncoder {
    public: 

        volatile long position;
        volatile int lastEncoded;
        volatile int thisDelay;
        volatile bool indexDetected;

        StaticJsonDocument<64> json;
        char packet[64];

        enum EncoderState {
            RUNNING,
            INDEX_RESET
        };

        EncoderState state;


        ExtEncoder():
            position(0),
            lastEncoded(0),
            indexDetected(false),
            thisDelay(ENCODER_UPDATE_DELAY),
            state(RUNNING)
        { }

        void setup() {

            pinMode(pinA, INPUT_PULLUP);
            pinMode(pinB, INPUT_PULLUP);
            // pinMode(pinZ, INPUT);

            lastEncoded = readEncoderPins();
            _instance = this;

            attachInterrupt(digitalPinToInterrupt(pinA), isrA, CHANGE);
            attachInterrupt(digitalPinToInterrupt(pinB), isrB, CHANGE);
            // attachInterrupt(digitalPinToInterrupt(pinZ), isrIndex, RISING);
        }

        int readEncoderPins() {
            int a = digitalRead(pinA);
            int b = digitalRead(pinB);
            return (a << 1) | b;
        }

        void updateEncoder() {
            int encoded = readEncoderPins();

            int sum = (lastEncoded << 2) | encoded;

            if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) {
                position++;
            } else if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) {
                position--;
            }
            lastEncoded = encoded;
        }

        void updateIndex() {
            position = 0;
            indexDetected = true;
            state = INDEX_RESET;
        }

        long getPosition() {
            noInterrupts();
            long pos = position;
            interrupts();
            return pos / PULSES_PER_CM * -1; //Convert pulses to cm.
        }

        void writeDistanceJson() {
            json["encoder_distance"] = getPosition();
            serializeJson(json, packet);
            Serial.println(packet);
        }

        void stateMachine() {
            if (thisDelay == 0) {
                writeDistanceJson();
                thisDelay = ENCODER_UPDATE_DELAY;
            }
            switch (state) {
                case RUNNING:
                    break;

                case INDEX_RESET:
                    indexDetected = false;
                    state = RUNNING;
                    break;

                default:
                    state = RUNNING;
                    break;
            }
        }

        /*
        void debugOutput() {
            Serial.print("Encoder Position: ");
            Serial.print(getPosition());
            Serial.print(" cm");
            //Serial.print(digitalRead(pinA));
            delay(100);
            if(indexDetected) {
                Serial.print(" (Index pulse detected!)");
                indexDetected = false;
            }
            Serial.println();
        }
        */

        /*
        void debugInput() {
            Serial.print("Pin A: ");
            Serial.print(digitalRead(pinA));
            Serial.print(" Pin B: ");
            Serial.println(digitalRead(pinB));
            delay(200);
        }
        */   

        static ExtEncoder* _instance;

        static void isrA() {
            if (_instance) _instance->updateEncoder();
        }

        static void isrB() {
            if (_instance) _instance->updateEncoder();
        }

        static void isrIndex() {
            if (_instance) _instance->updateIndex();
        }
};

ExtEncoder* ExtEncoder::_instance = nullptr;

#endif