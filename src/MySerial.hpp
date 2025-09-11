#ifndef MY_SERIAL_CLASS
#define MY_SERIAL_CLASS

#include <ArduinoJson.h>
#include <Actuator.hpp>
#include "Ext_Encoder.hpp"
#include "AndonLight.hpp"  
#include "Motors.hpp"
#include <vector>

#define RED_LED LEDR

class MySerial
{
public:
    StaticJsonDocument<128> jsonPacket;
    static const byte numChars = 128;
    char receivedChars[numChars];
    boolean newData = false;

    boolean serialStarted = false;
    boolean serialEnded = false;
    bool LED_STATE = false;
    

    struct Trigger {
        int threshold;
        int activate_channel;
        int deactivate_channel;
        int delay_seconds;
        bool triggered = false;
        unsigned long triggerTime = 0;
        bool waitingToDeactivate = false;
    };
    std::vector<Trigger> triggerBuffer;

    ActuatorControl* actuator;
    ExtEncoder* encoder;
    AndonLight* andonLight;
    Motors* motors;

    MySerial(ActuatorControl& actuatorRef, AndonLight& lightRef, Motors& motorsRef, ExtEncoder& encoderRef)
    : actuator(&actuatorRef), andonLight(&lightRef), motors(&motorsRef), encoder(&encoderRef) {}



    enum States{ CONNECTED, DISCONNECTED};
    States state;

    volatile int thisDelay = 0;
    volatile int timeout = 0;
    volatile int receiveDelay = 0;

    void setup() {
        state = DISCONNECTED;
        Serial.begin(115200);
        Serial.println("{\"status\":\"serial_started\"}");
;
    }

    void stateMachine(void) {
        receiveLinux(); // will tell us if the serial has ended or not
        checkTriggers(); //Check encoder triggers

        switch (state) {

        case CONNECTED:
            // while we are connected, if the timeout is zero, then we have lost connection
            if (!thisDelay && !timeout) {
                thisDelay = 500;
                state = DISCONNECTED;
                motors->STOP();
            }
            if (!thisDelay) {
                thisDelay = 500;
                //Serial.println("connected");
            }
            break;

        case DISCONNECTED:
            // while we are disconnected, look for messages
            if (!thisDelay && timeout > 0) {
                thisDelay = 500;
                state = CONNECTED;
            }
            if (!thisDelay) {
                thisDelay = 500;
                //Serial.println("disconnected");
            }
            break;

        default:
            break;
        }
    }

    void receiveLinux(void) {
        if (!receiveDelay) {
            digitalWrite(RED_LED, HIGH);
        }

        recvWithStartEndMarkers();
        processMessage();
    }

    void recvWithStartEndMarkers(void) {
        static boolean recvInProcess = false;
        static byte ndx = 0;
        char startMarker = '<';
        char endMarker = '>';
        char rc;

        while (Serial.available() > 0 && newData == false) {
            rc = Serial.read();

            if (recvInProcess == true) {
                if (rc != endMarker) {
                    receivedChars[ndx] = rc;
                    ndx++;
                    if (ndx >= numChars) {
                        ndx = numChars - 1;
                    }
                }
                else {
                    receivedChars[ndx] = '\0';
                    recvInProcess = false;
                    ndx = 0;
                    newData = true;
                }
            }
            else if (rc == startMarker) {
                recvInProcess = true;
            }
        }
    }

    void processMessage() { //message has tags removed
        if (newData == true) {
            depackage();
            newData = false;
            timeout = 2000;

            digitalWrite(RED_LED, LED_STATE);
            receiveDelay = 5;
        }
    }

    void depackage(void) {
        DeserializationError err = deserializeJson(jsonPacket, receivedChars);

        switch (err.code()) {
        case DeserializationError::Ok:
            updateParameters();
            break;

        case DeserializationError::InvalidInput:
            Serial.print(F("Invalid input!"));
            break;

        case DeserializationError::NoMemory:
            Serial.print(F("Not enough memory"));
            break;

        default:
            Serial.print(F("Deserialization failed"));
            break;
        }
    }

    void updateParameters() {
        // Update motor speeds if present
        for (int i = 0; i < 4; i++) {
            String speedKey = "speed" + String(i);
            if (jsonPacket.containsKey(speedKey)) {
                motors->speeds[i] = jsonPacket[speedKey];
            }
        }

        if (jsonPacket.containsKey("action")) {
            String action  = jsonPacket["action"];

            if (action.equalsIgnoreCase("set_voltage")) {
                int channel = jsonPacket["channel"];
                float voltage = jsonPacket["voltage"];
                actuator->actuatorPositions[channel] = voltage;
                actuator->writeDAC(channel, voltage);

                StaticJsonDocument<128> response;
                response["status"] = "OK";
                response["channel"] = channel;
                response["voltage"] = voltage;
                serializeJson(response, Serial);
                Serial.println();
            }

            else if (action.equalsIgnoreCase("read_feedback")) {
                if (!jsonPacket.containsKey("channel")) {
                    Serial.println("Missing channel for read_feedback");
                    return;
                }
                int channel = jsonPacket["channel"];
                float feedback = actuator->readADC(channel);
                actuator->feedbackSignals[channel] = feedback;

                StaticJsonDocument<128> response;
                response["feedback"] = feedback;
                response["channel"] = channel;
                serializeJson(response, Serial);
                Serial.println();
            }

            else if (action.equalsIgnoreCase("reset_encoder")) {
                encoder->position = 0;

                StaticJsonDocument<64> response;
                response["status"] = "Encoder reset";
                serializeJson(response, Serial);
                Serial.println();
            }

            else if (action.equalsIgnoreCase("STOP")) {
                motors->STOP();
            }

            else if (action.equalsIgnoreCase("set_light")) {
                if (!jsonPacket.containsKey("state")) {
                    Serial.println("Missing state for set_light");
                    return;
                }

                String stateStr = jsonPacket["state"];
                AndonLight::States newState;

                if (stateStr.equalsIgnoreCase("GREEN")) newState = AndonLight::GREEN;
                else if (stateStr.equalsIgnoreCase("YELLOW")) newState = AndonLight::YELLOW;
                else if (stateStr.equalsIgnoreCase("BLUE")) newState = AndonLight::BLUE;
                else if (stateStr.equalsIgnoreCase("RED")) newState = AndonLight::RED;
                else if (stateStr.equalsIgnoreCase("OFF")) newState = AndonLight::OFF;
                else if (stateStr.equalsIgnoreCase("BLINK_GREEN")) newState = AndonLight::BLINK_GREEN;
                else if (stateStr.equalsIgnoreCase("BLINK_YELLOW")) newState = AndonLight::BLINK_YELLOW;
                else if (stateStr.equalsIgnoreCase("BLINK_BLUE")) newState = AndonLight::BLINK_BLUE;
                else if (stateStr.equalsIgnoreCase("BLINK_RED")) newState = AndonLight::BLINK_RED;
                else {
                    Serial.println("Invalid light state");
                    return;
                }

                andonLight->setState(newState);

                StaticJsonDocument<64> response;
                response["status"] = "light_updated";
                response["state"] = stateStr;
                serializeJson(response, Serial);
                Serial.println();
            }


        else if (action.equalsIgnoreCase("set_triggers")) {
            // Clear existing triggers if specified
            bool shouldClear = jsonPacket["clear"] | false;  // Defaults to false if "clear" is missing
            if (shouldClear) {
                triggerBuffer.clear();
            }

            JsonArray triggers = jsonPacket["triggers"].as<JsonArray>();
            for (JsonObject t : triggers) {
                if (t.containsKey("threshold") && t.containsKey("activate") &&
                    t.containsKey("deactivate") && t.containsKey("delay")) {

                    Trigger trig;
                    trig.threshold = t["threshold"];
                    trig.activate_channel = t["activate"];
                    trig.deactivate_channel = t["deactivate"];
                    trig.delay_seconds = t["delay"];
                    triggerBuffer.push_back(trig);
                } else {
                    Serial.println("Invalid trigger format");
                }
            }

            StaticJsonDocument<64> response;
            response["status"] = "triggers_loaded";
            response["count"] = triggerBuffer.size();
            serializeJson(response, Serial);
            Serial.println();
        }

        else if (action.equalsIgnoreCase("reset_triggers")) {
            for (auto& trig : triggerBuffer) {
                trig.triggered = false;
                trig.waitingToDeactivate = false;
                trig.triggerTime = 0;
            }

            StaticJsonDocument<64> response;
            response["status"] = "triggers_reset";
            serializeJson(response, Serial);
            Serial.println();
        }

        else if (action.equalsIgnoreCase("start_process")) {
            ultrasonicEnabled = true;
            ultrasonic.processSpeed = 0.0;
            ultrasonic.currentSpeed = 0.0;
            andonLight->setState(AndonLight::BLINK_GREEN);

            StaticJsonDocument<64> response;
            response["status"] = "process_started";
            serializeJson(response, Serial);
            Serial.println();
        }

        else if (action.equalsIgnoreCase("stop_process")) {
            ultrasonicEnabled = false;
            motors->STOP();
            andonLight->setState(AndonLight::GREEN);

            StaticJsonDocument<64> response;
            response["status"] = "process_stopped";
            serializeJson(response, Serial);
            Serial.println();
        }

        else if (action.equalsIgnoreCase("set_speed")) {
            float speed = jsonPacket["speed"];
            ultrasonic.processSpeed = speed;
            ultrasonic.currentSpeed = speed;

            StaticJsonDocument<64> response;
            response["status"] = "speed_set";
            response["speed"] = speed;
            serializeJson(response, Serial);
            Serial.println();
            }

        else if (action.equalsIgnoreCase("shutdown")) {
            motors->STOP();
            ultrasonicEnabled = false;
            andonLight->setState(AndonLight::GREEN);
            triggerBuffer.clear();
            serialStarted = false;  // Optional: block further commands until reinitialized
            StaticJsonDocument<64> response;
            response["status"] = "shutdown_complete";
            serializeJson(response, Serial);
            Serial.println();
            }
        }
    }

    void checkTriggers() {
        int currentPos = encoder->getPosition();
        unsigned long now = millis();

        /* [TRIGGER DEBUG]  
        Serial.print("[Trigger Debug] Current encoder position: ");
        Serial.println(currentPos);
        */

        for (auto& trig : triggerBuffer) {
            /* [TRIGGER DEBUG]
            Serial.print("[Trigger Debug] Checking trigger: threshold=");
            Serial.print(trig.threshold);
            Serial.print(" activate=");
            Serial.print(trig.activate_channel);
            Serial.print(" deactivate=");
            Serial.print(trig.deactivate_channel);
            Serial.print(" delay=");
            Serial.print(trig.delay_seconds);
            Serial.print(" triggered=");
            Serial.print(trig.triggered);
            Serial.print(" waitingToDeactivate=");
            Serial.println(trig.waitingToDeactivate);
            */

            // Trigger activation
            if (!trig.triggered && currentPos >= trig.threshold) {
                Serial.println("[Trigger Debug] Trigger condition met! Activating...");

                actuator->actuatorPositions[trig.activate_channel] = 3.0;
                trig.triggerTime = now;
                trig.waitingToDeactivate = true;
                trig.triggered = true;

                StaticJsonDocument<128> response;
                response["trigger_reached"] = true;
                response["channel"] = trig.activate_channel;
                response["value"] = trig.threshold;
                serializeJson(response, Serial);
                Serial.println();
            }

            // Trigger deactivation after delay
            if (trig.waitingToDeactivate && (now - trig.triggerTime >= (unsigned long)(trig.delay_seconds * 1000))) {
                Serial.println("[Trigger Debug] Trigger deactivation condition met! Deactivating...");

                actuator->actuatorPositions[trig.deactivate_channel] = 0.0;
                trig.waitingToDeactivate = false;

                StaticJsonDocument<128> response;
                response["trigger_deactivated"] = true;
                response["channel"] = trig.deactivate_channel;
                serializeJson(response, Serial);
                Serial.println();
            }
        }
    }




};

#endif