
#ifndef MY_SERIAL_CLASS
#define MY_SERIAL_CLASS

#include <ArduinoJson.h>
#include <Actuator.hpp>
#include "Ext_Encoder.hpp"

#define RED_LED LEDR

class MySerial
{
public:
    StaticJsonDocument<64> jsonPacket;
    boolean serialStarted = false;
    boolean serialEnded = false;
    bool LED_STATE = false;
    static const byte numChars = 64;
    char receivedChars[numChars];
    boolean newData = false;
    volatile int thisDelay = 0;
    volatile int timeout = 0;
    volatile int receiveDelay = 0;

    ActuatorControl* actuator;
    ExtEncoder encoder;

    MySerial(ActuatorControl& actuatorRef) {actuator = &actuatorRef; }

    enum States
    {
        CONNECTED,
        DISCONNECTED,
        WAITING,
    };

    States state;

    void stateMachine(void)
    {
        receiveLinux(); // will tell us if the serial has ended or not

        switch (state)
        {

        case CONNECTED:
            // while we are connected, if the timeout is zero, then we have lost connection
            if (!thisDelay && !timeout)
            {
                thisDelay = 500;

                state = DISCONNECTED;

                motors.STOP();
            }
            if (!thisDelay)
            {
                thisDelay = 500;

                Serial.println("connected");
            }
            break;

        case DISCONNECTED:
            // while we are disconnected, look for messages
            if (!thisDelay && timeout > 0)
            {
                thisDelay = 500;

                state = CONNECTED;
            }
            if (!thisDelay)
            {
                thisDelay = 500;

                Serial.println("disconnected");
            }
            break;

        default:
            break;
        }
    }

    void setup(void)
    {
        state = DISCONNECTED;
        Serial.begin(115200);
        Serial.println("Serial Starting");
    }

    void receiveLinux(void)
    {
        if (!receiveDelay)
        {
            digitalWrite(RED_LED, HIGH);
        }

        recvWithStartEndMarkers();

        processMessage();
    }

    void recvWithStartEndMarkers(void)
    {

        static boolean recvInProcess = false;
        static byte ndx = 0;
        char startMarker = '<';
        char endMarker = '>';
        char rc;

        while (Serial.available() > 0 && newData == false)
        {

            rc = Serial.read();

            if (recvInProcess == true)
            {
                if (rc != endMarker)
                {
                    receivedChars[ndx] = rc;
                    ndx++;
                    if (ndx >= numChars)
                    {
                        ndx = numChars - 1;
                    }
                }
                else
                {
                    receivedChars[ndx] = '\0';
                    recvInProcess = false;
                    ndx = 0;
                    newData = true;
                }
            }
            else if (rc == startMarker)
            {
                recvInProcess = true;
            }
        }
    }

    void processMessage(void) // message has tags removed
    {
        if (newData == true)
        {
            depackage();
            newData = false;
            timeout = 2000;

            digitalWrite(RED_LED, LED_STATE);
            receiveDelay = 5;
        }
    }

    void depackage(void)
    {

        DeserializationError err = deserializeJson(jsonPacket, receivedChars);

        switch (err.code())
        {
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

    void updateParameters(void)
    {
        // Debug: Print recieved JSON
        //Serial.print("Recieved JSON: ");
        //serializeJson(jsonPacket, Serial);
        //Serial.println();

        //Handle actuator commands first
        if (jsonPacket.containsKey("action")) {
            String action = jsonPacket["action"];
            int channel = jsonPacket["channel"];

            if (action.equalsIgnoreCase("set_voltage")) {
                float voltage = jsonPacket["voltage"];
                actuator ->actuatorPositions[channel] = voltage;
                actuator->writeDAC(channel,voltage);

            //Debug print to confirm voltage was sent
                //Serial.print("Actuator ");
                //Serial.print(channel);
                //Serial.print(" Set to voltage: ");
                //Serial.print(voltage);

                //Send confirmation response
                StaticJsonDocument<128> response;
                response["status"] = "OK";
                response["channel"] = channel;
                response["voltage"] = voltage;
                serializeJson(response, Serial);
                Serial.println();
            }
            else if (action.equalsIgnoreCase("read_feedback")) {
                float feedback = actuator->readADC(channel);
                actuator->feedbackSignals[channel] = feedback;
                
            //Debug print for actuator feedback
                //Serial.print("Actuator ");
                //Serial.print(channel);
                //Serial.print(" feedback: ");
                //Serial.print(feedback);

                //Send feedback response
                StaticJsonDocument<128> response;
                response["feedback"] = feedback;
                response["channel"] = channel;
                serializeJson(response, Serial);
                Serial.println();
            }
            
            else if (action.equalsIgnoreCase("reset_encoder")) {
                //Reset encoder position to 0.
                encoder.position = 0;
                
                StaticJsonDocument<64> response;
                response["status"] = "Encoder reset";
                serializeJson(response, Serial);
                Serial.println();
            }

            else if (action.equalsIgnoreCase("STOP")) {
                motors.STOP();
            }
            
        }
        

        // Update motor speeds if present
        for (int i = 0; i < 4; i++) {
            String speedKey = "speed" + String(i);
            if (jsonPacket.containsKey(speedKey)) {
                motors.speeds[i] = jsonPacket[speedKey];
            }
        }

        if (jsonPacket.containsKey("reset_encoder")) {
            noInterrupts();
            encoder.updateIndex();
            interrupts();
        }

    }
};

#endif