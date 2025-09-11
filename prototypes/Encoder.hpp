#ifndef MY_ENCODER_CLASS
#define MY_ENCODER_CLASS

#include <Arduino.h>
#include <Arduino_CAN.h>

class Encoder {
public:
    enum State {
        IDLE,
        REQUEST_SENT,
        RESPONSE_RECIEVED,
        TIMEOUT_ERROR
    };

    Encoder(uint8_t node, uint16_t index, uint8_t subindex = 0)
      : nodeId(node),
        objIndex(index),
        subIndex(subindex),
        state(IDLE),
        lastRequestTime(0),
        timeoutInterval(1000)
    {}

    void stateMachine() {
        unsigned long currentTime = millis();
        switch (state) {
            case IDLE:
                if ((currentTime - lastRequestTime) >= requestInterval) {
                    if (requestEncoderData()) {
                        state = REQUEST_SENT;
                        lastRequestTime = currentTime;
                    }
                }
                break;
            case REQUEST_SENT:
                if ((currentTime - lastRequestTime) >= timeoutInterval) {
                    state = TIMEOUT_ERROR;
                    Serial.print("SDO request timeout on node ");
                    Serial.println(nodeId, DEC);
                }
                break;
            case RESPONSE_RECIEVED:
                Serial.print("Encoder [Node ]");
                Serial.print(nodeId, DEC);
                Serial.print("} Value: ");
                Serial.print(encoderValue);
                state = IDLE;
                lastRequestTime = currentTime;
                break;
            case TIMEOUT_ERROR:
                if ((currentTime - lastRequestTime) >= requestInterval) {
                    state = IDLE;
                }
                break;
            default:
                break;

        }
    }

    void dumpCanMsg(const CanMsg &msg) {
        Serial.print("CAN Msg: ID=0x");
        Serial.print(msg.id, HEX);
        Serial.print("Length=");
        Serial.print(msg.data_length);
        Serial.print("Data=[");
        for (int i = 0; i < msg.data_length; i++) {
            Serial.print("0x");
            Serial.print(msg.data[i], HEX);
            if (i < msg.data_length - 1) {
                Serial.print(", ");
            }
        }
        Serial.print("]");
    }

    bool requestEncoderData() {
        uint8_t sdoRequest[8] = {0};
        sdoRequest[0] = 0x40;
        sdoRequest[1] = objIndex & 0xFF;
        sdoRequest[2] = (objIndex >> 8) & 0xFF;
        sdoRequest[3] = subIndex;

        uint32_t txId = 0x600 + nodeId;
        CanMsg sdoMsg(CanExtendedId(txId), sizeof(sdoRequest), sdoRequest);
        int ret = CAN.write(sdoMsg);
        if (ret == 1) {
            Serial.print("Sent SDO request (node )");
            Serial.print(nodeId, DEC);
            Serial.print(", index 0x");
            Serial.print(objIndex, HEX);
            Serial.print(")");
            return true;
        } else {
            Serial.println("Error: SDO request not sent");
            return false;
        }
    }

    bool processCanMessage(const CanMsg &msg) {
        uint32_t expectedId = 0x580 + nodeId;
        if (msg.id == expectedId && msg.data_length >= 8) {
            if (msg.data[0] == 0x43) {
                encoderValue =
                    ((int32_t)msg.data[4]) |
                    (((int32_t)msg.data[4]) << 8) |
                    (((int32_t)msg.data[4]) << 16) |
                    (((int32_t)msg.data[4]) << 24);
                state = RESPONSE_RECIEVED;
                return true;
            } else {
                Serial.print("Unexpected SDO response from node ");
                Serial.println(nodeId, DEC);
            }
        }
        return false;
    }

    int32_t getEncoderValue() const {
        return encoderValue;
    }

    State getState() const {
        return state;
    }

private:
    uint8_t nodeId;
    uint16_t objIndex;
    uint8_t subIndex;
    int32_t encoderValue;

    State state;
    unsigned long lastRequestTime;
    const unsigned long requestInterval = 200;
    const unsigned long timeoutInterval;
};

#endif