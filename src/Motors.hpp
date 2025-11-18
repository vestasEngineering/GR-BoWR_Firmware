#ifndef MY_MOTOR_CLASS
#define MY_MOTOR_CLASS

#include <Arduino.h>
#include <Arduino_CAN.h>

class Motors {
public:
    volatile int thisDelay = 0;
    int index = 0;
    float erefs = 0.0;
    const float wheelDiameter = 0.048;
    const int ENCODER_CPR = 4096; // Adjust based on your encoder resolution

    uint8_t EREFS_HEXDATA[4];

    uint32_t const M1_EREFS_ID = 0x048020A8;
    uint32_t const M2_EREFS_ID = 0x048040A8;
    uint32_t const M3_EREFS_ID = 0x048060A8;
    uint32_t const M4_EREFS_ID = 0x048080A8;
    uint32_t const MD_EREFS_ID = 0x049FE0A8; // default ID

    uint32_t const MOTOR_EREFS_IDS[4] = {M1_EREFS_ID, M2_EREFS_ID, M3_EREFS_ID, M4_EREFS_ID};

    // APOS query CAN IDs
    const uint32_t MOTOR_APOS_IDS[4] = {
        0x16002005, 0x16004005, 0x16006005, 0x16008005
    };

    float speeds[4] = {0.0, 0.0, 0.0, 0.0};
    float positions[4] = {0.0, 0.0, 0.0, 0.0}; // Encoder positions in revolutions
    int receipts[4] = {0, 0, 0, 0};

    enum States { WAITING, WRITING };
    States state = WAITING;

    void setup() {
        if (!CAN.begin(CanBitRate::BR_250k)) {
            Serial.println("CAN.begin(...) failed.");
            while (true) {
                Serial.println("CAN ISSUE");
                delay(1000);
            }
        }

        for (int i = 0; i < 4; i++) {
            resetAPOS(i);
        }
    }

    void erefs_to_hexdata(float input_float, uint8_t hexdata[4])
    {
        uint32_t initialInt = int(round(input_float * 16 * 16 * 16 * 16));
        // Split the number into bytes
        hexdata[3] = (initialInt >> 24) & 0xFF; // Most significant byte
        hexdata[2] = (initialInt >> 16) & 0xFF;
        hexdata[1] = (initialInt >> 8) & 0xFF;
        hexdata[0] = initialInt & 0xFF; // Least significant byte
    }

    float ms_to_erefs(float ms, float wheelDiameter)
    {
        // meters per second linear speed to erefs
        // wheel diameter in meters 0.048
        float angularVelocity = ms / wheelDiameter;  // 0.1 m/s /m = 1/s
        float rpm = angularVelocity / 0.10471975057; // convert from rads/s to rpm
        // std::cout<<"EREFS: " << rpm * 18.7187185 << "\n";
        return rpm * 18.7187185; // this was pulled from the eletrocraft setup file
    }

    // Stop all motors
    void STOP() {
        for (int i = 0; i < 4; i++) {
            speeds[i] = 0.0;
            float erefs = ms_to_erefs(0.0, wheelDiameter);
            uint8_t hexdata[4];
            erefs_to_hexdata(erefs, hexdata);
            CanMsg stopCmd(CanExtendedId(MOTOR_EREFS_IDS[i]), sizeof(hexdata), hexdata);
            CAN.write(stopCmd);
        }
        Serial.println("[Motors] STOP command sent to all axes.");
    }


    // Send APOS query for motors 3 & 4 and return their average position
    float requestAPOS() {
        // Send APOS query only to motors 3 & 4 (axes 2 and 3)
        for (uint8_t axis = 2; axis <= 3; axis++) {
            uint8_t aposCmd[4] = {0x11, 0x00, 0x28, 0x02}; // MPL query for APOS
            CanMsg query(CanExtendedId(MOTOR_APOS_IDS[axis]), sizeof(aposCmd), aposCmd);
            CAN.write(query);
        }

        // Return the average of APOS readings for motors 3 & 4
        return (positions[2] + positions[3]) * 0.5f; // in revolutions
    }


    void resetAPOS(uint8_t axis) {
        if (axis >= 4) {
            Serial.println("[RESET] Invalid axis index!");
            return;
        }

        // CAN IDs for APOS reset
        const uint32_t RESET_APOS_IDS[4] = {
            0x00802002, 0x00804002, 0x00806002, 0x00808002
        };

        // Empty payload (or zeros if required)
        uint8_t resetData[4] = {0x00, 0x00, 0x00, 0x00};

        // Create CAN message with the reset ID
        CanMsg resetCmd(CanExtendedId(RESET_APOS_IDS[axis]), sizeof(resetData), resetData);

        if (CAN.write(resetCmd)) {
            Serial.print("[RESET] Encoder reset sent to axis ");
            Serial.print(axis + 1);
            Serial.print(" (ID: 0x");
            Serial.print(RESET_APOS_IDS[axis], HEX);
            Serial.println(")");
        } else {
            Serial.print("[RESET] Failed to send encoder reset to axis ");
            Serial.println(axis + 1);
        }
    }

    // Handle CAN responses (APOS)
    void handleCANResponse(const CanMsg &msg) {
        // Detect APOS response by MPL signature (bytes 2 and 3 = 0x28 0x02)
        if (msg.data_length >= 8 && msg.data[2] == 0x28 && msg.data[3] == 0x02) {
            uint8_t axisIndex = (msg.data[0] / 0x10) - 1; // 0x10, 0x20, 0x30, 0x40
            if (axisIndex < 4) {
                int32_t rawPos = (int32_t)(
                    ((uint32_t)msg.data[4]) |
                    ((uint32_t)msg.data[5] << 8) |
                    ((uint32_t)msg.data[6] << 16) |
                    ((uint32_t)msg.data[7] << 24)
                );
                positions[axisIndex] = (float)rawPos / ENCODER_CPR;

                //Serial.print("[APOS] Axis ");
                //Serial.print(axisIndex + 1);
                //Serial.print(" position updated: raw=");
                //Serial.print(rawPos);
                //Serial.print(", rev=");
                //Serial.println(positions[axisIndex], 4);
            } else {
                //Serial.println("[APOS] Invalid axis index in response!");
            }
        } else {
            //Serial.print("[CAN] Unexpected message ID: 0x");
            //Serial.println(msg.id, HEX);
        }

        // Debug: print full CAN frame
        //Serial.print("[CAN] Data: ");
        //for (int i = 0; i < msg.data_length; i++) {
            //Serial.print(msg.data[i], HEX);
            //Serial.print(" ");
        //}
        //Serial.println();
    }

    // State machine for motor control and APOS requests
    void stateMachine() {
        switch (state) {
            case WAITING:
                if (!thisDelay) {
                    thisDelay = 10;
                    state = WRITING;
                }
                break;

            case WRITING:
                if (!thisDelay)
                {

                    thisDelay = 10;
                    erefs = ms_to_erefs(speeds[index], wheelDiameter);                                                   // convert to erefs
                    erefs_to_hexdata(erefs, EREFS_HEXDATA);                                                              // convert to hexdata
                    CanMsg MOTOR_SET_EREFS(CanExtendedId(MOTOR_EREFS_IDS[index]), sizeof(EREFS_HEXDATA), EREFS_HEXDATA); // to can message
                    //Serial.println(MOTOR_SET_EREFS);
                    receipts[index] = CAN.write(MOTOR_SET_EREFS);
                    requestAPOS();
                    //Serial.println(requestAPOS());
                    index++; // 0 , 1 , 2 , 3

                    if (index == 4)
                    {
                        index = 0;
                        thisDelay = 10;
                        state = WAITING;
                    }
                }
                break;
        }
    }
};

#endif
