#include <stdio.h>
#include <stdint.h>
#include <iostream>
#include <cmath>


void erefs_to_hexdata(float erefs, uint8_t hexdata[4]);

float ms_to_erefs(float ms, float wheelDiameter);


int main() {

    float speed = 1;
    float erefs = 0;
    uint8_t hexdata[4];


    while(speed != 0){

        std::cout<<"ENTER a linear speed value: ";
        std::cin>>speed;

        erefs = ms_to_erefs(speed, 0.048);

        erefs_to_hexdata(erefs, hexdata);

        printf("%02X %02X %02X %02X\n", hexdata[0], hexdata[1], hexdata[2], hexdata[3]);


    }


    return 0;
}


void erefs_to_hexdata(float input_float, uint8_t hexdata[4]) {
        uint32_t initialInt = int(round(input_float*16*16*16*16));
        // Split the number into bytes
        hexdata[3] = (initialInt >> 24) & 0xFF; // Most significant byte
        hexdata[2] = (initialInt >> 16) & 0xFF;
        hexdata[1] = (initialInt >> 8) & 0xFF;
        hexdata[0] = initialInt & 0xFF;         // Least significant byte
}

float ms_to_erefs(float ms, float wheelDiameter) {
    // meters per second linear speed to erefs
    // wheel diameter in meters 0.048

    float angularVelocity = ms/wheelDiameter; // 0.1 m/s /m = 1/s
    float rpm = angularVelocity/0.10471975057; // convert from rads/s to rpm

    std::cout<<"EREFS: " << rpm * 18.7187185 << "\n";
    return rpm * 18.7187185; // this was pulled from the eletrocraft setup file
}

  
