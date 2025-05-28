#include <stdio.h>
#include <stdint.h>
#include <iostream>

int main() {

    uint32_t number = 2000;

    while(number != 0){

        std::cout<<"ENTER A NUMBER: ";
        std::cin>>number;

        uint8_t bytes[4];

        // Split the number into bytes
        bytes[0] = (number >> 24) & 0xFF; // Most significant byte
        bytes[1] = (number >> 16) & 0xFF;
        bytes[2] = (number >> 8) & 0xFF;
        bytes[3] = number & 0xFF;         // Least significant byte

        // Print the bytes in the desired format
        printf("%02X %02X %02X %02X\n", bytes[0], bytes[1], bytes[3], bytes[2]);

    }


    return 0;
}
