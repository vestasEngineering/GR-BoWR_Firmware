
#include <stdio.h>
#include <stdint.h>
#include <iostream>
#include <cmath>

using namespace std;
int main() {

    uint32_t const M1_EREFS_ID = 0x048020A8;
    uint32_t const M2_EREFS_ID = 0x048040A8;
    uint32_t const M3_EREFS_ID = 0x048060A8;
    uint32_t const M4_EREFS_ID = 0x048080A8;
    uint32_t const MOTOR_EREFS_IDS [4] = {M1_EREFS_ID, M2_EREFS_ID, M3_EREFS_ID, M4_EREFS_ID};

    std::cout<<MOTOR_EREFS_IDS[3];

    return 0;
}

