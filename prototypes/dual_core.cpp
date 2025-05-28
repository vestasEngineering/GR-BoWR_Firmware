#include <Arduino.h>
// Original by Khoi Hoang (Github @khoih-prog ) and Jeremy Ellis (Github @hpssjellis )
// Note: each core can be separated to it's own IDE folder

#define STARTING_M4_VALUE         0
#define STARTING_M7_VALUE         0

#include "mbed.h"
#include <vector>

using namespace mbed;
using namespace rtos;

#define USING_SRAM4               true

#define SRAM3_START_ADDRESS       ((uint32_t) 0x30040000)
#define SRAM4_START_ADDRESS       ((uint32_t) 0x38000000)

#if USING_SRAM4
  // Using AHB SRAM4 at 0x38000000
  #define SRAM_START_ADDRESS        SRAM4_START_ADDRESS

  //#define ARRAY_SIZE                16000
  const uint16_t ARRAY_SIZE = 16000;
  
  // Max 64K - 8 bytes (1 * uint32_t + 2 * uint8_t)
  #if ( ARRAY_SIZE > (65536 - 8) / 4 )
    #error ARRAY_SIZE must be < 16382
  #endif
#else
  // Using AHB SRAM3 at 0x30040000
  #define SRAM_START_ADDRESS        SRAM3_START_ADDRESS

  //#define ARRAY_SIZE                8000
  const uint16_t ARRAY_SIZE = 8000;        
  
  // Max 32K - 8 bytes (1 * uint32_t + 2 * uint8_t)
  #if ( ARRAY_SIZE > (32768 - 8) / 4 )
    #error ARRAY_SIZE must be < 8190
  #endif
#endif

struct shared_data
{
  uint32_t M7toM4;
  uint16_t M4toM7_Index;
  uint16_t dummy;
  uint32_t M4toM7[ARRAY_SIZE];
};

int LED_Blink = 0;
////////////////////////////////////////////////////////////////////


#ifdef CORE_CM7    // Start M7 Core programming

uint32_t localm7m4      = STARTING_M7_VALUE;

uint32_t myStoreFromM4  = STARTING_M4_VALUE;

// Current 1 sec
#define M7_DELAY_US     1000UL //20230223 changed to 25ms was --> 30000UL     // microSeconds not milliSeconds (30 ms)

unsigned long delayStart = 0;

static struct shared_data * const xfr_ptr = (struct shared_data *) SRAM_START_ADDRESS;

Thread M7Thread;

////////////////////////////////////////////

void MPU_Config()
{

  /*
  MPU - The MPU is an optional component for the memory protection. Including the MPU
        in the STM32 microcontrollers (MCUs) makes them more robust and reliable.
        The MPU must be programmed and enabled before using it. If the MPU is not enabled,
        there is no change in the memory system behavior.

  HAL - The STM32 Hardware Abstraction Layer (HAL) provides a simple, generic multi-instance
        set of APIs (application programming interfaces) to interact with the upper layers
        like the user application, libraries and stacks.

  */
  MPU_Region_InitTypeDef MPU_InitStruct;

  /* Disable the MPU */
  HAL_MPU_Disable();

  /////////////
  
  /* Configure the MPU attributes as WT for SDRAM */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;

  // Base address of the region to protect
#if USING_SRAM4
  MPU_InitStruct.BaseAddress = SRAM4_START_ADDRESS;             // For SRAM4 only
  // Size of the region to protect, 64K for SRAM4
  MPU_InitStruct.Size = MPU_REGION_SIZE_64KB;                   // Important to access more memory
#else
  MPU_InitStruct.BaseAddress = SRAM3_START_ADDRESS;             // For SRAM3 only
  // Size of the region to protect, only 32K for SRAM3
  MPU_InitStruct.Size = MPU_REGION_SIZE_32KB;                   // Important to access more memory
#endif

  // Region access permission type
  MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;

  // Shareability status of the protected region
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;

  /////////////

  // Optional

  // Bufferable status of the protected region
  //MPU_InitStruct.IsBufferable = MPU_ACCESS_BUFFERABLE;
  // Cacheable status of the protected region
  //MPU_InitStruct.IsCacheable = MPU_ACCESS_CACHEABLE;
  // Number of the region to protect
  //MPU_InitStruct.Number = MPU_REGION_NUMBER7;
  // TEX field level
  //MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  // number of the subregion protection to disable
  //MPU_InitStruct.SubRegionDisable = 0x00;
  // instruction access status
  //MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_ENABLE;

  /////////////

  HAL_MPU_ConfigRegion(&MPU_InitStruct);

  /* Enable the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);
}

////////////////////////////////////////////

void M7ThreadFunc()
{
  static uint16_t localIndex;
  
  localm7m4 = STARTING_M7_VALUE;

  while (true)
  {
    if ( (micros() - delayStart ) >= M7_DELAY_US ) //every 5ms
    {
      delayStart = micros(); //  reset the delay time
      
      yield();
      
      localm7m4++;
      xfr_ptr->M7toM4 = localm7m4;

      //--20230223 out Serial.print("M7 to M4: "); Serial.println(xfr_ptr->M7toM4);
      //--20230223 out localIndex = xfr_ptr->M4toM7_Index;
      //--20230223 out Serial.print("M4 to M7: Index = "); Serial.print(localIndex); 
      //--20230223 out Serial.print(", value = "); Serial.println(xfr_ptr->M4toM7[localIndex]);

      // test if M7 detects struct changed on M4 (xfr_ptr->M4toM7_Index)
      if (myStoreFromM4 !=  xfr_ptr->M4toM7_Index)
      {   
        myStoreFromM4 = xfr_ptr->M4toM7_Index;
        if (LED_Blink++ >= 4) { // 7
          LED_Blink = 0;
          digitalWrite(LEDR, !digitalRead(LEDR)); // flip red LED on and off
        }
      }
    }
  }
}

////////////////////////////////////////////

void setup()
{
  MPU_Config();
  bootM4();

  pinMode(LEDR, OUTPUT);  // portenta red LED

  Serial.begin(2000000);

  //while (!Serial);

  // Following issue solved by starting RPC. Not sure why.
  // Following line is needed, not sure why?
  //Serial.println(SRAM_START_ADDRESS, HEX); // MUST HAVE

  M7Thread.start(callback(M7ThreadFunc));
}

////////////////////////////////////////////

void loop()
{
  // yield();
}

#endif

////////////////////////////////////////////////////////////////////

#ifdef CORE_CM4    // Start M4 Core programming

uint32_t localm4m7 = STARTING_M4_VALUE;

static struct shared_data * const xfr_ptr = (struct shared_data *) SRAM_START_ADDRESS;

uint32_t myStoreFromM7 = STARTING_M7_VALUE;


Thread M4Thread;

////////////////////////////////////////////

void M4ThreadFunc()
{
  localm4m7 = STARTING_M4_VALUE;

  xfr_ptr->M4toM7[0] = 0xDEADBEEF;
  
  while (true)
  {
    // without this => not increase ???
    //delayMicroseconds(0); //--20230223 <--- I removed. Not needed.

    // To verify if SRAM change if OK by checking index = array[index]
    //xfr_ptr->M4toM7[xfr_ptr->M4toM7_Index] = xfr_ptr->M4toM7_Index;
    //xfr_ptr->M4toM7[xfr_ptr->M4toM7_Index] = localm4m7++;
    xfr_ptr->M4toM7[xfr_ptr->M4toM7_Index] = analogRead(A0);

    xfr_ptr->M4toM7_Index = (xfr_ptr->M4toM7_Index + 1) % ARRAY_SIZE;

    // test if M4 detects struct changed on M7 (xfr_ptr->M7toM4)
    if (myStoreFromM7 !=  xfr_ptr->M7toM4)
    {   
      myStoreFromM7 = xfr_ptr->M7toM4;
      if (LED_Blink++ >= 100) { //10
        LED_Blink = 0;
        digitalWrite(LEDB, !digitalRead(LEDB)); // flip blue LED on and off
      }
    }
 
  }
}

void setup()
{
  pinMode(LEDB, OUTPUT);  // portenta blue LED
  
  memset( xfr_ptr, 0, sizeof(*xfr_ptr) );

  xfr_ptr->M4toM7_Index = 0;
  
  M4Thread.start(callback(M4ThreadFunc));
}

////////////////////////////////////////////

void loop()
{
}

#endif



//====================================================
// program m7 core
//====================================================
#ifdef CORE_CM7
//====================================================
// program m7 core
//====================================================