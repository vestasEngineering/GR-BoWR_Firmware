/**************************************************************************************
 * INCLUDE
 **************************************************************************************/

#include <Arduino_CAN.h>

/**************************************************************************************
 * CONSTANTS
 **************************************************************************************/

//uint32_t const CAN_ID = 00011 11000000 10000001 00000001;
//uint32_t const CAN_ID = 0x3C08101;

//uint32_t const CAN_ID = 0x3300101;
//uint32_t const CAN_ID = 0x330180;
//uint32_t const CAN_ID =  0x80813003; // not sure, example in google sheets

//uint32_t const CAN_ID =  0x03308180; // not sure, example in google sheets

//uint32_t const CAN_ID = 0x0a61bc95; // enable drive command

//uint32_t const CAN_ID = 0x0a61c37e;



uint32_t const RESET_CAN_ID = 0x00808002;
uint32_t const EREFS_10_ID = 0x048080A8;

uint32_t const EREFS_20_ID3 = 0x048060A8;


uint32_t const EREFS_0_ID = 0x048080A8;

uint32_t const EREFS_0_ID3 = 0x048060A8;

uint32_t const ENABLE_CAN_ID = 0x03000401;

uint32_t const GET_DRIVE_STATUS_ID = 0x16008004;

uint32_t const MER_ID = 0x16008004;   

uint32_t const DER_ID = 0x16008004;  

uint32_t const qEREFS_ID = 0x16008005;  

uint32_t const SRL_ID = 0x16008004;


uint32_t const SRH_ID = 0x16008004; 





//16008005 11 00 0E 09
//1600A005 11 00 0E 09 
/**************************************************************************************
 * SETUP/LOOP
 **************************************************************************************/
#define BLUE_LED    LEDB
bool ledState = LOW;
int rc;
void setup()
{
  Serial.begin(115200);

  if (!CAN.begin(CanBitRate::BR_250k))
  {
    Serial.println("CAN.begin(...) failed.");
    for (;;) {}
  }


  



}

static uint32_t msg_cnt = 0;

void loop()
{

  uint8_t const RESET_DATA[] = {0x11, 0x00, 0xAD, 0x03};
  uint8_t const MER_DATA[] = {0x11, 0x00, 0xFC, 0x08};

  uint8_t const DER_DATA[] = {0x11, 0x00, 0xAD, 0x03};
  
  uint8_t const EREFS_10_DATA[] = {0xA6, 0x65, 0x74, 0x01};
  uint8_t const EREFS_20_DATA3[] = {0x00, 0x00, 0x14, 0x00};

  uint8_t const EREFS_0_DATA[] = {0x00, 0x00, 0x00, 0x00};
  uint8_t const qEREFS_DATA[] = {0x10, 0x00, 0xA8, 0x02};
  uint8_t const SRL_DATA[] = {0x11, 0x00, 0x0E, 0x09};
  uint8_t const SRH_DATA[] = {0x11, 0x00, 0x0F, 0x09};



  //memcpy((void *)(msg_data + 5), &msg_cnt, sizeof(msg_cnt));
  CanMsg RESET(CanExtendedId(RESET_CAN_ID), sizeof(RESET_DATA), RESET_DATA);
  CanMsg qEREFS(CanExtendedId(qEREFS_ID), sizeof(qEREFS_DATA), qEREFS_DATA);

  CanMsg EREFS_10(CanExtendedId(EREFS_10_ID), sizeof(EREFS_10_DATA), EREFS_10_DATA);
  CanMsg EREFS_20(CanExtendedId(EREFS_20_ID3), sizeof(EREFS_20_DATA3), EREFS_20_DATA3);


  CanMsg EREFS_0_ID4(CanExtendedId(EREFS_0_ID), sizeof(EREFS_0_DATA), EREFS_0_DATA);
  CanMsg AXIS3_EREFS_0(CanExtendedId(EREFS_0_ID3), sizeof(EREFS_0_DATA), EREFS_0_DATA);



  CanMsg MER(CanExtendedId(MER_ID), sizeof(MER_DATA), MER_DATA);
  CanMsg DER(CanExtendedId(DER_ID), sizeof(DER_DATA), DER_DATA);

  CanMsg SRL(CanExtendedId(SRL_ID), sizeof(SRL_DATA), SRL_DATA);

  CanMsg SRH(CanExtendedId(SRH_ID), sizeof(SRH_DATA), SRH_DATA);




  Serial.println("===================================================");
  Serial.println("===================================================");
  Serial.println("===================================================");
  Serial.println("===================================================");


  /* Transmit the CAN message, capture and display an
   * error core in case of failure.
   */
  Serial.println("MESSAGE: EREFS = 10");
  Serial.println(EREFS_10);
  rc = CAN.write(EREFS_10);
  rc = CAN.write(EREFS_20);

  Serial.println("CAN WRITE RESULT: ");
  Serial.println(rc);
  if (CAN.available()){
  // should be no response from reset
  Serial.println("RECEIVED MESSAGE");
  CanMsg new_msg = CAN.read();
  Serial.println(new_msg);
  }
  delay(10000);

    Serial.println("===================================================");
  Serial.println("===================================================");
  Serial.println("===================================================");
  Serial.println("===================================================");


  /* Transmit the CAN message, capture and display an
   * error core in case of failure.
   */
  Serial.println("MESSAGE: EREFS = 0");
  Serial.println(EREFS_0_ID4);
  rc = CAN.write(EREFS_0_ID4);
  rc = CAN.write(AXIS3_EREFS_0);

  Serial.println("CAN WRITE RESULT: ");
  Serial.println(rc);
  if (CAN.available()){
  // should be no response from reset
  Serial.println("RECEIVED MESSAGE");
  CanMsg new_msg = CAN.read();
  Serial.println(new_msg);
  }
  delay(5000);

    Serial.println("===================================================");
  Serial.println("===================================================");
  Serial.println("===================================================");
  Serial.println("===================================================");



  digitalWrite(BLUE_LED, ledState );
  ledState = !ledState;
  msg_cnt++;

}



// EREFS = 10 in mpl from output:
// 08 00 40 24 A8 00 00 00 0A 1E
//       04 80 80 A8 00 00 0A 00
// EREFS = 10 from binary viewer