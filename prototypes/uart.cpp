/* Portenta Serial UART

* Code modified from:
Thank you, Jeremy Ellis
https://github.com/hpssjellis/portenta-pro-community-solutions/blob/main/examples/dot1-portentaH7-examples/dot19-cores-uart-serial-com/dot198-uart-serialtransfer/dot198-uart-serialtransfer.ino

* Code modified from:
Thank you, Jeremy Ellis
//This, go there... ---> https://github.com/hpssjellis/portenta-pro-community-solutions

* ================================================================================
* 
* * UART Information for the PortentaH7 
* 
* UART myUART0(PA_0,  PI_9,  NC, NC);  // TX, RX, RTS, CTC  NOTE: NC means not connected
* UART myUART1(PA_9,  PA_10, NC, NC);   // pin D14 TX, pin D13 RX same as pre-defined Serial1
* UART myUART2(PG_14, PG_9,  NC, NC);
* UART myUART3(PJ_8,  PJ_9,  NC, NC);
* UART myUART6(PC_6,  PC_7, NC, NC);    // pin D5 TX, pin D4 RX
* UART myUART8(NC,    PA_8, NC, NC);    // pin D6 RX only can receive
* 
*/

//#include <Arduino.h>
//#include <Arduino_PortentaBreakout.h>
//#include "SerialTransfer.h"
#include <Arduino.h>
//////////////////// Start All Core M7 Programing /////////////////////
#ifdef CORE_CM7 


//--------------------------------------------- NOTE ------------------------------------------------------
// I used the Portenta Breakout Board and soldered some jumpers on the four UART breakouts. 
// Everything worked.
//
// UART Test1 is a bidirectional test of UART0 and UART1
// UART Test2 is a bidirectional test of UART2 and UART3


//--assign Serial UART
//https://github.com/hpssjellis/portenta-pro-community-solutions <--- Most excellent URL
//Thank you, Jeremy Ellis.
UART myUART0(PA_0,  PI_9,  NC, NC); // put this back to test with with myUART1 on M4 Test1 - worked 20230218  
//UART myUART2(PG_14, PG_9,  NC, NC); // put this back to test with with myUART3 on M4 Test2 - worked 20230218  

int i=65; //(char)"A";

void setup() { 
  pinMode(LEDR, OUTPUT);   // LEDB = blue, LEDG or LED_BUILTIN = green, LEDR = red 


   //myUART0.begin(1000000);   
   //myUART0.setTimeout(10);
   myUART0.begin(115200);   
   myUART0.setTimeout(10);
}

void loop() {

  if (myUART0.available()) {
    digitalWrite(LEDR, !digitalRead(LEDR));     // switch on / off
    do { // Read all to clear buffer
      Serial.println(myUART0.read());    // Read it and send it out Serial (USB)
    } while (myUART0.available());
  }
  delay(500); 
}

#endif

