//====================================================
// TITLE BLOCK
//====================================================
/*  
 *  Project: GR-LRR
 *  Author: Jacob Owens - Vestas Blades America
 *  Date: 07/23/2024
 * 
 *  Author: Giovanni Cordova - CREADIS
 *  Date: 03/24/2025
 */

#include <Arduino.h>
#include <mbed.h>
#include <math.h>


//#include <BlueLed.hpp>
//BlueLed blueLed;

#include <Motors.hpp>
Motors motors;

#include <Ultrasonic.hpp>
Ultrasonic ultrasonic;

//#include <Encoder.hpp>
//Encoder encoder(3, 0x6064);

#include <Ext_Encoder.hpp>
ExtEncoder encoder;

#include <Actuator.hpp>
ActuatorControl actuator;

#include <MySerial.hpp>
MySerial mySerial(actuator);

#include "Portenta_H7_TimerInterrupt.h"
volatile int interruptCounter = 0;
void m7timer() { 
  // every 1/10,000 second - 10,000hz - 0.0001 second
  interruptCounter++;

  if(encoder.thisDelay) encoder.thisDelay--;
  //if(mySerial.delay) mySerial.delay--;

  if(mySerial.receiveDelay) mySerial.receiveDelay--;

  // every 10/10,000 second - 1,000hz - 0.001 second
  if ((interruptCounter % 10) == 0) { 
     // this can indicate if something is taking way to long?

      if(motors.thisDelay) motors.thisDelay--;
      if(mySerial.thisDelay) mySerial.thisDelay--;
      if(mySerial.timeout) mySerial.timeout--;

  }

  // every 100/10,000 second - 100hz - 0.01 second
  if ((interruptCounter % 100) == 0) { 
    if(ultrasonic.delay) ultrasonic.delay--;
    if(encoder.thisDelay) encoder.thisDelay--;
  }

  // every 1,000/10,000 second - 10hz - 0.1 second
  if ((interruptCounter % 1000) == 0) { 
    //if (redLedDelay) redLedDelay--;
  }

  // every 10,000/10,000 second - 1hz
  if ((interruptCounter % 10000) == 0) {
    //if (blueLed.delay) blueLed.delay--;
    interruptCounter = 0;
  }

}
Portenta_H7_Timer M7Timer(TIM7);


void setup(void);
void loop(void);


void setup() {
  Serial.begin(115200);
  while (!Serial) {
    delay(10);
  }
  Serial.println("Serial Starting");

  delay(2000);
  M7Timer.attachInterruptInterval(100, m7timer);
  mySerial.setup();
  motors.setup();
  ultrasonic.setup();
  encoder.setup();
  actuator.setup ();
}

void loop() {
  //blueLed.stateMachine();
  mySerial.stateMachine();
  motors.stateMachine();
  ultrasonic.stateMachine();
  actuator.stateMachine();
  encoder.stateMachine();
  //Serial.print("Current Encoder Position BEFORE JSON: ");
  //Serial.println(encoder.getPosition());  
  //encoder.debugRawInputs();
}
