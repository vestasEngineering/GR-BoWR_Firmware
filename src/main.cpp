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

#include <Motors.hpp>
Motors motors;

#include <Ultrasonic.hpp>
Ultrasonic ultrasonic;
bool ultrasonicEnabled = false;

#include <Ext_Encoder.hpp>
ExtEncoder encoder;

#include <Actuator.hpp>
ActuatorControl actuator;

#include <AndonLight.hpp>
AndonLight andonLight;

#include <MySerial.hpp>
MySerial mySerial(actuator, andonLight, motors, encoder);

#include <BatteryMonitor.hpp>
BatteryMonitor batteryMonitor;

#include "JogControl.hpp"
JogControl jogControl(motors, mySerial);

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

  delay(200);
  M7Timer.attachInterruptInterval(100, m7timer);
  andonLight.setup();
  mySerial.setup();
  motors.setup();
  ultrasonic.setup();
  ultrasonic.attachMotors(motors);
  encoder.setup();
  actuator.setup ();
  jogControl.setup();
  //batteryMonitor.setup();
}

void loop() {
  andonLight.loop();
  mySerial.stateMachine();
  motors.stateMachine();
  
  if (ultrasonicEnabled) {
    ultrasonic.stateMachine();  // PID loop only runs when enabled
  }
  
  actuator.stateMachine();
  encoder.stateMachine();
  jogControl.update();
  //batteryMonitor.stateMachine();
}
