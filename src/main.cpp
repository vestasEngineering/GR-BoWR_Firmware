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
#include "Config.hpp"

#include <Motors.hpp>
Motors motors;

#include <Ultrasonic.hpp>
Ultrasonic ultrasonic;
bool ultrasonicEnabled = false;

#include <UltrasonicServo.hpp>
UltrasonicServo ultrasonicServo;

#include <Actuator.hpp>
ActuatorControl actuator;

#include <AndonLight.hpp>
AndonLight andonLight;

#include <MySerial.hpp>
MySerial mySerial(actuator, andonLight, motors);

#include <BatteryMonitor.hpp>
BatteryMonitor batteryMonitor;

#include "JogControl.hpp"
JogControl jogControl(motors, mySerial);

#include "AndonManager.hpp"
AndonManager andonMgr(andonLight, mySerial, motors, actuator, jogControl, batteryMonitor, ultrasonic, ultrasonicServo);

#include "BootHealth.hpp"

#include "Portenta_H7_TimerInterrupt.h"
volatile int interruptCounter = 0;
void m7timer() { 
  // every 1/10,000 second - 10,000hz - 0.0001 second
  interruptCounter++;

  //if(mySerial.delay) mySerial.delay--;

  // every 10/10,000 second - 1,000hz - 0.001 second
  if ((interruptCounter % 10) == 0) { 
      if(motors.thisDelay) motors.thisDelay--;
      if(mySerial.thisDelay) mySerial.thisDelay--;
      if(mySerial.timeout) mySerial.timeout--;
      if (mySerial.receiveDelay) mySerial.receiveDelay--;

  }

  // every 100/10,000 second - 100hz - 0.01 second
  if ((interruptCounter % 100) == 0) { 
    if(ultrasonic.delay) ultrasonic.delay--;
    //if(encoder.thisDelay) encoder.thisDelay--;
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

  Serial.println("{\"status\":\"boot\",\"msg\":\"Serial Starting\"}");

  delay(200);
  M7Timer.attachInterruptInterval(100, m7timer);
  andonLight.setup();
  mySerial.setup();
  motors.setup();
  ultrasonic.setup();
  mySerial.attachAndonManager(andonMgr);
  mySerial.attachUltrasonic(ultrasonic, ultrasonicEnabled);
  mySerial.attachUltrasonicServo(ultrasonicServo);
  ultrasonic.attachMotors(motors);
  ultrasonicServo.setup();
  actuator.setup ();
  jogControl.setup();
  batteryMonitor.setup();
  andonMgr.setup();

  // --- Boot health check: probe subsystems and emit one JSON line to Raspberry Pi ---
  BootHealth::Report rep = BootHealth::run(
      andonLight,
      motors,
      actuator,
      ultrasonic,
      ultrasonicServo,
      /*battery=*/&batteryMonitor,
      /*can_timeout_ms=*/500
  );
  BootHealth::sendReport(rep);

  //If boot health fails, latch Andon to BLINK_RED (until manual override)
  //if (!rep.ok) andonMgr.setOverride(AndonLight::BLINK_RED);
}

void loop() {
  andonLight.loop();
  mySerial.stateMachine();
  motors.stateMachine();

  // Poll CAN for incoming messages
  while (CAN.available()) {
    CanMsg msg = CAN.read();
    motors.handleCANResponse(msg);
     //Serial.print("Received CAN ID: ");
    //Serial.println(msg.id, HEX);
    //Serial.print("Data: ");
    for (int i = 0; i < msg.data_length; i++) {
      //Serial.print(msg.data[i], HEX);
      //Serial.print(" ");
    }
    //Serial.println();
  }

  static unsigned long lastPrint = 0;
  if (millis() - lastPrint > 500) { // every 0.5s
      lastPrint = millis();
      //Serial.println("Encoder Positions (rev):");
      for (int i = 0; i < 4; i++) {
          //Serial.print("Axis ");
          //Serial.print(i + 1);
          //Serial.print(": ");
          //Serial.println(motors.positions[i], 4); // 4 decimal places
      }
      //Serial.println("----------------------");
  }

  if (ultrasonicEnabled) {
      ultrasonicServo.activate();  // Servo active when ultrasonic is enabled
      ultrasonic.stateMachine();
  } else {
      ultrasonicServo.deactivate(); // Servo inactive when ultrasonic is disabled
  }

  actuator.stateMachine();
  jogControl.update();
  batteryMonitor.stateMachine();
  andonMgr.tick();
}
