//====================================================
// TITLE BLOCK
//====================================================
/*  
 *  Project: GR-ToWR Firmware
 *  Description: Firmware for the Glue Robot - Top of Web Rover (GR-ToWR), running on an Arduino Portenta H7. 
 *  Author: Giovanni Cordova - CREADIS
 *  Date: 03/24/2025
 */

#include <Arduino.h>
#include <mbed.h>
#include <math.h>
#include "Config.hpp"

UART myUART0(PA_0, PI_9, NC, NC);   // TX, RX, RTS, CTS
UART roboclaw_uart_a(PA_9, PA_10, NC, NC);
UART roboclaw_uart_b(PJ_8, PJ_9, NC, NC);

#include "EStop.hpp"
EStop estop;

#include <Motors.hpp>
Motors motors(&roboclaw_uart_a, &roboclaw_uart_b);

#include <Ultrasonic.hpp>
Ultrasonic ultrasonic;
bool ultrasonicEnabled = false;
  
#include <UltrasonicServo.hpp>
UltrasonicServo ultrasonicServo;

#include <Actuator.hpp>
ActuatorControl actuator(myUART0);

#include <AndonLight.hpp>
AndonLight andonLight(myUART0);

#include "ClampSensor.hpp"
ClampSensor clamp(CFG.clamp);

#include <MySerial.hpp>
MySerial mySerial(myUART0, actuator, andonLight, motors);

#include <BatteryMonitor.hpp>
BatteryMonitor batteryMonitor(myUART0);

#include "JogControl.hpp"
JogControl jogControl(motors, mySerial);

#include "AndonManager.hpp"
AndonManager andonMgr(andonLight, mySerial, motors, actuator, jogControl, batteryMonitor, ultrasonic, ultrasonicServo, estop, clamp);

#include "BootHealth.hpp"

#include "stm32h7xx_hal_rcc.h"

#include "Portenta_H7_TimerInterrupt.h"
volatile int interruptCounter = 0;
void m7timer() { 
  // every 1/10,000 second - 10,000hz - 0.0001 second
  interruptCounter++;

  //if(mySerial.delay) mySerial.delay--;

  // every 10/10,000 second - 1,000hz - 0.001 second
  if ((interruptCounter % 10) == 0) { 
      if(mySerial.thisDelay) mySerial.thisDelay--;
      if(mySerial.timeout) mySerial.timeout--;
      if (mySerial.receiveDelay) mySerial.receiveDelay--;

  }

  // every 100/10,000 second - 100hz - 0.01 second
  if ((interruptCounter % 100) == 0) { 
    if(ultrasonic.delay) ultrasonic.delay--;
    if (ultrasonic.servoSettleDelay) ultrasonic.servoSettleDelay--;

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

namespace ColdBootReset {

  // Start with 2 seconds; increase to 3 s if needed.
  static constexpr uint32_t kAutoResetDelayMs = 2000;

  inline bool isColdPowerBoot()
  {
    const bool por = (__HAL_RCC_GET_FLAG(RCC_FLAG_PORRST) != RESET);
    const bool bor = (__HAL_RCC_GET_FLAG(RCC_FLAG_BORRST) != RESET);
    const bool sft = (__HAL_RCC_GET_FLAG(RCC_FLAG_SFTRST) != RESET);

    // Cold boot if power-related reset is present, but not software reset.
    return (por || bor) && !sft;
  }

  inline void clearResetFlags()
  {
    __HAL_RCC_CLEAR_RESET_FLAGS();
    __DSB();
    __ISB();
  }

  [[noreturn]] inline void doSoftwareReset()
  {
    __disable_irq();
    NVIC_SystemReset();
    while (true) { }
  }

} // namespace ColdBootReset


void setup(void);
void loop(void);


void setup() {

  // --- One-time delayed self-reset on cold power boot ---
  {
    const bool coldBoot = ColdBootReset::isColdPowerBoot();

    // Clear flags now so the next reset is classified cleanly.
    ColdBootReset::clearResetFlags();

    if (coldBoot) {
      // Optional: bring up the Pi-facing UART just enough to leave breadcrumbs.
      myUART0.begin(115200);
      delay(50);
      myUART0.println("{\"type\":\"status\",\"status\":\"boot\",\"msg\":\"cold_boot_detected_autoreset_pending\"}");

      delay(ColdBootReset::kAutoResetDelayMs);

      myUART0.println("{\"type\":\"status\",\"status\":\"boot\",\"msg\":\"autoreset_now\"}");
      delay(20);

      ColdBootReset::doSoftwareReset();
    }
  }

  //Serial.begin(115200);
  //while (!Serial) {
  //  delay(10);
  //}
  //Serial.println("{\"status\":\"boot\",\"msg\":\"USB debug Serial starting\"}");

  // Main Pi-facing UART
  myUART0.begin(115200);
  delay(3000);
  myUART0.println("{\"type\":\"status\",\"status\":\"boot\",\"msg\":\"uart_starting\"}");

  delay(200);
  mySerial.setup();
  motors.begin();
  ultrasonic.setup();
  ultrasonicServo.attachUltrasonic(ultrasonic);
  estop.setup();
  mySerial.attachAndonManager(andonMgr);
  mySerial.attachUltrasonic(ultrasonic, ultrasonicEnabled);
  mySerial.attachUltrasonicServo(ultrasonicServo);
  ultrasonic.attachMotors(motors);
  ultrasonicServo.setup();
  actuator.setup ();
  jogControl.setup();
  mySerial.attachJogControl(jogControl);
  batteryMonitor.setup();
  clamp.setup();
  andonLight.setup();
  andonMgr.setup();

  
  // --- Boot health check: probe subsystems and emit one JSON line to Raspberry Pi ---
  BootHealth::Report rep = BootHealth::run(
      andonLight,
      motors,
      actuator,
      ultrasonic,
      ultrasonicServo,
      &batteryMonitor,
      estop,
      /*can_timeout_ms=*/500
  );
  BootHealth::sendReport(rep, myUART0);

  //If boot health fails, latch Andon to BLINK_RED (until manual override)
  //if (!rep.ok) andonMgr.setOverride(AndonLight::BLINK_RED);
  M7Timer.attachInterruptInterval(100, m7timer);
}

void loop() {
  
  if (andonLight.booting) {
    motors.setMotionInhibited(true);
    andonLight.loop();
    return;
  }
  andonLight.loop();
  estop.tick();
  clamp.tick();
  batteryMonitor.stateMachine();
  andonMgr.enforceMotionSafety();
  mySerial.stateMachine();
  jogControl.update();

  if (ultrasonicEnabled) {
      ultrasonicServo.activate();  // Servo active when ultrasonic is enabled
      ultrasonic.stateMachine();
      //myUART0.print("Ultrasonic distance: ");
      //myUART0.println(ultrasonic.measuredDistance);
  } else {
      ultrasonicServo.deactivate(); // Servo inactive when ultrasonic is disabled
      ultrasonic.servoSettleDelay = 0;
  };

  actuator.stateMachine();
  andonMgr.enforceMotionSafety();
  motors.update();
  andonMgr.tick();
}
