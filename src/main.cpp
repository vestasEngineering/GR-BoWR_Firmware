//====================================================
// TITLE BLOCK
//====================================================
/*  
 *  Project: GR-ToWR Firmware
 *  Description: Firmware for the Glue Robot - Top of Web Rover (GR-ToWR), running on an Arduino Portenta H7. 
 *  Author: Giovanni Cordova - CREADIS
 *  Date: 09/18/2026
 */

#include <Arduino.h>
#include <ArduinoJson.h>
#include <mbed.h>
#include <math.h>
#include "Config.hpp"

UART myUART0(PA_0, PI_9, NC, NC);   // TX, RX, RTS, CTS

#include "EStop.hpp"
EStop estop;

#include <MotorCP.hpp>
MotorCP motors;

#include <Ultrasonic.hpp>
Ultrasonic ultrasonic;
bool ultrasonicEnabled = false;
bool hmiConnected = false;
bool laserBaseModeActive = false;
bool laserBaseSteering = false;
bool laserBaseAcquiring = false;
uint32_t laserBaseAcquireStartMs = 0;
  

#include <Actuator.hpp>
ActuatorControl actuator(myUART0);
bool actuatorWasConnected = false;
bool actuatorDisconnectActive = false;

#include <AndonLight.hpp>
AndonLight andonLight(myUART0);

#include "ContactorMonitor.hpp"
ContactorMonitor contactor;

#include "EncoderSession.hpp"
EncoderSession encoderSession(contactor, motors, myUART0);

#include <MySerial.hpp>
MySerial mySerial(myUART0, actuator, andonLight, motors);

#include <BatteryMonitor.hpp>
BatteryMonitor batteryMonitor(myUART0);

#include "JogControl.hpp"
JogControl jogControl(motors, mySerial);

#include "AndonManager.hpp"
AndonManager andonMgr(
    andonLight,
    mySerial,
    motors,
    actuator,
    jogControl,
    batteryMonitor,
    ultrasonic,
    estop,
    contactor,
    hmiConnected
);

#include "DiagnosticRunner.hpp"
DiagnosticRunner diagnosticRunner(myUART0, motors, actuator, batteryMonitor, ultrasonic, andonMgr, estop, jogControl, ultrasonicEnabled
);
#include "BootHealth.hpp"
bool runtimeReadyPublished = false;

#include "stm32h7xx_hal_rcc.h"

#include "Portenta_H7_TimerInterrupt.h"
volatile int interruptCounter = 0;
void m7timer() {
  // 100 kHz (10 us). The step generator must run on every tick.
  MotorCP::stepIsr();

  interruptCounter++;

  // 1,000 Hz - 1 ms
  if ((interruptCounter % 100) == 0) {
      if(mySerial.thisDelay) mySerial.thisDelay--;
      if(mySerial.timeout) mySerial.timeout--;
      if (mySerial.receiveDelay) mySerial.receiveDelay--;
  }

  // 100 Hz - 10 ms
  if ((interruptCounter % 1000) == 0) {
    if(ultrasonic.delay) ultrasonic.delay--;
  }

  // 1 Hz - wrap the counter
  if ((interruptCounter % 100000) == 0) {
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
  mySerial.attachEncoderSession(encoderSession);
  mySerial.attachHmiConnectionState(hmiConnected);
  motors.begin();
  contactor.setup();
  encoderSession.setup();
  ultrasonic.setup();
  estop.setup();
  mySerial.attachAndonManager(andonMgr);
  mySerial.attachUltrasonic(ultrasonic, ultrasonicEnabled);
  ultrasonic.attachMotors(motors);

  actuator.setup ();
  actuatorWasConnected = !actuator.hasPCBFault();
  actuatorDisconnectActive = !actuatorWasConnected;

  jogControl.setup();
  mySerial.attachJogControl(jogControl);
  mySerial.attachDiagnosticRunner(
      diagnosticRunner
  );
  batteryMonitor.setup();
  andonLight.setup();
  andonMgr.setup();
  
  mySerial.emitProcessStatus(
    false,
    "firmware_startup"
  );

  // --- Boot health check: probe subsystems and emit one JSON line to Raspberry Pi ---
  BootHealth::Report rep = BootHealth::run(
      andonLight,
      motors,
      actuator,
      ultrasonic,
      &batteryMonitor,
      estop,
      /*can_timeout_ms=*/500
  );
  BootHealth::sendReport(rep, myUART0);

  //If boot health fails, latch Andon to BLINK_RED (until manual override)
  //if (!rep.ok) andonMgr.setOverride(AndonLight::BLINK_RED);
  M7Timer.attachInterruptInterval(10, m7timer);
}

void emitLaserBaseStatus(const char* status, const char* reason = nullptr)
{
    StaticJsonDocument<192> doc;
    doc["type"] = "status";
    doc["module"] = "laser_base";
    doc["status"] = status;
    if (reason) {
        doc["reason"] = reason;
    }
    serializeJson(doc, myUART0);
    myUART0.println();
}

const char* laserBaseEntryBlocker()
{
    if (!CFG.steering.enabled) return "steering_disabled";
    if (estop.isActive()) return "estop_active";
    if (andonMgr.motionStopRequired()) return "fault_active";
    if (ultrasonicEnabled) return "process_active";
    if (diagnosticRunner.active()) return "diagnostic_active";
    if (jogControl.isActive()) return "jog_active";
    if (motors.isMotionInhibited()) return "motion_inhibited";
    return nullptr;
}

// Base mode starts in a search phase with steering disarmed until the laser
// angle is within the acquisition window. Speed comes from the feedforward UT
// tracker, so the robot stays stopped until glue is detected.
void enterLaserBaseMode()
{
    laserBaseModeActive = true;
    laserBaseSteering = false;
    laserBaseAcquiring = false;
    ultrasonic.reset();
    jogControl.setLocked(true);
    motors.steering.setHoldLastAngle(true);
    motors.steering.setArmed(false);
    emitLaserBaseStatus("entered");
}

void exitLaserBaseMode(const char* reason)
{
    laserBaseModeActive = false;
    laserBaseSteering = false;
    motors.steering.setArmed(true);
    motors.steering.setHoldLastAngle(false);
    jogControl.setLocked(false);
    ultrasonic.reset();
    motors.setSpeeds(0.0f, 0.0f);
    motors.BRAKE_STOP();
    emitLaserBaseStatus("exited", reason);
}

void updateLaserSteerAcquire(uint32_t now)
{
    const SteeringControl& steering = motors.steering;
    const bool inWindow =
        !steering.isStale(now)
        && fabsf(steering.lastAngleDeg()) <= CFG.laserBase.steer_acquire_angle_deg;

    if (!inWindow) {
        laserBaseAcquiring = false;
        return;
    }

    if (!laserBaseAcquiring) {
        laserBaseAcquiring = true;
        laserBaseAcquireStartMs = now;
        return;
    }

    if (static_cast<uint32_t>(now - laserBaseAcquireStartMs) < CFG.laserBase.steer_acquire_ms) {
        return;
    }

    laserBaseAcquiring = false;
    laserBaseSteering = true;
    motors.steering.setArmed(true);
    emitLaserBaseStatus("steering");
}

// Two-button hold enters laser base mode; only estop, a fault, or a jog press
// exits it. The chord latches after each attempt so one hold produces at most
// one entry or rejection.
void updateLaserBaseMode(uint32_t now)
{
    static bool chordStarted = false;
    static bool chordLatched = false;
    static bool exitArmed = false;
    static uint32_t chordStartMs = 0;
    static bool rawLast = false;
    static uint32_t rawChangeMs = 0;
    static bool stableAny = false;

    const bool forwardPressed = !digitalRead(CFG.jog.pin_forward);
    const bool backwardPressed = !digitalRead(CFG.jog.pin_backward);
    const bool anyPressed = forwardPressed || backwardPressed;
    const bool bothPressed = forwardPressed && backwardPressed;

    if (anyPressed != rawLast) {
        rawLast = anyPressed;
        rawChangeMs = now;
    }
    else if (anyPressed != stableAny
             && static_cast<uint32_t>(now - rawChangeMs) >= CFG.laserBase.exit_debounce_ms) {
        stableAny = anyPressed;
    }

    if (laserBaseModeActive) {
        if (!laserBaseSteering) {
            updateLaserSteerAcquire(now);
        }

        if (estop.isActive()) {
            exitLaserBaseMode("estop");
        }
        else if (andonMgr.motionStopRequired()) {
            exitLaserBaseMode("fault_active");
        }
        else if (!stableAny) {
            exitArmed = true;
        }
        else if (exitArmed) {
            exitLaserBaseMode("jog_button");
        }
        else {
            return;
        }

        chordLatched = true;
        chordStarted = false;
        return;
    }

    if (!anyPressed) {
        chordStarted = false;
        chordLatched = false;
        return;
    }

    if (!bothPressed) {
        chordStarted = false;
        return;
    }

    if (chordLatched) {
        return;
    }

    if (!chordStarted) {
        chordStarted = true;
        chordStartMs = now;
        return;
    }

    if (static_cast<uint32_t>(now - chordStartMs) < CFG.laserBase.entry_hold_ms) {
        return;
    }

    chordLatched = true;
    chordStarted = false;

    const char* blocker = laserBaseEntryBlocker();
    if (blocker) {
        emitLaserBaseStatus("entry_rejected", blocker);
        andonMgr.flash(AndonLight::SWEEP_PURPLE, CFG.laserBase.entry_reject_flash_ms);
        return;
    }

    exitArmed = false;
    enterLaserBaseMode();
}

void updateActuatorConnectionMode()
{
    const bool actuatorConnected = !actuator.hasPCBFault();

    // Connected -> disconnected transition
    if (actuatorWasConnected && !actuatorConnected) {
        // Record whether the ultrasonic process owned the motors before
        // changing its state.
        const bool ultrasonicWasActive =
            ultrasonicEnabled &&
            !actuatorDisconnectActive;

        // A disconnect terminates the ultrasonic process rather than
        // temporarily pausing it.
        ultrasonicEnabled = false;
        actuatorDisconnectActive = true;

        // Clear the final command issued by ultrasonic control.
        if (ultrasonicWasActive) {
            motors.BRAKE_STOP();
        }

        mySerial.emitProcessStatus(
            false,
          "actuator_dac_adc_disconnected"
        );
    }

    // Disconnected -> connected transition
    if (!actuatorWasConnected && actuatorConnected) {
        // The hardware is available again, but the ultrasonic process
        // remains disabled. A new operator command is required to restart it.
        actuatorDisconnectActive = false;

        // No BRAKE_STOP() is needed here because the ultrasonic process
        // is disabled and must not take ownership from JogControl.
        myUART0.println(
            "{\"type\":\"status\","
            "\"module\":\"motion\","
            "\"status\":\"actuator_dac_adc_reconnected\","
            "\"ultrasonic\":\"stopped\"}"
        );
    }

    actuatorWasConnected = actuatorConnected;
}

// The autonomous process steers from the compute module's laser angle. If
// that angle stops arriving, stop the process rather than drive blind.
void enforceSteeringFreshness(uint32_t now)
{
    static bool wasRunning = false;
    static uint32_t startedMs = 0;

    const bool running = ultrasonicEnabled;
    if (running && !wasRunning) {
        startedMs = now;
    }
    wasRunning = running;

    if (!running
        || !CFG.steering.enabled
        || !CFG.steering.stop_process_when_stale
        || jogControl.isActive()
        || diagnosticRunner.active()) {
        return;
    }

    if (static_cast<uint32_t>(now - startedMs) < CFG.steering.startup_grace_ms) {
        return;
    }

    if (!motors.steering.isStale(now)) {
        return;
    }

    ultrasonicEnabled = false;
    ultrasonic.stopAndBrake();
    mySerial.emitProcessStatus(false, "steering_angle_stale");
}

void loop() {

    const uint32_t now = millis();
    
    if (andonLight.booting) {
        motors.setMotionInhibited(true);
        andonLight.loop();
        return;
    }

    andonLight.setMotionActive(motors.isStepping());
    andonLight.loop();
    estop.tick();
    contactor.tick();
    encoderSession.tick(ultrasonicEnabled);
    batteryMonitor.stateMachine();

    // Check actuator DAC/ADC connection before choosing which controller
    // is allowed to issue motor commands.
    actuator.stateMachine();
    updateActuatorConnectionMode();

    // Process incoming commands.
    mySerial.stateMachine();
    diagnosticRunner.update();

    if (!runtimeReadyPublished) {
      runtimeReadyPublished = true;
      myUART0.println(
      "{\"type\":\"status\","
      "\"module\":\"runtime\","
      "\"status\":\"runtime_ready\"}"
      );
    }
  
    mySerial.emitActuatorStatus();

    // Apply hard safety conditions before accepting motion commands.
    andonMgr.enforceMotionSafety();

    updateLaserBaseMode(now);

    const bool actuatorConnected = !actuator.hasPCBFault();

    if (diagnosticRunner.active()) {
        // The diagnostic runner owns the motors.
    }
    else if (
        ultrasonicEnabled &&
        actuatorConnected &&
        !actuatorDisconnectActive
    ) {
        jogControl.update();

        if (!jogControl.isActive()) {
            ultrasonic.stateMachine();
        }
    }
    else {
        jogControl.update();

        if (laserBaseModeActive) {
            ultrasonic.stateMachine();
        }
    }

    andonMgr.enforceMotionSafety();

    enforceSteeringFreshness(now);

    motors.update();
    andonMgr.tick();
}
