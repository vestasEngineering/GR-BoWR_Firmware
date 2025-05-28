//====================================================
// title block
//====================================================
/*  
 *  Author: Jacob Owens - Avid Product Development
 *  Customer: Vestas
 *  Date: 08082023
 */
//====================================================
// end title block
//====================================================


//====================================================
// shared data
//====================================================
struct shared_data {
  long stepperSpeed = 0; // buffer
  int stepperCommand = 0; // stepper state : 0 = stopped, 1 = forward, 2 = backward
};
volatile struct shared_data * const xfr_ptr = (struct shared_data *)0x38001000;
//====================================================
// end shared data
//====================================================

//====================================================
// included libraries
//====================================================
#include <Arduino.h>
#include <math.h>
//====================================================
// end included libraries
//====================================================



//====================================================
// timing debug
//====================================================
bool testState = false;
volatile int testDelay = 0;

//====================================================
// end timing debug
//====================================================


//====================================================
// wifi and websockets definitions
//====================================================
#define USE_WIFI_PORTENTA_H7  true
#define USE_WIFI_NINA         false
#define USE_WIFI_CUSTOM       false
#define WEBSOCKETS_USE_PORTENTA_H7_WIFI           true
#define DEBUG_WEBSOCKETS_PORT     Serial
// Debug Level from 0 to 4
#define _WEBSOCKETS_LOGLEVEL_     4
#include <WebSockets2_Generic.h>
#include <WiFi.h>
#define WEBSOCKETS_PORT     8080

const uint16_t websockets_server_port = WEBSOCKETS_PORT;
const char* websockets_server_host = "10.42.0.109";

const char* ssid = "grlrr2024"; //Enter SSID
const char* password = "grlrr2024"; //Enter Password
using namespace websockets2_generic;
#define HEARTBEAT_INTERVAL      300000 // 5 Minutes
uint64_t heartbeatTimestamp     = 0;
uint64_t now                    = 0;
bool wsConnected = false;
bool wsFlag = 0;
volatile int wsReconnectDelay = 0;
volatile int wsDelay = 0;

int uptime = 0;

byte mac[6];

//====================================================
// end wifi and websockets definitions
//====================================================




//====================================================
// stepper definitions
//====================================================
#include <SPI.h>
#include <HighPowerStepperDriver.h>

#define EXTRA_STEPPER_ENABLE_PIN D0

const uint8_t CSPin = D7;
HighPowerStepperDriver sd;

int stepperCommand = 0;
long stepperSpeed = 0;
long targetStepperSpeed = 0;
int stepperMode = 0; // 0 set speed, 1 auto speed
bool stepperEnable = 1; // 0 steppers off, 1 steppers on
volatile int stepperDelay = 0;
long milliamps = 1000;
//====================================================
// end stepper definitions
//====================================================

//====================================================
// motor definitions
//====================================================
#include <Portenta_H7_PWM.h>
const uint8_t pin_inv = D3;
const uint8_t pin_D1 = D2;
volatile int motorDelay = 0;
mbed::PwmOut* pwm   = NULL;
double wheelDiameter = 0.040; // 0.040 meters
double wheelSpeed = 0.0; // m/sa
//====================================================
// end motor definitions
//====================================================
//
//
//
//
//====================================================
// encoder definitions
//====================================================
const uint8_t encoderPin1 = D0;
const uint8_t encoderPin2 = D1;
volatile int encoderDelay = 0;
bool initEncoderState1 = false;
bool lastEncoderState1 = false;
bool initEncoderState2 = false;
bool lastEncoderState2 = false;
volatile int  encoderCount1 = 0;
volatile int  encoderCount2 = 0;
bool encoderFlag1 = 0;
bool encoderFlag2 = 0;
const int encoderRotation = 1120;
double rotations = 0;
double speed = 0.0;
int rotationDuration = 0;
unsigned long this_time = 0;
//====================================================
// end encoder definitions
//====================================================

//====================================================
// encoder1 timer
//====================================================
void incrementEncoder1() { 
  encoderCount1++;  
}
//====================================================
// end encoder1 timer
//====================================================

//====================================================
// encoder2 timer
//====================================================
void incrementEncoder2() { 
  encoderCount2++;
}
//====================================================
// end encoder2 timer
//====================================================


//====================================================
// ultrasonic definitions
//====================================================
#define ULTRASONIC_PIN A1
long duration = 0; 
long distance = 0;
long ultrasonic_value = 0;
volatile int ultrasonicDelay = 0;
bool ultrasonicFlag = 0;
long current_time = 0;

double aveUltrasonicValue = 0;
const int numUltrasonicSamples = 20;
double ultrasonicSamples[numUltrasonicSamples] = {0};
int ultrasonicSampleNumber = 0;
//====================================================
// end ultrasonic definitions
//====================================================




//====================================================
// battery monitor definitions
//====================================================
#define BATTERY_PIN A0
double battery_value = 0;
volatile int batteryDelay = 0;

double aveBatteryValue = 0;
const int numBatterySamples = 20;
double batterySamples[numBatterySamples] = {0};
int batterySampleNumber = 0;
//====================================================
// end battery monitor definitions
//====================================================




//====================================================
// PID definitions
//====================================================
double setpoint = 0; // target ultrasonic value
double Kp = 0.1;
double Ki = 0;
double Kd = 0;

double last_error = 0;
double current_error = 0;
double changeError = 0;
double totalError = 0;
double pidTerm = 0;
double pidTerm_scaled = 0;
volatile int PIDDelay = 0;


// P[n] = Kp * e[n]
// I[n] = Ki * T / 2 * (E[n] + E[n-1]) + i[n-1]
// D[n] = 2Kd /( 2tau + T) * (E[n] - E[n-1]) + 2tau - T/ 2tau + T * D[n-1]


double initMeasurement = 0;      //   ultrasonic sensor reading
double prevMeasurement = 0;
double initError = 0;        // diff from setpoint
double prevError = 0;
double p = 0; //  proportional term
double i = 0; // integral term
double d = 0; // derivative term

double u = 0; // control output

// double Kp = 0; // proportional gain
// double Ki = 0; // integral gain
// double Kd = 0; // derivative gain

double T = 0.1; // sampling time constant 1/10 second
double tau = 0.2; // low pass time constant 2/10 second

//====================================================
// end PID definitions
//====================================================




//====================================================
// clamp definitions
//====================================================
#define CLAMP_PIN D5
bool clamped = false;
volatile int clampDelay = 0;
//====================================================
// end clamp definitions
//====================================================




//====================================================
// led definitions
//====================================================
#define RED_LED     LEDR
#define GREEN_LED   LEDG
#define BLUE_LED    LEDB
bool redLedFlag = false;
bool blueLedFlag = false;
volatile int redLedDelay = 0;
volatile int blueLedDelay = 0;
//====================================================
// end led definitions
//====================================================




//====================================================
// state led definitions
//====================================================
#define STATE_LED_RED PE_3      // gpio 4
#define STATE_LED_YELLOW PG_3   // gpio 5
#define STATE_LED_GREEN PG_10   // gpio 6
volatile int stateLEDDelay = 0;
bool stateLEDFlag = false;
int stateLEDcount = 0;
bool stateLEDchange = false;
//====================================================
// end state led definitions
//====================================================




//====================================================
// estop
//====================================================
#define ESTOP_PIN D6
bool estopFlag = 0; // 0 is false
volatile int estopDelay = 0;
bool estop = 0;
//====================================================
// end estop definitions
//====================================================




//====================================================
// json definitions
//====================================================
#include <ArduinoJson.h>
// json packet allows us to use a 'dict' like structure in the form of "jsonPacket['prop'] = value"
StaticJsonDocument<512> jsonPacket;
// json message is a string that contains all the info smashed together
String jsonMessage = "";
//====================================================
// end json definitions
//====================================================




//====================================================
// ip address definitions
//====================================================
// Since the esp is also on its own network... it has a local ip that isn't used for anything. 
IPAddress local(10, 42, 0, 135);
// throwing googles dns in for a temp fix
IPAddress testdns(8,8,8,8); 
// Since we are serving as an access point this is the address where the websockets will be posted
IPAddress gateway(10, 42, 0, 1);
// These are set to the same just for ease of use. 
IPAddress nmask(255, 255, 255, 0);
//====================================================
// end ip address definitions
//====================================================




//====================================================
// function prototypes
//====================================================
void setup(void);
void loop(void);
void RedLedMachine(void);
void BlueLedMachine(void);
void UltrasonicMachine(void);
void BatteryMachine(void);
int voltageToPercent(int);
void ClampMachine(void);
void StepperSpeedMachine(void);
void WebSocketMachine(void);
void onMessageCallback(WebsocketsMessage);
void onEventsCallback(WebsocketsEvent, String);
void StepperMachine(void);
void ReceiveJsonMachine(void);
void SendJsonMachine(void);
void CommandToEnumState(void);
long microsecondsToCentimeters(long);
void StepperPID(void);
void PID(void);
void StateLEDMachine(void);
void EstopMachine(void);
void motorMachine(void);
void test(void);
void encoderMachine(void);
void incrementEncoder1(void);
void incrementEncoder2(void);
//====================================================
// end function prototypes
//====================================================




//====================================================
// states
//====================================================
enum LedStates {
  LED_OFF,
  LED_ON
};
LedStates RedLedState;
LedStates BlueLedState;

enum wsStates { 
  WS_DISCONNECTED,
  WS_CONNECTED
};
wsStates wsState;

enum SpeedStates { 
  SET_MODE,
  AUTO_MODE
};
SpeedStates speedState;


enum UltrasonicStates {
  UT_WAITING,
  UT_READING
};
UltrasonicStates UltrasonicState;

enum BatteryStates {
  BAT_WAITING,
  BAT_READING
};
BatteryStates BatteryState;

enum ClampStates {
  CLAMP_DISENGAGED,
  CLAMP_ENGAGED
};
ClampStates ClampState;

enum StepperStates {
  OFF,
  INIT,
  RUN,
  ERR
};
StepperStates StepperState;


enum StateLEDStates {
  READY,                // green 2 second 2 flash
  UNSAFE,        // red constant
  WS_DOWN,             // red 1/2 second flash
  RESETTING_MOTORS,     // flash red and yellow
  MOTORS_DISABLED
};
StateLEDStates StateLEDState;


// this will be high, active if either estop button is pressed
enum EstopStates {
  INACTIVE,          // this means we have no detected an estop signal 0
  ACTIVE             // this means we have detected an estop signal 1
};
EstopStates EstopState;


enum motorStates {
   MOTOR_OFF,
   MOTOR_ON
};
motorStates motorState;

//====================================================
// end states
//====================================================



//====================================================
// objects
//====================================================
WebsocketsServer wsServer;
WebsocketsMessage wsMessage;
WebsocketsClient wsClient;
//====================================================
// end objects
//====================================================




//====================================================
// system timer
//====================================================
#include "Portenta_H7_TimerInterrupt.h"
volatile int interruptCounter;
void TimerHandler() { 
  interruptCounter++;
  // going about 100khz
  
  if (testDelay) testDelay--;

  // every 1/10000 sec - 10khz
  if ((interruptCounter % 100) == 0) {
    if (motorDelay) motorDelay--;
    
  }

  // every 1/1000 second - 1khz
  if ((interruptCounter % 1000) == 0) {
    if (ultrasonicDelay) ultrasonicDelay--;
    if (encoderDelay) encoderDelay--;
    
  }

  // every 1/10 second - 0.1khz
  if ((interruptCounter % 10000) == 0) {
    if (redLedDelay) redLedDelay--;
    if (wsReconnectDelay) wsReconnectDelay--;
    if (wsDelay) wsDelay--;
    if (clampDelay) clampDelay--;
    if (stateLEDDelay) stateLEDDelay--;
    if (estopDelay) estopDelay--;
    if (stepperDelay) stepperDelay--;
    if (PIDDelay) PIDDelay--;
    if (batteryDelay) batteryDelay--;
    
    
  }

  // every second - 1hz
  if ((interruptCounter % 100000) == 0) {
    if (blueLedDelay) blueLedDelay--;
    
    interruptCounter = 0;
  }

}
Portenta_H7_Timer ITimer(TIM3);
//====================================================
// end system timer
//====================================================

//====================================================
// m4 core timer
//====================================================
#include "Portenta_H7_TimerInterrupt.h"
volatile int interruptCounter = 0;
void timer() { 
  // every 1/10,000 second - 10,000hz - 0.0001 second
  interruptCounter++;


  // every 10/10,000 second - 1,000hz - 0.001 second
  if ((interruptCounter % 10) == 0) { 
    if(testDelay) testDelay--;
  }

  // every 100/10,000 second - 100hz - 0.01 second
  if ((interruptCounter % 100) == 0) { 
    
  }

  // every 1,000/10,000 second - 10hz - 0.1 second
  if ((interruptCounter % 1000) == 0) { 
    

  }

  // every 10,000/10,000 second - 1hz
  if ((interruptCounter % 10000) == 0) {
    interruptCounter = 0;
  }

}
Portenta_H7_Timer M4Timer(TIM16);
//====================================================
// end m4 core timer
//====================================================



//====================================================
// setup
//====================================================
void setup() {
  // state led setup
  pinMode(STATE_LED_RED, OUTPUT);
  pinMode(STATE_LED_YELLOW, OUTPUT);
  pinMode(STATE_LED_GREEN, OUTPUT);

  digitalWrite(STATE_LED_RED, HIGH);
  digitalWrite(STATE_LED_YELLOW, HIGH);
  digitalWrite(STATE_LED_GREEN, HIGH);


  digitalWrite(RED_LED, LOW);

  // extra stepper pin
  pinMode(EXTRA_STEPPER_ENABLE_PIN, OUTPUT);
  digitalWrite(EXTRA_STEPPER_ENABLE_PIN, HIGH);


  // spi setup for steppers:
  SPI.begin();
  sd.setChipSelectPin(CSPin);
  // Give the driver some time to power up.
  delay(10);
  // Reset the driver to its default settings and clear latched status
  // conditions.
  sd.resetSettings();
  sd.clearStatus();
  // Select auto mixed decay.  TI's DRV8711 documentation recommends this mode
  // for most applications, and we find that it usually works well.
  sd.setDecayMode(HPSDDecayMode::Fast);
  // Set the current limit. You should change the number here to an appropriate
  // value for your particular system.
  sd.setCurrentMilliamps36v4(milliamps);
  // Set the number of microsteps that correspond to one full step.
  sd.setStepMode(HPSDStepMode::MicroStep1);
  // Enable the motor outputs.
  //sd.enableDriver();

  // initialize m4 core
  bootM4();


  // timer setup
  ITimer.attachInterruptInterval(10, TimerHandler); // is thje timer messing with timeout of the websockets connection??????????
  // debug setup
  Serial.begin(115200);


  //done with steppers, timers, serial configs
  digitalWrite(STATE_LED_RED, LOW);

  // wifi setup
  while (!Serial && millis() < 2000);

  WiFi.begin(ssid, password);

  delay(500);
  WiFi.config(local, testdns, gateway, nmask);
  delay(500);

  while (WiFi.status() != WL_CONNECTED){
    delay(500);
    WiFi.begin(ssid, password);
    delay(500);
    WiFi.config(local, testdns, gateway, nmask);
    delay(500);
    Serial.println("trying to connect");
  }
  digitalWrite(RED_LED, HIGH);

  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
  Serial.println("Mac address: ");
  
  WiFi.macAddress(mac);

  Serial.print(mac[5],HEX);
  Serial.print(":");
  Serial.print(mac[4],HEX);
  Serial.print(":");
  Serial.print(mac[3],HEX);
  Serial.print(":");
  Serial.print(mac[2],HEX);
  Serial.print(":");
  Serial.print(mac[1],HEX);
  Serial.print(":");
  Serial.println(mac[0],HEX);
  // end wifi setup


  // done and connected with wifi config
  digitalWrite(STATE_LED_YELLOW, LOW);


  // websocket setup
  wsServer.listen(websockets_server_port);
  Serial.println("websocket server listening...");
  // end websocket setup


  // ultrasonic setup
  analogReadResolution(12);
  pinMode(ULTRASONIC_PIN, INPUT);

  //battery monitor setup
  pinMode(BATTERY_PIN, INPUT);

  // clamp setup
  pinMode(CLAMP_PIN, INPUT);

  // estop detect setup
  //pinMode(ESTOP_PIN, INPUT);
  delay(2000);

  // done with all setup
  digitalWrite(STATE_LED_GREEN, LOW);
  
  StateLEDState = READY;

  // encoder setup
  pinMode(encoderPin1, INPUT);
  pinMode(encoderPin2, INPUT);

  attachInterrupt(encoderPin1, incrementEncoder1, RISING);
  attachInterrupt(encoderPin2, incrementEncoder2, RISING);

}
//====================================================
// end setup
//====================================================




//====================================================
// main loop
//====================================================
void loop() {

  BlueLedMachine();
  // RedLedMachine();
  //WebSocketMachine();
  //UltrasonicMachine();
  //BatteryMachine();
  //ClampMachine();
  //StepperSpeedMachine();
  //StepperMachine();
  //StateLEDMachine();
  //EstopMachine();
  //test();
  //motorMachine();
  //encoderMachine();
}
//====================================================
// end main loop
//====================================================




//====================================================
// functions
//====================================================
//====================================================
// blue led machine
//====================================================
void BlueLedMachine() {
  switch(BlueLedState) {
    case LED_OFF:
      if (!blueLedDelay && !blueLedFlag) {
        //ledFlag = false;
        blueLedDelay = 1;
        BlueLedState = LED_ON;
        digitalWrite(BLUE_LED, LOW);
        uptime = uptime + 1;
        //Serial.println(String(millis()/1000.0/60.0));
      }
    break;
    case LED_ON:
      if (!blueLedDelay && !blueLedFlag) {
        //ledFlag
        blueLedDelay = 1;
        BlueLedState = LED_OFF;
        digitalWrite(BLUE_LED, HIGH);
        //Serial.println(String(millis()/1000.0/60.0));
        //Serial.println("stepper speed");
        //Serial.println(stepperSpeed);
        //Serial.println("stepper pid term");
        //Serial.println(pidTerm);
      }
    break;
    default:
    break;
  }
}
//====================================================
// end blue led machine
//====================================================




//====================================================
// red led machine
//====================================================
void RedLedMachine() {
  switch(RedLedState) {
    case LED_OFF:
      if (!redLedDelay && !redLedFlag) {
        //ledFlag = false;
        redLedDelay = 1;
        RedLedState = LED_ON;
        digitalWrite(RED_LED, LOW);
        //Serial.println("led on");
      }
    break;
    case LED_ON:
      if (!redLedDelay && !redLedFlag) {
        //ledFlag
        redLedDelay = 1;
        RedLedState = LED_OFF;
        digitalWrite(RED_LED, HIGH);
        //Serial.println("led off");
      }
    break;
    default:
    break;
  }
}
//====================================================
// end red led machine
//====================================================





//====================================================
// ultrasonic machine
//====================================================
void UltrasonicMachine() {
  switch(UltrasonicState){
    case UT_WAITING:
      if (!ultrasonicDelay) {
        UltrasonicState = UT_READING;
      }
    break;
    case UT_READING:
      if (!ultrasonicDelay) {
        ultrasonic_value = analogRead(ULTRASONIC_PIN);
        ultrasonicSamples[ultrasonicSampleNumber++] = ultrasonic_value;

        if (ultrasonicSampleNumber >= numUltrasonicSamples) {ultrasonicSampleNumber = 0;}

        aveUltrasonicValue = 0;

        for(int i=0; i< numUltrasonicSamples; ++i){aveUltrasonicValue += ultrasonicSamples[i];}
        aveUltrasonicValue /= numUltrasonicSamples;

        ultrasonicDelay = 50; // 50/1000 seconds
        UltrasonicState = UT_WAITING;
      }

    break;
    default:
    break;
  }

}


long microsecondsToCentimeters(long microseconds) {
  // The speed of sound is 340 m/s or 29 microseconds per centimeter.
  // The ping travels out and back, so to find the distance of the object we
  // take half of the distance travelled.
  return microseconds / 29 / 2;
}
//====================================================
// end ultrasonic machine
//====================================================

//====================================================
// battery machine
//====================================================
void BatteryMachine() {
  switch(BatteryState){
    case BAT_WAITING:
      if (!batteryDelay) {
        BatteryState = BAT_READING;
        //Serial.println("aveBatteryValue");
        //Serial.println(aveBatteryValue);
      }
    break;
    case BAT_READING:
      if (!batteryDelay) {
        // 1. double() to change analogRead() units
        // 2. times 3.1 to scale to max ADC voltage for Portenta
        // 3. divide by 4096 to convert from the ADC precision 
        // 4. times 11.0 to scale to original battery voltage - this comes from the voltage divider on the board
        // overall I think this gets us the battery voltage +/- 0.15
        battery_value = (double(analogRead(BATTERY_PIN)) * 3.1 / 4096.0) * (11.0);
        batterySamples[batterySampleNumber++] = battery_value;

        if (batterySampleNumber >= numBatterySamples) {batterySampleNumber = 0;}

        aveBatteryValue = 0.0;

        for(int i=0; i< numBatterySamples; ++i){aveBatteryValue += batterySamples[i];}
        aveBatteryValue /= double(numBatterySamples);

        batteryDelay = 2; // 2/10 seconds
        BatteryState = BAT_WAITING;
      }
    break;
    default:
    break;
  }
}


int voltageToPercent(int voltage) {
  return voltage / 4095;
}


//====================================================
// end battery machine
//====================================================




//====================================================
// clamp machine
//====================================================
void ClampMachine(void){
    switch(ClampState) {
    case CLAMP_DISENGAGED:
      if (digitalRead(CLAMP_PIN) == 1 && !clampDelay) {
        ClampState = CLAMP_ENGAGED;
        clampDelay = 5; // 1/10 seconds
        clamped = true; // true is engaged
      
      }
      
    break;
    case CLAMP_ENGAGED:
      if (digitalRead(CLAMP_PIN) == 0 && !clampDelay) {
        ClampState = CLAMP_DISENGAGED;
        clampDelay = 5; // 1/10 seconds
        clamped = false; // false is disengaged
      }

    break;
    default:
    break;
  }
}
//====================================================
// end clamp machine
//====================================================


//====================================================
// estop machine
//====================================================
void EstopMachine(void){
    switch(EstopState) {
    case INACTIVE:
      if (!estopDelay && !digitalRead(ESTOP_PIN)) { //estop pin high means there is power on the steppers, estop not pressed
        estop = true; 
        estopDelay = 2;
        EstopState = ACTIVE;
      }
      
    break;
    case ACTIVE:
      if (!estopDelay && digitalRead(ESTOP_PIN)) {
        estop = false; 
        estopDelay = 2;
        EstopState = INACTIVE;
      }

    break;
    default:
    break;
  }
}
//====================================================
// end estop machine
//====================================================


//====================================================
// stepper speed machine
//====================================================
void StepperSpeedMachine(void) {
  switch(speedState) {
    case SET_MODE:
      if (stepperMode) {
        Serial.println("Going to auto mode");
        speedState = AUTO_MODE;
      }

    break;
    case AUTO_MODE:
      if (!stepperMode) {
        Serial.println("Going to set mode");
        speedState = SET_MODE;
        stepperSpeed = 0;
        xfr_ptr->stepperSpeed = stepperSpeed;

      }
      if (!PIDDelay) {
      StepperPID();
      xfr_ptr->stepperSpeed = stepperSpeed;
      PIDDelay = 1; //1/10 seconds
      }

    break;
    default:
    break;
  }

}

void StepperPID(void) {
  current_error = setpoint - aveUltrasonicValue;
  
  changeError = current_error - last_error; // derivative term
  totalError += current_error; //accumalate errors to find integral term
  pidTerm = (Kp * current_error) + (Ki * totalError) + (Kd * changeError);//total gain
  pidTerm = constrain(pidTerm, -10000, 10000);//constraining to appropriate value

  stepperSpeed = constrain(pidTerm + targetStepperSpeed, 0, 2100);

  last_error = current_error;
}

void PID(void){
  // compute error
  initError = setpoint - initMeasurement;
  
  // calculate the proportional term
  p = Kp * initError;

  i = i + 0.5f * Ki * T * (initError + prevError);

  i = constrain(i, -10000.0, 10000.0); // this limits are random right now
  
  d = -(2.0 * Kd * (initMeasurement-prevMeasurement) + (2.0 * tau - T) * d) / (2.0 * tau + T);
  
  // sum control terms
  u = p + i + d;

  // store value for next iteration
  prevError = initError;
  prevMeasurement = initMeasurement;
}
//====================================================
// end stepper speed machine
//====================================================




//====================================================
// state LED machine
//====================================================
void StateLEDMachine(void) {
    switch(StateLEDState) {
    case READY: // yellow 1 second flash
      if (estop || !clamped) {
        StateLEDState = UNSAFE;
      }

      //default, let everything be good
      stateLEDFlag = !stateLEDFlag;
      stateLEDDelay = 10;
      digitalWrite(STATE_LED_GREEN, stateLEDFlag ? HIGH : LOW);
      stateLEDcount++;
      if(stateLEDcount == 4){
        stateLEDcount = 0;
        stateLEDDelay = 10;
        stateLEDFlag = false;
        digitalWrite(STATE_LED_GREEN, LOW);
      }
      
    break;

    case UNSAFE: // red constant
      digitalWrite(STATE_LED_RED, HIGH);

      if(!stateLEDDelay && wsConnected && !estop && clamped && !stepperDelay && stepperEnable) {
        StateLEDState = READY;
      }
    break;

    case WS_DOWN: // red 1 second flash
      if (!stateLEDDelay && !wsConnected) {
        stateLEDFlag = !stateLEDFlag;
        stateLEDDelay = 10;
        digitalWrite(STATE_LED_RED, stateLEDFlag ? HIGH : LOW);
      }
    break;

    case RESETTING_MOTORS: // red 1 second 2 flash
      if (!stateLEDDelay && wsConnected && !estop && clamped && !stepperDelay && stepperEnable) {
        stateLEDFlag = !stateLEDFlag;
        stateLEDDelay = 10;
        digitalWrite(STATE_LED_YELLOW, stateLEDFlag ? LOW : HIGH);
        digitalWrite(STATE_LED_RED, stateLEDFlag ? HIGH : LOW);
      }
    break;

    case MOTORS_DISABLED: // red 1/2 second flash
      if (!stateLEDDelay || !wsConnected || estop || !clamped || !stepperEnable) {
        stateLEDFlag = !stateLEDFlag;
        stateLEDDelay = 10;
        digitalWrite(STATE_LED_RED, stateLEDFlag ? HIGH : LOW);
      }
    break;

    default:
    break;
  }
}
//====================================================
// end state LED machine
//====================================================


//====================================================
// websocket machine
//====================================================
void WebSocketMachine() {
wsClient.poll();
// the websocket machine here has two states
// it can be connected or not
// if connected it needs to send and receive messages
// if not connected it needs to tell us and try to get connected
  switch(wsState){
    case WS_DISCONNECTED:
      if (!wsReconnectDelay) {
        Serial.println("accepting new");

        wsClient.close();
        wsClient = wsServer.accept();
        wsConnected = wsClient.available();

        

        if (wsConnected){
          // register callback when messages are received
          wsClient.onMessage(onMessageCallback);
          // register callback when events are occuring          
          wsClient.onEvent(onEventsCallback);
          // change state to connected
          wsState = WS_CONNECTED;
          Serial.println("Connected!");
          digitalWrite(GREEN_LED, LOW);
          digitalWrite(RED_LED, HIGH);

        }
        wsReconnectDelay = 5;
      }
    break;
    case WS_CONNECTED:
      if (!wsClient.available()) {
        wsState = WS_DISCONNECTED;
        Serial.println("Disconnected!");
        digitalWrite(GREEN_LED, HIGH);
        digitalWrite(RED_LED, LOW);
      }

      if (!wsDelay && wsClient.available()) {   
        wsDelay = 2; // 2/10 seconds
        SendJsonMachine();
        wsClient.send(jsonMessage);
        //Serial.println("Just send a message");
      }
    break;
    default:
    break;
  }


}


void onMessageCallback(WebsocketsMessage message) {
    Serial.print("Got Message: ");
    Serial.println(message.data());
    // save string message to global variable 
    jsonMessage = message.data();
    ReceiveJsonMachine();
  }


void onEventsCallback(WebsocketsEvent event, String data) {
  (void) data;
  
  if (event == WebsocketsEvent::ConnectionOpened) {
    Serial.println("Connnection Opened");
    
  } 

  else if (event == WebsocketsEvent::ConnectionClosed) {
    Serial.println(String(millis()/1000));
    Serial.println("Connnection Closed");
    Serial.println(wsClient.getCloseReason());
    uptime = 0;
    //delay(2000);
  }

  else if (event == WebsocketsEvent::GotPing) {
    Serial.println("Got a Ping!");
    Serial.println(wheelSpeed);
  }

  else if (event == WebsocketsEvent::GotPong) {
    Serial.println("Got a Pong!");
  }
}
//====================================================
// end websocket machine
//====================================================


//====================================================
// send json machine
//====================================================
void SendJsonMachine(void) {
  jsonPacket.clear();
  jsonMessage = "";

  jsonPacket["stepper_command"]  = stepperCommand;
  jsonPacket["stepper_speed"]    = stepperSpeed;
  jsonPacket["stepper_mode"]     = stepperMode;
  jsonPacket["stepper_enable"]   = stepperEnable;

  jsonPacket["PID_setpoint"]     = setpoint;
  jsonPacket["PID_Kp"]           = Kp;
  jsonPacket["PID_Ki"]           = Ki;
  jsonPacket["PID_Kd"]           = Kd;

  jsonPacket["ultrasonic_value"] = aveUltrasonicValue;
  jsonPacket["battery_value"]    = aveBatteryValue;

  jsonPacket["estop"] = estop;
  jsonPacket["uptime"]          = uptime;

  serializeJson(jsonPacket, jsonMessage);
}
//====================================================
// end send json machine
//====================================================


//====================================================
// receive json machine
//====================================================
void ReceiveJsonMachine(void) {
    DeserializationError error = deserializeJson(jsonPacket, jsonMessage);
  // Test if parsing succeeds.
  if (error) {
    Serial.print(F("deserializeJson() failed: "));
    Serial.println(error.f_str());
  }

  // stepper stuff 
  // get and set the value to the local var and to the shared memory space
  stepperCommand = int(jsonPacket["stepper_command"]);
  xfr_ptr->stepperCommand = stepperCommand;

  // get and set the value to the local var and to the shared memory space
  stepperSpeed = double(jsonPacket["stepper_speed"]);
  targetStepperSpeed = double(jsonPacket["stepper_target_speed"]);
  xfr_ptr->stepperSpeed = stepperSpeed;

  stepperMode = bool(jsonPacket["stepper_mode"]);

  stepperEnable = bool(jsonPacket["stepper_enable"]);

  
  // get the pid values from the tablet
  setpoint = double(jsonPacket["PID_setpoint"]);
  Kp       = double(jsonPacket["PID_Kp"]);
  Ki       = double(jsonPacket["PID_Ki"]);
  Kd       = double(jsonPacket["PID_Kd"]);



  //CommandToEnumState();
  jsonPacket.clear();
  jsonMessage = "";
}
//====================================================
// end receive json machine
//====================================================




//====================================================
// stepper machine
//====================================================
void StepperMachine() {
  switch(StepperState) {
    case OFF:
      if (wsConnected && !estop && clamped && !stepperDelay && stepperEnable) { // if we are connected and the estop is not pressed go to init
        StepperState = INIT;

      }

    break;

    case INIT:
      Serial.println("Initializing Stepper Drivers");
      //delay(10);
      sd.setChipSelectPin(CSPin);
      // Give the driver some time to power up.
      delay(1);
      // Reset the driver to its default settings and clear latched status
      // conditions.
      sd.resetSettings();
      sd.clearStatus();
      // Select auto mixed decay.  TI's DRV8711 documentation recommends this mode
      // for most applications, and we find that it usually works well.
      sd.setDecayMode(HPSDDecayMode::Fast);
      // Set the current limit. You should change the number here to an appropriate
      // value for your particular system.
      sd.setCurrentMilliamps36v4(milliamps);
    
      // Set the number of microsteps that correspond to one full step.
      sd.setStepMode(HPSDStepMode::MicroStep1);
      sd.enableDriver();
      delay(1);

      stepperDelay = 2; // seconds to let things setttle?
      xfr_ptr->stepperCommand = 0; // set stepper command to 0 to stop motion
      stepperCommand = 0;          // set both to 0
      StepperState = RUN;
    break;

    case RUN:

      if (!wsConnected || estop || !clamped || !stepperEnable) { // if the websocket is disconnected or the estop is pressed or the clamp is not engaged  or if the steppers are disabled go to off
        Serial.println("DISABLING DRIVERS");
        sd.disableDriver();
        xfr_ptr->stepperCommand = 0; // set stepper command to 0 to stop motion
        stepperCommand = 0;          // set both to 0
        stepperDelay = 2;
        StepperState = OFF;
      }


    break;
    case ERR:
      if (!stepperDelay) {
        Serial.println("Steppers are in error");
        Serial.print("The stepper error is: ");
        Serial.println(sd.readStatus());

        StepperState = INIT;
        stepperDelay = 2;
      }

    default:
    break;
  }
}


//====================================================
// end stepper machine
//====================================================


//====================================================
// motor machine
//====================================================
void motorMachine() {
  setPWM(pwm, pin_D1, 100000.0, float(stepperSpeed));
  }
//====================================================
// end motor machine
//====================================================


//====================================================
// debug test machine
//====================================================
void test() {

  if (!testDelay){
    if (testState){
      digitalWrite(D6, HIGH);    
    }
    else {
      digitalWrite(D6, LOW);  
    }

    testDelay = 1;  // sets the countdown timer to 100ms.  
    testState = !testState;
  }
}

//====================================================
// end debug test machine
//====================================================

//====================================================
// encoder machine
//====================================================
void encoderMachine() {
  if (!encoderDelay){
    
    encoderDelay = 20; //counting 100 herts

      rotations = (double(encoderCount1)/double(encoderRotation)); // fraction of a rotation in 0.01 of a second
      speed = rotations / 0.01;                                     // the amount of rotation in that 0.01 of a second
      // V = R*W
      wheelSpeed = wheelDiameter * 0.5 * speed;                  // converted to linear velocity V = D * 1/2 * omega
      //Serial.println("speed: r/s");                              // rotatations per second
      //Serial.println(speed);
//
      //Serial.println("Wheel speed");
      //Serial.println(wheelSpeed);
//
      //Serial.println("encoder1 count");
      //Serial.println(encoderCount1);

      encoderCount1 = 0;


    }
  else {
    
  }
}

//====================================================
// end encoder machine
//====================================================


//====================================================
// end functions
//====================================================