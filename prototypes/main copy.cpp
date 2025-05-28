//====================================================
// TITLE BLOCK
//====================================================
/*  
 *  Author: Jacob Owens - Avid Product Development
 *  Customer: Vestas
 *  Date: 08082023
 */
//====================================================
// END TITLE BLOCK
//====================================================


//====================================================
// INITIALIZATION AND VARIABLES
//====================================================


//====================================================
// shared data
//====================================================
//struct shared_data {
//  long motorSpeed = 0;    // speed from PID control
//  long measuredMotorSpeed = 0;   // speed from encoders
//  int motorDirection = 0; // stepper state : 0 = stopped, 1 = forward, 2 = backward
//};
//volatile struct shared_data * const xfr_ptr = (struct shared_data *)0x38001000;
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
#define _WEBSOCKETS_LOGLEVEL_     1
#include <WebSockets2_Generic.h>
#include <WiFi.h>

#define WEBSOCKETS_PORT     8080
const uint16_t websockets_server_port = WEBSOCKETS_PORT;
const char* websockets_server_host = "10.42.0.109";
const char* hostname = "GRLLR";

const char* ssid = "grlrr2024"; //Enter SSID
const char* password = "grlrr2024"; //Enter Password

using namespace websockets2_generic;
bool wsConnected = false;
bool wsFlag = 0;
volatile int wsReconnectDelay = 0;
volatile int wsDelay = 0;

int uptime = 0;

WebsocketsServer wsServer;
WebsocketsMessage wsMessage;
WebsocketsClient wsClient;
//====================================================
// end wifi and websockets definitions
//====================================================



//====================================================
// vision definitions
//====================================================

volatile int visionDelay = 0;
String trajectory = "";

//====================================================
// end vision definitions
//====================================================

// old stuff
// #define EXTRA_STEPPER_ENABLE_PIN D0
// const uint8_t CSPin = D7;

//====================================================
// ultrasonic definitions
//====================================================
#define ULTRASONIC_PIN A1
long duration = 0; 
long distance = 0;
double ultrasonicValue = 0;
volatile int ultrasonicDelay = 0;
bool ultrasonicFlag = 0;
long current_time = 0;

double aveUltrasonicValue = 0;
double ultrasonicDistance = 0;
const int numUltrasonicSamples = 50;
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
double targetGlueHeight = 0; // target ultrasonic value
double UT_Kp = 0;
double UT_Ki = 0;
double UT_Kd = 0;

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

double T = 0.1; // sampling time constant 1/10 second
double tau = 2.0*T; // low pass time constant 2/10 second
//====================================================
// end PID definitions
//====================================================



//====================================================
// Motor PID definitions
//====================================================
double M_setSpeed = 0; // target motor speed
double M_Kp = 5; // proportional gain
double M_Ki = 10; // integral gain
double M_Kd = 10; // derivative gain

// P[n] = Kp * e[n]
// I[n] = Ki * T / 2 * (E[n] + E[n-1]) + i[n-1]
// D[n] = 2Kd /( 2tau + T) * (E[n] - E[n-1]) + 2tau - T/ 2tau + T * D[n-1]

struct  MotorEncoder {
  int dutyCycle = 0;
  double encoderSpeed = 0;
  double initMeasurement = 0;
  double prevMeasurement = 0;
  double initError = 0;
  double prevError = 0;
};

struct MotorEncoder M1, M2, M3, M4;


double M_p = 0; //  proportional term
double M_i = 0; // integral term
double M_d = 0; // derivative term
double M_u = 0; // control output

double M_T = 0.1; // sampling time constant 1/100 second based on the rate at which the function is executed
double M_tau = 2.0*M_T; // low pass time constant 2/100 second
//====================================================
// end Motor PID definitions
//====================================================


//====================================================
// motor definitions
//====================================================
#include <mbed.h>

mbed::PwmOut motorStepPin1(PK_1);  // d1 // connected to driver 1
mbed::PwmOut motorStepPin2(PJ_11); // d2 // connected to driver 2
mbed::PwmOut motorStepPin3(PC_6);  // d5 // connected to driver 3
mbed::PwmOut motorStepPin4(PC_7);  // d4 // connected to driver 4

const uint8_t DirPin1 = LEDB + 1 + PC_13; // gpio 0
const uint8_t DirPin2 = LEDB + 1 + PC_15; // gpio 1
const uint8_t DirPin3 = LEDB + 1 + PD_4; // gpio 2
const uint8_t DirPin4 = LEDB + 1 + PD_5; // gpio 3


//const uint8_t pin_inv = D3;
double lastMotorSpeed = 0;
double motorSpeed = 0;        // This is the speed set in the tablet in rotations per second
double speedOffset = 0;
double newMotorSpeed = 0;

//long finalMotorSpeed = 0;
//long initialMotorSpeed = 0;
////long motorAcceleration = 5; // const set from experience units are %/0.1 sec
long motorDuration = 0;     // the time needed to accelerate or decelerate 
//long velocityIncrement = 0;
double measuredMotorSpeed = 0;       // speed from encoder
int motorDirection = 0;     // stepper state : 0 = stopped, 1 = forward, 2 = backward
int currentMotorDirection = 0;
//int lastMotorDirection = 0;
//double targetMotorSpeed = 0;  // changing the offset of the set point in the PID controller
bool motorMode = 0;         // toggle to use PID mode (1) or set speed mode (0)
bool motorEnable = 0;         // toggle to diable the motors
volatile int motorDelay = 0;
volatile int motorDirectionDelay = 0;
bool direction = false; // false is forward, true is backward
double currentMotorSpeed = 0;
long deltaMotorDutyCycle = 0;
volatile int directionDelay = 0;


int motorDutyCycle1 = 0;
int motorDutyCycle2 = 0;
int motorDutyCycle3 = 0;
int motorDutyCycle4 = 0;

double wheelDiameter = 0.05; // mm

//==================================================
// end motor definitions
//====================================================


//====================================================
// clamp definitions
//====================================================
//#define CLAMP_PIN D5
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
//#define ESTOP_PIN D6
bool estopFlag = 0; // 0 is false
volatile int estopDelay = 0;
bool estop = 0;
//====================================================
// end estop definitions
//====================================================





//====================================================
// encoder definitions
//====================================================
const uint8_t encoderPin3 = D14; // connected to driver 3

const uint8_t encoderPin4 = D13; // connected to driver 4

const uint8_t encoderPin2 = D12; // connected to driver 2

const uint8_t encoderPin1 = D11; // connected to driver 1



volatile int encoderDelay = 0;

volatile int  encoderCount1 = 0;
volatile int  encoderCount2 = 0;
volatile int  encoderCount3 = 0;
volatile int  encoderCount4 = 0;

const int encoderRotation = 1120;
double encoderTime = 0;
double rotations = 0;
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
// encoder1 timer
//====================================================
void incrementEncoder3() { 
  encoderCount3++;  
}
//====================================================
// end encoder1 timer
//====================================================

//====================================================
// encoder2 timer
//====================================================
void incrementEncoder4() { 
  encoderCount4++;
}
//====================================================
// end encoder2 timer
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


enum visionStates {
  VISION_WAITING,
  VISION_READING
};
visionStates visionState;

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
  // ready for input, Unsafe / error, comms down, active process
  READY,               // green 1 second flash
  UNSAFE,              // red 1/2 sec flash
  WS_DOWN,             // red constant
  ACTIVE_PROCESS,      // yellow 1 second flash
};
StateLEDStates StateLEDState;
StateLEDStates LastStateLEDState;


// this will be high, active if either estop button is pressed
enum EstopStates {
  INACTIVE,          // this means we have no detected an estop signal 0
  ACTIVE             // this means we have detected an estop signal 1
};
EstopStates EstopState;


enum motorStates {
  MOTOR_STOPPED,
  MOTOR_RUNNING, 
  MOTOR_CHANGING_DIRECTION,
  MOTOR_ERROR,
};
motorStates motorState;



//====================================================
// end states
//====================================================
//====================================================
// END INITIALIZATION AND VARIABLES
//====================================================



//====================================================
// TIMER AND FUNCTION PROTOTYPES
//====================================================
//====================================================
// M7 CORE TIMER
//====================================================
#include "Portenta_H7_TimerInterrupt.h"
volatile int interruptCounter = 0;
void m7timer() { 
  // every 1/10,000 second - 10,000hz - 0.0001 second
  interruptCounter++;

  if(testDelay) testDelay--;
  // every 10/10,000 second - 1,000hz - 0.001 second
  if ((interruptCounter % 10) == 0) { 
     // this can indicate if something is taking way to long?
  }

  // every 100/10,000 second - 100hz - 0.01 second
  if ((interruptCounter % 100) == 0) { 
    if (ultrasonicDelay) ultrasonicDelay--;

    
  }

  // every 1,000/10,000 second - 10hz - 0.1 second
  if ((interruptCounter % 1000) == 0) { 

    if (redLedDelay) redLedDelay--;
    if (wsReconnectDelay) wsReconnectDelay--;
    if (wsDelay) wsDelay--;
    if (clampDelay) clampDelay--;
    if (stateLEDDelay) stateLEDDelay--;
    if (estopDelay) estopDelay--;
    //if (stepperDelay) stepperDelay--;
    if (PIDDelay) PIDDelay--;
    if (batteryDelay) batteryDelay--;
    if (encoderDelay) encoderDelay--;
    if (motorDelay) motorDelay--;
    if (motorDirectionDelay) motorDirectionDelay--;

    

  }

  // every 10,000/10,000 second - 1hz
  if ((interruptCounter % 10000) == 0) {
    if (blueLedDelay) blueLedDelay--;
    if (visionDelay) visionDelay--;
    interruptCounter = 0;
  }

}
Portenta_H7_Timer M7Timer(TIM7);
//====================================================
// END M7 CORE TIMER
//====================================================

//====================================================
// function prototypes
//====================================================
void setup(void);
void loop(void);
void RedLedMachine(void);
void BlueLedMachine(void);

void UltrasonicMachine(void);
double linearToRotational(double);
double voltagetoDistance(double);

void BatteryMachine(void);
int voltageToPercent(int);
//void ClampMachine(void);
void StepperSpeedMachine(void);
void WebSocketMachine(void);
void onMessageCallback(WebsocketsMessage);
void onEventsCallback(WebsocketsEvent, String);

void visionMachine(void);

void StepperMachine(void);
void receiveJson(void);
void sendJson(void);
void CommandToEnumState(void);
//long microsecondsToCentimeters(long);
void StepperPID(void);
void PID(void);
void motorPID(struct MotorEncoder *m);
void StateLEDMachine(void);
void EstopMachine(void);
void motorMachine(void);
void motorSpeedMachine(void);
void test(void);
void encoderMachine(void);

void incrementEncoder1(void);
void incrementEncoder2(void);
void incrementEncoder3(void);
void incrementEncoder4(void);

void changeDirection(void);
void accelerateMotors(void);
void decelerateMotors(void);
void stopMotors(void);
void spinMotors(double); // at the rate given by the tablet
void setMotorsForward(void);
void setMotorsBackward(void);
void updateSpeed(void);

//====================================================
// end function prototypes
//====================================================
//====================================================
// END TIMER AND FUNCTION PROTOTYPES
//====================================================




//====================================================
// MAIN
//====================================================
//====================================================
// setup
//====================================================
void setup() {

  // initialize m4 core
  //bootM4();
  // timer setup
  M7Timer.attachInterruptInterval(100, m7timer);
  // debug setup
  Serial.begin(115200);

  // turn red led on for initialization setup
  digitalWrite(RED_LED, LOW);
  // extra stepper pin

  // state led setup
  pinMode(STATE_LED_RED, OUTPUT);
  pinMode(STATE_LED_YELLOW, OUTPUT);
  pinMode(STATE_LED_GREEN, OUTPUT);
  // turn on all big leds
  digitalWrite(STATE_LED_RED, HIGH);
  digitalWrite(STATE_LED_YELLOW, HIGH);
  digitalWrite(STATE_LED_GREEN, HIGH);


  //done with steppers, timers, serial configs
  digitalWrite(STATE_LED_RED, LOW);

  // wifi setup
  while (!Serial && millis() < 2000);
  
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED){
    delay(500);
    WiFi.begin(ssid, password);
    Serial.println("trying to connect");
  }

  // wifi connected
  digitalWrite(RED_LED, HIGH);

  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
  Serial.println("Mac address: ");
  

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
  //pinMode(CLAMP_PIN, INPUT);

  // estop detect setup
  //pinMode(ESTOP_PIN, INPUT);
  //pinMode(D5, OUTPUT);
  delay(1000);

  // done with all setup
  digitalWrite(STATE_LED_GREEN, LOW);



  // motor setup
  motorStepPin1.period_us(100);
  motorStepPin1.pulsewidth_us(0);

  motorStepPin2.period_us(100);
  motorStepPin2.pulsewidth_us(0);

  motorStepPin3.period_us(100);
  motorStepPin3.pulsewidth_us(0);
  //motorStepPin4.period_us(100000);

  motorStepPin4.period_us(100);
  motorStepPin4.pulsewidth_us(0);


  pinMode(DirPin1, OUTPUT);
  pinMode(DirPin2, OUTPUT);
  pinMode(DirPin3, OUTPUT);
  pinMode(DirPin4, OUTPUT);

  digitalWrite(DirPin1, direction);
  digitalWrite(DirPin2, direction);
  digitalWrite(DirPin3, direction);
  digitalWrite(DirPin4, direction);  
  
  StateLEDState = READY;

  attachInterrupt(encoderPin1, incrementEncoder1, RISING);
  attachInterrupt(encoderPin2, incrementEncoder2, RISING);
  attachInterrupt(encoderPin3, incrementEncoder3, RISING);
  attachInterrupt(encoderPin4, incrementEncoder4, RISING);

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
  WebSocketMachine();
  UltrasonicMachine();
  BatteryMachine();
  //ClampMachine();
  StateLEDMachine();
  //EstopMachine();
  motorMachine();
  PID();
  encoderMachine();
  //test();
}
//====================================================
// end loop
//====================================================
//====================================================
// END MAIN
//====================================================




//====================================================
// FUNCTIONS
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
// blue led machine
//====================================================
void BlueLedMachine() {
  switch(BlueLedState) {
    case LED_OFF:
      if (!blueLedDelay) {
        blueLedDelay = 1;
        BlueLedState = LED_ON;
        digitalWrite(BLUE_LED, LOW);
        uptime = uptime + 1;
        //motorStepPin4.pulsewidth_us(2000);
      }
    break;
    case LED_ON:
      if (!blueLedDelay) {
        blueLedDelay = 1;
        BlueLedState = LED_OFF;
        digitalWrite(BLUE_LED, HIGH);
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
        //Serial.println("ultrasonic Value");
        //Serial.println(aveUltrasonicValue);
      }
    break;
    case UT_READING:
      if (!ultrasonicDelay) {
        

        // 1. double() to change analogRead() units
        // 2. times 3.1 to scale to max ADC voltage for Portenta
        // 3. divide by 4096 to convert from the ADC precision 
        // 4. times 5.0 to scale to original battery voltage - this comes from the voltage divider on the board
        // overall I think this gets us the battery voltage +/- 0.15
        ultrasonicValue = (double(analogRead(ULTRASONIC_PIN)) * 3.1 / 4096.0) * (5.0);
        //4.92v t0 160mm
        //2.84 to 100mm


        // check if the reading is any good, if its below a specific threshold it may be a bad value
        // if the value is less than 1 or greater than 6.0 its a bad value
        // this is about 50mm and 190mm
        if (ultrasonicValue > 1.0 && ultrasonicValue < 6.0) {
          // update dist, otherwise dont
          //leaky integrator gain of 0.5
          aveUltrasonicValue += (ultrasonicValue - aveUltrasonicValue) * 0.1; // updated the gain
          ultrasonicDistance = round(voltagetoDistance(aveUltrasonicValue));
        }


        Serial.println("ultrasonicValue");
        Serial.println(ultrasonicValue);
        ultrasonicDelay = 25; // 0.25 seconds
        UltrasonicState = UT_WAITING;
      }

    break;
    default:
    break;
  }

}


double voltagetoDistance(double ultrasonicVoltage) {
  
  double ultrasonicDist = 28.986 * ultrasonicVoltage + 17.389;
  
  return ultrasonicDist;


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

        // convert voltage to percent

        aveBatteryValue = 33.333*aveBatteryValue - 17.0;

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
//void ClampMachine(void){
//    switch(ClampState) {
//    case CLAMP_DISENGAGED:
//      if (digitalRead(CLAMP_PIN) == 1 && !clampDelay) {
//        ClampState = CLAMP_ENGAGED;
//        clampDelay = 5; // 1/10 seconds
//        clamped = true; // true is engaged
//      
//      }
//      
//    break;
//    case CLAMP_ENGAGED:
//      if (digitalRead(CLAMP_PIN) == 0 && !clampDelay) {
//        ClampState = CLAMP_DISENGAGED;
//        clampDelay = 5; // 1/10 seconds
//        clamped = false; // false is disengaged
//      }
//
//    break;
//    default:
//    break;
//  }
//}
//====================================================
// end clamp machine
//====================================================




//====================================================
// estop machine
//====================================================
//void EstopMachine(void){
//  switch(EstopState) {
//    case INACTIVE:
//      if (!estopDelay && !digitalRead(ESTOP_PIN)) { //estop pin high means there is power on the steppers, estop not pressed
//        estop = true; 
//        estopDelay = 2;
//        EstopState = ACTIVE;
//      }
//      
//    break;
//    case ACTIVE:
//      if (!estopDelay && digitalRead(ESTOP_PIN)) {
//        estop = false; 
//        estopDelay = 2;
//        EstopState = INACTIVE;
//      }
//
//    break;
//    default:
//    break;
//  }
//}
//====================================================
// end estop machine
//====================================================




//====================================================
// pid machine
//====================================================
void PID(void){
  // found that values of p=0.5, i=0.0, and d=0.0 work ok
  // found that values of P=0.05, i=0.0, d=0.0 work well when the reference is set to the zero glue ultrasonic reading
  if (!PIDDelay){
    PIDDelay = 1;
    initMeasurement = ultrasonicDistance;

    // compute error
    initError = targetGlueHeight - initMeasurement;
    // error == 160 - 150
    // error == 10=

    //Serial.println("init eror");
    //Serial.println(initError);

    // calculate the proportional term
    p = UT_Kp * initError;

    //Serial.println("p");
    //Serial.println(p);

    i = i + 0.5f * UT_Ki * T * (initError + prevError);

        //Serial.println("i");
    //Serial.println(i);

    i = constrain(i, -3.0, 3.0); // these limits are the max rotations per second

    d = -(2.0 * UT_Kd * (initMeasurement-prevMeasurement) + (2.0 * tau - T) * d) / (2.0 * tau + T);
    //Serial.println("d");
    //Serial.println(d);
    // sum control terms
    u = constrain(p + i + d, 0, 3.0); // between the rotations per sec of the motors // minus sign to invert the direction

  // pid runs all the time but only shows values and changes speed when enabled
  if (motorMode) {
    motorSpeed = u;

    // leaky intergrator
    //motorSpeed += (u - motorSpeed)*0.1;

    Serial.println("u: ");
    Serial.println(u);
    Serial.println("motorspeed");
    Serial.println(motorSpeed);

    Serial.println("UT_Kp");
    Serial.println(UT_Kp);
    Serial.println("UT_Ki");
    Serial.println(UT_Ki);
    Serial.println("UT_Kd");
    Serial.println(UT_Kd);
  }



  // store value for next iteration
  prevError = initError;
  prevMeasurement = initMeasurement;
  }
}
//====================================================
// end pid machine
//====================================================




//====================================================
// motor pid machine
//====================================================
void motorPID(struct MotorEncoder *m){
  // good values seem to be p=25 and i=10, d = 10
  // better values seem to be p=10, I=15, d = 0
  // better values seem to be p=25, i=15, d = 0

  //Serial.print("Error In function ");
  //Serial.println(m->initError);

  m->initMeasurement = m->encoderSpeed;
  // compute error // motor speed is the set point so constant // encoder speed is the measurement
  m->initError = motorSpeed - m->initMeasurement;

  // calculate the proportional term
  M_p = M_Kp * m->initError;

  M_i = M_i + 0.5f * M_Ki * M_T * (m->initError + m->prevError);

  M_i = constrain(M_i, -100.0, 100.0); // these limits are the duty cycle

  M_d = -(2.0 * M_Kd * (m->initMeasurement-m->prevMeasurement) + (2.0 * M_tau - M_T) * M_d) / (2.0 * M_tau + M_T);

  // sum control terms
  M_u = constrain(M_p + M_i + M_d, 0.0, 100.0);

  // store value for next iteration
  m->prevError = m->initError;
  m->prevMeasurement = m->initMeasurement;
  m->dutyCycle = M_u; // set duty cycle
}
//====================================================
// end motor pid machine
//====================================================





//====================================================
// encoder machine
//====================================================
void encoderMachine() {
  if (!encoderDelay){
    
    encoderDelay = 5; // 0.5 seconds

      // fraction of a rotation in 0.01 of a second  
      // the amount of rotation in that 0.01 of a second    
      // leaky integrator, gain of 0.1
      //M1.encoderSpeed += ( (double(encoderCount1) / double(encoderRotation)) / 0.5 - M1.encoderSpeed) * 0.75;
      M1.encoderSpeed += ((double(encoderCount1)/double(encoderRotation)) / 0.5 - M1.encoderSpeed)*0.9;

      // V = R*W
      //wheelSpeed = wheelDiameter * 0.5 * speed;                  // converted to linear velocity V = D * 1/2 * omega
      //M2.encoderSpeed = (double(encoderCount2)/double(encoderRotation)) / 0.5;
      //M3.encoderSpeed = (double(encoderCount3)/double(encoderRotation)) / 0.5;
      //M4.encoderSpeed = (double(encoderCount4)/double(encoderRotation)) / 0.5;

      M2.encoderSpeed += ((double(encoderCount2)/double(encoderRotation)) / 0.5 - M2.encoderSpeed)*0.9;
      M3.encoderSpeed += ((double(encoderCount3)/double(860)) / 0.5 - M3.encoderSpeed)*0.9;
      M4.encoderSpeed += ((double(encoderCount4)/double(encoderRotation)) / 0.5 - M4.encoderSpeed)*0.9;

      Serial.println("encoder counts");
      Serial.println(encoderCount1);
      Serial.println(encoderCount2);
      Serial.println(encoderCount3);
      Serial.println(encoderCount4);


      encoderCount1 = 0;
      encoderCount2 = 0;
      encoderCount3 = 0;
      encoderCount4 = 0;

    }
  else {
    
  }
}

//====================================================
// end encoder machine
//====================================================






//====================================================
// motor machine
//====================================================
void motorMachine() {

  switch(motorState){

    case MOTOR_STOPPED:
      //Serial.println("motor stopped");
      stopMotors();
      if(motorDirection == 1 || motorDirection == 2) {
        motorState = MOTOR_CHANGING_DIRECTION;
      }

    break;

    case MOTOR_RUNNING:

      if(motorDirection != currentMotorDirection) {
          currentMotorDirection = motorDirection;
          motorState = MOTOR_CHANGING_DIRECTION;
          stopMotors();
          motorDirectionDelay = 5; // 0.5 seconds
      }

      updateSpeed();

    break;

    case MOTOR_CHANGING_DIRECTION:

      if (motorDirection == 0) {
        motorState = MOTOR_STOPPED;
      }

      if (!motorDirectionDelay && motorDirection == 1) { 
        setMotorsForward();
        motorState = MOTOR_RUNNING;
      }

      if (!motorDirectionDelay && motorDirection == 2) {
        setMotorsBackward();
        motorState = MOTOR_RUNNING;
      }

      
      updateSpeed();


    break;


    case MOTOR_ERROR:
    break;

    default:
    break;
  }

}

void updateSpeed(void) { 
  if (!motorDelay) { // motor delay is the time between changes, motor duration is the number of those cycles
    motorDelay = 1; // run at 1 of the 10hz cycles
    motorPID(&M1);
    motorPID(&M2);
    motorPID(&M3);
    motorPID(&M4);
    measuredMotorSpeed = roundf((float(M1.encoderSpeed+M2.encoderSpeed+M3.encoderSpeed+M4.encoderSpeed)/4.0)*100)/100; //average motor speeds



  }

  motorStepPin1.pulsewidth_us(abs(M1.dutyCycle)); // limited to: 40 to 85 roughly
  motorStepPin2.pulsewidth_us(abs(M2.dutyCycle));
  motorStepPin3.pulsewidth_us(abs(M3.dutyCycle));
  motorStepPin4.pulsewidth_us(abs(M4.dutyCycle));

}


void stopMotors(void){
  lastMotorSpeed = motorSpeed;
  motorSpeed = 0;

  M1.dutyCycle = 0;
  M2.dutyCycle = 0;
  M3.dutyCycle = 0;
  M4.dutyCycle = 0; 

  motorStepPin4.pulsewidth_us(int(0)); // limited to: 40 to 85 roughly
  motorStepPin3.pulsewidth_us(int(0));
  motorStepPin2.pulsewidth_us(int(0));
  motorStepPin1.pulsewidth_us(int(0));
}

void setMotorsForward(void) {
  motorSpeed = lastMotorSpeed;
  digitalWrite(DirPin1, false);
  digitalWrite(DirPin2, false);
  digitalWrite(DirPin3, false);
  digitalWrite(DirPin4, false);  
}

void setMotorsBackward(void) {  
  motorSpeed = lastMotorSpeed;
  digitalWrite(DirPin1, true);
  digitalWrite(DirPin2, true);
  digitalWrite(DirPin3, true);
  digitalWrite(DirPin4, true);  
}


//====================================================
// end motor machine
//====================================================


//====================================================
// linear to rotational speed
//====================================================

double linearToRotational(double linear){
  return linear / (wheelDiameter * PI);
}

//====================================================
// end linear to rotational speed
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
        sendJson();
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
    receiveJson();
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
void sendJson(void) {
  jsonPacket.clear();
  jsonMessage = "";
  jsonPacket["wheelDiameter"] = wheelDiameter;
  jsonPacket["motorDirection"]  = motorDirection;
  jsonPacket["measuredMotorSpeed"]    = measuredMotorSpeed;
  jsonPacket["motorSpeed"]            = motorSpeed;
  jsonPacket["motorMode"]     = motorMode;
  jsonPacket["motorEnable"]   = motorEnable;

  jsonPacket["targetGlueHeight"]     = targetGlueHeight;
  jsonPacket["UT_Kp"]           = UT_Kp;
  jsonPacket["UT_Ki"]           = UT_Ki;
  jsonPacket["UT_Kd"]           = UT_Kd;

  jsonPacket["ultrasonicValue"] = ultrasonicDistance;
  jsonPacket["batteryLevel"]    = aveBatteryValue;

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
void receiveJson(void) {
    DeserializationError error = deserializeJson(jsonPacket, jsonMessage);
  // Test if parsing succeeds.
  if (error) {
    Serial.print(F("deserializeJson() failed: "));
    Serial.println(error.f_str());
  }

  wheelDiameter = double(jsonPacket["wheelDiameter"]);

  // motor stuff 
  // get and set the value to the local var and to the shared memory space
  motorDirection = int(jsonPacket["motorDirection"]);

  // get and set the value to the local var and to the shared memory space
  motorSpeed = linearToRotational(double(jsonPacket["motorSpeed"])); //motor speed is rotational
  
  M_Kp       = double(jsonPacket["M_Kp"]);
  M_Ki       = double(jsonPacket["M_Ki"]);
  M_Kd       = double(jsonPacket["M_Kd"]);

  Serial.println(motorSpeed);

  //targetMotorSpeed = double(jsonPacket["targetMotorSpeed"]);
  motorMode = bool(jsonPacket["motorMode"]);
  motorEnable = bool(jsonPacket["motorEnable"]);

  
  // get the pid values from the tablet
  targetGlueHeight = double(jsonPacket["targetGlueHeight"]);
  UT_Kp       = double(jsonPacket["UT_Kp"]);
  UT_Ki       = double(jsonPacket["UT_Ki"]);
  UT_Kd       = double(jsonPacket["UT_Kd"]);


  jsonPacket.clear();
  jsonMessage = "";
}
//====================================================
// end receive json machine
//====================================================




//====================================================
// ultrasonic machine
//====================================================
void visionMachine() {
  switch(visionState){
    case VISION_WAITING:
      if (!visionDelay) {
        visionState = VISION_READING;
      }
    break;
    case VISION_READING:
      if (!visionDelay) {
        
        trajectory = Serial.readString();

        if (trajectory!=""){
          digitalWrite(LED_BLUE, LOW);
          digitalWrite(LED_RED, LOW);
          digitalWrite(LED_GREEN, LOW);
        }

        else{
          digitalWrite(LED_BLUE, HIGH);
          digitalWrite(LED_RED,  HIGH);
          digitalWrite(LED_GREEN,HIGH);
        }

        visionDelay = 1; // 0.25 seconds
        visionState = VISION_WAITING;
      }

    break;
    default:
    break;
  }

}

//====================================================
// end ultrasonic machine
//====================================================





//====================================================
// state LED machine
//====================================================

//READY,               // green 1 second flash
//UNSAFE,              // red 1/2 sec flash
//WS_DOWN,             // red constant
//ACTIVE_PROCESS,      // yellow 1 second

void StateLEDMachine(void) {

  if(LastStateLEDState!=StateLEDState){
    LastStateLEDState = StateLEDState;
    digitalWrite(STATE_LED_GREEN, LOW);    
    digitalWrite(STATE_LED_YELLOW, LOW);
    digitalWrite(STATE_LED_RED, LOW);    
  }

  // removed || !clamed

  if (estop || !wsConnected) {
    StateLEDState = UNSAFE;
  }

  // removed && clamped
  if (motorMode && !wsConnected && !estop){
    StateLEDState = WS_DOWN;
  }
  
  if (motorMode && wsConnected && !estop) {
    StateLEDState = ACTIVE_PROCESS;
  }
  
  if (!motorMode && wsConnected && !estop){
    StateLEDState = READY;
  }

  switch(StateLEDState) {
    case READY: // green 1 second flash
      if(!stateLEDDelay){
      stateLEDFlag = !stateLEDFlag;
      stateLEDDelay = 10;
      digitalWrite(STATE_LED_GREEN, stateLEDFlag ? HIGH : LOW);
      Serial.println("READY");
      }
    break;

    case UNSAFE: // red 1/2 second flash
      if (!stateLEDDelay) {
        stateLEDFlag = !stateLEDFlag;
        stateLEDDelay = 2;
        digitalWrite(STATE_LED_RED, stateLEDFlag ? HIGH : LOW);
        Serial.println("UNSAFE");
      }
    break;

    case WS_DOWN: // red 1 second flash constant
      if (!stateLEDDelay) {
        stateLEDFlag = !stateLEDFlag;
        stateLEDDelay = 7;
        digitalWrite(STATE_LED_RED, stateLEDFlag ? HIGH : LOW);
        Serial.println("WS down");
      }
    break;

    case ACTIVE_PROCESS: // green and yellow flash
      if (!stateLEDDelay) {
        stateLEDFlag = !stateLEDFlag;
        stateLEDDelay = 5;
        digitalWrite(STATE_LED_GREEN, stateLEDFlag ? HIGH : LOW);
        digitalWrite(STATE_LED_YELLOW, !stateLEDFlag ? HIGH : LOW);
        Serial.println("active process");
      }
    break;

    default:
    break;
  }
}
//====================================================
// end state LED machine
//===================================================



















//====================================================
// END FUNCTIONS
//====================================================



