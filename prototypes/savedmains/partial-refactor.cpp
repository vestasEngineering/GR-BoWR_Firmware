//====================================================
// TITLE BLOCK
//====================================================
/*  
 *  Project: GR-LRR
 *  Author: Jacob Owens - Vestas Blades America
 *  Date: 07/23/2024
 */
//====================================================
// END TITLE BLOCK
//====================================================
//
//
//
//
//====================================================
// GENERAL LIBRARIES
//====================================================
#include <Arduino.h>
#include <mbed.h>
#include <math.h>
//====================================================
// END GENERAL LIBRARIES
//====================================================
//
//
//
//
//====================================================
// COMMUNICATION DEFINTIONS
//====================================================
#include <ArduinoJson.h>
// json packet allows us to use a 'dict' like structure in the form of "jsonPacket['prop'] = value"
StaticJsonDocument<512> jsonPacket;
// json message is a string that contains all the info smashed together
String jsonMessage = "";
//====================================================
// END COMMUNICATION DEFINTIONS
//====================================================
//
//
//
//
//====================================================
// ULTRASONIC SENSOR DEFINITIONS
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


enum UltrasonicStates {
  UT_WAITING,
  UT_READING
};
UltrasonicStates UltrasonicState;

//====================================================
// END ULTRASONIC SENSOR DEFINITIONS
//====================================================
//
//
//
//
//====================================================
// BATTERY MONITOR DEFINITIONS
//====================================================
#define BATTERY_PIN A0
double battery_value = 0;
volatile int batteryDelay = 0;

double aveBatteryValue = 0;
const int numBatterySamples = 20;
double batterySamples[numBatterySamples] = {0};
int batterySampleNumber = 0;

enum BatteryStates {
  BAT_WAITING,
  BAT_READING
};
BatteryStates BatteryState;

//====================================================
// END BATTERY MONITOR DEFINITIONS
//====================================================
//
//
//
//
//====================================================
// CLAMP DEFINITIONS
//====================================================
// #define CLAMP_PIN D5
bool clamped = false;
volatile int clampDelay = 0;



enum ClampStates {
  CLAMP_DISENGAGED,
  CLAMP_ENGAGED
};
ClampStates ClampState;

//====================================================
// END CLAMP DEFINITIONS
//====================================================
//
//
//
//
//====================================================
// ON BOARD LED DEFINITIONS
//====================================================
#define RED_LED     LEDR
#define GREEN_LED   LEDG
#define BLUE_LED    LEDB
bool redLedFlag = false;
bool blueLedFlag = false;
volatile int redLedDelay = 0;
volatile int blueLedDelay = 0;

enum LedStates {
  LED_OFF,
  LED_ON
};
LedStates RedLedState;
LedStates BlueLedState;

void RedLedMachine(void);
void BlueLedMachine(void);
//====================================================
// END ON BOARD LED DEFINITIONS
//====================================================
//
//
//
//
//====================================================
// STATE LED DEFINITIONS
//====================================================
#define STATE_LED_RED PE_3      // gpio 4
#define STATE_LED_YELLOW PG_3   // gpio 5
#define STATE_LED_GREEN PG_10   // gpio 6
volatile int stateLEDDelay = 0;
bool stateLEDFlag = false;
int stateLEDcount = 0;
bool stateLEDchange = false;

enum StateLEDStates {
  // ready for input, Unsafe / error, comms down, active process
  READY,               // green 1 second flash
  UNSAFE,              // red 1/2 sec flash
  WS_DOWN,             // red constant
  ACTIVE_PROCESS,      // yellow 1 second flash
};
StateLEDStates StateLEDState;
StateLEDStates LastStateLEDState;


//====================================================
// END STATE LED DEFINITIONS
//====================================================
//
//
//
//
//====================================================
// ESTOP DEFINITIONS
//====================================================
// #define ESTOP_PIN D6
bool estopFlag = 0; // 0 is false
volatile int estopDelay = 0;
bool estop = 0;

// this will be high, active if either estop button is pressed
enum EstopStates {
  INACTIVE,          // this means we have no detected an estop signal 0
  ACTIVE             // this means we have detected an estop signal 1
};
EstopStates EstopState;

//====================================================
// END ESTOP DEFINITIONS
//====================================================
//
//
//
//
//====================================================
// ENCODER DEFINITIONS
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
// END ENCODER DEFINITIONS
//====================================================
//
//
//
//
//====================================================
// TIMER DEFINITIONS
//====================================================
bool testState = false;
volatile int testDelay = 0;


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
    //if (motorDelay) motorDelay--;
    //if (motorDirectionDelay) motorDirectionDelay--;

    

  }

  // every 10,000/10,000 second - 1hz
  if ((interruptCounter % 10000) == 0) {
    if (blueLedDelay) blueLedDelay--;
    //if (visionDelay) visionDelay--;
    interruptCounter = 0;
  }

}
Portenta_H7_Timer M7Timer(TIM7);


//====================================================
// END TIMER DEFINITIONS
//====================================================
//
//
//
//
//====================================================
// BEGIN SETUP FUNCTION PROTOTYPES
//====================================================
void setup(void);
void loop(void);
//====================================================
// END SETUP FUNCTION PROTOTYPES
//====================================================
//
//
//
//
//====================================================
// BEGIN SETUP
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
// END SETUP
//====================================================
//
//
//
//
//====================================================
// BEGIN LOOP
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
// END LOOP
//====================================================







//====================================================
// BEGIN FUNCTIONS
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
//
//
//
//
//====================================================
// END FUNCTIONS
//====================================================



