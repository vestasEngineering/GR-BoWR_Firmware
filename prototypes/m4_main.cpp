/////////====================================================
///////// TITLE BLOCK
/////////====================================================
////////*  
/////// *  Author: Jacob Owens - Avid Product Development
/////// *  Customer: Vestas
/////// *  Date: 08082023
/////// */
/////////====================================================
///////// END TITLE BLOCK
/////////====================================================
///////
///////
/////////====================================================
///////// INITIALIZATION AND VARIABLES
/////////====================================================
///////
///////
/////////====================================================
///////// shared data
/////////====================================================
///////struct shared_data {
///////  long motorSpeed = 0;    // speed from PID control
///////  long measuredMotorSpeed = 0;   // speed from encoders
///////  int motorDirection = 0; // stepper state : 0 = stopped, 1 = forward, 2 = backward
///////};
///////volatile struct shared_data * const xfr_ptr = (struct shared_data *)0x38001000;
/////////====================================================
///////// end shared data
/////////====================================================
///////
///////
/////////====================================================
///////// included libraries
/////////====================================================
#include <Arduino.h>
///////#include <math.h>
/////////====================================================
///////// end included libraries
/////////====================================================
///////
/////////====================================================
///////// timing debug
/////////====================================================
///////bool testState = false;
///////volatile int testDelay = 0;
/////////====================================================
///////// end timing debug
/////////====================================================
///////
///////
/////////====================================================
///////// motor definitions
/////////====================================================
///////#include "mbed.h"
///////const uint8_t pin_inv = D3;
/////////const uint8_t pin_D1 = D2;
///////volatile int motorDelay = 0;
///////using namespace mbed;
///////mbed::PwmOut motorStepPin(PJ_11);
///////
/////////const uint8_t Dir3Pin = LEDB + 1 + PD_4;  //gpio 2
///////
///////double wheelDiameter = 0.040; // 0.040 meters
///////double wheelSpeed = 0.0; // m/sa
///////
///////long motorSpeed = 0;    // speed from PID control
///////long measuredMotorSpeed = 0;   // speed from encoders
///////int motorDirection = 0; // stepper state : 0 = stopped, 1 = forward, 2 = backward
///////bool direction = LOW;
/////////====================================================
///////// end motor definitions
/////////====================================================
///////
/////////====================================================
///////// encoder definitions
/////////====================================================
///////const uint8_t encoderPin1 = D0;
///////const uint8_t encoderPin2 = D1;
///////volatile int encoderDelay = 0;
///////bool initEncoderState1 = false;
///////bool lastEncoderState1 = false;
///////bool initEncoderState2 = false;
///////bool lastEncoderState2 = false;
///////volatile int  encoderCount1 = 0;
///////volatile int  encoderCount2 = 0;
///////bool encoderFlag1 = 0;
///////bool encoderFlag2 = 0;
///////const int encoderRotation = 1120;
///////double rotations = 0;
///////double speed = 0.0;
///////int rotationDuration = 0;
///////unsigned long this_time = 0;
///////
///////
/////////====================================================
///////// encoder1 timer
/////////====================================================
///////void incrementEncoder1() { 
///////  encoderCount1++;  
///////}
/////////====================================================
///////// end encoder1 timer
/////////====================================================
///////
/////////====================================================
///////// encoder2 timer
/////////====================================================
///////void incrementEncoder2() { 
///////  encoderCount2++;
///////}
/////////====================================================
///////// end encoder2 timer
/////////====================================================
/////////====================================================
///////// end encoder definitions
/////////====================================================
///////
///////
/////////====================================================
///////// END INITIALIZATION AND VARIABLES
/////////====================================================
///////
///////
///////
///////
/////////====================================================
///////// TIMER AND FUNCTION PROTOTYPES
/////////====================================================
/////////====================================================
///////// function prototypes
/////////====================================================
///////void setup(void);
///////void loop(void);
///////void test(void);
///////void encoderMachine(void);
///////void motorMachine(void);
/////////====================================================
///////// end function prototypes
/////////====================================================
/////////====================================================
///////// m4 core timer
/////////====================================================
///////#include "Portenta_H7_TimerInterrupt.h"
///////volatile int interruptCounter = 0;
///////void m4timer() { 
///////  // every 1/10,000 second - 10,000hz - 0.0001 second
///////  interruptCounter++;
///////
///////    if(testDelay) testDelay--;
///////  // every 10/10,000 second - 1,000hz - 0.001 second
///////  if ((interruptCounter % 10) == 0) { 
///////    
///////  }
///////
///////  // every 100/10,000 second - 100hz - 0.01 second
///////  if ((interruptCounter % 100) == 0) { 
///////    
///////  }
///////
///////  // every 1,000/10,000 second - 10hz - 0.1 second
///////  if ((interruptCounter % 1000) == 0) { 
///////    
///////
///////  }
///////
///////  // every 10,000/10,000 second - 1hz
///////  if ((interruptCounter % 10000) == 0) {
///////    interruptCounter = 0;
///////  }
///////
///////}
///////Portenta_H7_Timer M4Timer(TIM14);
//====================================================
// end m4 core timer
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

  // timer setup
  ///M4Timer.attachInterruptInterval(100, m4timer); 
///
  ///pinMode(D6, OUTPUT);
///
///
  ///  // encoder setup
  ///pinMode(encoderPin1, INPUT);
  ///pinMode(encoderPin2, INPUT);
///
  /////attachInterrupt(encoderPin1, incrementEncoder1, RISING);
  /////attachInterrupt(encoderPin2, incrementEncoder2, RISING);
///
  ///motorStepPin.period_us(100);
  ///motorStepPin.pulsewidth_us(0);

  
}
//====================================================
// end setup
//====================================================
//====================================================
// loop
//====================================================
void loop() {

  //test();
  //encoderMachine();
  //motorMachine();

}
////====================================================
//// end loop
////====================================================
//
////====================================================
//// END MAIN
////====================================================
//
//
//
//
////====================================================
//// FUNCTIONS
////====================================================
//
////====================================================
//// debug test machine
////====================================================
//void test() {
//
//  if (!testDelay){
//    if (testState){
//      digitalWrite(D6, HIGH);
//    }
//    else {
//      digitalWrite(D6, LOW);  
//    }
//
//    testDelay = 1;  // sets the countdown timer to 100ms.  
//    testState = !testState;
//  }
//}
//
////====================================================
//// end debug test machine
////====================================================
//
////====================================================
//// encoder machine
////====================================================
//void encoderMachine() {
//  if (!encoderDelay){
//    
//    encoderDelay = 20; //counting 100 herts
//
//      rotations = (double(encoderCount1)/double(encoderRotation)); // fraction of a rotation in 0.01 of a second
//      speed = rotations / 0.01;                                     // the amount of rotation in that 0.01 of a second
//      // V = R*W
//      wheelSpeed = wheelDiameter * 0.5 * speed;                  // converted to linear velocity V = D * 1/2 * omega
//      //Serial.println("speed: r/s");                              // rotatations per second
//      //Serial.println(speed);
////
//      //Serial.println("Wheel speed");
//      //Serial.println(wheelSpeed);
////
//      //Serial.println("encoder1 count");
//      //Serial.println(encoderCount1);
//
//      encoderCount1 = 0;
//
//
//    }
//  else {
//    
//  }
//}
//
//====================================================
// end encoder machine
//====================================================


//====================================================
// motor machine
////====================================================
//void motorMachine() {
//  motorSpeed = xfr_ptr->motorSpeed;
//  motorDirection = xfr_ptr->motorDirection;
//  motorStepPin.pulsewidth_us(int(motorSpeed)); // limited to: 40 to 85 roughly
//  motorDirection ? digitalWrite(pin_inv, !direction) : digitalWrite(pin_inv, direction);
//  }
////====================================================
// end motor machine
//====================================================

//====================================================
// END FUNCTIONS
//====================================================