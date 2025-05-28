#ifndef MY_RED_LED_CLASS
#define MY_RED_LED_CLASS

#include <Arduino.h>

#define RED_LED     LEDR

class RedLed {
  public:
    volatile int cycleTime;
    volatile int delay = 0;
    // default constructor, set cycle to 1
    RedLed(){
      cycleTime = 1;
    }

    RedLed(int ct){
      cycleTime = ct;
    }


    enum States {
      OFF,
      ON
    };
    States state;


    void stateMachine(void) {
      switch(state) {
        case OFF:
          if (!delay) {
            delay = cycleTime;
            state = ON;
            digitalWrite(RED_LED, LOW);

          }
        break;
        case ON:
          if (!delay) {
            delay = cycleTime;
            state = OFF;
            digitalWrite(RED_LED, HIGH);
          }
        break;
        default:
        break;
      }
    }
};
#endif