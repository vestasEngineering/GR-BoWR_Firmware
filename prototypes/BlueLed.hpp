#ifndef MY_BLUE_LED_CLASS
#define MY_BLUE_LED_CLASS

#include <Arduino.h>

#define BLUE_LED    LEDB

class BlueLed {
  public:
    volatile int cycleTime;
    volatile int delay = 0;
    // default constructor, set cycle to 1
    BlueLed(){
      cycleTime = 1;
    }

    BlueLed(int ct){
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
            digitalWrite(BLUE_LED, LOW);

          }
        break;
        case ON:
          if (!delay) {
            delay = cycleTime;
            state = OFF;
            digitalWrite(BLUE_LED, HIGH);
          }
        break;
        default:
        break;
      }
    }
};

#endif
