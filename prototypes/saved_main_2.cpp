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
// included libraries
//====================================================
#include <Arduino.h>
#include <Vector.h>

//====================================================
// end included libraries
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
#define _WEBSOCKETS_LOGLEVEL_     3
#include <WebSockets2_Generic.h>
#include <WiFi.h>

#define WEBSOCKETS_PORT     8080
const char* ssid = "grover123456"; //Enter SSID
const char* password = "grover123456"; //Enter Password

using namespace websockets2_generic;
//====================================================
// end wifi and websockets definitions
//====================================================




//====================================================
// ip address definitions
//====================================================
// Since the esp is also on its own network... it has a local ip that isn't used for anything. 
IPAddress local(192, 168, 3, 1);
// throwing googles dns in for a temp fix
IPAddress testdns(8,8,8,8); 
// Since we are serving as an access point this is the address where the websockets will be posted
IPAddress gateway(192, 168, 3, 1);
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
void WebSocketMachine(void);
void pollAllClients(void);
void onMessageCallback(WebsocketsMessage);
void sendMessage(void);
//====================================================
// end function prototypes
//====================================================


//====================================================
// pin definitions
//====================================================
#define RED_LED     LEDR
#define GREEN_LED   LEDG
#define BLUE_LED    LEDB
//====================================================
// end pin definitions
//====================================================


//====================================================
// global variables
//====================================================
bool redLedFlag = false;
bool blueLedFlag = false;
volatile int redLedDelay = 0;
volatile int blueLedDelay = 0;

bool wsFlag = 0;
volatile int wsDelay = 0;


bool connected = false;

//====================================================
// end global variables
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

//====================================================
// end states
//====================================================





//====================================================
// objects
//====================================================
WebsocketsServer wsServer;
WebsocketsMessage wsMessage;
WebsocketsClient wsClient;

//Vector<WebsocketsClient> allClients;

//====================================================
// end objects
//====================================================



//====================================================
// timer
//====================================================
#include "Portenta_H7_TimerInterrupt.h"
volatile int interruptCounter;
void TimerHandler() { 
  interruptCounter++;

    // every 1/10 second
  if ((interruptCounter % 10000) == 0) {
        if (redLedDelay) redLedDelay--;
        if (wsDelay) wsDelay--;
  }

  // every second
  if ((interruptCounter % 100000) == 0) {
    if (blueLedDelay) blueLedDelay--;
    interruptCounter = 0;
  }

}
Portenta_H7_Timer ITimer(TIM16);
//====================================================
// end timer
//====================================================



//====================================================
// setup
//====================================================
void setup() {
  // timer setup
  ITimer.attachInterruptInterval(10, TimerHandler); // is thje timer messing with timeout of the websockets connection??????????
  // debug setup
  Serial.begin(115200);

  // wifi setup
  while (!Serial && millis() < 3000);
  WiFi.config(local, testdns, gateway, nmask);
  delay(500);
  WiFi.beginAP(ssid, password);
  while (WiFi.status() != WL_CONNECTED && millis() < 10000);

  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());

  // websocket setup
  wsServer.listen(WEBSOCKETS_PORT);
  Serial.println("websocket server listening...");



}
//====================================================
// end setup
//====================================================





//====================================================
// main loop
//====================================================


void loop() {

  // BlueLedMachine();
  //RedLedMachine();
  WebSocketMachine();


}
//====================================================
// end main loop
//====================================================





//====================================================
// functions
//====================================================
void BlueLedMachine() {
  switch(BlueLedState) {
    case LED_OFF:
      if (!blueLedDelay && !blueLedFlag) {
        //ledFlag = false;
        blueLedDelay = 1;
        BlueLedState = LED_ON;
        digitalWrite(BLUE_LED, LOW);
        //Serial.println("blue led on");
      }
    break;
    case LED_ON:
      if (!blueLedDelay && !blueLedFlag) {
        //ledFlag
        blueLedDelay = 1;
        BlueLedState = LED_OFF;
        digitalWrite(BLUE_LED, HIGH);
        //Serial.println("led off");
      }
    break;
    default:
    break;
  }
}

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





void WebSocketMachine() {

  //server.handleClient();

  WebsocketsClient client = wsServer.accept();

  if (client.available())
  {
    WebsocketsMessage msg = client.readNonBlocking();

    // log
    Serial.print("Got Message: ");
    Serial.println(msg.data());

    // return echo
    client.send("Echo: " + msg.data());

    // close the connection
    client.close();
  }
}

//====================================================
// end functions
//====================================================

/*  if (millis() > times + delays) {
    times = millis();
    Serial.println(millis()/1000.0);
    wsClient.send("Heart beat")
    wsClient.poll();



  }
  if (wsServer.poll()) {
    //Serial.println("Accepting a new client");
    WebsocketsClient wsClient = wsServer.accept();
    wsClient.onMessage(onMessageCallback);
    wsClient.send("Brooke is evil");
    wsClient.poll();
    //allClients.push_back(wsClient);
  }
  if ( wsClient.getCloseReason() != -1 ) {
    Serial.println(wsClient.getCloseReason());
    delay(10000);
  }*/