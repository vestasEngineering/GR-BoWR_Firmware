
class MySerial
{

private:
  
public:
  // json packet allows us to use a 'dict' like structure in the form of "jsonPacket['prop'] = value"
  // StaticJsonDocument<1024> jsonPacket;
  // json message is a string that contains all the info smashed together

  enum States
  {
    READING,
    WAITING,
  };
  States state;

  volatile int delay;

  void setup(void)
  {
    // debug setup
    Serial.begin(115200);
  }

  // void serialStream(void){
  //
  //  if (Serial.available()){
  //
  //    char outgoingJsonString[512];
  //    String incomingJsonString = Serial.readStringUntil('\n');
  //
  //    deserializeJson(jsonPacket, incomingJsonString);
  //
  //    if (jsonPacket["msgtyp"] == "get"){
  //      // get messages
  //      jsonPacket["device"] = "h7";
  //      jsonPacket["wroteMotor0"] = motors.writeReceipts[0];
  //      jsonPacket["wroteMotor1"] = motors.writeReceipts[1];
  //      jsonPacket["wroteMotor2"] = motors.writeReceipts[2];
  //      jsonPacket["wroteMotor3"] = motors.writeReceipts[3];
  //      //jsonPacket["erefsMotor0"] = motors.erefValues[0];
  //      //jsonPacket["erefsMotor1"] = motors.erefValues[1];
  //      //jsonPacket["erefsMotor2"] = motors.erefValues[2];
  //      //jsonPacket["erefsMotor3"] = motors.erefValues[3];
  //      //jsonPacket["ultrasonic"] = ultrasonic.ultrasonicArrayValue[0];
  //
  //      //jsonPacket["thisValue"] = ultrasonic.ultrasonicDistance;
  //
  //    }
  //
  //    if(jsonPacket["msgtyp"] == "set"){
  //      motors.motorSpeeds[0] = jsonPacket["motorSpeed0"];
  //      motors.motorSpeeds[1] = jsonPacket["motorSpeed1"];
  //      motors.motorSpeeds[2] = jsonPacket["motorSpeed2"];
  //      motors.motorSpeeds[3] = jsonPacket["motorSpeed3"];
  //    }
  //
  //
  //    serializeJson(jsonPacket, outgoingJsonString);
  //    Serial.write(outgoingJsonString);
  //    Serial.write('\n');
  //
  //  }
  //
  //}
  //

  void reading()
  {

    String incomingJsonString = Serial.readStringUntil('\n');
  }
  void stateMachine()
  {
    switch (state)
    {
    case READING:
      if (!delay)
      {

        reading();
        delay = 1; // 1khz
        state = WAITING;
        motors.setMotorSpeed(5.5);
      }
      break;
    case WAITING:
      if (!delay)
      {
        delay = 1; // 1khz
        state = READING;
      }
      break;
    default:
      break;
    }
  }
};

MyPacket::distance = 5.5;