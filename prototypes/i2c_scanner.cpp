//this code is adapted for the I2C_1 port of the portenta breakout

#include <Arduino.h>
#include <Wire.h>
void setup() {

  Wire.begin();
  Serial.begin(115200);
  while (!Serial);  // wait for serial monitor
  Serial.println("\nI2C Scanner");
}

void loop() {
  int nDevices = 0;

  Serial.println("Scanning...");

  for (byte address = 1; address < 127; ++address) {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    Wire.beginTransmission(address);
    byte error = Wire.endTransmission();

    if (error == 0) {
      Serial.print("I2C device found at address 0x");  //0 = success
      if (address < 16) {
        Serial.print("0");
      }
      Serial.print(address, HEX);
      Serial.println("  !");

      ++nDevices;
    } else if (error == 4) {
      Serial.print("Unknown error at address 0x");
      if (address < 16) {
        Serial.print("0");
      }
      Serial.println(address, HEX);
    } else if (error == 1) {
      Serial.println("1: data too long to fit in transmit buffer.");
    } else if (error == 2) {
      Serial.println("2: received NACK on transmit of address.");
    } else if (error == 3) {
      Serial.println("3: received NACK on transmit of data.");
    } else if (error == 5) {
      Serial.println("5: timeout");
    }
  }
  if (nDevices == 0) {
    Serial.println("No I2C devices found\n");
  } else {
    Serial.println("done\n");
  }
  delay(5000);  // Wait 5 seconds for next scan
}