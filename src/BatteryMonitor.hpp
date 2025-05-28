#ifndef BATTERY_MONITOR
#define BATTERY_MONITOR

#include <Arduino.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

LiquidCrystal_I2C lcd(0,27, 16, 2);

void setup() {
    lcd.begin();
    lcd.backlight();
    lcd.setCuron(0,0);
    lcd.print("Battery:");
}

void loop() {
    float batteryVoltage = 18;
    lcd.setCursor(0,1);
}