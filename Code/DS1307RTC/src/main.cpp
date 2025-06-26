#include <Arduino.h>

#include <Wire.h>

#include "DS1307.h"
#define k_I2C_SDA_PIN 21 // Default SDA pin for ESP32
#define k_I2C_SCL_PIN 22 // Default SCL pin for ESP32
MyTime_t currentTime;

void setup() {
  Serial.begin(9600);
  while (!Serial);
   // Initialize I2C with custom SDA and SCL pins

  Serial.println("beginning I2C communication...");
  initializeRTC(k_I2C_SDA_PIN, k_I2C_SCL_PIN);
  Serial.println("I2C communication started successfully!");
}

void loop() {
  
  if (getTimeFromRTC(&currentTime) == 0) {
    Serial.print("Current Time: ");
    Serial.print(currentTime.hour);
    Serial.print(":");
    Serial.print(currentTime.minute);
    Serial.print(":");
    Serial.println(currentTime.second);
  } else {
    Serial.println("Failed to read time from RTC.");
  }
  delay(1000); // Wait for a second before the next iteration
}

