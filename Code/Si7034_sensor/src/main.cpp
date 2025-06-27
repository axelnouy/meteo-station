#include <Arduino.h>
#include <Wire.h>
#include "SI7034.h"



void setup() {
  Wire.begin();
  Serial.begin(9600);
  Serial.println("SI7034 I2C Test");
}



void loop() {
  float temp = readTemperature();
  float hum = readHumidity();

  if (!isnan(temp) && !isnan(hum)) {
    Serial.print("Temperature: ");
    Serial.print(temp);
    Serial.println(" °C");
    Serial.print("Humidity: ");
    Serial.print(hum);
    Serial.println(" %RH");
  } else {
    Serial.println("Sensor read failed.");
  }

  delay(2000);
}
