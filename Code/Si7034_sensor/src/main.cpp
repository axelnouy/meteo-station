#include <Arduino.h>

#include <Wire.h>

#define k_SI7034_ADDRESS 0x70 // I2C address for SI7034
#define k__MEASURE_HOLD1 0x7C // Command for temperature measurement without hold master
#define k__MEASURE_HOLD2 0xA2 // Command for humidity measurement without hold master


void setup() {
  Wire.begin(); // Initialize I2C
  Serial.begin(9600);

  Serial.println("SI7034 I2C Test");
}

void loop() {
  delay(1000); // Wait for a second
  Wire.beginTransmission(k_SI7034_ADDRESS);

}

