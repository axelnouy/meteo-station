#include <Arduino.h>

#include <LiquidCrystal.h>
#include <Wire.h>
#include "BMP180.h"
#include "LoraMeteo.h"
#include "error.h"
#include "Battery.h"
#include "SHT21.h"

#define RS 2
#define EN 3
#define D4 4
#define D5 5
#define D6 6
#define D7 7

#define I2C_SHT21 0x40

#define ADDR_RH 0xF5   //address for relative humidity measurement, hold master
#define ADDR_T 0xF3    //address for temperature measurement, hold master

#define k_LED 14

LiquidCrystal lcd(RS, EN, D4, D5, D6, D7);


//function definition
void display_sensor_SHT21(float temp, float humi);

void setup() {
  // Initialize the built-in LED pin
  pinMode(k_LED, OUTPUT); // Initialize the built-in LED pin as an output

  // Initialize the Serial communication
  Serial.begin(9600);           // Start serial communication at 9600 baud rate

  // Initialize the LCD
  lcd.begin(8, 2);              // screen has 2 lignes and 8 columns

  lcd.createChar(k_BATTERY_EMPTY_CHAR, (byte*)batteryEmpty);
  lcd.createChar(k_BATTERY_25_CHAR, (byte*)battery25);
  lcd.createChar(k_BATTERY_50_CHAR, (byte*)battery50);
  lcd.createChar(k_BATTERY_75_CHAR, (byte*)battery75);
  lcd.createChar(k_BATTERY_FULL_CHAR, (byte*)batteryFull);


  lcd.clear();

  for (size_t i = 0; i < k_BATTERY_FULL_CHAR+1; i++)
  {
    lcd.setCursor(0, 0);
    lcd.write(byte(i));
    lcd.print("heehee");
    delay(500);
  }

  lcd.clear();
  delay(200);

  // Initialize I2C communication
  Wire.begin();

  if(bmp_get_cal_param() != ERROR_NONE)
  {
    Serial.println("Failed to initialize BMP180");
    while(1);
  }

  if(InitLoraSensor() != ERROR_NONE)
  {
    Serial.println("Failed to initialize LoRa");
    while(1);
  }
}

void loop()
{
  static int   FrameCount = 0;
  static float rh = 0;
  static float st = 0;
  static int32_t Pressure = 0;
  static float BatteryLevel = 0;
  static tDataPacket DataPacket;
  digitalWrite(k_LED, HIGH); // Turn the LED on
  Serial.println("LED is ON");     // Print message to serial monitor
  delay(2500);                     // Wait for 2.5 second
  digitalWrite(k_LED, LOW);  // Turn the LED off
  Serial.println("LED is OFF");    // Print message to serial monitor
  delay(2500);                     // Wait for 2.5 second

  // Read temperature and humidity from SHT21 sensor
  rh = get_info_SHT21(ADDR_RH);
  st = get_info_SHT21(ADDR_T);
  Pressure = compute_pressure(); // Read pressure from BMP180 sensor

  BatteryLevel = getBatteryLevelRaw(); // Read the battery level
  DataPacket.Temp = st; // Store temperature in DataPacket
  DataPacket.Hum = rh;  // Store humidity in DataPacket
  DataPacket.Pres = Pressure;  // Store pressure in DataPacket
  DataPacket.BatteryLevelRaw = BatteryLevel; // Store battery level in DataPacket

  // Print the sensor data to the Serial Monitor
  Serial.print("Temperature: ");
  Serial.print(st);
  Serial.print(" C, Humidity: ");
  Serial.print(rh);
  Serial.println(" %");
  Serial.print("Pressure: ");
  Serial.print(Pressure);
  Serial.println(" Pa");


  // Display the sensor data on the LCD
  switch (FrameCount)
  {
  case 1:
    display_sensor_SHT21(st, rh);
    break;
  case 2:
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Pressure:");
    lcd.setCursor(0, 1);
    lcd.print(Pressure / 100.0, 0); // Display pressure in hPa
    lcd.print("hPa");
    break;
  default:
    FrameCount = 0; // Reset FrameCount if it exceeds 2
    break;
  }
  FrameCount++;

  
  
  Serial.print("Battery : ");
  Serial.println(BatteryLevel);

  // Send the LoRa packet
  if (SendLoRaPacket(DataPacket) != ERROR_NONE)
  {
    Serial.println("Failed to send LoRa packet");
  }
  else
  {
    Serial.println("LoRa packet sent successfully");
  }
  
}



//display of informations from sensor SHT21
void display_sensor_SHT21(float temp, float humi){ 
    lcd.clear();
    lcd.setCursor(0,0); //display temperature on ligne 1
    lcd.print("T:");
    lcd.print(temp);
    lcd.print("C");
    lcd.setCursor(0,1); //display humidity on ligne 2
    lcd.print("H:");
    lcd.print(humi);
    lcd.print("%");
}

