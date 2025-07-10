#include <Arduino.h>

#include <LiquidCrystal.h>
#include <Wire.h>
#include "LoraMeteo.h"
#include "error.h"
#include "Battery.h"

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
float get_info_SHT21(char addr);
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

  if(InitLoraSensor() != ERROR_NONE)
  {
    Serial.println("Failed to initialize LoRa");
    while(1);
  }
}

void loop()
{
  static float rh = 0;
  static float st = 0;
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
  DataPacket.Temp = st; // Store temperature in DataPacket
  DataPacket.Hum = rh;  // Store humidity in DataPacket
  DataPacket.Pres = 0;  // Store pressure in DataPacket (not available)

  // Print the sensor data to the Serial Monitor
  Serial.print("Temperature: ");
  Serial.print(st);
  Serial.print(" C, Humidity: ");
  Serial.print(rh);
  Serial.println(" %");

  // Display the sensor data on the LCD
  display_sensor_SHT21(st, rh);
  Serial.print("Battery : ");
  Serial.println(getBatteryLevel());

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


float get_info_SHT21(char addr){
    uint16_t temp;
    uint8_t data[2];
    float result=0.0;

    //Request that sht21 transmits temperature data to processor
    Wire.beginTransmission(I2C_SHT21);
    Wire.write(addr);     //warns temperature sensor for future data collection
    Wire.endTransmission();
    delay(85);              //minimal delay for a good temperature transmission
    Wire.requestFrom(I2C_SHT21,3);       //gets 3 bytes of data from SHT21 sensor
    
    if(Wire.available()!=3)
    {
        return -1;
    }

    data[0] = Wire.read();  // read data (MSB)
    data[1] = Wire.read();  // read data (LSB)
    Wire.read();

    temp = (data[0] << 8);
    temp += data[1];
 
    temp &= ~0x0003;  // clean last two bits
    
    if(addr==(char)ADDR_RH){
        //result = -6.0 + 125.0*temp/pow(2.0,RES_RH);
        //result = -6.0 + 125.0*temp/RES_RHV2;
        //temp &= ~0x0003;  // clean last two bits
        result = -6.0 + 125.0/65536 * (float)temp; // return relative humidity
    }
    else if(addr==(char)ADDR_T){
        //result = -46.85 + 175.72*temp/RES_TV2;
        
        result = -46.85 + 175.72/65536 * (float)temp; // return relative humidity
    }
    else result = 42.0;

    return result;
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

