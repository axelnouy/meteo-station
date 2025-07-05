#include <Arduino.h>

#include <Wire.h>
#include <LiquidCrystal.h>
#include "DS1307.h"
#include "SI7034.h"
#include "LoraMeteo.h"

#include <pthread.h>

#define RS 15
#define EN 2
#define D4 0
#define D5 4
#define D6 16
#define D7 17

#define ScreenWidth 20
#define ScreenHeight 4

LiquidCrystal lcd(RS, EN, D4, D5, D6, D7);

TaskHandle_t TaskCore0;
TaskHandle_t TaskCore1;

pthread_mutex_t MutexLocalData;
pthread_mutex_t MutexSensorData;
pthread_mutex_t MutexTime;


MyTime_t currentTime;
tDataPacket dataPacket;
float TemperatureLocal = 0.0;
float HumidityLocal = 0.0;


void printLocalDataLCD(float temperature, float humidity, MyTime_t currentTime);
void PrintSensorDataLCD(float Temperature, float Humidity, int Pressure, MyTime_t currentTime);
void RecoverDataTaskCore1(void* pvParameters);
void PrintDataTaskCore0(void* pvParameters);

void setup()
{
  Serial.begin(9600);
  Serial.print("setup() running on core ");
  Serial.println(xPortGetCoreID());
  

  
  Serial.println("Starting ESP32 LoRa Meteo Station...");
  Wire.begin(); // Initialize I2C communication
  Serial.println("I2C initialized.");

  if (initRTC() != 0)
  {
    Serial.println("Failed to initialize RTC.");
    return; // Exit setup if RTC initialization fails
  }

  if (getTimeFromRTC(&currentTime) == 0)
  {
    Serial.print("Current Time: ");
    Serial.print(currentTime.hour);
    Serial.print(":");
    Serial.print(currentTime.minute);
    Serial.print(":");
    Serial.println(currentTime.second);
  }
  else
  {
    Serial.println("Failed to read time from RTC.");
  }

  lcd.begin(20, 4);
  lcd.print("hee hee");
  delay(5000);
  xTaskCreatePinnedToCore(
                    PrintDataTaskCore0,   /* Task function. */
                    "TaskPrintData",     /* name of task. */
                    10000,       /* Stack size of task */
                    NULL,        /* parameter of the task */
                    1,           /* priority of the task */
                    &TaskCore0,      /* Task handle to keep track of created task */
                    0); // Run on core 0

  xTaskCreatePinnedToCore(
                    RecoverDataTaskCore1,   /* Task function. */
                    "TaskRecoverData",     /* name of task. */
                    10000,       /* Stack size of task */
                    NULL,        /* parameter of the task */
                    1,           /* priority of the task */
                    &TaskCore1,      /* Task handle to keep track of created task */
                    1); // Run on core 1

}

void loop()
{
  sleep(1000); // Main loop does nothing, tasks handle everything
  // This is to prevent the loop from running too fast and consuming unnecessary CPU resources
}



void PrintDataTaskCore0(void * pvParameters)
{
  Serial.println("Task1code running on core ");
  Serial.println(xPortGetCoreID());
  int countFrame = 0;
  
  while (true) {
    delay(1000);
    if(countFrame <= 10) 
    {
      pthread_mutex_lock(&MutexTime);
      pthread_mutex_lock(&MutexLocalData);
      printLocalDataLCD(TemperatureLocal, HumidityLocal, currentTime);
      pthread_mutex_unlock(&MutexLocalData);
      pthread_mutex_unlock(&MutexTime);
    }
    else if(countFrame <= 20)
    {
      pthread_mutex_lock(&MutexSensorData);
      pthread_mutex_lock(&MutexTime);
      PrintSensorDataLCD(dataPacket.Temp, dataPacket.Hum, dataPacket.Pres, currentTime);
      pthread_mutex_unlock(&MutexSensorData);
      pthread_mutex_unlock(&MutexTime);

    }
    else
    {
      countFrame = 0; // Reset the frame count
    }
    countFrame++;
  }
}



void RecoverDataTaskCore1(void *pvParameters)
{
  int PacketSize = 0;
  Serial.println("RecoverDataTask running on core ");
  Serial.println(xPortGetCoreID());
  while (true)
  {
    delay(100);
    pthread_mutex_lock(&MutexLocalData);
    // Read local sensor data
    readSi7034Data(&TemperatureLocal, &HumidityLocal);
    pthread_mutex_unlock(&MutexLocalData);
    
    // Check if a packet is available
    PacketSize = PacketAvailable();
    if(PacketSize!=0)
    {
      pthread_mutex_lock(&MutexSensorData);
      if(ReceivePacket(&dataPacket) != 0)
      {
        Serial.println("Error receiving packet");
      }
      pthread_mutex_unlock(&MutexSensorData);
    }

    pthread_mutex_lock(&MutexTime);
    getTimeFromRTC(&currentTime);
    pthread_mutex_unlock(&MutexTime);

    Serial.print("Temperature: ");
    Serial.print(TemperatureLocal);
    Serial.println(" °C");
    Serial.print("Humidity: ");
    Serial.print(HumidityLocal);
    Serial.println(" %RH");
    Serial.print("Current Time: ");
    Serial.print(currentTime.hour);
    Serial.print(":");
    Serial.print(currentTime.minute);
    Serial.print(":");
    Serial.println(currentTime.second);
  }
}





//-----------------------------------------------------------------------------
// Function to print local data on the LCD
//-----------------------------------------------------------------------------
// parameters:
// - Temperature: float value of temperature in Celsius
// - Humidity: float value of humidity in percentage
// - currentTime: MyTime_t structure containing the current time
//-----------------------------------------------------------------------------
void printLocalDataLCD(float temperature, float humidity, MyTime_t currentTime)
{
  // Clear the LCD and print the local data

  // print temperature
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Temp: ");
  lcd.print(temperature);
  lcd.print(" C");
  
  // print humidity
  lcd.setCursor(0, 1);
  lcd.print("Hum: ");
  lcd.print(humidity);
  lcd.print(" %");

  // print time
  lcd.setCursor(0, 2);
  lcd.print("Time: ");
  lcd.print(currentTime.hour);
  lcd.print(":");
  if (currentTime.minute < 10) {
    lcd.print("0");
  }
  lcd.print(currentTime.minute);
  lcd.print(":");
  if (currentTime.second < 10) {
    lcd.print("0");
  }
  lcd.print(currentTime.second);
}

//-----------------------------------------------------------------------------
// Function to print sensor data on the LCD
//-----------------------------------------------------------------------------
// parameters:
// - Temperature: float value of temperature in Celsius
// - Humidity: float value of humidity in percentage
// - Pressure: int value of pressure in hPa
// - currentTime: MyTime_t structure containing the current time
//-----------------------------------------------------------------------------
void PrintSensorDataLCD(float Temperature, float Humidity, int Pressure, MyTime_t currentTime)
{
  // Clear the LCD and print the sensor data

  // print temperature
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Temp: ");
  lcd.print(Temperature);
  lcd.print(" C");
  
  // print humidity
  lcd.setCursor(0, 1);
  lcd.print("Hum: ");
  lcd.print(Humidity);
  lcd.print(" %");

  // print pressure
  lcd.setCursor(0, 2);
  lcd.print("Pres: ");
  lcd.print(Pressure);
  lcd.print(" hPa");

  // print time in the left down corner (last row, last possible position for 20x4 LCD)
  lcd.setCursor(13, 3);
  lcd.print(currentTime.hour);
  lcd.print(":");
  if (currentTime.minute < 10) {
    lcd.print("0");
  }
  lcd.print(currentTime.minute);
  lcd.print(":");
  if (currentTime.second < 10) {
    lcd.print("0");
  }
  lcd.print(currentTime.second);
}