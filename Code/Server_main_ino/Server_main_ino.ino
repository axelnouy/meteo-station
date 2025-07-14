#include <Arduino.h>

#include <Wire.h>
#include <LiquidCrystal.h>
#include <WiFi.h>
#include <WebServer.h>
#include <pthread.h>
#include "time.h"


#include "DS1307.h"
#include "SI7034.h"
#include "LoraMeteo.h"
#include "error.h"
#include "Battery.h"



#define RS 15
#define EN 2
#define D4 0
#define D5 4
#define D6 16
#define D7 17

#define ScreenWidth 20
#define ScreenHeight 4

#define SizeOfTimeMessage 8

LiquidCrystal lcd(RS, EN, D4, D5, D6, D7);

// Task handles for FreeRTOS
TaskHandle_t TaskCore0;
TaskHandle_t TaskCore1;

// Mutexes for thread safety
pthread_mutex_t MutexLocalData;
pthread_mutex_t MutexSensorData;
pthread_mutex_t MutexTime;

// Shared data structures
MyTime_t currentTime;
tDataPacket dataPacket;

// Local sensor data
float TemperatureLocal = 0.0;
float HumidityLocal = 0.0;

// WiFi credentials
// Replace with your network credentials
const char* ssid     = "Livebox-72C0";
const char* password = "QmGt53gsmqRTfzJj49";

// NTP server configuration
const char* ntpServer = "pool.ntp.org";
const long  gmtOffset_sec = 3600;
const int   daylightOffset_sec = 3600;


// Set web server port number to 80
WebServer server(80);

// task execution functions
void RecoverDataTaskCore1(void* pvParameters);
void PrintDataTaskCore0(void* pvParameters);


// LCD print Functions
void printLocalDataLCD(float temperature, float humidity);
void PrintSensorDataLCD(float Temperature, float Humidity, int Pressure);

void PrintTimeOnLCD(MyTime_t currentTime);
void PrintTemperatureOnLCD(float temperature);
void PrintHumidityOnLCD(float humidity);
void PrintServerInfoOnLCD();
void PrintBatteryLevelIconOnLCD(int batteryLevelRaw);

// Web server functions
void handleRoot();
void handleNotFound();
void startWebServer();
void initWebServer();

void setup()
{
  Serial.begin(9600);
  Serial.print("setup() running on core ");
  Serial.println(xPortGetCoreID());
  
  Serial.println("Starting ESP32 LoRa Meteo Station...");
  Wire.begin(); // Initialize I2C communication
  Serial.println("I2C initialized.");

  // init Lora 
  if (InitLoraServer() != ERROR_NONE)
  {
    Serial.println("Failed to initialize LoRa.");
    return; // Exit setup if LoRa initialization fails
  }

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


  //connecting to WiFi
  WiFi.begin(ssid, password);

  // Print local IP address and start web server
  Serial.println("");
  Serial.println("WiFi connected.");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
  initWebServer();
  startWebServer();

  //inittialize LCD
  lcd.begin(20, 4);
  // initialize LCD with custom characters
  lcd.createChar(k_BATTERY_EMPTY_CHAR, (byte*)batteryEmpty);
  lcd.createChar(k_BATTERY_25_CHAR, (byte*)battery25);
  lcd.createChar(k_BATTERY_50_CHAR, (byte*)battery50);
  lcd.createChar(k_BATTERY_75_CHAR, (byte*)battery75);
  lcd.createChar(k_BATTERY_FULL_CHAR, (byte*)batteryFull);
  lcd.createChar(k_BATTERY_CHARGING_CHAR, (byte*)batteryCharging);

  delay(1000);

  // Create tasks
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


//-----------------------------------------------------------------------------
// Task to print data on the LCD
// This task runs on core 0 and prints local data, sensor data, and server info on the LCD
//-----------------------------------------------------------------------------
void PrintDataTaskCore0(void * pvParameters)
{
  Serial.println("Task1code running on core ");
  Serial.println(xPortGetCoreID());
  int countFrame = 0;
  
  while (true) {
    delay(1000);
    if(countFrame < 10) 
    {
      pthread_mutex_lock(&MutexLocalData);
      printLocalDataLCD(TemperatureLocal, HumidityLocal);
      pthread_mutex_unlock(&MutexLocalData);
    }
    else if(countFrame < 20) 
    {
      pthread_mutex_lock(&MutexSensorData);
      PrintSensorDataLCD(dataPacket.Temp, dataPacket.Hum, dataPacket.Pres);
      PrintBatteryLevelIconOnLCD(dataPacket.BatteryLevelRaw);
      pthread_mutex_unlock(&MutexSensorData);
    }
    else if(WiFi.status() == WL_CONNECTED)
    {
      PrintServerInfoOnLCD();
    }
    if(countFrame >= 25)
    {
      countFrame = 0; // Reset the frame count
    }
    pthread_mutex_lock(&MutexTime);
    PrintTimeOnLCD(currentTime);
    pthread_mutex_unlock(&MutexTime);
    countFrame++;

    
    if (WiFi.status() == WL_CONNECTED) 
    {
      struct tm timeinfo;
      // Handle web server requests
      server.handleClient();
      // Synchronize time with NTP server
      configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
      if(!getLocalTime(&timeinfo))
      {
        Serial.println("Failed to obtain time");
      }
      timeinfo.tm_year += 1900; // Adjust year
      timeinfo.tm_mon += 1; // Adjust month
      Serial.printf("Current time: %04d-%02d-%02d %02d:%02d:%02d\n", 
                    timeinfo.tm_year, timeinfo.tm_mon, timeinfo.tm_mday, 
                    timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);
      // Update RTC with the current time
      pthread_mutex_lock(&MutexTime);
      currentTime.hour = timeinfo.tm_hour;
      currentTime.minute = timeinfo.tm_min;
      currentTime.second = timeinfo.tm_sec;
      currentTime.day = timeinfo.tm_mday;
      currentTime.month = timeinfo.tm_mon + 1; // tm_mon is 0-11
      currentTime.year = timeinfo.tm_year - 2000; // tm_year is years since 1900

      setTimeToRTC(currentTime);
      pthread_mutex_unlock(&MutexTime);
    }
    else
    {
      // If WiFi is disconnected, try to reconnect
      Serial.print("trying to reconnect to WiFi...");
      WiFi.begin(ssid, password);
    }

    
    
  }
}


//-----------------------------------------------------------------------------
// Task to recover data from LoRa and update local sensor data
// This task runs on core 1 and reads sensor data from LoRa packets
//-----------------------------------------------------------------------------
void RecoverDataTaskCore1(void *pvParameters)
{
  int LoraPacketAvailable = 0;
  int error = 0;
  Serial.println("RecoverDataTask running on core ");
  Serial.println(xPortGetCoreID());
  while (true)
  {
    delay(100);
    pthread_mutex_lock(&MutexLocalData);
    // Read local sensor data
    readSi7034Data(&TemperatureLocal, &HumidityLocal);
    pthread_mutex_unlock(&MutexLocalData);

    pthread_mutex_lock(&MutexSensorData);
    // Check if a LoRa packet is available
    error = ReceivePacket(&dataPacket);
    if (error != 0)
    {
      Serial.print("Error receiving packet: ");
      Serial.println(error);
    }
    Serial.print("Received packet: ");
    Serial.print("Temp: ");
    Serial.print(dataPacket.Temp);
    Serial.print(" Hum: ");
    Serial.print(dataPacket.Hum);
    Serial.print(" Pres: ");
    Serial.print(dataPacket.Pres);
    pthread_mutex_unlock(&MutexSensorData);

    // Update the current time from RTC
    pthread_mutex_lock(&MutexTime);
    getTimeFromRTC(&currentTime);
    pthread_mutex_unlock(&MutexTime);
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
void printLocalDataLCD(float Temperature, float Humidity)
{
  // Clear the LCD and print the local data

  // print temperature
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Local Data");
  PrintTemperatureOnLCD(Temperature);
  PrintHumidityOnLCD(Humidity);
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
void PrintSensorDataLCD(float Temperature, float Humidity, int Pressure)
{
  // Clear the LCD and print the sensor data

  // print temperature
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Sensor Data");
  PrintTemperatureOnLCD(Temperature);
  
  // print humidity
  PrintHumidityOnLCD(Humidity);

  // print pressure
  lcd.setCursor(0, 3);
  lcd.print(Pressure/100.0);
  lcd.print("hPa");
}

//-----------------------------------------------------------------------------
// Function to print server information on the LCD
//-----------------------------------------------------------------------------
void PrintServerInfoOnLCD()
{
  // Clear the LCD and print the server information
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("IP: ");
  lcd.print(WiFi.localIP().toString());
  
  lcd.setCursor(0, 1);
  lcd.print("SSID: ");
  lcd.print(ssid);
  
  lcd.setCursor(0, 2);
  lcd.print("Status: ");
  
  if (WiFi.status() == WL_CONNECTED) {
    lcd.print("Connected");
  } else {
    lcd.print("Disconnected");
  }
}


//-----------------------------------------------------------------------------
// Function to print Time on the LCD
//-----------------------------------------------------------------------------
// parameters:
// - currentTime: MyTime_t structure containing the current time
//-----------------------------------------------------------------------------
void PrintTimeOnLCD(MyTime_t currentTime)
{
  // Print the current time on the LCD
  lcd.setCursor(ScreenWidth-SizeOfTimeMessage, 3);
  if (currentTime.hour < 10) {
    lcd.print("0");
  }
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
// Function to print Temperature on the LCD
//-----------------------------------------------------------------------------
// parameters:
// - temperature: float value of temperature in Celsius
//-----------------------------------------------------------------------------
void PrintTemperatureOnLCD(float temperature)
{
  // Print the temperature on the LCD
  lcd.setCursor(0, 1);
  lcd.print(temperature);
  lcd.print(" C");
}

//-----------------------------------------------------------------------------
// Function to print Humidity on the LCD
//-----------------------------------------------------------------------------
// parameters:
// - humidity: float value of humidity in percentage
//-----------------------------------------------------------------------------
void PrintHumidityOnLCD(float humidity)
{
  // Print the humidity on the LCD
  lcd.setCursor(0, 2);
  lcd.print(humidity);
  lcd.print(" %");
}

//-----------------------------------------------------------------------------
// Function to print Battery Level Icon on the LCD
//-----------------------------------------------------------------------------
// parameters:
// - batteryLevelRaw: int value of the raw battery level
//-----------------------------------------------------------------------------
// This function converts the raw battery level to a percentage and prints it on the LCD
// It uses the convertBatteryLevelToPourcentage function to get the appropriate icon
// and displays it on the LCD at the specified position.
//-----------------------------------------------------------------------------

void PrintBatteryLevelIconOnLCD(int batteryLevelRaw)
{
  // Print the battery level on the LCD    
  lcd.setCursor(14, 0); // Set cursor to the last column of the first row
  lcd.print(convertBatteryLevelToVoltage(batteryLevelRaw), 2); // Print the voltage with 2 decimal places
  lcd.print("V ");
  lcd.setCursor(19, 0);
  lcd.write((byte)convertBatteryLevelToPourcentage(batteryLevelRaw));
}
















//-----------------------------------------------------------------------------
// Web Server Functions
//-----------------------------------------------------------------------------


//-----------------------------------------------------------------------------
// Function to handle the root path
// This function serves the main HTML page of the web server
//-----------------------------------------------------------------------------
void handleRoot() {
  String page = R"rawliteral(
<!DOCTYPE html>
<html lang="fr">
<head>
  <meta charset="UTF-8">
  <title>ESP32 Server</title>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <style>
    body { font-family: Arial, sans-serif; margin: 40px; background-color: #f7f7f7; color: #333; }
    h1 { color: #007BFF; }
    p { font-size: 1.2em; }
    .label { font-weight: bold; }

    .main {
  display: flex;
  align-items: center;
  justify-content: center;
}

.btn {
  width: 170px;
  height: 60px;
  font-size: 18px;
  background: #fff;
  border: none;
  border-radius: 50px;
  color: #000;
  outline: none;
  cursor: pointer;
  transition: all 0.4s;
}

.btn:hover {
  box-shadow: inset 0 0 0 4px #ef476f, 
              inset 0 0 0 8px #ffd166, 
              inset 0 0 0 12px #06d6a0,
              inset 0 0 0 16px #118ab2;
  background: #073b4c;
  color: #fff;
}
  </style>
  <script>
    function updateData() {
      fetch('/data')
        .then(response => response.json())
        .then(data => {
          document.getElementById('temperature').innerText = data.temperature + " °C";
          document.getElementById('humidity').innerText = data.humidity + " %";
          document.getElementById('sensor_temperature').innerText = data.sensor_temperature + " °C";
          document.getElementById('sensor_humidity').innerText = data.sensor_humidity + " %";
          document.getElementById('sensor_pressure').innerText = data.sensor_pressure + " hPa";
        })
        .catch(err => {
          console.error("Error fetching data:", err);
        });
    }

    setInterval(updateData, 5000); // Update every 5 seconds
    window.onload = updateData;
  </script>
</head>
<body>
  <h1>Station Météo ESP32</h1>

  <h2>Informations locales</h2>
  <p><span class="label">Température locale:</span> <span id="temperature">...</span></p>
  <p><span class="label">Humidité locale:</span> <span id="humidity">...</span></p>
  <h2>Informations du capteur distant</h2>
  <p><span class="label">Température distante:</span> <span id="sensor_temperature">...</span></p>
  <p><span class="label">Humidité distante:</span> <span id="sensor_humidity">...</span></p>
  <p><span class="label">Pression:</span> <span id="sensor_pressure">...</span></p>
    <button class="btn">Hover Me</button>
</body>
</html>
)rawliteral";

  server.send(200, "text/html", page);
}


//-----------------------------------------------------------------------------
// Function to handle 404 Not Found errors
// This function is called when a requested page is not found
//-----------------------------------------------------------------------------
void handleNotFound()
{  // Page Not found
  server.send(404, "text/plain","404: Not found");
}


//-----------------------------------------------------------------------------
// Function to initialize the web server
// This function sets up the routes and handlers for the web server
//-----------------------------------------------------------------------------
void initWebServer() {
  server.on("/", handleRoot);  // Chargement de la page d'accueil
  server.on("/data", []()
  {
  float temperature = 0.0;
  float humidity = 0.0;
  float SensorTemperature = 0.0;
  float SensorHumidity = 0.0;
  int SensorPressure = 0;

  pthread_mutex_lock(&MutexLocalData);
  temperature = TemperatureLocal;
  humidity = HumidityLocal;
  pthread_mutex_unlock(&MutexLocalData);

  pthread_mutex_lock(&MutexSensorData);
  SensorTemperature = dataPacket.Temp;
  SensorHumidity = dataPacket.Hum;
  SensorPressure = dataPacket.Pres;
  pthread_mutex_unlock(&MutexSensorData);

  String json = "{";
  json += "\"temperature\": " + String(temperature, 1) + ",";
  json += "\"humidity\": " + String(humidity, 1) + ",";
  json += "\"sensor_temperature\": " + String(SensorTemperature, 1) + ",";
  json += "\"sensor_humidity\": " + String(SensorHumidity, 1) + ",";
  json += "\"sensor_pressure\": " + String(SensorPressure);
  json += "}";

  server.send(200, "application/json", json); 
  });

  server.onNotFound(handleNotFound);
}

//-----------------------------------------------------------------------------
// Function to start the web server
//-----------------------------------------------------------------------------
void startWebServer() {
  server.begin();
  Serial.println("Web server started.");
}