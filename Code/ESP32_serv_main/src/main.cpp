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

TaskHandle_t TaskCore0;
TaskHandle_t TaskCore1;

pthread_mutex_t MutexLocalData;
pthread_mutex_t MutexSensorData;
pthread_mutex_t MutexTime;


MyTime_t currentTime;
tDataPacket dataPacket;
float TemperatureLocal = 0.0;
float HumidityLocal = 0.0;

const char* ssid     = "NothingAxel";
const char* password = "motdepasse";

// Set web server port number to 80
WebServer server(80);

// Global variables for sensor data
void RecoverDataTaskCore1(void* pvParameters);
void PrintDataTaskCore0(void* pvParameters);


void printLocalDataLCD(float temperature, float humidity);
void PrintSensorDataLCD(float Temperature, float Humidity, int Pressure);

void PrintTimeOnLCD(MyTime_t currentTime);
void PrintTemperatureOnLCD(float temperature);
void PrintHumidityOnLCD(float humidity);
void PrintServerInfoOnLCD();
void PrintBatteryLevelIconOnLCD(int batteryLevelRaw);

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

  lcd.begin(20, 4);
  lcd.createChar(k_BATTERY_EMPTY_CHAR, (byte*)batteryEmpty);
  lcd.createChar(k_BATTERY_25_CHAR, (byte*)battery25);
  lcd.createChar(k_BATTERY_50_CHAR, (byte*)battery50);
  lcd.createChar(k_BATTERY_75_CHAR, (byte*)battery75);
  lcd.createChar(k_BATTERY_FULL_CHAR, (byte*)batteryFull);
  lcd.print("hee hee");
  lcd.setCursor(0,1);
  lcd.print(WiFi.localIP().toString());

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
      server.handleClient();
    }
    else
    {
      Serial.print("trying to reconnect to WiFi...");
      WiFi.begin(ssid, password);
    }

    
    
  }
}



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
    error = ReceivePacket(&dataPacket);
    if (error != 0)
    {
      Serial.print("Error receiving packet: ");
      Serial.println(error);
    }
    pthread_mutex_unlock(&MutexSensorData);

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
  lcd.print("P ");
  lcd.print(Pressure);
  lcd.print(" hPa");
}


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


void PrintTemperatureOnLCD(float temperature)
{
  // Print the temperature on the LCD
  lcd.setCursor(0, 1);
  lcd.print("T ");
  lcd.print(temperature);
  lcd.print(" C");
}


void PrintHumidityOnLCD(float humidity)
{
  // Print the humidity on the LCD
  lcd.setCursor(0, 0);
  lcd.print("H ");
  lcd.print(humidity);
  lcd.print(" %");
}

void PrintBatteryLevelIconOnLCD(int batteryLevelRaw)
{
  // Print the battery level on the LCD
  lcd.setCursor(19, 1);
  lcd.write((byte)convertBatteryLevelToPourcentage(batteryLevelRaw));
}





















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

    setInterval(updateData, 2000); // Update every 2 seconds
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

void handleNotFound()
{  // Page Not found
  server.send(404, "text/plain","404: Not found");
}



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


void startWebServer() {
  server.begin();
  Serial.println("Web server started.");
}