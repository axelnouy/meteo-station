#include <Arduino.h>

#define k_BATTERY_EMPTY_CHAR 0
#define k_BATTERY_25_CHAR 1
#define k_BATTERY_50_CHAR 2
#define k_BATTERY_75_CHAR 3
#define k_BATTERY_FULL_CHAR 4

#define Sensor 0

float convertBatteryLevelToVoltage(int batteryLevel);
int convertBatteryLevelToPourcentage(int batteryLevel);
#if Sensor
int getBatteryLevelRaw();
#endif


const byte batteryEmpty[8] = {
  B01110,
  B10001,
  B10001,
  B10001,
  B10001,
  B10001,
  B10001,
  B11111
};

const byte battery25[8] = {
  B01110,
  B10001,
  B10001,
  B10001,
  B10001,
  B11111,
  B11111,
  B11111
};

const byte battery50[8] = {
  B01110,
  B10001,
  B10001,
  B10001,
  B11111,
  B11111, 
  B11111,
  B11111
};

const byte battery75[8] = {
  B01110,
  B10001,
  B10001,
  B11111,
  B11111,
  B11111,
  B11111,
  B11111
};

const byte batteryFull[8] = {
  B01110,
  B11111,
  B11111,
  B11111,
  B11111,
  B11111,
  B11111,
  B11111
};
