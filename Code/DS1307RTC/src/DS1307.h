#ifndef DS1307_H
#define DS1307_H

#include <stdint.h>

#define k_I2C_DS1307_ADDRESS 0x68 // Example I2C address, change as needed
#define k_I2C_FREQUENCY 100000 // Default I2C frequency (100kHz)

struct MyTime_t{
  int second; // Seconds (0-59)
  int minute; // Minutes (0-59)
  int hour;   // Hours (0-23)
  int day;    // Day of the month (1-31)
  int month;  // Month (1-12)
  int year;   // Year (0-99, where 0 = 2000)
};

void initializeRTC(int sdaPin, int sclPin);

int getTimeFromRTC(MyTime_t* pTime);
int setTimeToRTC(MyTime_t tTime);

uint8_t bcd2dec(uint8_t num);
uint8_t dec2bcd(uint8_t num);

#endif // DS1307_H
