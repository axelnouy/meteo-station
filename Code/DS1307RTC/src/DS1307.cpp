#include <Wire.h>
#include "DS1307.h"
#include "Arduino.h"

void initializeRTC(int sdaPin, int sclPin) {
    Wire.begin(); // Initialize I2C with custom SDA and SCL pins
    //Wire.setClock(k_I2C_FREQUENCY);
    return; // Return 0 to indicate success
}

int getTimeFromRTC(MyTime_t* pTime) 
{
  uint8_t buffer[7];

  Wire.beginTransmission(k_I2C_DS1307_ADDRESS);
  Wire.write(0x00); // Set the register pointer to the seconds register
  Wire.endTransmission(); // End the transmission to prepare for reading
  Wire.requestFrom(k_I2C_DS1307_ADDRESS, 7); // Request 7 bytes
  Wire.readBytes(buffer, 7); // Read the bytes into the buffer
  pTime->second = bcd2dec(buffer[0] & 0x7F); // Read seconds, mask to ignore the CH bit
  pTime->minute = bcd2dec(buffer[1]);
  pTime->hour = bcd2dec(buffer[2] & 0x3F); // Read hour, mask to ignore the 24-hour format bit
  pTime->day = bcd2dec(buffer[4]); // Read day of the month
  pTime->month = bcd2dec(buffer[5]); // Read month
  pTime->year = bcd2dec(buffer[6]); // Read year

  return 0; // Success
}

int setTimeToRTC(MyTime_t tTime) 
{
  int error = 0;
  if (tTime.hour < 0 || tTime.hour > 23 || tTime.minute < 0 || tTime.minute > 59 || tTime.second < 0 || tTime.second > 59 ||
      tTime.year < 0 || tTime.year > 99 || tTime.month < 1 || tTime.month > 12 || tTime.day < 1 || tTime.day > 31) {
    error = -1; // Invalid time or date
    goto ERREUR; // Jump to error handling
  }
  uint8_t buffer[7];

  buffer[0] = dec2bcd(tTime.second); // Convert seconds to BCD
  //buffer[0] |= 0x80; // Set the CH bit to 1 to stop the oscillator
  buffer[1] = dec2bcd(tTime.minute); // Convert minutes to BCD
  buffer[2] = dec2bcd(tTime.hour); // Convert hours to BCD
  buffer[3] = 0; // Day of week (not used)
  buffer[4] = dec2bcd(tTime.day); // Day of month (1st)
  buffer[5] = dec2bcd(tTime.month); // Month (January)
  buffer[6] = dec2bcd(tTime.year); // Year (0 for 2000)

  Wire.beginTransmission(k_I2C_DS1307_ADDRESS);
  Wire.write(0x00); // Set the register pointer to the seconds register
  Wire.write(buffer, 7); // Write the buffer to the RTC
  error = Wire.endTransmission(); // Return the status of the transmission
  
ERREUR:

  return error;
}

// Convert Decimal to Binary Coded Decimal (BCD)
uint8_t dec2bcd(uint8_t num)
{ 
  return((num/10 * 16) + (num % 10));
}

// Convert Binary Coded Decimal (BCD) to Decimal
uint8_t bcd2dec(uint8_t num)
{
  return((num/16 * 10) + (num % 16));
}