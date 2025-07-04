#include "SI7034.h"

#include <Wire.h>


/*float readTemperature() 
{
  Wire.beginTransmission(SI7034_ADDR);
  Wire.write(CMD_MEASURE_TEMP_NOHOLD);
  Wire.endTransmission();

  delay(25); // Attente du résultat (datasheet: typ 22ms)

  Wire.requestFrom(SI7034_ADDR, 3); // 2 data bytes + 1 CRC
  if (Wire.available() == 3) {
    uint8_t msb = Wire.read();
    uint8_t lsb = Wire.read();
    Wire.read(); // CRC (non utilisé ici)
    uint16_t rawTemp = (msb << 8) | lsb;
    return ((175.0 * rawTemp) / 65536.0) - 45.0;
  }
  return NAN;
}*/

/*float readHumidity() 
{
  Wire.beginTransmission(SI7034_ADDR);
  Wire.write(CMD_MEASURE_HUM_NOHOLD);
  Wire.endTransmission();

  delay(25); // Attente du résultat

  Wire.requestFrom(SI7034_ADDR, 3);
  if (Wire.available() == 3) {
    uint8_t msb = Wire.read();
    uint8_t lsb = Wire.read();
    Wire.read(); // CRC
    uint16_t rawHum = (msb << 8) | lsb;
    return ((100.0 * rawHum) / 65536.0);
  }
  return NAN;
}*/


int readSi7034Data(float* pTemperature, float* pHumidity) 
{
  uint8_t Msb = 0;
  uint8_t Lsb = 0;
  uint16_t Raw = 0;
  Wire.beginTransmission(SI7034_ADDR);
  Wire.write(CMD_MEASURE_TEMP_HUMIDITY_NO_HOLD1);
  Wire.write(CMD_MEASURE_TEMP_HUMIDITY_NO_HOLD2);
  Wire.endTransmission();
  
  delay(80); // Attente du résultat
  Wire.requestFrom(SI7034_ADDR, 6);

  if (Wire.available() == 6) {
    Msb = Wire.read();
    Lsb = Wire.read();
    Wire.read(); // CRC
    Raw = (Msb << 8) | Lsb;
    *pTemperature = ((175.0 * Raw) / 65536.0) - 45.0;
    Msb = Wire.read();
    Lsb = Wire.read();
    Wire.read(); // CRC
    *pHumidity = ((100.0 * Raw) / 65536.0);
  }

  return 0; // Succès
}
