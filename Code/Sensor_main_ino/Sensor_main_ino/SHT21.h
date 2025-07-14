/*****************************************************************
 * This librairy contains the useful data, structure, 
 * function prototypes to use for the SHT21 sensor
 * 
 * author : Adele De Saint-Martin and Axel Nouy
 *****************************************************************/
#include <Arduino.h>
#include <Wire.h>

//define i2c address for temperature and humidity sensor
#define ADDR_SHT21_R 0b10000001    //SHT21 adress in read, sensor for humidity and temperature
#define ADDR_SHT21_W 0b10000000    //SHT21 adress in write, sensor for humidity and temperature
#define I2C_SHT21 0x40

#define ADDR_RH 0xF5   //address for relative humidity measurement, hold master
#define ADDR_T 0xF3    //address for temperature measurement, hold master

//function prototypes
float get_info_SHT21(uint8_t addr);
