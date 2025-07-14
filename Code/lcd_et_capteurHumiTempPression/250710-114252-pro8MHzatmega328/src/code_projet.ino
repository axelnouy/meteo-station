/********************************************************
* This code manages a portable meteo station            *
*                                                       *
* Authors : Adèle DE SAINT-MARTIN, Axel NOUY            *
* Date : 13/07/2025                                   *
* Device : ATMega328P                                   *
*********************************************************/
#include <Arduino.h>
#include <LiquidCrystal.h>
#include <Wire.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "BMP180.h"
#include "SHT21.h"

#define PROTEUS 0
#if PROTEUS
    //PROTEUS define LCD ports -> arduino 
    #define RS 9    //RS pin of the LCD screen connected to pin 9 of the arduino
    #define EN 8
    #define D4 13
    #define D5 12
    #define D6 11
    #define D7 10

    //define i2c ports -> Arduino
    #define SCL A5  //serial clock line
    #define SDA A4  //Serial data line
#else

    //REEL define LCD ports -> arduino 
    #define RS 2
    #define EN 3
    #define D4 4
    #define D5 5
    #define D6 6
    #define D7 7
#endif


LiquidCrystal lcd(RS, EN, D4, D5, D6, D7);

/*
//define i2c address for temperature and humidity sensor
#define ADDR_SHT21_R 0b10000001    //SHT21 adress in read, sensor for humidity and temperature
#define ADDR_SHT21_W 0b10000000    //SHT21 adress in write, sensor for humidity and temperature
#define I2C_SHT21 0x40

#define ADDR_RH 0xF5   //address for relative humidity measurement, hold master
#define ADDR_T 0xF3    //address for temperature measurement, hold master
*/
//function prototypes ******************************************************
//float get_info_SHT21(uint8_t addr);
void display_sensor_SHT21(float temp, float humi);
void display_sensor_BMP180(float pressure);

void setup() {
    // put your setup code here, to run once:
    Serial.begin(9600); //start serial communication at 9600 baud rate
    Serial.println("Meteo station starting...");
    lcd.begin(8, 2);  //screen has 2 lignes and 8 columns
    lcd.print("heehee");  //test text
    delay(1000);
    lcd.clear();
    delay(200);
    Wire.begin(); //start i2c communication
    Wire.setClock(100000); //set i2c clock to 100kHz
    
    lcd.print("Meteo station");
    delay(1000);
    lcd.clear();

}

void loop() {
    // put your main code here, to run repeatedly:
    //float rh=5; //test value for relative humidity
    //float t=2;  //test value for temperature
    float rh, st, pressure;
    //bmp180_coeff coeffListe;
    rh = 0;
    st = 0;
    pressure = 0;
   
    rh = get_info_SHT21(ADDR_RH);
    st = get_info_SHT21(ADDR_T);

    //if(bmp_get_cal_param()!=0) lcd.print("erreur init param");
    bmp_get_cal_param();

    pressure = compute_pressure();

    display_sensor_SHT21(st,rh); 
    //display_sensor_SHT21(78.5 ,78.5); 
    delay(3000);
    display_sensor_BMP180(pressure); //display pressure in hPa
    delay(3000);
}

/*
//function definition *********************************************************
float get_info_SHT21(uint8_t addr){
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
    
    if(addr==ADDR_RH){
        //result = -6.0 + 125.0*temp/pow(2.0,RES_RH);
        //result = -6.0 + 125.0*temp/RES_RHV2;
        //temp &= ~0x0003;  // clean last two bits
        result = -6.0 + 125.0/65536 * (float)temp; // return relative humidity
    }
    else if(addr==ADDR_T){
        //result = -46.85 + 175.72*temp/RES_TV2;
        
        result = -46.85 + 175.72/65536 * (float)temp; // return relative humidity
    }
    else result = 42.0;

    return result;
}
*/

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
    lcd.print("RH");
    //delay(700);
}

//display of informations from sensor BMP180
void display_sensor_BMP180(float pressure){ 
    lcd.clear();
    lcd.setCursor(0,0); //display temperature on ligne 1
    lcd.print(pressure/100.0);
    lcd.print("hPa");
}



