/********************************************************
* This code manages a portable meteo station            *
*                                                       *
* Authors : Adèle DE SAINT-MARTIN, Axel NOUY            *
* Date : 27/06/2025                                     *
* Device : ATMega328P                                   *
*********************************************************/

#include <LiquidCrystal.h>
#include <Wire.h>
#include <math.h>

//define LCD ports -> arduino
/*
#define RS 12
#define EN 11
#define D4 5
#define D5 4
#define D6 3
#define D7 2
*/
#define RS 2
#define EN 3
#define D4 4
#define D5 5
#define D6 6
#define D7 7


LiquidCrystal lcd(RS, EN, D4, D5, D6, D7);

//define i2c ports -> Arduino
#define SCL A5  //serial clock line
#define SDA A4  //Serial data line

//define address for i2c transmitted info
#define ADDR_SHT21_R 0b10000001    //SHT21 adress in read, sensor for humidity and temperature
#define ADDR_SHT21_W 0b10000000    //SHT21 adress in write, sensor for humidity and temperature

#define ADDR_RH 0b11100101   //address for relative humidity measurement, hold master
#define ADDR_T 0b11100011    //address for temperature measurement, hold master
#define RES_RH 14   //resolution of the RH sensor
#define RES_T 8     //resolution of the temperature sensor

//#define ADDR_P '5'    //address for pressure

#if 0
#define I2C_ADD 0x40	// I2C device address
//informations from description of 
//==============================================================================
#define TRIGGER_T_MEASUREMENT_HM 0XE3   // command trig. temp meas. hold master
#define TRIGGER_RH_MEASUREMENT_HM 0XE5  // command trig. hum. meas. hold master
#define TRIGGER_T_MEASUREMENT_NHM 0XF3  // command trig. temp meas. no hold master
#define TRIGGER_RH_MEASUREMENT_NHM 0XF5 // command trig. hum. meas. no hold master
#define USER_REGISTER_W 0XE6		    // command writing user register
#define USER_REGISTER_R 0XE7            // command reading user register
#define SOFT_RESET 0XFE                 // command soft reset
//==============================================================================
// HOLD MASTER - SCL line is blocked (controlled by sensor) during measurement
// NO HOLD MASTER - allows other I2C communication tasks while sensor performing
// measurements.
#endif


//function definition
float get_info_iic(char addr);
void display_sensor_SHT21(float temp, float humi);

/*
float getHumidity(void);
float getTemperature(void);


float getHumidity(void){
	uint16_t result; 	// return variable
    uint8_t checksum;
	uint8_t data[2];
	uint8_t n = 0;
	uint8_t d;
	
	if(command == TRIGGER_RH_MEASUREMENT_HM || command == TRIGGER_RH_MEASUREMENT_NHM) d = 30;
	if(command == TRIGGER_T_MEASUREMENT_HM || command == TRIGGER_T_MEASUREMENT_NHM) d = 85;
	
	Wire.beginTransmission(I2C_ADD);
	Wire.write(command);
	Wire.endTransmission();
	delay(d);
	Wire.requestFrom(I2C_ADD,3);
	
	while(Wire.available() < 3) {
		delay(10);
		n++;
		if(n>10) return 0;
	}

	data[0] = Wire.read(); 	// read data (MSB)
	data[1] = Wire.read(); 	// read data (LSB)
	checksum = Wire.read();	// read checksum

	result = (data[0] << 8);
	result += data[1];

	if(CRC_Checksum(data,2,checksum)) {
		reset();
		return 1;
	}

    result = -6 + 125*result/(2**RES_RH);      //relative humidity above liquid water conversion
    
	else return result;

}


float getTemperature(void){
    uint16_t temp;
    uint8_t data[2];
    float result;

    //Request that sht21 transmits temperature data to processor
    Wire.beginTransmission(ADDR_SHT21_W);
	Wire.write(ADDR_T);     //warns temperature sensor for future data collection
	Wire.endTransmission();
	delay(85);              //minimal delay for a good temperature transmission
	Wire.requestFrom(ADDR_SHT21_W,3);       //gets 3 bytes of data from SHT21 sensor

    data[0] = Wire.read(); 	// read data (MSB)
	data[1] = Wire.read(); 	// read data (LSB)

	temp = (data[0] << 8);
	temp += data[1];

    result = -46.85 + 175.72*temp/pow(2,RES_T);

    return result;
}
*/


float get_info_iic(char addr){
    uint16_t temp;
    uint8_t data[2];
    float result;

    //Request that sht21 transmits temperature data to processor
    Wire.beginTransmission(ADDR_SHT21_W);
	Wire.write(addr);     //warns temperature sensor for future data collection
	Wire.endTransmission();
	delay(85);              //minimal delay for a good temperature transmission
	Wire.requestFrom(ADDR_SHT21_W,3);       //gets 3 bytes of data from SHT21 sensor

    data[0] = Wire.read(); 	// read data (MSB)
	data[1] = Wire.read(); 	// read data (LSB)

	temp = (data[0] << 8);
	temp += data[1];

    switch(addr){
        case ADDR_RH:
            result = -6.0 + 125.0*temp/pow(2.0,RES_RH); 
            break;
        case ADDR_T:
            result = -46.85 + 175.72*temp/pow(2.0,RES_T);
            break;
    }
    return result;
}

//display of informations from sensor
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
    delay(700);
}

//LCD initialization
void setup() {
    // put your setup code here, to run once:
    lcd.begin(8, 2);  //screen has 2 lignes and 8 columns
    lcd.print("heehee");  //test text
    delay(1000);
    lcd.clear();
    delay(200);
    Wire.begin();
    Serial.begin(9600);
    Serial.print("hello world");
}

void loop() {
    // put your main code here, to run repeatedly:
    //float rh=5; //test value for relative humidity
    //float t=2;  //test value for temperature
    float rh, t;
    rh = get_info_iic(ADDR_RH);
    t= get_info_iic(ADDR_T);
    Serial.print("temperature:");
    Serial.println(t);
    Serial.print("humidity:");
    Serial.println(rh);
    display_sensor_SHT21(t,rh); 
    //display_sensor_SHT21(78.5 ,78.5); 
}
