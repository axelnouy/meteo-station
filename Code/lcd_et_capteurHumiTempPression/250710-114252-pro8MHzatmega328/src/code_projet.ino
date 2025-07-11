/********************************************************
* This code manages a portable meteo station            *
*                                                       *
* Authors : Adèle DE SAINT-MARTIN, Axel NOUY            *
* Date : 11/06/2025                                     *
* Device : ATMega328P                                   *
*********************************************************/
#include <Arduino.h>
#include <LiquidCrystal.h>
#include <Wire.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

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

//define i2c address for temperature and humidity sensor
#define ADDR_SHT21_R 0b10000001    //SHT21 adress in read, sensor for humidity and temperature
#define ADDR_SHT21_W 0b10000000    //SHT21 adress in write, sensor for humidity and temperature
#define I2C_SHT21 0x40

#define ADDR_RH 0xF5   //address for relative humidity measurement, hold master
#define ADDR_T 0xF3    //address for temperature measurement, hold master

//define i2c address for pressure sensor
#define I2C_BMP180 0xF4     //start measurment  register
#define I2C_BMP180_R 0b11101111
#define I2C_BMP180_W 0b11101110

#define BMP180_ADDRESS                0x77   //i2c address
#define BMP180_CHIP_ID                0x55   //id number
#define BMP180_ERROR                  255    //returns 255, if communication error is occurred


/* BMP180_START_MEASURMENT_REG controls */
#define BMP180_GET_TEMPERATURE_CTRL   0x2E   //get temperature control
#define BMP180_GET_PRESSURE_OSS0_CTRL 0x34   //get pressure oversampling 1 time/oss0 control
#define BMP180_GET_PRESSURE_OSS1_CTRL 0x74   //get pressure oversampling 2 time/oss1 control
#define BMP180_GET_PRESSURE_OSS2_CTRL 0xB4   //get pressure oversampling 4 time/oss2 control
#define BMP180_GET_PRESSURE_OSS3_CTRL 0xF4   //get pressure oversampling 8 time/oss3 control

#define oversampling_setting 0      //has value 0, 1, 2 or 3 for ultra low power, 
                                    //standard, high, ultra high resolution

#define BMP180_START_MEASURMENT_REG   0xF4   //start measurment  register
#define BMP180_READ_ADC_MSB_REG       0xF6   //read adc msb  register
#define BMP180_READ_ADC_LSB_REG       0xF7   //read adc lsb  register
#define BMP180_READ_ADC_XLSB_REG      0xF8   //read adc xlsb register


//calibration coefficients BMP180**************************************
#define AC1_MSB_addr    0xAA
#define AC1_LSB_addr    0xAB
#define AC2_MSB_addr    0xAC
#define AC2_LSB_addr    0xAD
#define AC3_MSB_addr    0xAE
#define AC3_LSB_addr    0xAF
#define AC4_MSB_addr    0xB0
#define AC4_LSB_addr    0xB1
#define AC5_MSB_addr    0xB2
#define AC5_LSB_addr    0xB3
#define AC6_MSB_addr    0xB4
#define AC6_LSB_addr    0xB5
#define B1_MSB_addr     0xB6
#define B1_LSB_addr     0xB7
#define B2_MSB_addr     0xB8
#define B2_LSB_addr     0xB9
#define MB_MSB_addr     0xBA
#define MB_LSB_addr     0xBB
#define MC_MSB_addr     0xBC
#define MC_LSB_addr     0xBD
#define MD_MSB_addr     0xBE
#define MD_LSB_addr     0xBF
//***************************************************************

typedef struct {
    short cAC1;
    short cAC2;
    short cAC3;
    unsigned short cAC4;
    unsigned short cAC5;
    unsigned short cAC6;
    short cB1;
    short cB2;
    short cMB;
    short cMC;
    short cMD;
}bmp180_coeff;

static bmp180_coeff p_param;

#if 0
#define I2C_ADD 0x40  // I2C device address
//informations from description of 
//==============================================================================
#define TRIGGER_T_MEASUREMENT_HM 0XE3   // command trig. temp meas. hold master
#define TRIGGER_RH_MEASUREMENT_HM 0XE5  // command trig. hum. meas. hold master
#define TRIGGER_T_MEASUREMENT_NHM 0XF3  // command trig. temp meas. no hold master
#define TRIGGER_RH_MEASUREMENT_NHM 0XF5 // command trig. hum. meas. no hold master
#define USER_REGISTER_W 0XE6        // command writing user register
#define USER_REGISTER_R 0XE7            // command reading user register
#define SOFT_RESET 0XFE                 // command soft reset
//==============================================================================
// HOLD MASTER - SCL line is blocked (controlled by sensor) during measurement
// NO HOLD MASTER - allows other I2C communication tasks while sensor performing
// measurements.
#endif


//function prototypes ******************************************************
float get_info_SHT21(uint8_t addr);
void display_sensor_SHT21(float temp, float humi);
int bmp_get_cal_param(void);
void display_sensor_BMP180(float pressure);
uint16_t bmp180_get_ut(void);
uint32_t bmp180_get_up(void);
float compute_pressure(void);

void setup() {
    // put your setup code here, to run once:
    lcd.begin(8, 2);  //screen has 2 lignes and 8 columns
    lcd.print("heehee");  //test text
    delay(1000);
    lcd.clear();
    delay(200);
    Wire.begin();
}

void loop() {
    // put your main code here, to run repeatedly:
    //float rh=5; //test value for relative humidity
    //float t=2;  //test value for temperature
    float rh, st, press;
    //bmp180_coeff coeffListe;
    rh = 0;
    st = 0;
    press = 0;
   
    rh = get_info_SHT21(ADDR_RH);
    st= get_info_SHT21(ADDR_T);

    if(bmp_get_cal_param()!=0) puts("erreur init param");

    press = compute_pressure();

    display_sensor_SHT21(st,rh); 
    //display_sensor_SHT21(78.5 ,78.5); 
    delay(5000);
    display_sensor_BMP180(press);
}

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
    delay(700);
}

//display of informations from sensor BMP180
void display_sensor_BMP180(float pressure){ 
    lcd.clear();
    lcd.setCursor(0,0); //display temperature on ligne 1
    lcd.print("P:");
    lcd.print(pressure);
    lcd.print("hPa");
}

int bmp_get_cal_param(void){
    int nb_coeff = 22;          //22 bytes to retrieve to form the 11 coeff
    uint8_t data[nb_coeff];     //buffer to collect data
    int i=0;
    //short AC1, AC2, AC3, B1, B2, MB, MC, MD;
    //unsigned short AC4, AC5, AC6;
    bmp180_coeff p_param;
    //p_param = (bmp180_coeff*)malloc(sizeof(bmp_coeff));
    
    Wire.beginTransmission(I2C_BMP180);
    Wire.write(AC1_MSB_addr);     //calibration adresses begin with AC1_MSB
    Wire.endTransmission();
    //delay(85);            
    Wire.requestFrom(I2C_BMP180,nb_coeff);       //gets 22 bytes of data from BMP180 sensor

    for(i=0; i<nb_coeff; i++){
        data[i] = Wire.read();
    }

    if(Wire.available()!=nb_coeff) return -1;

    //memcpy(p_param, data, 22);
    
    p_param.cAC1 = (data[0]<<8);
    p_param.cAC1 += data[1];

    p_param.cAC2 = (data[2]<<8);
    p_param.cAC2 += data[3];

    p_param.cAC3 = (data[4]<<8);
    p_param.cAC3 += data[5];

    p_param.cAC4 = (data[6]<<8);
    p_param.cAC4 += data[7];

    p_param.cAC5 = (data[8]<<8);
    p_param.cAC5 += data[9];

    p_param.cAC6 = (data[10]<<8);
    p_param.cAC6 += data[11];

    p_param.cB1 = (data[12]<<8);
    p_param.cB1 += data[13];

    p_param.cB2 = (data[14]<<8);
    p_param.cB2 += data[15];

    p_param.cMB = (data[16]<<8);
    p_param.cMB += data[17];

    p_param.cMC = (data[18]<<8);
    p_param.cMC += data[19];

    p_param.cMD = (data[20]<<8);
    p_param.cMD += data[21];

   return 0;
    
}

//read uncompensated temperature value
uint16_t bmp180_get_ut(void){
    uint16_t UT;
    Wire.beginTransmission(BMP180_ADDRESS);
    Wire.write(BMP180_START_MEASURMENT_REG);    //instructs the sensor to measure temperature
    Wire.write(BMP180_GET_TEMPERATURE_CTRL);     //gets msb and lsb for temperature data
    Wire.endTransmission();

    delay(5); 
    
    Wire.beginTransmission(BMP180_ADDRESS);
    Wire.write(BMP180_READ_ADC_MSB_REG);
    Wire.endTransmission();
    Wire.requestFrom(BMP180_ADDRESS,2);       //gets 2 bytes of data from BMP180 sensor
    UT  = Wire.read() << 8;                                //read msb
    UT |= Wire.read();
    
    return UT;

}

//read uncompensated pressure value
uint32_t bmp180_get_up(void){
    int nb_data = 3;    //number of data to get from component
    uint32_t UP;
    uint8_t data[nb_data];
    uint8_t  regControl = 0;

    switch (oversampling_setting)
    {
        case 0:                 //oss0
        regControl = BMP180_GET_PRESSURE_OSS0_CTRL;
        break;

        case 1:                 //oss1
        regControl = BMP180_GET_PRESSURE_OSS1_CTRL;
        break;

        case 2:                 //oss2
        regControl = BMP180_GET_PRESSURE_OSS2_CTRL;
        break;

        case 3:                 //oss3
        regControl = BMP180_GET_PRESSURE_OSS3_CTRL;
        break;
    }

    Wire.beginTransmission(BMP180_ADDRESS);
    Wire.write(BMP180_START_MEASURMENT_REG);    //instructs the sensor to start measure
    Wire.write(regControl);     //instructs the sensor to measure pressure
    Wire.endTransmission();

    switch(oversampling_setting){
        case 0:
            delay(5);
            break;
        case 1:
            delay(8);
            break;
        case 2:
            delay(14);
            break;
        case 3:
            delay(26);
            break;
    }

    
    Wire.beginTransmission(BMP180_ADDRESS);
    Wire.write(BMP180_READ_ADC_MSB_REG);       //informs sensor to get pressure data from sensor
    Wire.endTransmission();
    Wire.requestFrom(BMP180_ADDRESS,3);       //gets 3 bytes of data from BMP180 sensor
    
    data[0] = Wire.read();      //read msb
    data[1] = Wire.read();      //read lSB
    data[2] = Wire.read();      //read xlsb, resolution adjustment

    //UP = data[0]<<16;    
    UP = data[0];
    UP<<=16;
    UP |= data[1]<<8;       
    UP |= data[2];          

    UP >>=(8-oversampling_setting);

    return UP;
}

int32_t computeB5(int32_t UT){
  int32_t X1 = ((UT - (int32_t)p_param.cAC6) * (int32_t)p_param.cAC5) >> 15;
  int32_t X2 = ((int32_t)p_param.cMC << 11) / (X1 + (int32_t)p_param.cMD);

  return X1 + X2;
}


float compute_pressure(void){ //pressure sensor
    int32_t  UT       = 0;
    int32_t  UP       = 0;
    int32_t  cB3       = 0;
    int32_t  cB5       = 0;
    int32_t  cB6       = 0;
    int32_t  X1       = 0;
    int32_t  X2       = 0;
    int32_t  X3       = 0;
    int32_t  pressure = 0;
    uint32_t cB4       = 0;
    uint32_t cB7       = 0;

    UT = bmp180_get_ut();                                            //read uncompensated temperature, 16-bit
    if (UT == BMP180_ERROR) return BMP180_ERROR;                          //error handler, collision on i2c bus

    UP = bmp180_get_up();                                               //read uncompensated pressure, 19-bit
    if (UP == BMP180_ERROR) return BMP180_ERROR;                          //error handler, collision on i2c bus

    cB5 = computeB5(UT);

    /* pressure calculation */
    cB6 = cB5 - 4000;
    X1 = ((int32_t)p_param.cB2 * ((cB6 * cB6) >> 12)) >> 11;
    X2 = ((int32_t)p_param.cAC2 * cB6) >> 11;
    X3 = X1 + X2;
    cB3 = ((((int32_t)p_param.cAC1 * 4 + X3) << oversampling_setting) + 2) / 4;

    X1 = ((int32_t)p_param.cAC3 * cB6) >> 13;
    X2 = ((int32_t)p_param.cB1 * ((cB6 * cB6) >> 12)) >> 16;
    X3 = ((X1 + X2) + 2) >> 2;
    cB4 = ((uint32_t)p_param.cAC4 * (X3 + 32768L)) >> 15;
    cB7 = (UP - cB3) * (50000UL >> oversampling_setting);
    
    if (cB4 == 0) return BMP180_ERROR;            //safety check, avoiding division by zero

    if   (cB7 < 0x80000000) pressure = (cB7 * 2) / cB4;
    else                   pressure = (cB7 / cB4) * 2;

    X1 = pow((pressure >> 8), 2);
    X1 = (X1 * 3038L) >> 16;
    X2 = (-7357L * pressure) >> 16;

    return pressure = pressure + ((X1 + X2 + 3791L) >> 4);

}


