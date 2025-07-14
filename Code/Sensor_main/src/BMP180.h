/*****************************************************************
 * This librairy contains the useful data, structure, 
 * function prototypes to use for the bmp180
 * 
 * author : Adele De Saint-Martin and Axel Nouy
 *****************************************************************/
#include <Arduino.h>
#include <Wire.h>

typedef struct {    //calibration coefficients
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


//define i2c address for pressure sensor
//#define I2C_BMP180_MEAS 0xF4     //start measurment  register

#define BMP180_ADDRESS                0x77   //i2c address
#define BMP180_CHIP_ID                0x55   //id number
#define BMP180_ERROR                  255    //returns 255, if communication error is occurred


/* BMP180_START_MEASURMENT_REG controls */
#define BMP180_GET_TEMPERATURE_CTRL   0x2E   //get temperature control
#define BMP180_GET_PRESSURE_OSS0_CTRL 0x34   //get pressure oversampling 1 time/oss0 control
#define BMP180_GET_PRESSURE_OSS1_CTRL 0x74   //get pressure oversampling 2 time/oss1 control
#define BMP180_GET_PRESSURE_OSS2_CTRL 0xB4   //get pressure oversampling 4 time/oss2 control
#define BMP180_GET_PRESSURE_OSS3_CTRL 0xF4   //get pressure oversampling 8 time/oss3 control

#define oversampling_setting 1      //has value 0, 1, 2 or 3 for ultra low power, 
                                    //standard, high, ultra high resolution

#define BMP180_START_MEASURMENT_REG   0xF4   //start measurment  register
#define BMP180_READ_ADC_MSB_REG       0xF6   //read adc msb  register
#define BMP180_READ_ADC_LSB_REG       0xF7   //read adc lsb  register
#define BMP180_READ_ADC_XLSB_REG      0xF8   //read adc xlsb register


//calibration coefficients BMP180**************************************
#define AC1_MSB_addr    0xAA
/*#define AC1_LSB_addr    0xAB
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
#define MD_LSB_addr     0xBF*/
//***************************************************************

//function prototypes
int bmp_get_cal_param(void);
uint16_t bmp180_get_ut(void);
uint32_t bmp180_get_up(void);
int32_t computeB5(int32_t UT);
float compute_pressure(void);

