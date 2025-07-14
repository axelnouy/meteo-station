#include "BMP180.h"
#include <Arduino.h>

static bmp180_coeff p_param;

int bmp_get_cal_param(void){
    int nb_coeff = 22;          //22 bytes to retrieve to form the 11 coeff
    uint8_t data[nb_coeff];     //buffer to collect data
    int i=0;
    //short AC1, AC2, AC3, B1, B2, MB, MC, MD;
    //unsigned short AC4, AC5, AC6;
    //bmp180_coeff p_param;
    //p_param = (bmp180_coeff*)malloc(sizeof(bmp_coeff));
    
    Wire.beginTransmission(BMP180_ADDRESS);
    Wire.write(AC1_MSB_addr);     //calibration adresses begin with AC1_MSB
    //Wire.write(AC1_LSB_addr);     //calibration adresses begin with AC1_LSB
    Wire.endTransmission();
    //delay(85);            
    Wire.requestFrom(BMP180_ADDRESS,nb_coeff);       //gets 22 bytes of data from BMP180 sensor

    Serial.print("bmp_get_cal_param: ");
    Serial.print(Wire.available());
    if(Wire.available()!=nb_coeff) return -1;

    for(i=0; i<nb_coeff; i++){
        data[i] = Wire.read();
    }

    Serial.println("data received");
    for(i=0; i<nb_coeff; i++){
        Serial.print(data[i], HEX);
        Serial.print(" ");
    }

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

    //UP = (uint32_t)data[0]<<16;    
    UP = (uint32_t)data[0]<<16;
    UP |= (uint32_t)data[1]<<8;       
    UP |= (uint32_t)data[2];          

    UP >>=(8-oversampling_setting);

    // Assemble UP as a 19-bit value according to datasheet
    //UP = (((uint32_t)data[0] << 16) | ((uint32_t)data[1] << 8) | (uint32_t)data[2]) >> (8 - oversampling_setting);

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


