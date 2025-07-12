#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <stdint.h>

typedef struct 
{
    int a;
    int b;
    int c;
    short u;
}test;

typedef struct {
    short AC1;
    short AC2;
    short AC3;
    unsigned short AC4;
    unsigned short AC5;
    unsigned short AC6;
    short B1;
    short B2;
    short MB;
    short MC;
    short MD;
}bmp180_coeff;

#define oversampling_setting 3
#define BMP180_GET_PRESSURE_OSS0_CTRL 1
#define BMP180_GET_PRESSURE_OSS1_CTRL 2
#define BMP180_GET_PRESSURE_OSS2_CTRL 3
#define BMP180_GET_PRESSURE_OSS3_CTRL 4

int main(){
    printf("\ntest\n");
#if 0
    bmp180_coeff* t = NULL;
    uint8_t data[]={0,1,0,2,0,3,0,4,0,5,0,6,0,7,0,8,0,9,0,10,0,11};
    uint8_t a=0;
    uint8_t b =1;

    t = (bmp180_coeff*)malloc(sizeof(bmp180_coeff));
    /*t->a = 0;
    t->b = 0;
    t->c = 0;*/
    memcpy(t, data, sizeof(bmp180_coeff));
    printf("%hd, %hd, %hd, %hu, %hu, %hu, %hd, %hd, %hd, %hd, %hd\n",t->AC1, t->AC2, t->AC3, t->AC4, t->AC5, t->AC6, t->B1, t->B2, t->MB, t->MC, t->MD);
    printf("\n%hd", (b<<8)+a);
#endif
    uint8_t regControl =0;

    switch(oversampling_setting)
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
    printf("regControl %d", regControl);


    return 0;
}
