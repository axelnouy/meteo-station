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

int main(){
    printf("\ntest\n");
    
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
    return 0;
}
