#include <LiquidCrystal.h>
#define RS 12
#define EN 11
#define D4 5
#define D5 4
#define D6 3
#define D7 2

LiquidCrystal lcd(RS, EN, D4, D5, D6, D7);

int get_info(capteur){
    
    return info;
}

void affich_capteurs(void){
    lcd.clear();
    int temp=get_info(capteur_temp);
    int hum=get_info(capteur_humi);
    lcd.setCursor(0,0);
    lcd.print("T:");
    lcd.print(temp);
    lcd.print("°C");
    lcd.setCursor(0,1);
    lcd.print("H:");
    lcd.print(hum);
}


void setup() {
    // put your setup code here, to run once:
    lcd.begin(8, 2);
    lcd.print("heeeeeee");
}

void loop() {
    // put your main code here, to run repeatedly:
    affich_capteurs();
  
}
