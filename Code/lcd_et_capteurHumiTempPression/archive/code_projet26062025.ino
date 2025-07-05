#include <LiquidCrystal.h>
#define RS 2
#define EN 3
#define D4 4
#define D5 5
#define D6 6
#define D7 7

LiquidCrystal lcd(RS, EN, D4, D5, D6, D7);

int get_info(int capteur){
    int info = 5;
    return info;
}

void affich_capteurs(void){
    lcd.clear();
    int temp= 10;//get_info(capteur_temp);
    int hum= 50; //get_info(capteur_humi);
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
    lcd.print("hee hee");
    delay(5000);
}

void loop() {
    // put your main code here, to run repeatedly:
    affich_capteurs();
    delay(1000);
  
}
