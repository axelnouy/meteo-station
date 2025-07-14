#include "Battery.h"

#define k_BATTERY_EMPTY_LEVEL 3.0 // Threshold for low battery level 3.0V
#define k_BATTERY_25_LEVEL    3.4 // Threshold for 25% battery level 3.4V
#define k_BATTERY_50_LEVEL    3.7 // Threshold for 50% battery level 3.7V
#define k_BATTERY_75_LEVEL    3.9 // Threshold for 75% battery level 3.9V
#define k_BATTERY_FULL_LEVEL  4.2 // Threshold for 100% battery level 4.2V


float convertBatteryLevelToVoltage(int batteryLevel) 
{
    // Convert the analog reading to voltage
    // The analog reading is between 0 and 1023, and the reference voltage is 5V
    float voltage = (batteryLevel / 1023.0) * 3.3;
    
    // Since we have a voltage divider, we multiply by 2 to get the actual battery voltage
    return voltage * 2.0;
}

uint8_t convertBatteryLevelToPourcentage(int batteryLevel) 
{
    float voltage = convertBatteryLevelToVoltage(batteryLevel);
    if (voltage < k_BATTERY_EMPTY_LEVEL) {
        return k_BATTERY_EMPTY_CHAR;
    } else if (voltage < k_BATTERY_25_LEVEL) {
        return k_BATTERY_25_CHAR;
    } else if (voltage < k_BATTERY_50_LEVEL) {
        return k_BATTERY_50_CHAR;
    } else if (voltage < k_BATTERY_75_LEVEL) {
        return k_BATTERY_75_CHAR;
    } else if (voltage < k_BATTERY_FULL_LEVEL) {
        return k_BATTERY_FULL_CHAR;
    } else {
        return k_BATTERY_CHARGING_CHAR; // Full battery
    }
}

uint16_t getBatteryLevelRaw() {
    return analogRead(A1); // Read the raw analog value from pin A1
}