#include "Battery.h"

#define k_BATTERY_EMPTY_LEVEL 465 // Threshold for low battery level
#define k_BATTERY_25_LEVEL    515 // Threshold for 25% battery level
#define k_BATTERY_50_LEVEL    580 // Threshold for 50% battery level
#define k_BATTERY_75_LEVEL    620 // Threshold for 75% battery level
#define k_BATTERY_FULL_LEVEL  651 // Threshold for 100% battery level


float convertBatteryLevelToVoltage(int batteryLevel) {
    // Convert the analog reading to voltage
    // The analog reading is between 0 and 1023, and the reference voltage is 5V
    float voltage = (batteryLevel / 1023.0) * 3.3;
    
    // Since we have a voltage divider, we multiply by 2 to get the actual battery voltage
    return voltage * 2.0;
}

int convertBatteryLevelToPourcentage(int batteryLevel) {
    if (batteryLevel < k_BATTERY_EMPTY_LEVEL) {// <3.0V
        return k_BATTERY_EMPTY_CHAR;
    } else if (batteryLevel < k_BATTERY_25_LEVEL) {
        return k_BATTERY_25_CHAR;
    } else if (batteryLevel < k_BATTERY_50_LEVEL) {
        return k_BATTERY_50_CHAR;
    } else if (batteryLevel < k_BATTERY_75_LEVEL) {
        return k_BATTERY_75_CHAR;
    } else {
        return k_BATTERY_FULL_CHAR;
    }
}

#if Sensor
int getBatteryLevelRaw() {
    return analogRead(A1); // Read the raw analog value from pin A1
}
#endif