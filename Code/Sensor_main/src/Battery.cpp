#include "Battery.h"


int getBatteryLevel() {
    int batteryLevel = analogRead(A1); // Read the analog value from pin A1
    batteryLevel = map(batteryLevel, 0, 1023, 0, 100); // Map the value to a percentage (0-100)
    
    if (batteryLevel < 25) {
        return k_BATTERY_EMPTY_CHAR;
    } else if (batteryLevel < 50) {
        return k_BATTERY_25_CHAR;
    } else if (batteryLevel < 75) {
        return k_BATTERY_50_CHAR;
    } else if (batteryLevel < 100) {
        return k_BATTERY_75_CHAR;
    } else {
        return k_BATTERY_FULL_CHAR;
    }
}