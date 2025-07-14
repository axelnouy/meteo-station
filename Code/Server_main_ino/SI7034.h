#define SI7034_ADDR 0x70 // Vérifie cette adresse selon ton module
#define CMD_MEASURE_TEMP_HUMIDITY_NO_HOLD1 0x78 
#define CMD_MEASURE_TEMP_HUMIDITY_NO_HOLD2 0x66


//float readHumidity();
//float readTemperature();
int readSi7034Data(float* pTemperature, float* pHumidity);