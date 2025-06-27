#define SI7034_ADDR 0x40 // Vérifie cette adresse selon ton module
#define CMD_MEASURE_TEMP_NOHOLD 0xF3
#define CMD_MEASURE_HUM_NOHOLD  0xF5


float readHumidity();
float readTemperature();