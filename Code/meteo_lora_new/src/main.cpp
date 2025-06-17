#include <Arduino.h>

#include <SPI.h>
#include <LoRa.h>

#include "error.h"

#define k_RFM95_RESET 9
#define k_RFM95_CS 10
#define k_LORA_SPREADING_FACTOR 10

#define LORA_DEFAULT_TEMP 21.5
#define LORA_DEFAULT_HUM 45.0
#define LORA_DEFAULT_PRES 1013

struct tDataPacket
{
  float Temp;
  float Hum;
  int Pres;
};


int InitLora(void);
int SendLoRaPacket(tDataPacket DataPacket);

void setup() {
  Serial.begin(9600);
  while (!Serial);

  Serial.println("LoRa Sender");
  InitLora();
}

void loop() {
  static tDataPacket DataPacket;
  DataPacket.Temp = LORA_DEFAULT_TEMP;
  DataPacket.Hum = LORA_DEFAULT_HUM;
  DataPacket.Pres = LORA_DEFAULT_PRES;
  Serial.print("Sending packet: ");

  // send packet
  SendLoRaPacket(DataPacket);

  delay(5000);
}


int InitLora(void)
{
  int error = ERROR_NONE;
  if (!LoRa.begin(868E6)) {
    Serial.println("Starting LoRa failed!");
    error = ERROR_INIT_LORA;
    goto ERREUR;
  }
  LoRa.setPins(k_RFM95_CS, k_RFM95_RESET, 8); // DIO0 pin)

  LoRa.setSpreadingFactor(k_LORA_SPREADING_FACTOR);

ERREUR:
  return error;
}

int SendLoRaPacket(tDataPacket DataPacket)
{
  int error = ERROR_NONE;

  LoRa.beginPacket();
  if (!LoRa.write((uint8_t*)&DataPacket.Temp, sizeof(DataPacket.Temp)))
  {
    error = ERROR_SEND_PACKET;
    goto ERREUR;
  }
  if (!LoRa.write((uint8_t*)&DataPacket.Hum, sizeof(DataPacket.Hum)))
  {
    error = ERROR_SEND_PACKET;
    goto ERREUR;
  }
  if (!LoRa.write((uint8_t*)&DataPacket.Pres, sizeof(DataPacket.Pres)))
  {
    error = ERROR_SEND_PACKET;
    goto ERREUR;
  }
  LoRa.endPacket();
ERREUR:
  return error;
}

