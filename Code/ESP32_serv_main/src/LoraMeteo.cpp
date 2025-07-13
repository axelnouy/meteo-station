#include "LoraMeteo.h"
#include "error.h"


int InitLoraServer()
{
  int error = 0;
  SPIClass* spi = new SPIClass(VSPI);
  spi->begin(k_LORA_SCK, k_LORA_MISO, k_LORA_MOSI, k_LORA_CS); 
  LoRa.setSPI(*spi);
  LoRa.setPins(k_LORA_CS, k_LORA_RESET, k_LORA_DIO0);
  if (!LoRa.begin(868E6)) {
    Serial.println("Starting LoRa failed!");
    error = -1;
    goto ERREUR;
  }

  LoRa.setSpreadingFactor(k_LORA_SPREADING_FACTOR);

ERREUR:
  return error;
}

int ReceivePacket(tDataPacket* pDataPacket)
{
  int  error                             = 0;
  int  packetSize                        = 0;
  int  PacketIndex                       = 0;
  byte packetBuffer[sizeof(tDataPacket)] = {0};

  // verify if a packet was received
  packetSize = LoRa.parsePacket();
  if (packetSize == 0) {
    error = -1; // No packet received
    goto ERREUR;
  }

  // verify if the packet size matches the expected size
  Serial.print("Packet Size: ");
  Serial.println(packetSize);
  if(packetSize != 13) {
    error = -2; // Packet size mismatch
    goto ERREUR;
  }


  // Read the packet
  while (LoRa.available()) {
    packetBuffer[PacketIndex] = LoRa.read();
    PacketIndex++;
  }
  memcpy(pDataPacket, packetBuffer, sizeof(tDataPacket));

ERREUR:
  return error;
}






int InitLoraSensor()
{
  int error = ERROR_NONE;
  SPIClass* spi = new SPIClass();
  spi->begin();
  LoRa.setSPI(*spi);
  LoRa.setPins(k_LORA_CS_SENSOR, k_LORA_RS_SENSOR);
  if (!LoRa.begin(868E6)) {
    Serial.println("Starting LoRa failed!");
    error = ERROR_INIT_LORA;
    goto ERREUR;
  }
 
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
  // Check if the packet was sent successfully
  LoRa.endPacket();
ERREUR:
  return error;
}