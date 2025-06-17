#include <Arduino.h>

#include <SPI.h>
#include <LoRa.h>

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
int ReceivePacket(tDataPacket* pDataPacket);

void setup() {
  Serial.begin(9600);
  while (!Serial);

  Serial.println("LoRa Server");
  InitLora();

}

void loop() {
  static tDataPacket pDataPacket;
  Serial.print("\nWaiting for packet: ");

  while (ReceivePacket(&pDataPacket) != 0) {
    Serial.print(".");
    delay(1000);
  }
  Serial.println("Packet received!");
  Serial.print("Temperature: ");
  Serial.println(pDataPacket.Temp);
  Serial.print("Humidity: ");
  Serial.println(pDataPacket.Hum);
  Serial.print("Pressure: ");
  Serial.println(pDataPacket.Pres);
  Serial.println("Packet processed successfully!");
}


int InitLora(void)
{
  int error = 0;
  if (!LoRa.begin(868E6)) {
    Serial.println("Starting LoRa failed!");
    error = -1;
    goto ERREUR;
  }
  LoRa.setPins(k_RFM95_CS, k_RFM95_RESET, 8); // DIO0 pin)

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
  if(packetSize != sizeof(tDataPacket)) {
    Serial.println("Received packet size mismatch");
    error = -2; // Packet size mismatch
    goto ERREUR;
  }

  Serial.print("Received packet with size: ");
  Serial.println(packetSize);

  // Read the packet
  while (LoRa.available()) {
    packetBuffer[PacketIndex] = LoRa.read();
    PacketIndex++;
  }
  memccpy(pDataPacket, packetBuffer, 0, sizeof(tDataPacket));

ERREUR:
  return error;
}
