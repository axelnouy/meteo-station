#include <SPI.h>
#include <LoRa.h>

#define k_LORA_RESET 14
#define k_LORA_SPREADING_FACTOR 10
#define k_LORA_MOSI 27
#define k_LORA_MISO 19
#define k_LORA_SCK 5
#define k_LORA_CS 18
#define k_LORA_DIO0 26
#define k_LORA_CS_SENSOR 10
#define k_LORA_RS_SENSOR 9

struct tDataPacket
{
  float Temp;
  float Hum;
  int Pres;
};



int InitLoraServer(int CsPin, int ResetPin, int LoraSpreadingFactor);

int ReceivePacket(tDataPacket* pDataPacket);
int PacketAvailable(void);



int InitLoraSensor();

int SendLoRaPacket(tDataPacket DataPacket);