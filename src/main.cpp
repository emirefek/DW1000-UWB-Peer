#include <Arduino.h>
#include <DW1000Ng.hpp>

// connection pins

#define SPI_SCK 18
#define SPI_MISO 19
#define SPI_MOSI 23
#define DW_CS 4

// connection pins
const uint8_t PIN_RST = 27; // reset pin
const uint8_t PIN_IRQ = 34; // irq pin
const uint8_t PIN_SS = 4;   // spi select pin

void setup()
{
  // DEBUG monitoring
  Serial.begin(115200);
  delay(1000);

  // init the driver
  SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI);
  // initialize the driver
  DW1000Ng::initialize(PIN_SS, PIN_IRQ, PIN_RST);
  Serial.println(F("DW1000Ng initialized ..."));
  // general configuration
  DW1000Ng::setDeviceAddress(5);
  DW1000Ng::setNetworkId(10);
  Serial.println(F("Committed configuration ..."));
  // wait a bit
  delay(1000);
}

void loop()
{
  // DEBUG chip info and registers pretty printed
  char msg[128];
  DW1000Ng::getPrintableDeviceIdentifier(msg);
  Serial.print("Device ID: ");
  Serial.println(msg);
  DW1000Ng::getPrintableExtendedUniqueIdentifier(msg);
  Serial.print("Unique ID: ");
  Serial.println(msg);
  DW1000Ng::getPrintableNetworkIdAndShortAddress(msg);
  Serial.print("Network ID & Device Address: ");
  Serial.println(msg);
  DW1000Ng::getPrintableDeviceMode(msg);
  Serial.print("Device mode: ");
  Serial.println(msg);
  // wait a bit
  delay(10000);
}