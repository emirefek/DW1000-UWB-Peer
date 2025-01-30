#include <Arduino.h>
#include <SPI.h>
#include <DW1000Ng.hpp>
#include <WiFi.h>
#include "utils.h" // Include the new header file

// CONNECTION PINS BEGIN
#define SPI_SCK 18
#define SPI_MISO 19
#define SPI_MOSI 23
#define DW_CS 4
const uint8_t PIN_RST = 27; // reset pin
const uint8_t PIN_IRQ = 34; // irq pin
const uint8_t PIN_SS = 4;   // spi select pin
// CONNECTION PINS END

// Extended Unique Identifier register. 64-bit device identifier. Register file: 0x01
char EUI[] = "AA:BB:CC:DD:EE:FF:00:00";
volatile uint32_t beacon_rate = 1000;
volatile uint32_t last_beacon = 0;

device_configuration_t DEFAULT_CONFIG = {
    false,
    true,
    true,
    true,
    false,
    SFDMode::STANDARD_SFD,
    Channel::CHANNEL_5,
    DataRate::RATE_850KBPS,
    PulseFrequency::FREQ_16MHZ,
    PreambleLength::LEN_256,
    PreambleCode::CODE_3};

void printCore()
{
  Serial.print("Core: ");
  Serial.println(xPortGetCoreID());
}

void transmitMeta()
{

  Serial.println("[BEACON]Transmitting Beacon meta");
  char message[128];
  snprintf(message, sizeof(message), "Beacon: %s | timestamp: %lu", EUI, millis());

  DW1000Ng::setTransmitData((uint8_t *)message, strlen(message));
  DW1000Ng::startTransmit(TransmitMode::IMMEDIATE);

  Serial.println("[BEACON]Transmitting ...");

  printCore();

  while (!DW1000Ng::isTransmitDone())
  {
    yield();
  }

  last_beacon = millis();
  DW1000Ng::clearTransmitStatus();
}

void collectMeta()
{
  Serial.println("[INFO]Collecting meta data");

  DW1000Ng::startReceive();
  unsigned long startTime = millis();
  const unsigned long timeout = 50; // 50 ms timeout

  while (!DW1000Ng::isReceiveDone())
  {
    if (millis() - startTime > timeout)
    {
      Serial.println("[INFO]Receive timeout");
      DW1000Ng::clearReceiveStatus();
      DW1000Ng::clearReceiveTimeoutStatus();
      return;
    }
    yield();
  }

  String message;
  DW1000Ng::getReceivedData(message);
  Serial.print("[INFO]Received message: ");
  Serial.println(message);
  Serial.print("[INFO]dBm: ");
  Serial.println(DW1000Ng::getReceivePower());
  Serial.print("[INFO]Quality: ");
  Serial.println(DW1000Ng::getReceiveQuality());

  byte sourceAddress[2];
  DW1000Ng::getDeviceAddress(sourceAddress);

  DW1000Ng::clearReceiveStatus();
  DW1000Ng::clearReceiveTimeoutStatus();

  printCore();
}

void setup()
{
  Serial.begin(115200);

#if defined(ESP32)
  Serial.println("!! ESP32 DEVICE VAR DEFINED !!");
#endif

  Serial.println(F("### DW1000Ng-ESP32-peer-communicator ###"));
  // initialize the driver
  DW1000Ng::initializeNoInterrupt(PIN_SS);
  Serial.println(F("DW1000Ng initialized ..."));

  DW1000Ng::applyConfiguration(DEFAULT_CONFIG);

  DW1000Ng::setDeviceAddress(6);
  DW1000Ng::setNetworkId(10);

  DW1000Ng::setAntennaDelay(16436);
  Serial.println(F("Committed configuration ..."));

  // DEBUG chip info and registers pretty printed
  char msg[128];
  DW1000Ng::getPrintableDeviceIdentifier(msg);
  Serial.print("[INFO]Device ID: ");
  Serial.println(msg);
  DW1000Ng::getPrintableExtendedUniqueIdentifier(msg);
  Serial.print("[INFO]Unique ID: ");
  Serial.println(msg);
  DW1000Ng::getPrintableNetworkIdAndShortAddress(msg);
  Serial.print("[INFO]Network ID & Device Address: ");
  Serial.println(msg);
  DW1000Ng::getPrintableDeviceMode(msg);
  Serial.print("[INFO]Device mode: ");
  Serial.println(msg);

  generateUniqueId(EUI);
  Serial.print("[INFO]Generated Unique Device ID: ");
  Serial.println(EUI);
}

void loop()
{
  if (millis() - last_beacon > beacon_rate)
  {
    transmitMeta();
    Serial.println("[INFO]Beacon transmitted");
  }
  else
  {
    collectMeta();
  }
}
