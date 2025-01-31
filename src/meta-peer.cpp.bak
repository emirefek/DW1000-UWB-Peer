#include <Arduino.h>
#include <SPI.h>
#include <DW1000Ng.hpp>
#include <WiFi.h>
#include "utils.h"
#include <vector>
#include <string>

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
volatile uint32_t beacon_rate = 100;
volatile uint32_t last_beacon = 0;

enum device_type
{
  FORKLIFT,
  DOOR,
  LIFT
};
device_type myDeviceType = FORKLIFT;

struct DeviceInfo
{
  device_type type;
  String eui;
  uint32_t timestamp;
};
std::vector<DeviceInfo> metadataBuffer;

unsigned long lastMetadataPrintTime = 0;

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

void addToMetadataBuffer(device_type type, const String &eui, uint32_t timestamp)
{
  for (auto &device : metadataBuffer)
  {
    if (device.eui == eui)
    {
      device.timestamp = timestamp; // Update the timestamp if the EUI matches
      return;
    }
  }
  // If the EUI is not found, add a new entry
  DeviceInfo device = {type, eui, timestamp};
  metadataBuffer.push_back(device);
}

void printAndClearMetadataBuffer()
{
  if (millis() - lastMetadataPrintTime > 1000)
  {
    Serial.println("[INFO]Collected Beacons:");
    for (const auto &device : metadataBuffer)
    {
      Serial.print("[INFO]Device Type: ");
      switch (device.type)
      {
      case FORKLIFT:
        Serial.print("FORKLIFT");
        break;
      case DOOR:
        Serial.print("DOOR");
        break;
      case LIFT:
        Serial.print("LIFT");
        break;
      }
      Serial.print(" | EUI: ");
      Serial.print(device.eui);
      Serial.print(" | Timestamp: ");
      Serial.println(device.timestamp);
    }
    metadataBuffer.clear();
    lastMetadataPrintTime = millis();
  }
}

void transmitMeta()
{
  uint8_t message[13]; // 1 byte for device type, 8 bytes for EUI, 4 bytes for timestamp
  uint32_t timestamp = millis();

  // Construct the message
  message[0] = myDeviceType;                         // 1 byte for device type
  memcpy(&message[1], EUI, 8);                       // 8 bytes for EUI
  memcpy(&message[9], &timestamp, sizeof(uint32_t)); // 4 bytes for timestamp

  DW1000Ng::setTransmitData(message, sizeof(message));
  DW1000Ng::startTransmit(TransmitMode::IMMEDIATE);

  unsigned long startTime = millis();
  const unsigned long timeout = 10; // 200 ms timeout

  while (!DW1000Ng::isTransmitDone())
  {
    if (millis() - startTime > timeout)
    {
      DW1000Ng::clearTransmitStatus();
      return;
    }
    yield();
  }

  last_beacon = millis();
  DW1000Ng::clearTransmitStatus();
}

void collectMeta()
{
  DW1000Ng::startReceive();
  unsigned long startTime = millis();
  const unsigned long timeout = 10; // 200 ms timeout

  while (!DW1000Ng::isReceiveDone())
  {
    if (millis() - startTime > timeout)
    {
      // Serial.println("[ERROR]Receive timeout");
      // DW1000Ng::reset();
      DW1000Ng::clearReceiveStatus();
      DW1000Ng::clearReceiveFailedStatus();
      DW1000Ng::clearReceiveTimeoutStatus();
      return;
    }
    yield();
  }

  DW1000Ng::clearReceiveStatus();
  uint8_t message[13];
  DW1000Ng::getReceivedData(message, sizeof(message));

  device_type type = static_cast<device_type>(message[0]);
  char eui[9];
  memcpy(eui, &message[1], 8);
  eui[8] = '\0'; // Null-terminate the EUI string
  uint32_t timestamp;
  memcpy(&timestamp, &message[9], sizeof(uint32_t));

  addToMetadataBuffer(type, String(eui), timestamp);
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
  }
  else
  {
    collectMeta();
    printAndClearMetadataBuffer();
  }
}
