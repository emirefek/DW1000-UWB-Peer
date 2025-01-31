#include <Arduino.h>
#include <SPI.h>
#include <DW1000Ng.hpp>
#include "utils.h"
#include <vector>
#include <string>
#include <cstring>

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
char EUID[] = "AA:BB:CC:DD:EE:FF:00:00";

enum CarrierType
{
  FORKLIFT,
  DOOR
};
CarrierType thisCarrierType = FORKLIFT;

enum WaveType
{
  BLINK,
  ANSWER,
  RESULT
};

struct BlinkWave
{
  WaveType type = WaveType::BLINK;
  String euid;
  uint32_t departure_time;
  CarrierType carrier;
};

struct AnswerWave
{
  WaveType type = WaveType::ANSWER;
  String euid;
  String initiator_euid;
  uint32_t departure_time;
  uint32_t taxi_time;
  CarrierType carrier;
};

struct ResultWave
{
  WaveType type = WaveType::RESULT;
  String initiator_euid;
  String responder_euid;
  float calculated_distance;
};

struct WaveMessage
{
  WaveType type;
  union
  {
    BlinkWave blink;
    AnswerWave answer;
    ResultWave result;
  };

  // Constructor for BlinkWave
  WaveMessage(const BlinkWave &bw) : type(WaveType::BLINK), blink(bw) {}

  // Constructor for AnswerWave
  WaveMessage(const AnswerWave &aw) : type(WaveType::ANSWER), answer(aw) {}

  // Constructor for ResultWave
  WaveMessage(const ResultWave &rw) : type(WaveType::RESULT), result(rw) {}

  // Destructor - Needed for union
  ~WaveMessage() {}
};

const uint16_t blinkRate = 300;
uint32_t lastBlinkTime = 0;

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

void transmitWaveMessage(const WaveMessage &message);
void receiveWaveMessage(WaveMessage &message);
void serializeWaveMessage(const WaveMessage &message, uint8_t *buffer, size_t &length);
void deserializeWaveMessage(const uint8_t *buffer, size_t length, WaveMessage &message);

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

  generateUniqueId(EUID);
  Serial.print("[INFO]Unique Device ID: ");
  Serial.println(EUID);
}

void loop()
{
  // Blink every 300ms
  if (millis() - lastBlinkTime > blinkRate)
  {
    lastBlinkTime = millis();
    BlinkWave bw;
    bw.euid = EUID;
    bw.departure_time = millis();
    bw.carrier = thisCarrierType;

    WaveMessage message(bw);
    transmitWaveMessage(message);
  }

  // create a WaveMessage object with the type BLINK
}

void transmitWaveMessage(const WaveMessage &message)
{
  uint8_t buffer[128];
  size_t length;
  serializeWaveMessage(message, buffer, length);

  DW1000Ng::setTransmitData(buffer, length);
  DW1000Ng::startTransmit(TransmitMode::IMMEDIATE);

  unsigned long startTime = millis();
  const unsigned long timeout = 10;

  while (!DW1000Ng::isTransmitDone())
  {
    if (millis() - startTime > timeout)
    {
      DW1000Ng::clearTransmitStatus();
      return;
    }
    yield();
  }

  DW1000Ng::clearTransmitStatus();
};

void receiveWaveMessage(WaveMessage &message)
{
  DW1000Ng::startReceive();
  unsigned long startTime = millis();
  const unsigned long timeout = 10;

  while (!DW1000Ng::isReceiveDone())
  {
    if (millis() - startTime > timeout)
    {
      DW1000Ng::clearReceiveStatus();
      DW1000Ng::clearReceiveFailedStatus();
      DW1000Ng::clearReceiveTimeoutStatus();
      return;
    }
    yield();
  }

  DW1000Ng::clearReceiveStatus();
  uint8_t buffer[128];
  DW1000Ng::getReceivedData(buffer, sizeof(buffer));
  size_t length = sizeof(buffer);
  deserializeWaveMessage(buffer, length, message);
}

// Serialize WaveMessage to byte array
void serializeWaveMessage(const WaveMessage &message, uint8_t *buffer, size_t &length)
{
  buffer[0] = static_cast<uint8_t>(message.type);
  size_t offset = 1;

  switch (message.type)
  {
  case WaveType::BLINK:
    memcpy(buffer + offset, message.blink.euid.c_str(), message.blink.euid.length() + 1); // +1 for null terminator
    offset += message.blink.euid.length() + 1;
    memcpy(buffer + offset, &message.blink.departure_time, sizeof(message.blink.departure_time));
    offset += sizeof(message.blink.departure_time);
    buffer[offset++] = static_cast<uint8_t>(message.blink.carrier);
    break;

  case WaveType::ANSWER:
    memcpy(buffer + offset, message.answer.euid.c_str(), message.answer.euid.length() + 1);
    offset += message.answer.euid.length() + 1;
    memcpy(buffer + offset, message.answer.initiator_euid.c_str(), message.answer.initiator_euid.length() + 1);
    offset += message.answer.initiator_euid.length() + 1;
    memcpy(buffer + offset, &message.answer.departure_time, sizeof(message.answer.departure_time));
    offset += sizeof(message.answer.departure_time);
    memcpy(buffer + offset, &message.answer.taxi_time, sizeof(message.answer.taxi_time));
    offset += sizeof(message.answer.taxi_time);
    buffer[offset++] = static_cast<uint8_t>(message.answer.carrier);
    break;

  case WaveType::RESULT:
    memcpy(buffer + offset, message.result.initiator_euid.c_str(), message.result.initiator_euid.length() + 1);
    offset += message.result.initiator_euid.length() + 1;
    memcpy(buffer + offset, message.result.responder_euid.c_str(), message.result.responder_euid.length() + 1);
    offset += message.result.responder_euid.length() + 1;
    memcpy(buffer + offset, &message.result.calculated_distance, sizeof(message.result.calculated_distance));
    offset += sizeof(message.result.calculated_distance);
    break;
  }

  length = offset;
}

// Deserialize byte array to WaveMessage
void deserializeWaveMessage(const uint8_t *buffer, size_t length, WaveMessage &message)
{
  message.type = static_cast<WaveType>(buffer[0]);
  size_t offset = 1;

  // First, destroy the old object inside the union (since union does not automatically manage destruction)
  switch (message.type)
  {
  case WaveType::BLINK:
    new (&message.blink) BlinkWave(); // Placement new to properly initialize
    message.blink.euid = String((char *)(buffer + offset));
    offset += message.blink.euid.length() + 1; // +1 for null terminator
    memcpy(&message.blink.departure_time, buffer + offset, sizeof(message.blink.departure_time));
    offset += sizeof(message.blink.departure_time);
    message.blink.carrier = static_cast<CarrierType>(buffer[offset++]);
    break;

  case WaveType::ANSWER:
    new (&message.answer) AnswerWave();
    message.answer.euid = String((char *)(buffer + offset));
    offset += message.answer.euid.length() + 1;
    message.answer.initiator_euid = String((char *)(buffer + offset));
    offset += message.answer.initiator_euid.length() + 1;
    memcpy(&message.answer.departure_time, buffer + offset, sizeof(message.answer.departure_time));
    offset += sizeof(message.answer.departure_time);
    memcpy(&message.answer.taxi_time, buffer + offset, sizeof(message.answer.taxi_time));
    offset += sizeof(message.answer.taxi_time);
    message.answer.carrier = static_cast<CarrierType>(buffer[offset++]);
    break;

  case WaveType::RESULT:
    new (&message.result) ResultWave();
    message.result.initiator_euid = String((char *)(buffer + offset));
    offset += message.result.initiator_euid.length() + 1;
    message.result.responder_euid = String((char *)(buffer + offset));
    offset += message.result.responder_euid.length() + 1;
    memcpy(&message.result.calculated_distance, buffer + offset, sizeof(message.result.calculated_distance));
    offset += sizeof(message.result.calculated_distance);
    break;
  }
}