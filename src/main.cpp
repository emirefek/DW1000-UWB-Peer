#include <Arduino.h>
#include <SPI.h>
#include <DW1000Ng.hpp>
#include <DW1000NgUtils.hpp>
#include <DW1000NgConstants.hpp>
#include <DW1000NgRanging.hpp>
#include <DW1000NgTime.hpp>
#include <DW1000NgTime.hpp>
#include <DW1000NgUtils.hpp>

#include "utils.h"
#include "ranging.h"
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
  uint32_t t1;
  CarrierType carrier;
};

struct AnswerWave
{
  WaveType type = WaveType::ANSWER;
  String euid;
  String initiator_euid;
  uint32_t t1;
  uint32_t t2;
  uint32_t t3;
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

  // Default constructor
  WaveMessage() : type(WaveType::BLINK), blink() {}

  // Destructor - Needed for union
  ~WaveMessage() {}
};

struct NeighborEntry
{
  String euid;             // Unique identifier of the neighbor
  float lastDistance;      // Last calculated distance (or other metric)
  uint32_t lastUpdateTime; // Timestamp of the last received message
  int signalQuality;       // Optional quality/signal metric
};

// Global neighbor table
std::vector<NeighborEntry> neighborTable;

uint32_t lastNeighborTablePrint = 0;

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
void receiveWaveMessage(WaveMessage &message, uint32_t &recieveTimestamp);
void serializeWaveMessage(const WaveMessage &message, uint8_t *buffer, size_t &length);
void deserializeWaveMessage(const uint8_t *buffer, size_t length, WaveMessage &message);
void updateNeighborTable(const String &euid, float distance, int quality);
void printNeighborTable();
void recieveMode();

void setup()
{
  Serial.begin(115200);

#if defined(ESP32)
  Serial.println("!! ESP32 DEVICE VAR DEFINED !!");
#endif

  generateUniqueId(EUID);
  Serial.print("[INFO]Unique Device ID: ");
  Serial.println(EUID);

  Serial.println(F("### DW1000Ng-ESP32-peer-communicator ###"));
  // initialize the driver
  DW1000Ng::initializeNoInterrupt(PIN_SS);
  Serial.println(F("DW1000Ng initialized ..."));

  DW1000Ng::applyConfiguration(DEFAULT_CONFIG);

  DW1000Ng::setEUI(EUID);

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
}

void loop()
{
  // Blink every 300ms
  if (millis() - lastBlinkTime > blinkRate)
  {
    lastBlinkTime = millis();
    BlinkWave bw;
    bw.euid = EUID;
    bw.t1 = DW1000Ng::getSystemTimestamp();
    bw.carrier = thisCarrierType;

    WaveMessage message(bw);
    transmitWaveMessage(message);
  }

  recieveMode();

  if (millis() - lastNeighborTablePrint > 3000)
  {
    lastNeighborTablePrint = millis();
    printNeighborTable();
  }
}

void recieveMode()
{
  WaveMessage message;
  uint32_t receiveTimestamp;

  receiveWaveMessage(message, receiveTimestamp);

  switch (message.type)
  {
  case WaveType::BLINK:
  {
    if (message.blink.euid == "")
    {
      break;
    }
    {
      AnswerWave aw;
      aw.euid = EUID;
      aw.initiator_euid = message.blink.euid;
      aw.t1 = message.blink.t1;               // already from DW1000
      aw.t2 = receiveTimestamp;               // from DW1000Ng::getReceiveTimestamp()
      aw.t3 = DW1000Ng::getSystemTimestamp(); // now from DW1000 (not micros())
      aw.taxi_time = aw.t3 - aw.t2;           // in DW1000 raw units
      aw.carrier = thisCarrierType;

      // Serial.printf("Blink received: t1: %u, t2: %u, t3: %u, receiveTimestamp: %u\n", aw.t1, aw.t2, aw.t3, receiveTimestamp);

      WaveMessage response(aw);
      transmitWaveMessage(response);
    }

    break;
  }

  case WaveType::ANSWER:
  {
    if (message.answer.euid == "" || message.answer.euid == EUID)
    {
      Serial.printf("ANSWER DITCHED EUID: %s\n", message.answer.euid.c_str());
      break;
    }

    // Now all times are assumed to be in microseconds:
    uint64_t t1 = message.answer.t1;            // transmit time of Blink (in µs)
    uint64_t t4 = receiveTimestamp;             // receive time of Answer (in µs)
    uint64_t T_taxi = message.answer.taxi_time; // processing time in µs

    // Serial.printf("Answer received: t1: %llu, t2: %u, t3: %u, t4: %llu, T_taxi: %llu\n",
    //               t1, message.answer.t2, message.answer.t3, t4, T_taxi);

    double distance = calculateDistance(t1, t4, T_taxi);
    // Serial.printf("Calculated distance: %f\n", distance);

    updateNeighborTable(message.answer.euid, distance, 0);
    break;
  }

  case WaveType::RESULT:
  {
    updateNeighborTable(message.result.initiator_euid, message.result.calculated_distance, 0);
    break;
  }
  }
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

void receiveWaveMessage(WaveMessage &message, uint32_t &recieveTimestamp)
{
  DW1000Ng::startReceive();
  unsigned long startTime = millis();
  const unsigned long timeout = 100;

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

  recieveTimestamp = DW1000Ng::getReceiveTimestamp();

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
    // Serialize BlinkWave: euid, t1, carrier
    memcpy(buffer + offset, message.blink.euid.c_str(), message.blink.euid.length() + 1); // include null terminator
    offset += message.blink.euid.length() + 1;
    memcpy(buffer + offset, &message.blink.t1, sizeof(message.blink.t1));
    offset += sizeof(message.blink.t1);
    buffer[offset++] = static_cast<uint8_t>(message.blink.carrier);
    break;

  case WaveType::ANSWER:
    // Serialize AnswerWave: euid, initiator_euid, t1, t2, t3, taxi_time, carrier
    memcpy(buffer + offset, message.answer.euid.c_str(), message.answer.euid.length() + 1);
    offset += message.answer.euid.length() + 1;
    memcpy(buffer + offset, message.answer.initiator_euid.c_str(), message.answer.initiator_euid.length() + 1);
    offset += message.answer.initiator_euid.length() + 1;
    memcpy(buffer + offset, &message.answer.t1, sizeof(message.answer.t1));
    offset += sizeof(message.answer.t1);
    memcpy(buffer + offset, &message.answer.t2, sizeof(message.answer.t2));
    offset += sizeof(message.answer.t2);
    memcpy(buffer + offset, &message.answer.t3, sizeof(message.answer.t3));
    offset += sizeof(message.answer.t3);
    memcpy(buffer + offset, &message.answer.taxi_time, sizeof(message.answer.taxi_time));
    offset += sizeof(message.answer.taxi_time);
    buffer[offset++] = static_cast<uint8_t>(message.answer.carrier);
    break;

  case WaveType::RESULT:
    // Serialize ResultWave: initiator_euid, responder_euid, calculated_distance
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

void deserializeWaveMessage(const uint8_t *buffer, size_t length, WaveMessage &message)
{
  message.type = static_cast<WaveType>(buffer[0]);
  size_t offset = 1;

  switch (message.type)
  {
  case WaveType::BLINK:
    new (&message.blink) BlinkWave(); // placement new to properly reinitialize union member
    message.blink.euid = String((char *)(buffer + offset));
    offset += message.blink.euid.length() + 1;
    memcpy(&message.blink.t1, buffer + offset, sizeof(message.blink.t1));
    offset += sizeof(message.blink.t1);
    message.blink.carrier = static_cast<CarrierType>(buffer[offset++]);
    break;

  case WaveType::ANSWER:
    new (&message.answer) AnswerWave();
    message.answer.euid = String((char *)(buffer + offset));
    offset += message.answer.euid.length() + 1;
    message.answer.initiator_euid = String((char *)(buffer + offset));
    offset += message.answer.initiator_euid.length() + 1;
    memcpy(&message.answer.t1, buffer + offset, sizeof(message.answer.t1));
    offset += sizeof(message.answer.t1);
    memcpy(&message.answer.t2, buffer + offset, sizeof(message.answer.t2));
    offset += sizeof(message.answer.t2);
    memcpy(&message.answer.t3, buffer + offset, sizeof(message.answer.t3));
    offset += sizeof(message.answer.t3);
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

void updateNeighborTable(const String &euid, float distance, int quality)
{
  uint32_t now = millis();
  // Search for an existing entry with the same EUID
  for (auto &neighbor : neighborTable)
  {
    if (neighbor.euid == euid)
    {
      neighbor.lastDistance = distance;
      neighbor.lastUpdateTime = now;
      neighbor.signalQuality = quality;
      return; // Entry updated, exit
    }
  }

  // If not found, add a new entry
  NeighborEntry newNeighbor;
  newNeighbor.euid = euid;
  newNeighbor.lastDistance = distance;
  newNeighbor.lastUpdateTime = now;
  newNeighbor.signalQuality = quality;
  neighborTable.push_back(newNeighbor);
}

// Function to print the neighbor table
void printNeighborTable()
{
  Serial.println("[INFO] Neighbor Table:");
  for (const auto &neighbor : neighborTable)
  {
    Serial.print("EUID: ");
    Serial.print(neighbor.euid);
    Serial.print(" | Last Distance: ");
    Serial.print(neighbor.lastDistance, 2); // Print float with 2 decimal places
    Serial.print(" | Last Update: ");
    Serial.print(neighbor.lastUpdateTime);
    Serial.print(" | Signal Quality: ");
    Serial.println(neighbor.signalQuality);
  }
}