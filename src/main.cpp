#include <Arduino.h>
#include <SPI.h>
#include <DW1000Ng.hpp>
#include <DW1000NgUtils.hpp>
#include <ranging.h>
#include <utils.h>
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

// Extended Unique Identifier register. 64-bit device identifier.
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
  // t1 is now not sent over the air, only stored locally.
  uint32_t blinkHash; // Random hash for this blink
  CarrierType carrier;
};

struct AnswerWave
{
  WaveType type = WaveType::ANSWER;
  String euid;
  String initiator_euid;
  // t1 removed from answer message.
  uint64_t t2;        // ANSWER reception timestamp (raw DW1000 value)
  uint64_t t3;        // ANSWER transmission timestamp (raw DW1000 value)
  uint64_t taxi_time; // Processing delay (t3 - t2) in raw DW1000 units
  uint32_t blinkHash; // Echoed blink hash from the original blink
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

  // Destructor - Required for union usage.
  ~WaveMessage() {}
};

struct NeighborEntry
{
  String euid;             // Unique identifier of the neighbor
  float lastDistance;      // Last calculated distance (or other metric)
  uint32_t lastUpdateTime; // Timestamp of the last received message (using millis() for display)
  int signalQuality;       // Optional quality/signal metric
};

// Global neighbor table
std::vector<NeighborEntry> neighborTable;

uint32_t lastNeighborTablePrint = 0;
const uint16_t blinkRate = 300;
uint32_t lastBlinkTime = 0;

// Global storage for the last BLINK event (local t1 stored but not sent)
uint64_t lastBlinkT1 = 0;
uint32_t lastBlinkHash = 0;

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
void receiveWaveMessage(WaveMessage &message, uint64_t &receiveTimestamp);
void serializeWaveMessage(const WaveMessage &message, uint8_t *buffer, size_t &length);
void deserializeWaveMessage(const uint8_t *buffer, size_t length, WaveMessage &message);
void updateNeighborTable(const String &euid, float distance, int quality);
void printNeighborTable();
void recieveMode();

// Forward declaration of calculateDistance from ranging.cpp
double calculateDistance(uint64_t t1, uint64_t t4, uint64_t T_taxi);

void setup()
{
  Serial.begin(115200);
#if defined(ESP32)
  Serial.println("!! ESP32 DEVICE VAR DEFINED !!");
#endif

  generateUniqueId(EUID);
  Serial.print("[INFO] Unique Device ID: ");
  Serial.println(EUID);

  Serial.println(F("### DW1000Ng-ESP32-peer-communicator ###"));
  DW1000Ng::initializeNoInterrupt(PIN_SS);
  Serial.println(F("DW1000Ng initialized ..."));

  DW1000Ng::applyConfiguration(DEFAULT_CONFIG);
  DW1000Ng::setEUI(EUID);
  DW1000Ng::setDeviceAddress(6);
  DW1000Ng::setNetworkId(10);
  DW1000Ng::setAntennaDelay(16436);
  Serial.println(F("Committed configuration ..."));

  char msg[128];
  DW1000Ng::getPrintableDeviceIdentifier(msg);
  Serial.print("[INFO] Device ID: ");
  Serial.println(msg);
  DW1000Ng::getPrintableExtendedUniqueIdentifier(msg);
  Serial.print("[INFO] Unique ID: ");
  Serial.println(msg);
  DW1000Ng::getPrintableNetworkIdAndShortAddress(msg);
  Serial.print("[INFO] Network ID & Device Address: ");
  Serial.println(msg);
  DW1000Ng::getPrintableDeviceMode(msg);
  Serial.print("[INFO] Device mode: ");
  Serial.println(msg);
}

void loop()
{
  // Send a BLINK every blinkRate milliseconds
  if (millis() - lastBlinkTime > blinkRate)
  {
    lastBlinkTime = millis();
    BlinkWave bw;
    bw.euid = EUID;
    lastBlinkT1 = DW1000Ng::getSystemTimestamp();   // Capture t1 locally (DW1000 raw system timestamp)
    bw.blinkHash = static_cast<uint32_t>(random()); // Generate a random hash for this BLINK
    lastBlinkHash = bw.blinkHash;
    bw.carrier = thisCarrierType;

    // Note: Do not transmit t1 over protocol.
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
  uint64_t receiveTimestamp;
  receiveWaveMessage(message, receiveTimestamp);

  switch (message.type)
  {
  case WaveType::BLINK:
  {
    if (message.blink.euid == "")
      break;
    // Respond to BLINK with an ANSWER containing the blinkHash.
    AnswerWave aw;
    aw.euid = EUID;
    aw.initiator_euid = message.blink.euid;
    // Do not copy t1 from blink, since blinker already stored it.
    aw.t2 = receiveTimestamp;               // ANSWER reception timestamp
    aw.t3 = DW1000Ng::getSystemTimestamp(); // ANSWER transmission timestamp
    aw.taxi_time = aw.t3 - aw.t2;           // Processing delay in raw DW1000 units
    aw.blinkHash = message.blink.blinkHash; // Echo the blink hash
    aw.carrier = thisCarrierType;

    WaveMessage response(aw);
    transmitWaveMessage(response);
    break;
  }

  case WaveType::ANSWER:
  {
    if (message.answer.euid == "" || message.answer.euid == EUID)
    {
      Serial.printf("ANSWER DITCHED EUID: %s\n", message.answer.euid.c_str());
      break;
    }
    if (message.answer.blinkHash != lastBlinkHash)
    {
      Serial.println("Received ANSWER for a different BLINK; discarding.");
      break;
    }
    // Use the stored BLINK t1 with ANSWER timestamps to calculate distance.
    uint64_t t1 = lastBlinkT1;
    uint64_t t4 = receiveTimestamp; // ANSWER reception timestamp at blinker
    uint64_t T_taxi = message.answer.taxi_time;

    double distance = calculateDistance(t1, t4, T_taxi);
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
}

void receiveWaveMessage(WaveMessage &message, uint64_t &receiveTimestamp)
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
  receiveTimestamp = DW1000Ng::getReceiveTimestamp(); // DW1000 raw timestamp for reception
  size_t length = sizeof(buffer);
  deserializeWaveMessage(buffer, length, message);
}

void serializeWaveMessage(const WaveMessage &message, uint8_t *buffer, size_t &length)
{
  buffer[0] = static_cast<uint8_t>(message.type);
  size_t offset = 1;
  switch (message.type)
  {
  case WaveType::BLINK:
    memcpy(buffer + offset, message.blink.euid.c_str(), message.blink.euid.length() + 1);
    offset += message.blink.euid.length() + 1;
    // Do not send t1.
    memcpy(buffer + offset, &message.blink.blinkHash, sizeof(message.blink.blinkHash));
    offset += sizeof(message.blink.blinkHash);
    buffer[offset++] = static_cast<uint8_t>(message.blink.carrier);
    break;
  case WaveType::ANSWER:
    memcpy(buffer + offset, message.answer.euid.c_str(), message.answer.euid.length() + 1);
    offset += message.answer.euid.length() + 1;
    memcpy(buffer + offset, message.answer.initiator_euid.c_str(), message.answer.initiator_euid.length() + 1);
    offset += message.answer.initiator_euid.length() + 1;
    // Do not send t1.
    memcpy(buffer + offset, &message.answer.t2, sizeof(message.answer.t2));
    offset += sizeof(message.answer.t2);
    memcpy(buffer + offset, &message.answer.t3, sizeof(message.answer.t3));
    offset += sizeof(message.answer.t3);
    memcpy(buffer + offset, &message.answer.taxi_time, sizeof(message.answer.taxi_time));
    offset += sizeof(message.answer.taxi_time);
    memcpy(buffer + offset, &message.answer.blinkHash, sizeof(message.answer.blinkHash));
    offset += sizeof(message.answer.blinkHash);
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

void deserializeWaveMessage(const uint8_t *buffer, size_t length, WaveMessage &message)
{
  message.type = static_cast<WaveType>(buffer[0]);
  size_t offset = 1;
  switch (message.type)
  {
  case WaveType::BLINK:
    new (&message.blink) BlinkWave();
    message.blink.euid = String((char *)(buffer + offset));
    offset += message.blink.euid.length() + 1;
    // Do not receive t1.
    memcpy(&message.blink.blinkHash, buffer + offset, sizeof(message.blink.blinkHash));
    offset += sizeof(message.blink.blinkHash);
    message.blink.carrier = static_cast<CarrierType>(buffer[offset++]);
    break;
  case WaveType::ANSWER:
    new (&message.answer) AnswerWave();
    message.answer.euid = String((char *)(buffer + offset));
    offset += message.answer.euid.length() + 1;
    message.answer.initiator_euid = String((char *)(buffer + offset));
    offset += message.answer.initiator_euid.length() + 1;
    // Do not receive t1.
    memcpy(&message.answer.t2, buffer + offset, sizeof(message.answer.t2));
    offset += sizeof(message.answer.t2);
    memcpy(&message.answer.t3, buffer + offset, sizeof(message.answer.t3));
    offset += sizeof(message.answer.t3);
    memcpy(&message.answer.taxi_time, buffer + offset, sizeof(message.answer.taxi_time));
    offset += sizeof(message.answer.taxi_time);
    memcpy(&message.answer.blinkHash, buffer + offset, sizeof(message.answer.blinkHash));
    offset += sizeof(message.answer.blinkHash);
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
  for (auto &neighbor : neighborTable)
  {
    if (neighbor.euid == euid)
    {
      neighbor.lastDistance = distance;
      neighbor.lastUpdateTime = now;
      neighbor.signalQuality = quality;
      return;
    }
  }
  NeighborEntry newNeighbor;
  newNeighbor.euid = euid;
  newNeighbor.lastDistance = distance;
  newNeighbor.lastUpdateTime = now;
  newNeighbor.signalQuality = quality;
  neighborTable.push_back(newNeighbor);
}

void printNeighborTable()
{
  Serial.println("[INFO] Neighbor Table:");
  for (const auto &neighbor : neighborTable)
  {
    Serial.print("EUID: ");
    Serial.print(neighbor.euid);
    Serial.print(" | Last Distance: ");
    Serial.print(neighbor.lastDistance, 2);
    Serial.print(" | Last Update: ");
    Serial.print(neighbor.lastUpdateTime);
    Serial.print(" | Signal Quality: ");
    Serial.println(neighbor.signalQuality);
  }
}