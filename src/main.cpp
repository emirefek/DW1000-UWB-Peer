#include <Arduino.h>
#include <SPI.h>
#include <DW1000Ng.hpp>

// CONNECTION PINS BEGIN
#define SPI_SCK 18
#define SPI_MISO 19
#define SPI_MOSI 23
#define DW_CS 4
const uint8_t PIN_RST = 27; // reset pin
const uint8_t PIN_IRQ = 34; // irq pin
const uint8_t PIN_SS = 4;   // spi select pin
// CONNECTION PINS END

int16_t numReceived = 0; // todo check int type
String message;

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
}

void loop()
{
  DW1000Ng::startReceive();
  while (!DW1000Ng::isReceiveDone())
  {
#if defined(ESP8266)
    yield();
#endif
  }
  DW1000Ng::clearReceiveStatus();
  numReceived++;
  // get data as string
  DW1000Ng::getReceivedData(message);
  Serial.print("Received message ... #");
  Serial.println(numReceived);
  Serial.print("Data is ... ");
  Serial.println(message);
  Serial.print("RX power is [dBm] ... ");
  Serial.println(DW1000Ng::getReceivePower());
  Serial.print("Signal quality is ... ");
  Serial.println(DW1000Ng::getReceiveQuality());
}