#include <WiFi.h>
#include "utils.h"

void generateUniqueId(char *uniqueId)
{
    uint8_t mac[6];
    WiFi.macAddress(mac); // Get the MAC address of the ESP32

    Serial.print("WIFI MAC address: ");
    for (int i = 0; i < 6; i++)
    {
        Serial.print(mac[i], HEX);
        if (i < 5)
        {
            Serial.print(":");
        }
    }
    Serial.println();

    // Use the MAC address and append "00:00" at the end
    sprintf(uniqueId, "%02X:%02X:%02X:%02X:%02X:%02X:00:00",
            mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}
