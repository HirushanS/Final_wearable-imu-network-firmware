#ifndef OTA_UPDATE_H
#define OTA_UPDATE_H

// Connect to Wi-Fi and initialize the ESP32 Arduino OTA service.
void initOTA();

// Call frequently from loop() so OTA packets are processed.
// This also retries Wi-Fi automatically after a disconnection.
void handleOTA();

// Returns true while new firmware is being written to flash.
bool isOTAUpdating();

#endif
