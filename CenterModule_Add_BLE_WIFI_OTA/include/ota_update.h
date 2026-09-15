#ifndef OTA_UPDATE_H
#define OTA_UPDATE_H

// Starts the ESP32 Arduino OTA service. Call this AFTER Wi-Fi is already
// connected (e.g. after connectWiFiWithBLEProvisioning() returns).
// This function does NOT touch Wi-Fi itself; it only arms ArduinoOTA on
// the connection the rest of the app already established.
void initOTA();

// Call frequently (every loop iteration / task cycle) so OTA packets are
// processed. If Wi-Fi is connected but OTA hasn't been armed yet (e.g. it
// reconnected after a drop), this will re-arm it automatically. This does
// NOT manage or retry the Wi-Fi connection itself - that stays the job of
// wifi_ble_provisioning.
void handleOTA();

// Returns true while new firmware is being written to flash.
bool isOTAUpdating();

#endif
