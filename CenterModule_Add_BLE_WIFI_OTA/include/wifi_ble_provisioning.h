#ifndef WIFI_BLE_PROVISIONING_H
#define WIFI_BLE_PROVISIONING_H

#include <Arduino.h>

typedef void (*WiFiProvisioningServiceCallback)();

// Starts BLE provisioning during boot and does not return until Wi-Fi is
// connected. Previously saved credentials are tried automatically. BLE is
// stopped after a successful connection so the rest of the application can
// run with the maximum available memory.
bool connectWiFiWithBLEProvisioning(
  uint8_t wifiChannel,
  WiFiProvisioningServiceCallback serviceCallback = nullptr
);

#endif
