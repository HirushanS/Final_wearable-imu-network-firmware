#include "ota_update.h"

#include <Arduino.h>
#include <ArduinoOTA.h>
#include <WiFi.h>

// ---------------------------------------------------------------------------
// EDIT THESE SETTINGS BEFORE THE FIRST USB UPLOAD.
// Give this device a unique OTA_HOSTNAME if you ever add more OTA-capable
// boards to the same network (nodes, other centers, etc).
//
// NOTE: this module does NOT connect to Wi-Fi itself. This project already
// connects via connectWiFiWithBLEProvisioning() in main.cpp, on a specific
// ESP-NOW channel. If this module called WiFi.begin() with its own
// credentials it would tear down that connection and break ESP-NOW.
// ---------------------------------------------------------------------------
static const char* OTA_HOSTNAME = "imu-center";
static const char* OTA_PASSWORD = "Hirushan123";
static const uint16_t OTA_PORT = 3232;

static bool otaReady = false;
static volatile bool otaUpdateInProgress = false;
static int lastOtaProgress = -1;

static void startOTAService() {
  if (otaReady || WiFi.status() != WL_CONNECTED) {
    return;
  }

  ArduinoOTA.setPort(OTA_PORT);
  ArduinoOTA.setHostname(OTA_HOSTNAME);
  ArduinoOTA.setPassword(OTA_PASSWORD);

  ArduinoOTA.onStart([]() {
    otaUpdateInProgress = true;
    lastOtaProgress = -1;

    const char* updateType =
      (ArduinoOTA.getCommand() == U_FLASH) ? "firmware" : "filesystem";

    Serial.printf(
      "\nOTA update started (%s). Do not remove power.\n",
      updateType
    );
  });

  ArduinoOTA.onEnd([]() {
    Serial.println("\nOTA update complete. Restarting...");
  });

  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    const int percent = (total > 0)
      ? static_cast<int>((progress * 100U) / total)
      : 0;

    if (percent != lastOtaProgress && (percent % 10 == 0 || percent == 100)) {
      lastOtaProgress = percent;
      Serial.printf("OTA progress: %d%%\n", percent);
    }
  });

  ArduinoOTA.onError([](ota_error_t error) {
    otaUpdateInProgress = false;

    Serial.printf("\nOTA error [%u]: ", error);

    if (error == OTA_AUTH_ERROR) {
      Serial.println("authentication failed");
    } else if (error == OTA_BEGIN_ERROR) {
      Serial.println("begin failed");
    } else if (error == OTA_CONNECT_ERROR) {
      Serial.println("connection failed");
    } else if (error == OTA_RECEIVE_ERROR) {
      Serial.println("receive failed");
    } else if (error == OTA_END_ERROR) {
      Serial.println("end failed");
    } else {
      Serial.println("unknown error");
    }
  });

  ArduinoOTA.begin();
  otaReady = true;

  Serial.println("OTA is ready.");
  Serial.print("OTA hostname: ");
  Serial.println(OTA_HOSTNAME);
  Serial.print("OTA port: ");
  Serial.println(OTA_PORT);
  Serial.print("ESP32 IP address: ");
  Serial.println(WiFi.localIP());
  Serial.print("ESP32 Wi-Fi MAC address: ");
  Serial.println(WiFi.macAddress());
}

void initOTA() {
  Serial.println("OTA module: initialization started.");

  if (WiFi.status() != WL_CONNECTED) {
    Serial.println(
      "OTA module: Wi-Fi is not connected yet. Call initOTA() after "
      "Wi-Fi has connected. OTA will arm itself automatically from "
      "handleOTA() once Wi-Fi comes up."
    );
    return;
  }

  startOTAService();
}

void handleOTA() {
  // Wi-Fi connection/reconnection is owned by wifi_ble_provisioning.
  // This just arms/services OTA whenever a connection is present.
  if (WiFi.status() != WL_CONNECTED) {
    // Wi-Fi dropped: OTA's UDP listener won't survive this, so force a
    // re-arm the next time we see WL_CONNECTED.
    otaReady = false;
    return;
  }

  if (!otaReady) {
    startOTAService();
  }

  ArduinoOTA.handle();
}

bool isOTAUpdating() {
  return otaUpdateInProgress;
}
