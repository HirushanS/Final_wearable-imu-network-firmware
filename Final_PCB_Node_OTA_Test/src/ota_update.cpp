#include "ota_update.h"

#include <Arduino.h>
#include <ArduinoOTA.h>
#include <WiFi.h>

#include "project_config.h"

#define STRINGIFY_VALUE(value) #value
#define STRINGIFY(value) STRINGIFY_VALUE(value)

// ---------------------------------------------------------------------------
// EDIT THESE SETTINGS BEFORE THE FIRST USB UPLOAD.
// ESP32 uses a 2.4 GHz Wi-Fi network.
// Give every ESP32 node a different OTA_HOSTNAME.
// ---------------------------------------------------------------------------
static const char* WIFI_SSID = "SGM_D";
static const char* WIFI_PASSWORD = "BAD80Fd1";
static const char* OTA_HOSTNAME = "imu-node-" STRINGIFY(NODE_ID);
static const char* OTA_PASSWORD = "Hirushan";
static const uint16_t OTA_PORT = 3232;

static bool otaReady = false;
static volatile bool otaUpdateInProgress = false;
static unsigned long lastWifiRetryTime = 0;
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

  WiFi.persistent(false);
  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);
  WiFi.setHostname(OTA_HOSTNAME);

  Serial.print("Connecting to Wi-Fi for OTA");
  Serial.print(" on channel ");
  Serial.print(ESPNOW_WIFI_CHANNEL);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD, ESPNOW_WIFI_CHANNEL);
  lastWifiRetryTime = millis();

  const unsigned long connectionStart = millis();

  while (WiFi.status() != WL_CONNECTED &&
         millis() - connectionStart < 15000) {
    Serial.print('.');
    delay(250);
  }

  Serial.println();

  if (WiFi.status() == WL_CONNECTED) {
    startOTAService();
  } else {
    Serial.println(
      "Wi-Fi unavailable. Main program will continue; OTA will retry in the background."
    );
  }
}

void handleOTA() {
  if (WiFi.status() == WL_CONNECTED) {
    if (!otaReady) {
      startOTAService();
    }

    ArduinoOTA.handle();
    return;
  }

  if (millis() - lastWifiRetryTime >= 10000) {
    lastWifiRetryTime = millis();
    Serial.println("Retrying Wi-Fi connection for OTA...");
    WiFi.disconnect();
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD, ESPNOW_WIFI_CHANNEL);
  }
}

bool isOTAUpdating() {
  return otaUpdateInProgress;
}
