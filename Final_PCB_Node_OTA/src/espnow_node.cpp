#include "espnow_node.h"

#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "project_config.h"
#include "node_time_sync.h"

namespace {

uint16_t sequenceNumber = 0;
unsigned long lastLegacySendTime = 0;
volatile unsigned long lastSendSuccessTime = 0;
uint8_t activeEspNowChannel = ESPNOW_WIFI_CHANNEL;

bool espNowStarted = false;
uint8_t consecutiveFailCount = 0;

// Shared only between the ESP-NOW callback and the application transmit task.
portMUX_TYPE sendStateMux = portMUX_INITIALIZER_UNLOCKED;
volatile bool sendInProgress = false;
volatile bool callbackPending = false;
volatile esp_now_send_status_t pendingCallbackStatus = ESP_NOW_SEND_FAIL;
volatile unsigned long sendStartTime = 0;

void resetAsyncSendState() {
  portENTER_CRITICAL(&sendStateMux);
  sendInProgress = false;
  callbackPending = false;
  pendingCallbackStatus = ESP_NOW_SEND_FAIL;
  sendStartTime = 0;
  portEXIT_CRITICAL(&sendStateMux);
}

void recordSendFailure(const char *message) {
  if (consecutiveFailCount < UINT8_MAX) {
    consecutiveFailCount++;
  }

  setEspNowStatus(false);

  if (message != nullptr) {
    Serial.println(message);
  }
}

bool readSendBusyState(unsigned long &startedAtMs) {
  bool busy;

  portENTER_CRITICAL(&sendStateMux);
  busy = sendInProgress;
  startedAtMs = sendStartTime;
  portEXIT_CRITICAL(&sendStateMux);

  return busy;
}

void configureEspNowRadio() {
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);

  if (WiFi.status() == WL_CONNECTED) {
    activeEspNowChannel = WiFi.channel();

    Serial.print("Node OTA Wi-Fi connected. ESP-NOW using channel: ");
    Serial.println(activeEspNowChannel);

    if (activeEspNowChannel != ESPNOW_WIFI_CHANNEL) {
      Serial.print("WARNING: Expected ESP-NOW channel ");
      Serial.print(ESPNOW_WIFI_CHANNEL);
      Serial.print(" but router is on channel ");
      Serial.println(activeEspNowChannel);
    }
  } else {
    WiFi.disconnect(false, false);
    vTaskDelay(pdMS_TO_TICKS(100));

    esp_wifi_set_promiscuous(true);
    esp_wifi_set_channel(ESPNOW_WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);
    esp_wifi_set_promiscuous(false);

    activeEspNowChannel = ESPNOW_WIFI_CHANNEL;

    Serial.print("Node fixed ESP-NOW channel: ");
    Serial.println(activeEspNowChannel);
  }

  Serial.print("Node STA MAC Address: ");
  Serial.println(WiFi.macAddress());
}

// Executes in the high-priority Wi-Fi task. Store only the result.
void onDataSent(const uint8_t *macAddress, esp_now_send_status_t status) {
  (void)macAddress;

  portENTER_CRITICAL(&sendStateMux);
  pendingCallbackStatus = status;
  callbackPending = true;
  sendInProgress = false;
  portEXIT_CRITICAL(&sendStateMux);
}

// Keep receive callback work short. node_time_sync.cpp copies the response into
// a one-item FreeRTOS queue and validates it later in NodeESPNowTx.
void onEspNowDataRecv(const uint8_t *mac,
                      const uint8_t *incomingData,
                      int len) {
  handleTimeSyncResponseFromCenter(mac, incomingData, len);
}

bool restartAfterFailureLimitIfNeeded() {
  if (consecutiveFailCount < ESPNOW_RESTART_FAIL_LIMIT) {
    return true;
  }

  Serial.println("Too many ESP-NOW failures. Restarting ESP-NOW...");
  return restartEspNow();
}

}  // namespace

void initEspNowLedPins() {
  pinMode(LED_ESPNOW_OK, OUTPUT);
  pinMode(LED_ESPNOW_FAIL, OUTPUT);
  setEspNowStatus(false);
}

void setEspNowStatus(bool ok) {
  if (ok) {
    digitalWrite(LED_ESPNOW_OK, HIGH);
    digitalWrite(LED_ESPNOW_FAIL, LOW);
  } else {
    digitalWrite(LED_ESPNOW_OK, LOW);
    digitalWrite(LED_ESPNOW_FAIL, HIGH);
  }
}

bool restartEspNow() {
  if (espNowStarted) {
    esp_now_deinit();
    espNowStarted = false;
    vTaskDelay(pdMS_TO_TICKS(50));
  }

  resetAsyncSendState();
  configureEspNowRadio();

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed");
    setEspNowStatus(false);
    return false;
  }

  esp_now_register_send_cb(onDataSent);
  esp_now_register_recv_cb(onEspNowDataRecv);

  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, CENTER_MAC_ADDRESS, 6);
  peerInfo.channel = activeEspNowChannel;
  peerInfo.encrypt = false;
  peerInfo.ifidx = WIFI_IF_STA;

  if (esp_now_is_peer_exist(CENTER_MAC_ADDRESS)) {
    esp_now_del_peer(CENTER_MAC_ADDRESS);
  }

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add center/master peer");
    esp_now_deinit();
    setEspNowStatus(false);
    return false;
  }

  espNowStarted = true;
  consecutiveFailCount = 0;
  lastSendSuccessTime = millis();

  Serial.print("ESP-NOW ready on channel ");
  Serial.println(activeEspNowChannel);

  return true;
}

bool isEspNowStarted() {
  return espNowStarted;
}

bool isEspNowSendBusy() {
  unsigned long ignoredStartTime = 0;
  return readSendBusyState(ignoredStartTime);
}

void serviceEspNowSendStatus() {
  bool hasPendingCallback = false;
  esp_now_send_status_t callbackStatus = ESP_NOW_SEND_FAIL;

  portENTER_CRITICAL(&sendStateMux);

  if (callbackPending) {
    hasPendingCallback = true;
    callbackStatus = pendingCallbackStatus;
    callbackPending = false;
  }

  portEXIT_CRITICAL(&sendStateMux);

  if (!hasPendingCallback) {
    return;
  }

  if (callbackStatus == ESP_NOW_SEND_SUCCESS) {
    lastSendSuccessTime = millis();
    consecutiveFailCount = 0;
    setEspNowStatus(true);
  } else {
    recordSendFailure(nullptr);
    restartAfterFailureLimitIfNeeded();
  }
}

bool sendEspNowPacket(const uint8_t *data, size_t length) {
  if (data == nullptr || length == 0) {
    return false;
  }

  serviceEspNowSendStatus();

  if (!espNowStarted) {
    setEspNowStatus(false);
    return false;
  }

  unsigned long sendStartedAt = 0;

  if (readSendBusyState(sendStartedAt)) {
    const unsigned long elapsed = millis() - sendStartedAt;

    if (elapsed <= ESPNOW_SEND_CALLBACK_TIMEOUT_MS) {
      // Do not queue another packet before the previous callback completes.
      return false;
    }

    Serial.println("ESP-NOW callback timeout. Restarting ESP-NOW...");
    recordSendFailure(nullptr);

    if (!restartEspNow()) {
      return false;
    }
  }

  portENTER_CRITICAL(&sendStateMux);
  callbackPending = false;
  sendInProgress = true;
  sendStartTime = millis();
  portEXIT_CRITICAL(&sendStateMux);

  const esp_err_t result = esp_now_send(
    CENTER_MAC_ADDRESS,
    data,
    length
  );

  if (result != ESP_OK) {
    resetAsyncSendState();
    recordSendFailure("ESP-NOW send function failed.");

    Serial.print("ESP-NOW error code: ");
    Serial.println(result);

    restartAfterFailureLimitIfNeeded();
    return false;
  }

  return true;
}

bool sendImuFrameNow(IMU_Node_Frame &frame) {
  frame.timestamp_ms = getNodeTimestampMs();
  frame.sequenceNo = sequenceNumber;

  frame.checksum = 0;
  frame.checksum = calculateChecksum(
    reinterpret_cast<uint8_t *>(&frame),
    sizeof(IMU_Node_Frame)
  );

  if (!sendEspNowPacket(
        reinterpret_cast<const uint8_t *>(&frame),
        sizeof(IMU_Node_Frame)
      )) {
    return false;
  }

  sequenceNumber++;

  if (sequenceNumber % 20 == 0) {
    Serial.printf(
      "Sent seq %u | Time:%lu | CH:%d | W:%.2f X:%.2f Y:%.2f Z:%.2f\n",
      frame.sequenceNo,
      static_cast<unsigned long>(frame.timestamp_ms),
      activeEspNowChannel,
      frame.quatW,
      frame.quatX,
      frame.quatY,
      frame.quatZ
    );
  }

  return true;
}

void sendImuFrameIfDue(IMU_Node_Frame &frame) {
  const unsigned long now = millis();

  if (now - lastLegacySendTime < SEND_INTERVAL_MS) {
    return;
  }

  lastLegacySendTime = now;
  sendImuFrameNow(frame);
}

void handleEspNowHeartbeat() {
  if (millis() - lastSendSuccessTime > ESPNOW_HEARTBEAT_TIMEOUT_MS) {
    setEspNowStatus(false);
  }
}
