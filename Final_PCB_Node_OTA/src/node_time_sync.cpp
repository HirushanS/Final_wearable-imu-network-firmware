#include "node_time_sync.h"

#include <esp_now.h>
#include <string.h>

#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>

#include "espnow_node.h"

namespace {

uint8_t nodeTimeSyncNodeID = 0;
uint8_t nodeTimeSyncCenterMac[6] = {};

bool nodeTimeSynced = false;
uint32_t syncedTimeOfDayMs = 0;
uint32_t syncedLocalMillis = 0;

unsigned long lastTimeRequestMs = 0;
constexpr unsigned long TIME_REQUEST_INTERVAL_MS = 1000UL;

QueueHandle_t timeSyncResponseQueue = nullptr;

uint16_t calculateTimeSyncChecksum(const uint8_t *data, size_t len) {
  uint16_t checksum = 0;

  for (size_t i = 0; i < len; i++) {
    checksum += data[i];
  }

  return checksum;
}

uint16_t calculatePacketChecksum(TimeSyncPacket packet) {
  packet.checksum = 0;

  return calculateTimeSyncChecksum(
    reinterpret_cast<const uint8_t *>(&packet),
    sizeof(TimeSyncPacket)
  );
}

void processPendingTimeSyncResponse() {
  if (timeSyncResponseQueue == nullptr) {
    return;
  }

  TimeSyncPacket packet;

  while (xQueueReceive(timeSyncResponseQueue, &packet, 0) == pdTRUE) {
    if (
      packet.startByte != TIME_SYNC_START_BYTE ||
      packet.endByte != TIME_SYNC_END_BYTE ||
      packet.version != TIME_SYNC_VERSION ||
      packet.packetType != TIME_SYNC_RESPONSE ||
      packet.nodeID != nodeTimeSyncNodeID
    ) {
      continue;
    }

    const uint16_t receivedChecksum = packet.checksum;
    const uint16_t calculatedChecksum = calculatePacketChecksum(packet);

    if (receivedChecksum != calculatedChecksum) {
      Serial.println("Time sync checksum failed");
      continue;
    }

    syncedTimeOfDayMs = packet.timeOfDayMs;
    syncedLocalMillis = millis();
    nodeTimeSynced = true;

    Serial.print("Real time received from center: ");
    printTimeOnly(syncedTimeOfDayMs);
  }
}

}  // namespace

void initNodeTimeSync(uint8_t nodeID, const uint8_t centerMac[6]) {
  nodeTimeSyncNodeID = nodeID;
  memcpy(nodeTimeSyncCenterMac, centerMac, 6);

  nodeTimeSynced = false;
  syncedTimeOfDayMs = 0;
  syncedLocalMillis = 0;
  lastTimeRequestMs = 0;

  if (timeSyncResponseQueue == nullptr) {
    timeSyncResponseQueue = xQueueCreate(1, sizeof(TimeSyncPacket));
  } else {
    xQueueReset(timeSyncResponseQueue);
  }

  if (timeSyncResponseQueue == nullptr) {
    Serial.println("ERROR: Failed to create time-sync response queue.");
  } else {
    Serial.println("Node time sync initialized");
  }
}

bool requestTimeFromCenter() {
  if (!isEspNowStarted()) {
    return false;
  }

  // IMU and time-sync packets share one ESP-NOW transaction at a time.
  if (isEspNowSendBusy()) {
    return false;
  }

  TimeSyncPacket packet = {};
  packet.startByte = TIME_SYNC_START_BYTE;
  packet.packetType = TIME_SYNC_REQUEST;
  packet.nodeID = nodeTimeSyncNodeID;
  packet.version = TIME_SYNC_VERSION;
  packet.timeOfDayMs = 0;
  packet.senderMillis = millis();
  packet.endByte = TIME_SYNC_END_BYTE;
  packet.checksum = calculatePacketChecksum(packet);

  Serial.print("Requesting real time from center... ");

  const bool queued = sendEspNowPacket(
    reinterpret_cast<const uint8_t *>(&packet),
    sizeof(TimeSyncPacket)
  );

  if (queued) {
    lastTimeRequestMs = millis();
    Serial.println("OK");
  } else {
    Serial.println("DEFERRED");
  }

  return queued;
}

bool waitForNodeTimeSync(uint32_t timeoutMs) {
  const unsigned long startMs = millis();

  while ((millis() - startMs) < timeoutMs) {
    handleNodeTimeSync();

    if (nodeTimeSynced) {
      Serial.println("Node time sync OK");
      return true;
    }

    vTaskDelay(pdMS_TO_TICKS(10));
  }

  Serial.println("Node time sync timeout");
  Serial.println("Node continues with timestamp 0 until synchronization.");
  return false;
}

void handleNodeTimeSync() {
  processPendingTimeSyncResponse();

  if (nodeTimeSynced) {
    return;
  }

  if (millis() - lastTimeRequestMs >= TIME_REQUEST_INTERVAL_MS) {
    requestTimeFromCenter();
  }
}

void handleTimeSyncResponseFromCenter(const uint8_t *mac,
                                      const uint8_t *incomingData,
                                      int len) {
  // Called by the ESP-NOW Wi-Fi task. Only copy a correctly sized response
  // into a one-item queue; validation and Serial output happen later.
  if (
    timeSyncResponseQueue == nullptr ||
    mac == nullptr ||
    incomingData == nullptr ||
    len != static_cast<int>(sizeof(TimeSyncPacket)) ||
    memcmp(mac, nodeTimeSyncCenterMac, 6) != 0
  ) {
    return;
  }

  TimeSyncPacket packet;
  memcpy(&packet, incomingData, sizeof(packet));
  xQueueOverwrite(timeSyncResponseQueue, &packet);
}

uint32_t getNodeTimestampMs() {
  if (!nodeTimeSynced) {
    return 0;
  }

  const uint32_t elapsedMs = millis() - syncedLocalMillis;
  return (syncedTimeOfDayMs + elapsedMs) % DAY_MS;
}

bool isNodeTimeSynced() {
  return nodeTimeSynced;
}

void printTimeOnly(uint32_t timeMs) {
  const uint32_t hours = timeMs / 3600000UL;
  timeMs %= 3600000UL;

  const uint32_t minutes = timeMs / 60000UL;
  timeMs %= 60000UL;

  const uint32_t seconds = timeMs / 1000UL;
  const uint32_t ms = timeMs % 1000UL;

  Serial.printf(
    "%02lu:%02lu:%02lu.%03lu\n",
    static_cast<unsigned long>(hours),
    static_cast<unsigned long>(minutes),
    static_cast<unsigned long>(seconds),
    static_cast<unsigned long>(ms)
  );
}
