#include "center_data_manager.h"

#include <string.h>

#include "hardware.h"
#include "project_config.h"

namespace {

IMU_Node_Frame receiveBuffers
  [ProjectConfig::SOURCE_COUNT]
  [ProjectConfig::MAX_SAMPLES_PER_SOURCE];

IMU_Node_Frame pendingBuffers
  [ProjectConfig::SOURCE_COUNT]
  [ProjectConfig::MAX_SAMPLES_PER_SOURCE];

volatile uint8_t receiveCounts[
  ProjectConfig::SOURCE_COUNT
] = {};

uint8_t pendingCounts[
  ProjectConfig::SOURCE_COUNT
] = {};

volatile uint32_t droppedPacketCounts[
  ProjectConfig::SOURCE_COUNT
] = {};

volatile uint32_t nodeLastSeenMs[
  ProjectConfig::WIRELESS_NODE_COUNT
] = {};

volatile uint8_t nodeSeenMask = 0;

bool pendingBatchAvailable = false;

portMUX_TYPE bufferMux =
    portMUX_INITIALIZER_UNLOCKED;

bool isValidSourceID(uint8_t sourceID) {
  return sourceID < ProjectConfig::SOURCE_COUNT;
}

bool isWirelessNodeID(uint8_t sourceID) {
  return
      sourceID >= 1 &&
      sourceID <= ProjectConfig::WIRELESS_NODE_COUNT;
}

void storeFrameLocked(
  uint8_t sourceID,
  const IMU_Node_Frame &frame
) {
  const uint8_t count = receiveCounts[sourceID];

  if (
    count <
    ProjectConfig::MAX_SAMPLES_PER_SOURCE
  ) {
    receiveBuffers[sourceID][count] = frame;
    receiveCounts[sourceID] = count + 1;
  } else {
    droppedPacketCounts[sourceID]++;
  }
}

void refreshPendingAvailability() {
  pendingBatchAvailable = false;

  for (
    uint8_t sourceID = 0;
    sourceID < ProjectConfig::SOURCE_COUNT;
    sourceID++
  ) {
    if (pendingCounts[sourceID] > 0) {
      pendingBatchAvailable = true;
      return;
    }
  }
}

void printSourceLabel(uint8_t sourceID) {
  if (sourceID == ProjectConfig::CENTER_SOURCE_ID) {
    Serial.print("Center");
    return;
  }

  Serial.print("N");
  Serial.print(sourceID);
}

}  // namespace

void storeCenterFrame(const IMU_Node_Frame &frame) {
  portENTER_CRITICAL(&bufferMux);

  storeFrameLocked(
    ProjectConfig::CENTER_SOURCE_ID,
    frame
  );

  portEXIT_CRITICAL(&bufferMux);
}

void storeWirelessNodeFrame(
  const IMU_Node_Frame &frame
) {
  if (!isWirelessNodeID(frame.nodeID)) {
    return;
  }

  const uint32_t receivedTimeMs = millis();
  const uint8_t nodeIndex = frame.nodeID - 1;

  portENTER_CRITICAL(&bufferMux);

  nodeLastSeenMs[nodeIndex] = receivedTimeMs;
  nodeSeenMask |= (1U << nodeIndex);

  storeFrameLocked(frame.nodeID, frame);

  portEXIT_CRITICAL(&bufferMux);
}

void updateCenterEspNowNodeStatus() {
  static unsigned long lastStatusCheckMs = 0;

  const unsigned long now = millis();

  if (
    now - lastStatusCheckMs <
    ProjectConfig::NODE_STATUS_CHECK_INTERVAL_MS
  ) {
    return;
  }

  lastStatusCheckMs = now;

  uint32_t lastSeenCopy[
    ProjectConfig::WIRELESS_NODE_COUNT
  ];

  uint8_t seenMaskCopy;

  portENTER_CRITICAL(&bufferMux);

  for (
    uint8_t i = 0;
    i < ProjectConfig::WIRELESS_NODE_COUNT;
    i++
  ) {
    lastSeenCopy[i] = nodeLastSeenMs[i];
  }

  seenMaskCopy = nodeSeenMask;

  portEXIT_CRITICAL(&bufferMux);

  // uint8_t activeNodeCount = 0;
  uint8_t activeNodeMask = 0;

  for (
    uint8_t i = 0;
    i < ProjectConfig::WIRELESS_NODE_COUNT;
    i++
  ) {
    const bool hasBeenSeen =
        (seenMaskCopy & (1U << i)) != 0;

    const bool isStillActive =
        hasBeenSeen &&
        (
          now - lastSeenCopy[i] <=
          ProjectConfig::NODE_ACTIVE_TIMEOUT_MS
        );

    // if (isStillActive) {
    //   activeNodeCount++;
    // }
    if (isStillActive) {
      activeNodeMask |= (1U << i);
    }
  }

  // setCenterEspNowNodeCount(activeNodeCount);
  setCenterEspNowNodeStatus(activeNodeMask);
}

bool createPendingBatch() {
  if (pendingBatchAvailable) {
    return true;
  }

  portENTER_CRITICAL(&bufferMux);

  for (
    uint8_t sourceID = 0;
    sourceID < ProjectConfig::SOURCE_COUNT;
    sourceID++
  ) {
    pendingCounts[sourceID] =
        receiveCounts[sourceID];

    if (pendingCounts[sourceID] > 0) {
      memcpy(
        pendingBuffers[sourceID],
        receiveBuffers[sourceID],
        pendingCounts[sourceID] *
          sizeof(IMU_Node_Frame)
      );
    }

    receiveCounts[sourceID] = 0;
  }

  portEXIT_CRITICAL(&bufferMux);

  refreshPendingAvailability();

  if (!pendingBatchAvailable) {
    return false;
  }

  Serial.print("Pending batch created");

  for (
    uint8_t sourceID = 0;
    sourceID < ProjectConfig::SOURCE_COUNT;
    sourceID++
  ) {
    Serial.print(" | ");
    printSourceLabel(sourceID);
    Serial.print(": ");
    Serial.print(pendingCounts[sourceID]);
  }

  Serial.println();
  return true;
}

bool hasPendingBatch() {
  return pendingBatchAvailable;
}

PendingSourceView getPendingSource(
  uint8_t sourceID
) {
  if (!isValidSourceID(sourceID)) {
    return {nullptr, 0};
  }

  return {
    pendingBuffers[sourceID],
    pendingCounts[sourceID]
  };
}

void markPendingSourceUploaded(uint8_t sourceID) {
  if (!isValidSourceID(sourceID)) {
    return;
  }

  pendingCounts[sourceID] = 0;
  refreshPendingAvailability();
}

void clearPendingBatch() {
  for (
    uint8_t sourceID = 0;
    sourceID < ProjectConfig::SOURCE_COUNT;
    sourceID++
  ) {
    pendingCounts[sourceID] = 0;
  }

  pendingBatchAvailable = false;
}

void printDroppedPacketCounts() {
  static unsigned long lastPrintMs = 0;

  if (millis() - lastPrintMs < 5000UL) {
    return;
  }

  lastPrintMs = millis();

  uint32_t droppedCopy[
    ProjectConfig::SOURCE_COUNT
  ];

  portENTER_CRITICAL(&bufferMux);

  for (
    uint8_t sourceID = 0;
    sourceID < ProjectConfig::SOURCE_COUNT;
    sourceID++
  ) {
    droppedCopy[sourceID] =
        droppedPacketCounts[sourceID];
  }

  portEXIT_CRITICAL(&bufferMux);

  Serial.print("Dropped packets");

  for (
    uint8_t sourceID = 0;
    sourceID < ProjectConfig::SOURCE_COUNT;
    sourceID++
  ) {
    Serial.print(" | ");
    printSourceLabel(sourceID);
    Serial.print(": ");
    Serial.print(droppedCopy[sourceID]);
  }

  Serial.println();
}
