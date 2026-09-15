#ifndef CENTER_DATA_MANAGER_H
#define CENTER_DATA_MANAGER_H

#include <Arduino.h>

#include "imu_frame.h"

struct PendingSourceView {
  const IMU_Node_Frame *frames;
  uint8_t count;
};

void storeCenterFrame(const IMU_Node_Frame &frame);
void storeWirelessNodeFrame(const IMU_Node_Frame &frame);

void updateCenterEspNowNodeStatus();
void printDroppedPacketCounts();

bool createPendingBatch();
bool hasPendingBatch();
PendingSourceView getPendingSource(uint8_t sourceID);
void markPendingSourceUploaded(uint8_t sourceID);
void clearPendingBatch();

#endif
