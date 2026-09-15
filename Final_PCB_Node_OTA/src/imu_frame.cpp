#include "imu_frame.h"
#include <string.h>

void clearImuFrame(IMU_Node_Frame &frame, uint8_t nodeId, uint8_t payloadType) {
  memset(&frame, 0, sizeof(IMU_Node_Frame));

  frame.startByte = 0xAA;
  frame.nodeID = nodeId;
  frame.payloadType = payloadType;
  frame.endByte = 0x55;
}

uint16_t calculateChecksum(const uint8_t *data, size_t len) {
  uint16_t checksum = 0;

  for (size_t i = 0; i < len; i++) {
    checksum += data[i];
  }

  return checksum;
}
