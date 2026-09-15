#ifndef IMU_FRAME_H
#define IMU_FRAME_H

#include <Arduino.h>

// ===============================
// DATA FRAME
// ===============================
typedef struct __attribute__((packed)) {
  uint8_t  startByte;        // 0xAA
  uint8_t  nodeID;           // 1, 2, center = 0
  uint32_t timestamp_ms;
  uint16_t sequenceNo;

  float accelX; float accelY; float accelZ;
  float gyroX;  float gyroY;  float gyroZ;
  float magX;   float magY;   float magZ;
  float quatW;  float quatX;  float quatY; float quatZ;

  uint8_t  payloadType;
  uint16_t checksum;
  uint8_t  endByte;          // 0x55
} IMU_Node_Frame;

void clearImuFrame(IMU_Node_Frame &frame, uint8_t nodeId, uint8_t payloadType);
uint16_t calculateChecksum(const uint8_t *data, size_t len);

#endif
