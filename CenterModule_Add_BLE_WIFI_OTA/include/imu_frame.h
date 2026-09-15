#ifndef IMU_FRAME_H
#define IMU_FRAME_H

#include <Arduino.h>

constexpr uint8_t IMU_FRAME_START_BYTE = 0xAA;
constexpr uint8_t IMU_FRAME_END_BYTE = 0x55;

// This packet layout must match the packet used by every wireless node.
typedef struct __attribute__((packed)) {
  uint8_t startByte;
  uint8_t nodeID;
  uint32_t timestamp_ms;
  uint16_t sequenceNo;

  float accelX;
  float accelY;
  float accelZ;

  float gyroX;
  float gyroY;
  float gyroZ;

  float magX;
  float magY;
  float magZ;

  float quatW;
  float quatX;
  float quatY;
  float quatZ;

  uint8_t payloadType;
  uint16_t checksum;
  uint8_t endByte;
} IMU_Node_Frame;

static_assert(
  sizeof(IMU_Node_Frame) == 64,
  "IMU_Node_Frame size mismatch. Node and center packets must match."
);

uint16_t calculateByteChecksum(const uint8_t *data, size_t length);
uint16_t calculateIMUFrameChecksum(const IMU_Node_Frame &frame);
void finalizeIMUFrame(IMU_Node_Frame &frame);

bool decodeAndValidateIMUFrame(
  const uint8_t *incomingData,
  int length,
  IMU_Node_Frame &frame
);

#endif
