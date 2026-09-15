#ifndef IMU_FRAME_H
#define IMU_FRAME_H

#include <Arduino.h>

constexpr uint8_t IMU_FRAME_START_BYTE = 0xAA;
constexpr uint8_t IMU_FRAME_END_BYTE = 0x55;
constexpr uint8_t IMU_PAYLOAD_TYPE_FULL_FLOAT = 2;
constexpr uint8_t IMU_PAYLOAD_TYPE_COMPACT_INT16 = 3;

constexpr float IMU_ACCEL_SCALE = 1000.0f;
constexpr float IMU_QUAT_SCALE = 30000.0f;

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

typedef struct __attribute__((packed)) {
  uint8_t  startByte;
  uint8_t  nodeID;
  uint32_t timestamp_ms;
  uint16_t sequenceNo;

  int16_t accelX;
  int16_t accelY;
  int16_t accelZ;

  int16_t quatW;
  int16_t quatX;
  int16_t quatY;
  int16_t quatZ;

  uint8_t  payloadType;
  uint16_t checksum;
  uint8_t  endByte;
} CompactIMUFrame;

static_assert(
  sizeof(CompactIMUFrame) == 26,
  "CompactIMUFrame size mismatch. Node and center packets must match."
);

void clearImuFrame(IMU_Node_Frame &frame, uint8_t nodeId, uint8_t payloadType);
uint16_t calculateChecksum(const uint8_t *data, size_t len);
uint16_t calculateCompactIMUFrameChecksum(const CompactIMUFrame &frame);
void encodeCompactIMUFrame(const IMU_Node_Frame &source, CompactIMUFrame &target);

#endif
