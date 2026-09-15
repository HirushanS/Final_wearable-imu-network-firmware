#include "imu_frame.h"

#include <string.h>

uint16_t calculateByteChecksum(
  const uint8_t *data,
  size_t length
) {
  uint16_t checksum = 0;

  for (size_t i = 0; i < length; i++) {
    checksum += data[i];
  }

  return checksum;
}

uint16_t calculateIMUFrameChecksum(
  const IMU_Node_Frame &frame
) {
  IMU_Node_Frame checksumFrame = frame;
  checksumFrame.checksum = 0;

  return calculateByteChecksum(
    reinterpret_cast<const uint8_t *>(&checksumFrame),
    sizeof(checksumFrame)
  );
}

void finalizeIMUFrame(IMU_Node_Frame &frame) {
  frame.checksum = calculateIMUFrameChecksum(frame);
}

bool decodeAndValidateIMUFrame(
  const uint8_t *incomingData,
  int length,
  IMU_Node_Frame &frame
) {
  if (
    incomingData == nullptr ||
    length != static_cast<int>(sizeof(IMU_Node_Frame))
  ) {
    return false;
  }

  memcpy(&frame, incomingData, sizeof(frame));

  if (
    frame.startByte != IMU_FRAME_START_BYTE ||
    frame.endByte != IMU_FRAME_END_BYTE
  ) {
    return false;
  }

  const uint16_t receivedChecksum = frame.checksum;
  const uint16_t calculatedChecksum =
      calculateIMUFrameChecksum(frame);

  return receivedChecksum == calculatedChecksum;
}
