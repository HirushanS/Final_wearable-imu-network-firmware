#include "imu_frame.h"

#include <math.h>
#include <string.h>

namespace {

float clampFloat(float value, float minimumValue, float maximumValue) {
  if (value < minimumValue) {
    return minimumValue;
  }

  if (value > maximumValue) {
    return maximumValue;
  }

  return value;
}

int16_t encodeScaledFloat(float value, float scale, float limit) {
  const float clamped = clampFloat(value, -limit, limit);
  return static_cast<int16_t>(lroundf(clamped * scale));
}

}  // namespace

void clearImuFrame(IMU_Node_Frame &frame, uint8_t nodeId, uint8_t payloadType) {
  memset(&frame, 0, sizeof(IMU_Node_Frame));

  frame.startByte = IMU_FRAME_START_BYTE;
  frame.nodeID = nodeId;
  frame.payloadType = payloadType;
  frame.endByte = IMU_FRAME_END_BYTE;
}

uint16_t calculateChecksum(const uint8_t *data, size_t len) {
  uint16_t checksum = 0;

  for (size_t i = 0; i < len; i++) {
    checksum += data[i];
  }

  return checksum;
}

uint16_t calculateCompactIMUFrameChecksum(
  const CompactIMUFrame &frame
) {
  CompactIMUFrame checksumFrame = frame;
  checksumFrame.checksum = 0;

  return calculateChecksum(
    reinterpret_cast<const uint8_t *>(&checksumFrame),
    sizeof(checksumFrame)
  );
}

void encodeCompactIMUFrame(
  const IMU_Node_Frame &source,
  CompactIMUFrame &target
) {
  memset(&target, 0, sizeof(target));

  target.startByte = IMU_FRAME_START_BYTE;
  target.nodeID = source.nodeID;
  target.timestamp_ms = source.timestamp_ms;
  target.sequenceNo = source.sequenceNo;

  target.accelX = encodeScaledFloat(
    source.accelX,
    IMU_ACCEL_SCALE,
    32.767f
  );
  target.accelY = encodeScaledFloat(
    source.accelY,
    IMU_ACCEL_SCALE,
    32.767f
  );
  target.accelZ = encodeScaledFloat(
    source.accelZ,
    IMU_ACCEL_SCALE,
    32.767f
  );

  target.quatW = encodeScaledFloat(
    source.quatW,
    IMU_QUAT_SCALE,
    1.0f
  );
  target.quatX = encodeScaledFloat(
    source.quatX,
    IMU_QUAT_SCALE,
    1.0f
  );
  target.quatY = encodeScaledFloat(
    source.quatY,
    IMU_QUAT_SCALE,
    1.0f
  );
  target.quatZ = encodeScaledFloat(
    source.quatZ,
    IMU_QUAT_SCALE,
    1.0f
  );

  target.payloadType = IMU_PAYLOAD_TYPE_COMPACT_INT16;
  target.endByte = IMU_FRAME_END_BYTE;
  target.checksum = calculateCompactIMUFrameChecksum(target);
}
