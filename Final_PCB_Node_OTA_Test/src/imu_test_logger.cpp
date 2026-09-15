#include "imu_test_logger.h"

#include <math.h>

#include "project_config.h"

namespace {

bool headerPrinted = false;

float decodeAccel(int16_t value) {
  return static_cast<float>(value) / IMU_ACCEL_SCALE;
}

float decodeQuat(int16_t value) {
  return static_cast<float>(value) / IMU_QUAT_SCALE;
}

float quaternionNorm(const IMU_Node_Frame &frame) {
  return sqrtf(
    (frame.quatW * frame.quatW) +
    (frame.quatX * frame.quatX) +
    (frame.quatY * frame.quatY) +
    (frame.quatZ * frame.quatZ)
  );
}

float gyroMagnitudeDeg(const IMU_Node_Frame &frame) {
  return sqrtf(
    (frame.gyroX * frame.gyroX) +
    (frame.gyroY * frame.gyroY) +
    (frame.gyroZ * frame.gyroZ)
  ) * 57.2957795f;
}

bool accelWasClipped(
  const IMU_Node_Frame &frame,
  const CompactIMUFrame &compactFrame
) {
  return
      fabsf(frame.accelX) > 32.767f ||
      fabsf(frame.accelY) > 32.767f ||
      fabsf(frame.accelZ) > 32.767f ||
      compactFrame.accelX == INT16_MAX ||
      compactFrame.accelX == INT16_MIN ||
      compactFrame.accelY == INT16_MAX ||
      compactFrame.accelY == INT16_MIN ||
      compactFrame.accelZ == INT16_MAX ||
      compactFrame.accelZ == INT16_MIN;
}

bool quatWasClipped(
  const IMU_Node_Frame &frame,
  const CompactIMUFrame &compactFrame
) {
  return
      fabsf(frame.quatW) > 1.0f ||
      fabsf(frame.quatX) > 1.0f ||
      fabsf(frame.quatY) > 1.0f ||
      fabsf(frame.quatZ) > 1.0f ||
      compactFrame.quatW == INT16_MAX ||
      compactFrame.quatW == INT16_MIN ||
      compactFrame.quatX == INT16_MAX ||
      compactFrame.quatX == INT16_MIN ||
      compactFrame.quatY == INT16_MAX ||
      compactFrame.quatY == INT16_MIN ||
      compactFrame.quatZ == INT16_MAX ||
      compactFrame.quatZ == INT16_MIN;
}

}  // namespace

void printIMUTestHeaderIfNeeded() {
#if IMU_PREPROCESSING_TEST_MODE
  if (headerPrinted) {
    return;
  }

  headerPrinted = true;

  Serial.println(
    "TEST,millis,source,sequence,"
    "raw_ax,raw_ay,raw_az,"
    "filtered_ax,filtered_ay,filtered_az,"
    "bias_x,bias_y,bias_z,"
    "deadband_x,deadband_y,deadband_z,"
    "gyro_deg,stationary,"
    "quat_w,quat_x,quat_y,quat_z,quat_norm,"
    "encoded_ax,encoded_ay,encoded_az,"
    "decoded_ax,decoded_ay,decoded_az,"
    "encoded_qw,encoded_qx,encoded_qy,encoded_qz,"
    "decoded_qw,decoded_qx,decoded_qy,decoded_qz,"
    "accel_clip,quat_clip"
  );
#endif
}

void printIMUTestLine(
  uint8_t sourceID,
  const IMU_Node_Frame &rawFrame,
  const IMU_Node_Frame &processedFrame,
  const IMUPreprocessorState &state,
  const CompactIMUFrame &compactFrame
) {
#if IMU_PREPROCESSING_TEST_MODE
  printIMUTestHeaderIfNeeded();

  Serial.print("TEST,");
  Serial.print(millis());
  Serial.print(',');
  Serial.print(sourceID);
  Serial.print(',');
  Serial.print(processedFrame.sequenceNo);
  Serial.print(',');

  Serial.print(rawFrame.accelX, 6);
  Serial.print(',');
  Serial.print(rawFrame.accelY, 6);
  Serial.print(',');
  Serial.print(rawFrame.accelZ, 6);
  Serial.print(',');

  Serial.print(processedFrame.accelX, 6);
  Serial.print(',');
  Serial.print(processedFrame.accelY, 6);
  Serial.print(',');
  Serial.print(processedFrame.accelZ, 6);
  Serial.print(',');

  Serial.print(state.biasX, 6);
  Serial.print(',');
  Serial.print(state.biasY, 6);
  Serial.print(',');
  Serial.print(state.biasZ, 6);
  Serial.print(',');

  Serial.print(state.deadbandX, 6);
  Serial.print(',');
  Serial.print(state.deadbandY, 6);
  Serial.print(',');
  Serial.print(state.deadbandZ, 6);
  Serial.print(',');

  Serial.print(gyroMagnitudeDeg(rawFrame), 6);
  Serial.print(',');
  Serial.print(state.stationary ? 1 : 0);
  Serial.print(',');

  Serial.print(processedFrame.quatW, 8);
  Serial.print(',');
  Serial.print(processedFrame.quatX, 8);
  Serial.print(',');
  Serial.print(processedFrame.quatY, 8);
  Serial.print(',');
  Serial.print(processedFrame.quatZ, 8);
  Serial.print(',');
  Serial.print(quaternionNorm(processedFrame), 8);
  Serial.print(',');

  Serial.print(compactFrame.accelX);
  Serial.print(',');
  Serial.print(compactFrame.accelY);
  Serial.print(',');
  Serial.print(compactFrame.accelZ);
  Serial.print(',');
  Serial.print(decodeAccel(compactFrame.accelX), 6);
  Serial.print(',');
  Serial.print(decodeAccel(compactFrame.accelY), 6);
  Serial.print(',');
  Serial.print(decodeAccel(compactFrame.accelZ), 6);
  Serial.print(',');

  Serial.print(compactFrame.quatW);
  Serial.print(',');
  Serial.print(compactFrame.quatX);
  Serial.print(',');
  Serial.print(compactFrame.quatY);
  Serial.print(',');
  Serial.print(compactFrame.quatZ);
  Serial.print(',');
  Serial.print(decodeQuat(compactFrame.quatW), 8);
  Serial.print(',');
  Serial.print(decodeQuat(compactFrame.quatX), 8);
  Serial.print(',');
  Serial.print(decodeQuat(compactFrame.quatY), 8);
  Serial.print(',');
  Serial.print(decodeQuat(compactFrame.quatZ), 8);
  Serial.print(',');

  Serial.print(accelWasClipped(processedFrame, compactFrame) ? 1 : 0);
  Serial.print(',');
  Serial.println(quatWasClipped(processedFrame, compactFrame) ? 1 : 0);
#else
  (void)sourceID;
  (void)rawFrame;
  (void)processedFrame;
  (void)state;
  (void)compactFrame;
#endif
}
