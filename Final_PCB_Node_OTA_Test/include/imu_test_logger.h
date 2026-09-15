#ifndef IMU_TEST_LOGGER_H
#define IMU_TEST_LOGGER_H

#include <Arduino.h>

#include "imu_frame.h"
#include "imu_preprocessor.h"

void printIMUTestHeaderIfNeeded();
void printIMUTestLine(
  uint8_t sourceID,
  const IMU_Node_Frame &rawFrame,
  const IMU_Node_Frame &processedFrame,
  const IMUPreprocessorState &state,
  const CompactIMUFrame &compactFrame
);

#endif
