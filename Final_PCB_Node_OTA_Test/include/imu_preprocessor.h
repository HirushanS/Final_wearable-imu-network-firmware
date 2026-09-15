#ifndef IMU_PREPROCESSOR_H
#define IMU_PREPROCESSOR_H

#include <Arduino.h>

#include "imu_frame.h"

struct IMUPreprocessorConfig {
  float emaAlpha = 0.25f;
  float biasAdaptAlpha = 0.02f;
  float stationaryGyroDegPerSec = 1.0f;
  uint32_t stationaryConfirmMs = 350UL;
  float minDeadband = 0.05f;
};

struct IMUPreprocessorState {
  bool accelInitialized = false;
  float filteredX = 0.0f;
  float filteredY = 0.0f;
  float filteredZ = 0.0f;

  float biasX = 0.0f;
  float biasY = 0.0f;
  float biasZ = 0.0f;

  float deadbandX = 0.05f;
  float deadbandY = 0.05f;
  float deadbandZ = 0.05f;

  float previousQuatW = 1.0f;
  float previousQuatX = 0.0f;
  float previousQuatY = 0.0f;
  float previousQuatZ = 0.0f;
  bool hasPreviousQuaternion = false;

  uint32_t stationaryStartMs = 0;
  bool stationary = false;
};

void resetIMUPreprocessor(IMUPreprocessorState &state);
void preprocessIMUFrame(
  const IMUPreprocessorConfig &config,
  IMUPreprocessorState &state,
  IMU_Node_Frame &frame
);

#endif
