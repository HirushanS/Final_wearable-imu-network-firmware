#include "imu_preprocessor.h"

#include <math.h>

namespace {

constexpr float RADIANS_TO_DEGREES = 57.2957795f;

float applyDeadband(float value, float deadband) {
  return fabsf(value) < deadband ? 0.0f : value;
}

void normalizeQuaternion(
  float &w,
  float &x,
  float &y,
  float &z
) {
  const float norm = sqrtf(
    (w * w) +
    (x * x) +
    (y * y) +
    (z * z)
  );

  if (!isfinite(norm) || norm < 1.0e-6f) {
    w = 1.0f;
    x = 0.0f;
    y = 0.0f;
    z = 0.0f;
    return;
  }

  w /= norm;
  x /= norm;
  y /= norm;
  z /= norm;
}

void keepQuaternionContinuous(
  IMUPreprocessorState &state,
  float &w,
  float &x,
  float &y,
  float &z
) {
  if (!state.hasPreviousQuaternion) {
    return;
  }

  const float dot =
      (w * state.previousQuatW) +
      (x * state.previousQuatX) +
      (y * state.previousQuatY) +
      (z * state.previousQuatZ);

  if (dot < 0.0f) {
    w = -w;
    x = -x;
    y = -y;
    z = -z;
  }
}

void rememberQuaternion(
  IMUPreprocessorState &state,
  float w,
  float x,
  float y,
  float z
) {
  state.previousQuatW = w;
  state.previousQuatX = x;
  state.previousQuatY = y;
  state.previousQuatZ = z;
  state.hasPreviousQuaternion = true;
}

void updateStationaryState(
  const IMUPreprocessorConfig &config,
  IMUPreprocessorState &state,
  const IMU_Node_Frame &frame
) {
  const float gyroMagnitudeDeg =
      sqrtf(
        (frame.gyroX * frame.gyroX) +
        (frame.gyroY * frame.gyroY) +
        (frame.gyroZ * frame.gyroZ)
      ) * RADIANS_TO_DEGREES;

  if (gyroMagnitudeDeg < config.stationaryGyroDegPerSec) {
    if (state.stationaryStartMs == 0) {
      state.stationaryStartMs = millis();
    }

    state.stationary =
        millis() - state.stationaryStartMs >=
        config.stationaryConfirmMs;
  } else {
    state.stationaryStartMs = 0;
    state.stationary = false;
  }
}

}  // namespace

void resetIMUPreprocessor(IMUPreprocessorState &state) {
  state = IMUPreprocessorState();
}

void preprocessIMUFrame(
  const IMUPreprocessorConfig &config,
  IMUPreprocessorState &state,
  IMU_Node_Frame &frame
) {
  float quatW = frame.quatW;
  float quatX = frame.quatX;
  float quatY = frame.quatY;
  float quatZ = frame.quatZ;

  normalizeQuaternion(
    quatW,
    quatX,
    quatY,
    quatZ
  );

  keepQuaternionContinuous(
    state,
    quatW,
    quatX,
    quatY,
    quatZ
  );

  rememberQuaternion(
    state,
    quatW,
    quatX,
    quatY,
    quatZ
  );

  frame.quatW = quatW;
  frame.quatX = quatX;
  frame.quatY = quatY;
  frame.quatZ = quatZ;

  updateStationaryState(config, state, frame);

  if (state.stationary) {
    const float alpha = config.biasAdaptAlpha;
    state.biasX =
        ((1.0f - alpha) * state.biasX) +
        (alpha * frame.accelX);
    state.biasY =
        ((1.0f - alpha) * state.biasY) +
        (alpha * frame.accelY);
    state.biasZ =
        ((1.0f - alpha) * state.biasZ) +
        (alpha * frame.accelZ);
  }

  float correctedX = frame.accelX - state.biasX;
  float correctedY = frame.accelY - state.biasY;
  float correctedZ = frame.accelZ - state.biasZ;

  if (!state.accelInitialized) {
    state.filteredX = correctedX;
    state.filteredY = correctedY;
    state.filteredZ = correctedZ;
    state.accelInitialized = true;
  } else {
    const float alpha = config.emaAlpha;
    state.filteredX =
        ((1.0f - alpha) * state.filteredX) +
        (alpha * correctedX);
    state.filteredY =
        ((1.0f - alpha) * state.filteredY) +
        (alpha * correctedY);
    state.filteredZ =
        ((1.0f - alpha) * state.filteredZ) +
        (alpha * correctedZ);
  }

  frame.accelX =
      applyDeadband(state.filteredX, state.deadbandX);
  frame.accelY =
      applyDeadband(state.filteredY, state.deadbandY);
  frame.accelZ =
      applyDeadband(state.filteredZ, state.deadbandZ);

  if (state.stationary) {
    frame.accelX = 0.0f;
    frame.accelY = 0.0f;
    frame.accelZ = 0.0f;
  }
}
