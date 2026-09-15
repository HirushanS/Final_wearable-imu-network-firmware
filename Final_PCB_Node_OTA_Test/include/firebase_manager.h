#ifndef FIREBASE_MANAGER_H
#define FIREBASE_MANAGER_H

#include <Arduino.h>
#include "imu_frame.h"

// NOTE:
// Your uploaded node main.cpp does not contain Firebase code.
// Use this file only in the center/cloud ESP32, not in the pure ESP-NOW sensor node.

bool initFirebaseService(
  const char *wifiSsid,
  const char *wifiPassword,
  const char *databaseUrl,
  const char *databaseSecret
);

bool isFirebaseReady();
void handleFirebaseTask();
bool uploadImuFrameToFirebase(const IMU_Node_Frame &frame, const String &path);

#endif
