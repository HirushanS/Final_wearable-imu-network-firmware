#ifndef SENSOR_READER_H
#define SENSOR_READER_H

#include <Arduino.h>
#include "imu_frame.h"

bool initBNO085Sensor();

// Processes one available SH-2 report and updates the matching frame fields.
// Returns true when one report was processed.
bool updateBNO085Sensor(IMU_Node_Frame &frame);

// A frame is ready after linear acceleration, gyroscope and quaternion reports
// have all been received at least once. Gyroscope is used internally for
// stationary detection but is not transmitted in the compact ESP-NOW packet.
bool isBNO085FrameReady();

void sensorFailureLoop(uint8_t failLedPin);

#endif
