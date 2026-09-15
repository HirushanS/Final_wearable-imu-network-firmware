#ifndef ESPNOW_NODE_H
#define ESPNOW_NODE_H

#include <Arduino.h>
#include "imu_frame.h"

void initEspNowLedPins();
void setEspNowStatus(bool ok);

bool restartEspNow();
bool isEspNowStarted();
bool isEspNowSendBusy();

// Serialized ESP-NOW send path used by both IMU packets and time-sync packets.
bool sendEspNowPacket(const uint8_t *data, size_t length);

// FreeRTOS path: timestamps and transmits one prepared sensor frame.
bool sendImuFrameNow(IMU_Node_Frame &frame);

// Processes the result recorded by the ESP-NOW Wi-Fi callback.
void serviceEspNowSendStatus();

// Compatibility wrapper for the previous non-RTOS loop implementation.
void sendImuFrameIfDue(IMU_Node_Frame &frame);

void handleEspNowHeartbeat();

#endif
