#ifndef HARDWARE_H
#define HARDWARE_H

#include <Arduino.h>

void initHardware();
void handleHardwareTasks();

// GPIO12 is ADC2 on the ESP32. Read it once before Wi-Fi/ESP-NOW starts.
void checkBatteryLevelOnceBeforeWireless();

// Updates only the battery LEDs. This function never calls analogRead().
void handleStoredBatteryLedDisplay();

// Shows how many wireless nodes are currently active.
// void setCenterEspNowNodeCount(uint8_t connectedNodeCount);

// Each bit represents the status of Nodes 1–4.
void setCenterEspNowNodeStatus(uint8_t activeNodeMask);

// Shows whether the center BNO085 is producing readings.
void setCenterSensorWorking(bool working);

#endif