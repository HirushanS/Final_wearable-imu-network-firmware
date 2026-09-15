#ifndef PROJECT_CONFIG_H
#define PROJECT_CONFIG_H

#include <Arduino.h>

// ===============================
// NODE CONFIGURATION
// ===============================
// Use 1 for the first BNO085 node and 2 for the second BNO085 node.
#define NODE_ID 1

// Must match the center module's ESP-NOW/Wi-Fi channel.
#define ESPNOW_WIFI_CHANNEL 11

// ESP-NOW status LEDs.
#define LED_ESPNOW_OK 25
#define LED_ESPNOW_FAIL 26

// ===============================
// BNO085 ACQUISITION
// ===============================
// ===============================
// BNO085 SPI CONFIGURATION
// ===============================

#define IMU_MOSI 23
#define IMU_MISO 19
#define IMU_CLK  18
#define IMU_CS   5
#define IMU_INT  16
#define IMU_RST  15
// Request each SH-2 report at 20 Hz.
#define BNO085_REPORT_INTERVAL_US 50000UL

// Poll the BNO085 event queue frequently so all report types are collected.
#define BNO085_POLL_INTERVAL_MS 2UL

// Publish one complete snapshot to the transmitter at 20 Hz.
#define BNO085_FRAME_INTERVAL_MS 50UL

// ESP-NOW packet rate: 20 packets per second.
#define SEND_INTERVAL_MS 50UL

// A send callback should normally complete before the next packet period.
#define ESPNOW_SEND_CALLBACK_TIMEOUT_MS 200UL
#define ESPNOW_HEARTBEAT_TIMEOUT_MS 3000UL
#define ESPNOW_RESTART_FAIL_LIMIT 10

// Center/master gateway STA MAC address.
// IMPORTANT: Use the center ESP32 STA MAC, not its AP MAC.
static const uint8_t CENTER_MAC_ADDRESS[6] = {
  0x00, 0x70, 0x07, 0x47, 0x12, 0x80
};

#endif
