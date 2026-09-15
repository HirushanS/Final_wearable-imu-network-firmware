#ifndef PROJECT_CONFIG_H
#define PROJECT_CONFIG_H

#include <Arduino.h>

namespace ProjectConfig {

// =====================================================
// HARDWARE PINS
// =====================================================
constexpr uint8_t SYS_LATCH_PIN = 13;
constexpr uint8_t POWER_BUTTON_PIN = 17;
constexpr uint8_t RESET_BUTTON_PIN = 4;

constexpr uint8_t ESPNOW_NODE_LED_1_PIN = 25;
constexpr uint8_t ESPNOW_NODE_LED_2_PIN = 26;
constexpr uint8_t ESPNOW_NODE_LED_3_PIN = 32;
constexpr uint8_t ESPNOW_NODE_LED_4_PIN = 33;
constexpr uint8_t CENTER_SENSOR_LED_PIN = 12;

constexpr uint8_t BATTERY_LED_RED_PIN = 27;
constexpr uint8_t BATTERY_LED_GREEN_PIN = 14;
constexpr uint8_t BATTERY_ADC_PIN = 35; 
constexpr uint8_t CHARGING_DETECT_PIN = 2;

// =====================================================
// =====================================================
// CENTER BNO085 SPI
// =====================================================

constexpr uint8_t CENTER_IMU_MOSI = 23;
constexpr uint8_t CENTER_IMU_MISO = 19;
constexpr uint8_t CENTER_IMU_CLK  = 18;
constexpr uint8_t CENTER_IMU_CS   = 5;
constexpr uint8_t CENTER_IMU_INT  = 16;
constexpr uint8_t CENTER_IMU_RST  = 15;
// ===============================
// STATUS LED CONFIGURATION
// ===============================

constexpr uint8_t FIREBASE_STATUS_LED = 21;

// =====================================================
// NETWORK
// =====================================================
constexpr uint8_t ESPNOW_WIFI_CHANNEL = 11;
constexpr uint8_t WIRELESS_NODE_COUNT = 4;

// =====================================================
// SAMPLE AND BUFFER SETTINGS
// =====================================================
constexpr uint8_t CENTER_SOURCE_ID = 0;
constexpr uint8_t CENTER_PAYLOAD_TYPE = 2;
constexpr uint8_t SOURCE_COUNT = WIRELESS_NODE_COUNT + 1;

constexpr uint32_t CENTER_SENSOR_INTERVAL_MS = 50UL;
constexpr uint32_t CENTER_SENSOR_RETRY_INTERVAL_MS = 5000UL;


// Turn the center sensor LED off if no reading arrives for 1 second.
constexpr uint32_t CENTER_SENSOR_LED_TIMEOUT_MS = 1000UL;

constexpr uint8_t MAX_SAMPLES_PER_SOURCE = 25;
constexpr uint32_t NODE_ACTIVE_TIMEOUT_MS = 3000UL;
constexpr uint32_t NODE_STATUS_CHECK_INTERVAL_MS = 20UL;

// =====================================================
// FIREBASE
// =====================================================
constexpr uint32_t FIREBASE_UPDATE_INTERVAL_MS = 1000UL;
constexpr uint8_t MAX_FIREBASE_RETRIES = 2;
constexpr uint16_t FIREBASE_SSL_RX_BUFFER_SIZE = 8192;
constexpr uint16_t FIREBASE_SSL_TX_BUFFER_SIZE = 1024;
constexpr uint16_t FIREBASE_RESPONSE_SIZE = 1024;

}  // namespace ProjectConfig

#endif

