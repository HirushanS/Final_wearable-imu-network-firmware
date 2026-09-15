#ifndef CENTER_TIME_SYNC_H
#define CENTER_TIME_SYNC_H

#include <Arduino.h>

constexpr uint8_t TIME_SYNC_START_BYTE = 0xA5;
constexpr uint8_t TIME_SYNC_END_BYTE = 0x5A;
constexpr uint8_t TIME_SYNC_VERSION = 1;

constexpr uint8_t TIME_SYNC_REQUEST = 1;
constexpr uint8_t TIME_SYNC_RESPONSE = 2;

constexpr uint32_t DAY_MS = 86400000UL;

typedef struct __attribute__((packed)) {
  uint8_t startByte;
  uint8_t packetType;
  uint8_t nodeID;
  uint8_t version;

  uint32_t timeOfDayMs;
  uint32_t senderMillis;

  uint16_t checksum;
  uint8_t endByte;
} TimeSyncPacket;

typedef void (*CenterServiceCallback)();

bool initCenterRealTimeClock(
  const char *ssid,
  const char *password,
  uint8_t wifiChannel,
  CenterServiceCallback serviceCallback = nullptr
);

bool syncCenterTimeFromNTP(
  CenterServiceCallback serviceCallback = nullptr
);

bool handleTimeSyncRequestFromNode(
  const uint8_t *mac,
  const uint8_t *incomingData,
  int length
);

uint64_t getCenterRealTimestampMs();
uint32_t getCenterTimeOfDayMs();
bool isCenterTimeSynced();

void printCenterTimeOnly(uint32_t timeMs);
String getCenterTimeString(uint32_t timeMs);

#endif
