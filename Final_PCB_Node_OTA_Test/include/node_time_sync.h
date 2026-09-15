#ifndef NODE_TIME_SYNC_H
#define NODE_TIME_SYNC_H

#include <Arduino.h>

// ===============================
// TIME SYNC PACKET SETTINGS
// ===============================
#define TIME_SYNC_START_BYTE 0xA5
#define TIME_SYNC_END_BYTE   0x5A
#define TIME_SYNC_VERSION    1

#define TIME_SYNC_REQUEST    1
#define TIME_SYNC_RESPONSE   2

#define DAY_MS 86400000UL

// ===============================
// TIME SYNC PACKET
// ===============================
typedef struct __attribute__((packed)) {
  uint8_t  startByte;       // 0xA5
  uint8_t  packetType;      // 1=request, 2=response
  uint8_t  nodeID;
  uint8_t  version;

  uint32_t timeOfDayMs;     // Sri Lanka time of day in ms
  uint32_t senderMillis;    // millis() of sender when packet created

  uint16_t checksum;
  uint8_t  endByte;         // 0x5A
} TimeSyncPacket;

// ===============================
// NODE TIME SYNC FUNCTIONS
// ===============================
void initNodeTimeSync(uint8_t nodeID, const uint8_t centerMac[6]);

bool requestTimeFromCenter();
bool waitForNodeTimeSync(uint32_t timeoutMs);

void handleNodeTimeSync();
void handleTimeSyncResponseFromCenter(const uint8_t *mac,
                                      const uint8_t *incomingData,
                                      int len);

uint32_t getNodeTimestampMs();
bool isNodeTimeSynced();

void printTimeOnly(uint32_t timeMs);

#endif