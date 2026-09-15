#include "center_time_sync.h"

#include <WiFi.h>
#include <esp_now.h>
#include <string.h>
#include <sys/time.h>
#include <time.h>

namespace {

const char *NTP_SERVER = "pool.ntp.org";
constexpr long GMT_OFFSET_SEC =
    (5L * 3600L) + (30L * 60L);
constexpr int DAYLIGHT_OFFSET_SEC = 0;

bool centerTimeSynced = false;

uint16_t calculateTimeSyncChecksum(
  const uint8_t *data,
  size_t length
) {
  uint16_t checksum = 0;

  for (size_t i = 0; i < length; i++) {
    checksum += data[i];
  }

  return checksum;
}

uint16_t calculatePacketChecksum(
  TimeSyncPacket packet
) {
  packet.checksum = 0;

  return calculateTimeSyncChecksum(
    reinterpret_cast<const uint8_t *>(&packet),
    sizeof(packet)
  );
}

bool addNodePeerIfNeeded(const uint8_t *mac) {
  if (esp_now_is_peer_exist(mac)) {
    return true;
  }

  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, mac, 6);
  peerInfo.channel = WiFi.channel();
  peerInfo.encrypt = false;
  peerInfo.ifidx = WIFI_IF_STA;

  const esp_err_t result =
      esp_now_add_peer(&peerInfo);

  if (result != ESP_OK) {
    Serial.print(
      "Failed to add node peer for time response. Error: "
    );
    Serial.println(result);
    return false;
  }

  Serial.println("Node peer added for time response.");
  return true;
}

}  // namespace

bool initCenterRealTimeClock(
  const char *ssid,
  const char *password,
  uint8_t wifiChannel,
  CenterServiceCallback serviceCallback
) {
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.setAutoReconnect(true);

  Serial.print("Connecting to WiFi on channel ");
  Serial.println(wifiChannel);

  WiFi.begin(ssid, password, wifiChannel);

  while (WiFi.status() != WL_CONNECTED) {
    if (serviceCallback != nullptr) {
      serviceCallback();
    }

    delay(500);
    Serial.print(".");
  }

  Serial.println();
  Serial.println(
    "WiFi connected! IP: " +
    WiFi.localIP().toString()
  );

  Serial.print("Center STA MAC Address: ");
  Serial.println(WiFi.macAddress());

  Serial.print("Center WiFi Channel: ");
  Serial.println(WiFi.channel());

  if (WiFi.channel() != wifiChannel) {
    Serial.print(
      "WARNING: Center is not connected on fixed channel "
    );
    Serial.println(wifiChannel);
    Serial.println(
      "Check the router channel. "
      "All nodes must use the center channel."
    );
  }

  return syncCenterTimeFromNTP(serviceCallback);
}

bool syncCenterTimeFromNTP(
  CenterServiceCallback serviceCallback
) {
  configTime(
    GMT_OFFSET_SEC,
    DAYLIGHT_OFFSET_SEC,
    NTP_SERVER
  );

  struct tm timeInfo;

  Serial.print("Getting real time from NTP");

  while (!getLocalTime(&timeInfo)) {
    if (serviceCallback != nullptr) {
      serviceCallback();
    }

    Serial.print(".");
    delay(500);
  }

  centerTimeSynced = true;

  Serial.println();
  Serial.print("Real time synchronized: ");
  printCenterTimeOnly(getCenterTimeOfDayMs());

  return true;
}

uint64_t getCenterRealTimestampMs() {
  struct timeval timeValue;
  gettimeofday(&timeValue, nullptr);

  return
      (static_cast<uint64_t>(timeValue.tv_sec) * 1000ULL) +
      (timeValue.tv_usec / 1000ULL);
}

uint32_t getCenterTimeOfDayMs() {
  struct tm timeInfo;
  struct timeval timeValue;

  if (!getLocalTime(&timeInfo)) {
    return 0;
  }

  gettimeofday(&timeValue, nullptr);

  const uint32_t milliseconds =
      timeValue.tv_usec / 1000UL;

  return
      (timeInfo.tm_hour * 3600000UL) +
      (timeInfo.tm_min * 60000UL) +
      (timeInfo.tm_sec * 1000UL) +
      milliseconds;
}

bool isCenterTimeSynced() {
  return centerTimeSynced;
}

bool handleTimeSyncRequestFromNode(
  const uint8_t *mac,
  const uint8_t *incomingData,
  int length
) {
  if (length != static_cast<int>(sizeof(TimeSyncPacket))) {
    return false;
  }

  TimeSyncPacket requestPacket;
  memcpy(
    &requestPacket,
    incomingData,
    sizeof(requestPacket)
  );

  if (
    requestPacket.startByte != TIME_SYNC_START_BYTE ||
    requestPacket.endByte != TIME_SYNC_END_BYTE
  ) {
    return false;
  }

  if (
    requestPacket.version != TIME_SYNC_VERSION ||
    requestPacket.packetType != TIME_SYNC_REQUEST
  ) {
    Serial.println("Invalid time-sync packet received.");
    return true;
  }

  const uint16_t receivedChecksum =
      requestPacket.checksum;

  const uint16_t calculatedChecksum =
      calculatePacketChecksum(requestPacket);

  if (receivedChecksum != calculatedChecksum) {
    Serial.println("Time request checksum failed.");
    return true;
  }

  if (!centerTimeSynced) {
    Serial.println(
      "Time request received, but center time is not synced."
    );
    return true;
  }

  if (!addNodePeerIfNeeded(mac)) {
    return true;
  }

  TimeSyncPacket responsePacket = {};
  responsePacket.startByte = TIME_SYNC_START_BYTE;
  responsePacket.packetType = TIME_SYNC_RESPONSE;
  responsePacket.nodeID = requestPacket.nodeID;
  responsePacket.version = TIME_SYNC_VERSION;
  responsePacket.timeOfDayMs = getCenterTimeOfDayMs();
  responsePacket.senderMillis = millis();
  responsePacket.endByte = TIME_SYNC_END_BYTE;
  responsePacket.checksum =
      calculatePacketChecksum(responsePacket);

  const esp_err_t result = esp_now_send(
    mac,
    reinterpret_cast<uint8_t *>(&responsePacket),
    sizeof(responsePacket)
  );

  Serial.print("Time request from Node ");
  Serial.print(requestPacket.nodeID);
  Serial.print(" -> replied: ");
  printCenterTimeOnly(responsePacket.timeOfDayMs);

  if (result != ESP_OK) {
    Serial.print(
      "ESP-NOW time response failed. Error: "
    );
    Serial.println(result);
  }

  return true;
}

void printCenterTimeOnly(uint32_t timeMs) {
  Serial.println(getCenterTimeString(timeMs));
}

String getCenterTimeString(uint32_t timeMs) {
  const uint32_t hours = timeMs / 3600000UL;
  timeMs %= 3600000UL;

  const uint32_t minutes = timeMs / 60000UL;
  timeMs %= 60000UL;

  const uint32_t seconds = timeMs / 1000UL;
  const uint32_t milliseconds = timeMs % 1000UL;

  char buffer[20];

  snprintf(
    buffer,
    sizeof(buffer),
    "%02lu:%02lu:%02lu.%03lu",
    static_cast<unsigned long>(hours),
    static_cast<unsigned long>(minutes),
    static_cast<unsigned long>(seconds),
    static_cast<unsigned long>(milliseconds)
  );

  return String(buffer);
}
