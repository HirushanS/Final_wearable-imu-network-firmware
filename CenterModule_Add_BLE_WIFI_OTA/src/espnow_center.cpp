#include "espnow_center.h"

#include <WiFi.h>
#include <esp_now.h>

#include "center_data_manager.h"
#include "center_time_sync.h"
#include "imu_frame.h"
#include "project_config.h"
#include "sync_test_logger.h"

namespace {

void onEspNowDataReceived(
  const uint8_t *mac,
  const uint8_t *incomingData,
  int length
) {
  // First check whether this is a time-
  // synchronization request from a node.
  if (
    handleTimeSyncRequestFromNode(
      mac,
      incomingData,
      length
    )
  ) {
    return;
  }

  /*
   * Record the centre timestamp as early as
   * possible for normal IMU packets.
   *
   * This timestamp is compared with the
   * timestamp contained inside the node packet.
   */
  const uint32_t centerReceiveTimeMs =
    getCenterTimeOfDayMs();

  IMU_Node_Frame frame{};

  // Decode the packet and verify start byte,
  // end byte, length and checksum.
  if (
    !decodeAndValidateIMUFrame(
      incomingData,
      length,
      frame
    )
  ) {
    return;
  }

  // Accept only wireless Nodes 1-4.
  if (
    frame.nodeID < 1 ||
    frame.nodeID >
      ProjectConfig::WIRELESS_NODE_COUNT
  ) {
    return;
  }

  /*
   * Calculate and record synchronization error:
   *
   * node timestamp - centre receive timestamp
   */
  processTimeSyncTestRecord(
    frame,
    centerReceiveTimeMs
  );

  // Continue the normal final-system operation.
  // The packet remains available for Firebase,
  // node-status monitoring and data processing.
  storeWirelessNodeFrame(frame);
}

}  // namespace

bool initEspNowCenter() {
  if (esp_now_init() != ESP_OK) {
    Serial.println(
      "ESP-NOW initialization failed."
    );

    return false;
  }

  esp_now_register_recv_cb(
    onEspNowDataReceived
  );

  Serial.print(
    "ESP-NOW receiver ready on channel: "
  );
  Serial.println(WiFi.channel());

  return true;
}