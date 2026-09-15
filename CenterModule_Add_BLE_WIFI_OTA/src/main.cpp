// centreMainCode

#include <Arduino.h>
#include <WiFi.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "center_data_manager.h"
#include "center_sensor_task.h"
#include "center_time_sync.h"
#include "espnow_center.h"
#include "firebase_manager.h"
#include "hardware.h"
#include "ota_update.h"  
#include "project_config.h"
// #include "secrets.h"
#include "wifi_ble_provisioning.h"
#include "sync_test_logger.h"

namespace {

#if CONFIG_FREERTOS_UNICORE
constexpr BaseType_t CENTER_APPLICATION_CORE = 0;
#else
constexpr BaseType_t CENTER_APPLICATION_CORE = 1;
#endif

constexpr uint32_t HOUSEKEEPING_TASK_PERIOD_MS = 10UL;
constexpr uint32_t HOUSEKEEPING_TASK_STACK_BYTES = 3072UL;
constexpr UBaseType_t HOUSEKEEPING_TASK_PRIORITY = 3;

TaskHandle_t housekeepingTaskHandle = nullptr;

void serviceStartupHardware() {
  // Used only while Wi-Fi and NTP initialization are running.
  // The FreeRTOS housekeeping task starts after setup is complete.
  handleHardwareTasks();
  handleStoredBatteryLedDisplay();
}

void printChannelWarningIfNeeded() {
  if (
    WiFi.channel() ==
    ProjectConfig::ESPNOW_WIFI_CHANNEL
  ) {
    return;
  }

  Serial.println();
  Serial.println("ERROR: Wi-Fi channel mismatch!");

  Serial.print("Configured ESP-NOW channel: ");
  Serial.println(
    ProjectConfig::ESPNOW_WIFI_CHANNEL
  );

  Serial.print("Actual router channel: ");
  Serial.println(WiFi.channel());

  Serial.println(
    "Router, center and all nodes "
    "must use the same channel."
  );
}

void printStartupSummary() {
  Serial.println();

  Serial.println(
    "Center ready: Center BNO085 SPI+ "
    "Node1 + Node2 + Node3 + Node4 + "
    "Firebase + Real Time + FreeRTOS + "
    "Time-Synchronization Test"
  );

  Serial.print("Center STA MAC: ");
  Serial.println(WiFi.macAddress());

  Serial.print("Center BNO085 SPI status: ");
  Serial.println(
    isCenterSensorReady()
      ? "READY"
      : "NOT READY - retrying"
  );

  Serial.print("Free heap after initialization: ");
  Serial.println(ESP.getFreeHeap());
}

void centerHousekeepingTask(void *parameter) {
  (void)parameter;

  Serial.print(
    "Center housekeeping task running on core "
  );
  Serial.println(xPortGetCoreID());

  TickType_t lastWakeTime =
    xTaskGetTickCount();

  while (true) {
    // Service pending OTA firmware uploads. This blocks while a flash    
    // write chunk is in progress, which is expected during an update.

    handleOTA(); 
    // High-priority, short operations only.
    handleHardwareTasks();

    // LED display only. This does not read ADC2
    // after Wi-Fi and ESP-NOW have started.
    handleStoredBatteryLedDisplay();

    updateCenterEspNowNodeStatus();
    printDroppedPacketCounts();

    // Checks whether the synchronization test
    // has reached its configured duration.
    // It prints the final result when completed.
    handleTimeSyncTestLogger();

    vTaskDelayUntil(
      &lastWakeTime,
      pdMS_TO_TICKS(
        HOUSEKEEPING_TASK_PERIOD_MS
      )
    );
  }
}

bool startCenterHousekeepingTask() {
  if (housekeepingTaskHandle != nullptr) {
    return true;
  }

  const BaseType_t result =
    xTaskCreatePinnedToCore(
      centerHousekeepingTask,
      "CenterHousekeeping",
      HOUSEKEEPING_TASK_STACK_BYTES,
      nullptr,
      HOUSEKEEPING_TASK_PRIORITY,
      &housekeepingTaskHandle,
      CENTER_APPLICATION_CORE
    );

  if (result != pdPASS) {
    housekeepingTaskHandle = nullptr;

    Serial.println(
      "ERROR: Failed to create "
      "CenterHousekeeping task."
    );

    return false;
  }

  return true;
}

}  // namespace

void setup() {
  initHardware();

  Serial.begin(115200);
  delay(1000);

  Serial.println();
  Serial.println(
    "Starting center module with FreeRTOS..."
  );

  // GPIO12 is ADC2. Read the battery before
  // Wi-Fi and ESP-NOW start.
  checkBatteryLevelOnceBeforeWireless();

  // Connect to Wi-Fi and obtain the initial
  // real-world time using NTP.
  // initCenterRealTimeClock(
  //   WIFI_SSID,
  //   WIFI_PASSWORD,
  //   ProjectConfig::ESPNOW_WIFI_CHANNEL,
  //   serviceStartupHardware
  // );
  connectWiFiWithBLEProvisioning(
  ProjectConfig::ESPNOW_WIFI_CHANNEL,
  serviceStartupHardware
  );

   // Arm OTA now that Wi-Fi is connected. handleOTA() (called from the      
  // housekeeping task below) keeps it serviced and re-arms it if Wi-Fi     
  // ever drops and reconnects.                                             
  initOTA();  

  // After Wi-Fi connects, get real time from NTP.
  syncCenterTimeFromNTP(
    serviceStartupHardware
  );
  printChannelWarningIfNeeded();

  // Initialize synchronization-result storage
  // before receiving packets from the nodes.
  initTimeSyncTestLogger();

  // Start the centre BNO055 FreeRTOS task.
  startCenterSensorTask();

  // Start ESP-NOW reception.
  initEspNowCenter();

  // Configure Firebase.
  initFirebaseManager();

  printStartupSummary();

  // Start the Firebase FreeRTOS task.
  if (!startFirebaseUploadTask()) {
    Serial.println(
      "Center will continue, but Firebase "
      "uploads are disabled."
    );
  }

  // Start hardware, LED and test-summary task.
  if (!startCenterHousekeepingTask()) {
    Serial.println(
      "ERROR: Housekeeping task could not "
      "start. Restarting."
    );

    delay(1000);
    esp_restart();
  }
}

void loop() {
  // Work is performed by:
  // 1. CenterHousekeeping task
  // 2. CenterBNO055 task
  // 3. FirebaseUpload task
  // 4. ESP-NOW/Wi-Fi system tasks
  // 5. Time-synchronization logger

  vTaskDelay(
    pdMS_TO_TICKS(1000)
  );
}