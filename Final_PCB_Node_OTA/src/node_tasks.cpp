#include "node_tasks.h"

#include <Arduino.h>

#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>

#include "project_config.h"
#include "hardware.h"
#include "imu_frame.h"
#include "sensor_reader.h"
#include "espnow_node.h"
#include "node_time_sync.h"

namespace {

#if CONFIG_FREERTOS_UNICORE
constexpr BaseType_t NODE_APPLICATION_CORE = 0;
#else
constexpr BaseType_t NODE_APPLICATION_CORE = 1;
#endif

// The transmitter always receives the newest complete sensor snapshot.
constexpr UBaseType_t IMU_QUEUE_LENGTH = 1;

constexpr uint32_t SENSOR_TASK_STACK_BYTES = 6144UL;
constexpr uint32_t TRANSMIT_TASK_STACK_BYTES = 5120UL;
constexpr uint32_t HOUSEKEEPING_TASK_STACK_BYTES = 3072UL;

constexpr UBaseType_t SENSOR_TASK_PRIORITY = 3;
constexpr UBaseType_t TRANSMIT_TASK_PRIORITY = 2;
constexpr UBaseType_t HOUSEKEEPING_TASK_PRIORITY = 1;

constexpr uint32_t HOUSEKEEPING_PERIOD_MS = 10UL;

QueueHandle_t imuFrameQueue = nullptr;
TaskHandle_t sensorTaskHandle = nullptr;
TaskHandle_t transmitTaskHandle = nullptr;
TaskHandle_t housekeepingTaskHandle = nullptr;

void bno085AcquisitionTask(void *parameter) {
  (void)parameter;

  Serial.print("NodeBNO085 task running on core ");
  Serial.println(xPortGetCoreID());

  IMU_Node_Frame sampledFrame;
  clearImuFrame(sampledFrame, NODE_ID, 2);

  TickType_t lastPollWake = xTaskGetTickCount();
  TickType_t lastPublishTick = lastPollWake;
  bool receivedEventSincePublish = false;

  while (true) {
    // This is the only task that accesses the BNO085 and I2C bus.
    // Drain several queued SH-2 reports without monopolizing the processor.
    for (uint8_t eventIndex = 0; eventIndex < 8; eventIndex++) {
      if (!updateBNO085Sensor(sampledFrame)) {
        break;
      }

      receivedEventSincePublish = true;
    }

    const TickType_t nowTick = xTaskGetTickCount();

    if (
      receivedEventSincePublish &&
      isBNO085FrameReady() &&
      (nowTick - lastPublishTick >=
       pdMS_TO_TICKS(BNO085_FRAME_INTERVAL_MS))
    ) {
      // Atomic full-frame replacement prevents partial-frame transmission.
      xQueueOverwrite(imuFrameQueue, &sampledFrame);
      lastPublishTick = nowTick;
      receivedEventSincePublish = false;
    }

    vTaskDelayUntil(
      &lastPollWake,
      pdMS_TO_TICKS(BNO085_POLL_INTERVAL_MS)
    );
  }
}

void espNowTransmitTask(void *parameter) {
  (void)parameter;

  Serial.print("NodeESPNowTx task running on core ");
  Serial.println(xPortGetCoreID());

  IMU_Node_Frame frameToSend;
  clearImuFrame(frameToSend, NODE_ID, 2);

  // Do not transmit an empty frame; wait for the first complete snapshot.
  xQueueReceive(imuFrameQueue, &frameToSend, portMAX_DELAY);

  TickType_t lastWakeTime = xTaskGetTickCount();

  while (true) {
    // Process the lightweight result left by the Wi-Fi callback.
    serviceEspNowSendStatus();

    // Process a received time-sync response or request time again if needed.
    // Time-sync packets and IMU packets share one serialized send path.
    handleNodeTimeSync();

    // Queue length one means this retrieves the latest available snapshot.
    xQueueReceive(imuFrameQueue, &frameToSend, 0);

    // Adds synchronized timestamp, sequence number and checksum.
    sendImuFrameNow(frameToSend);

    vTaskDelayUntil(
      &lastWakeTime,
      pdMS_TO_TICKS(SEND_INTERVAL_MS)
    );
  }
}

void nodeHousekeepingTask(void *parameter) {
  (void)parameter;

  Serial.print("NodeHousekeeping task running on core ");
  Serial.println(xPortGetCoreID());

  TickType_t lastWakeTime = xTaskGetTickCount();

  while (true) {
    handleHardwareTasks();

    // LED display only; ADC2 is not read after wireless starts.
    handleStoredBatteryLedDisplay();

    handleEspNowHeartbeat();

    vTaskDelayUntil(
      &lastWakeTime,
      pdMS_TO_TICKS(HOUSEKEEPING_PERIOD_MS)
    );
  }
}

void deleteTaskIfCreated(TaskHandle_t &taskHandle) {
  if (taskHandle != nullptr) {
    vTaskDelete(taskHandle);
    taskHandle = nullptr;
  }
}

void cleanupTaskCreationFailure() {
  deleteTaskIfCreated(sensorTaskHandle);
  deleteTaskIfCreated(transmitTaskHandle);
  deleteTaskIfCreated(housekeepingTaskHandle);

  if (imuFrameQueue != nullptr) {
    vQueueDelete(imuFrameQueue);
    imuFrameQueue = nullptr;
  }
}

}  // namespace

bool startNodeFreeRTOSTasks() {
  if (
    imuFrameQueue != nullptr &&
    sensorTaskHandle != nullptr &&
    transmitTaskHandle != nullptr &&
    housekeepingTaskHandle != nullptr
  ) {
    return true;
  }

  imuFrameQueue = xQueueCreate(
    IMU_QUEUE_LENGTH,
    sizeof(IMU_Node_Frame)
  );

  if (imuFrameQueue == nullptr) {
    Serial.println("ERROR: Failed to create IMU frame queue.");
    return false;
  }

  BaseType_t result = xTaskCreatePinnedToCore(
    bno085AcquisitionTask,
    "NodeBNO085",
    SENSOR_TASK_STACK_BYTES,
    nullptr,
    SENSOR_TASK_PRIORITY,
    &sensorTaskHandle,
    NODE_APPLICATION_CORE
  );

  if (result != pdPASS) {
    Serial.println("ERROR: Failed to create NodeBNO085 task.");
    cleanupTaskCreationFailure();
    return false;
  }

  result = xTaskCreatePinnedToCore(
    espNowTransmitTask,
    "NodeESPNowTx",
    TRANSMIT_TASK_STACK_BYTES,
    nullptr,
    TRANSMIT_TASK_PRIORITY,
    &transmitTaskHandle,
    NODE_APPLICATION_CORE
  );

  if (result != pdPASS) {
    Serial.println("ERROR: Failed to create NodeESPNowTx task.");
    cleanupTaskCreationFailure();
    return false;
  }

  result = xTaskCreatePinnedToCore(
    nodeHousekeepingTask,
    "NodeHousekeeping",
    HOUSEKEEPING_TASK_STACK_BYTES,
    nullptr,
    HOUSEKEEPING_TASK_PRIORITY,
    &housekeepingTaskHandle,
    NODE_APPLICATION_CORE
  );

  if (result != pdPASS) {
    Serial.println("ERROR: Failed to create NodeHousekeeping task.");
    cleanupTaskCreationFailure();
    return false;
  }

  return true;
}
