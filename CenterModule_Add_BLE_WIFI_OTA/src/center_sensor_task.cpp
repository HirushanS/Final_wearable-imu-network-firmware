#include "center_sensor_task.h"

#include <string.h>

#include "center_data_manager.h"
#include "center_time_sync.h"
#include "imu_frame.h"
#include "project_config.h"
#include "sensor_reader.h"
#include "hardware.h"

namespace {

volatile bool centerSensorReady = false;
uint16_t centerSequenceNumber = 0;
constexpr uint8_t CENTER_SENSOR_EVENTS_PER_LOOP = 8;

void createCenterFrame(
  const BNO085Reading &reading,
  IMU_Node_Frame &frame
) {
  memset(&frame, 0, sizeof(frame));

  frame.startByte = IMU_FRAME_START_BYTE;
  frame.nodeID = ProjectConfig::CENTER_SOURCE_ID;
  frame.timestamp_ms = getCenterTimeOfDayMs();
  frame.sequenceNo = centerSequenceNumber++;

  frame.accelX = reading.accelX;
  frame.accelY = reading.accelY;
  frame.accelZ = reading.accelZ;

  frame.gyroX = reading.gyroX;
  frame.gyroY = reading.gyroY;
  frame.gyroZ = reading.gyroZ;

  frame.magX = reading.magX;
  frame.magY = reading.magY;
  frame.magZ = reading.magZ;

  frame.quatW = reading.quatW;
  frame.quatX = reading.quatX;
  frame.quatY = reading.quatY;
  frame.quatZ = reading.quatZ;

  frame.payloadType =
      ProjectConfig::CENTER_PAYLOAD_TYPE;

  frame.endByte = IMU_FRAME_END_BYTE;
  finalizeIMUFrame(frame);
}

void centerSensorTask(void *parameter) {
  (void)parameter;

  TickType_t lastWakeTime = xTaskGetTickCount();
  unsigned long lastInitAttempt = 0;
  unsigned long lastValidReadingMs = 0;
  BNO085Reading reading = {};

  for (;;) {
    if (!isBNO085SensorReady()) {
      setCenterSensorWorking(false);
      lastValidReadingMs = 0;
      const unsigned long now = millis();

      if (
        lastInitAttempt == 0 ||
        now - lastInitAttempt >=
          ProjectConfig::
            CENTER_SENSOR_RETRY_INTERVAL_MS
      ) {
        lastInitAttempt = now;
        centerSensorReady = initBNO085Sensor();

        if (centerSensorReady) {
          reading = {};
        }
      }

      vTaskDelay(pdMS_TO_TICKS(100));
      lastWakeTime = xTaskGetTickCount();
      continue;
    }

    centerSensorReady = true;

    bool receivedEvent = false;

    for (
      uint8_t eventIndex = 0;
      eventIndex < CENTER_SENSOR_EVENTS_PER_LOOP;
      eventIndex++
    ) {
      if (!readBNO085Sensor(reading)) {
        break;
      }

      receivedEvent = true;
    }

    // if (receivedEvent && isBNO085ReadingReady()) {
    //   IMU_Node_Frame frame;
    //   createCenterFrame(reading, frame);
    //   storeCenterFrame(frame);
    // }
    if (receivedEvent && isBNO085ReadingReady()) {
      IMU_Node_Frame frame;
      createCenterFrame(reading, frame);
      storeCenterFrame(frame);

      lastValidReadingMs = millis();
    }

    const bool centerSensorWorking =
        lastValidReadingMs != 0 &&
        millis() - lastValidReadingMs <=
          ProjectConfig::CENTER_SENSOR_LED_TIMEOUT_MS;

    setCenterSensorWorking(centerSensorWorking);

    vTaskDelayUntil(
      &lastWakeTime,
      pdMS_TO_TICKS(
        ProjectConfig::CENTER_SENSOR_INTERVAL_MS
      )
    );
  }
}

}  // namespace

bool startCenterSensorTask() {
  // Try once immediately. The task will keep retrying if it fails.
  centerSensorReady = initBNO085Sensor();

  const BaseType_t result =
      xTaskCreatePinnedToCore(
        centerSensorTask,
        "CenterBNO055",
        4096,
        nullptr,
        2,
        nullptr,
        1
      );

  if (result != pdPASS) {
    Serial.println(
      "Failed to create center BNO055 task."
    );
    return false;
  }

  return true;
}

bool isCenterSensorReady() {
  return centerSensorReady;
}
