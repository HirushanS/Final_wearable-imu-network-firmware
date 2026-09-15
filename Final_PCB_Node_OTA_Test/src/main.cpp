#include <Arduino.h>
#include "ota_update.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "project_config.h"
#include "hardware.h"
#include "sensor_reader.h"
#include "espnow_node.h"
#include "node_time_sync.h"
#include "node_tasks.h"

void setup() {
  // Latch power immediately and configure buttons/battery LEDs.
  initHardware();
  
  Serial.begin(115200);
  initOTA();
  delay(1000);

  initEspNowLedPins();

  // GPIO12 is ADC2. Read battery voltage before Wi-Fi/ESP-NOW starts.
  checkBatteryLevelOnceBeforeEspNow();

  Serial.println();
  Serial.print("NODE ");
  Serial.print(NODE_ID);
  Serial.println(" BNO085 STARTING WITH FreeRTOS");

  // After initialization, only NodeBNO085 accesses Wire and the sensor object.
  if (!initBNO085Sensor()) {
    Serial.println("BNO085 initialization failed.");
    sensorFailureLoop(LED_ESPNOW_FAIL);
  }

  // Start ESP-NOW only after the ADC2 battery check and sensor initialization.
  if (!restartEspNow()) {
    Serial.println("ESP-NOW failed to start. Restarting board...");
    delay(1000);
    esp_restart();
  }

  initNodeTimeSync(NODE_ID, CENTER_MAC_ADDRESS);
  requestTimeFromCenter();
  waitForNodeTimeSync(3000);

  if (!startNodeFreeRTOSTasks()) {
    Serial.println("Failed to create node FreeRTOS tasks. Restarting board...");
    delay(1000);
    esp_restart();
  }

  Serial.println();
  Serial.println("BNO085 SPI + ESP-NOW FreeRTOS node ready");
  Serial.print("Free heap after initialization: ");
  Serial.println(ESP.getFreeHeap());
}

void loop() {

  handleOTA();

  if (isOTAUpdating()) {
    delay(1);
    return;
  }
  // Application work is performed by:
  // 1. NodeBNO085 task
  // 2. NodeESPNowTx task
  // 3. NodeHousekeeping task
  // 4. ESP-NOW/Wi-Fi system tasks
  // Keep OTA responsive; application work runs in the FreeRTOS tasks.
  vTaskDelay(pdMS_TO_TICKS(10));
}
