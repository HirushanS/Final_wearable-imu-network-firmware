#include "hardware.h"

#include "project_config.h"

namespace {

volatile bool resetRequested = false;

float currentBatteryVoltage = 0.0f;

enum class BatteryState : uint8_t {
  Unknown,
  Good,
  Medium,
  Low
};

BatteryState storedBatteryState = BatteryState::Unknown;

void IRAM_ATTR resetBoardISR() {
  resetRequested = true;
}

void turnOffEspNowStatusLeds() {
  digitalWrite(ProjectConfig::ESPNOW_NODE_LED_1_PIN, LOW);
  digitalWrite(ProjectConfig::ESPNOW_NODE_LED_2_PIN, LOW);
  digitalWrite(ProjectConfig::ESPNOW_NODE_LED_3_PIN, LOW);
  digitalWrite(ProjectConfig::ESPNOW_NODE_LED_4_PIN, LOW);
}

void turnOffAllLeds() {
  turnOffEspNowStatusLeds();
  digitalWrite(ProjectConfig::CENTER_SENSOR_LED_PIN, LOW);
  digitalWrite(ProjectConfig::BATTERY_LED_GREEN_PIN, LOW);
  digitalWrite(ProjectConfig::BATTERY_LED_RED_PIN, LOW);
}

void powerButtonTask(void *parameter) {
  (void)parameter;

  // Ignore the button used to initially switch on the system.
  vTaskDelay(pdMS_TO_TICKS(2000));

  while (true) {
    if (
      digitalRead(ProjectConfig::POWER_BUTTON_PIN) ==
      HIGH
    ) {
      // Small debounce delay.
      vTaskDelay(pdMS_TO_TICKS(50));

      if (
        digitalRead(ProjectConfig::POWER_BUTTON_PIN) ==
        HIGH
      ) {
        Serial.println("Powering off...");

        turnOffAllLeds();
        vTaskDelay(pdMS_TO_TICKS(30));

        digitalWrite(
          ProjectConfig::SYS_LATCH_PIN,
          LOW
        );

        // Normally power is removed here.
        while (true) {
          vTaskDelay(pdMS_TO_TICKS(1000));
        }
      }
    }

    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

void applyStoredBatteryState() {
  static unsigned long lastBlinkTime = 0;
  static bool greenBlinkState = false;

  // Charging indication has priority over the stored battery level.
  if (digitalRead(ProjectConfig::CHARGING_DETECT_PIN) == HIGH) {
    digitalWrite(ProjectConfig::BATTERY_LED_GREEN_PIN, LOW);
    digitalWrite(ProjectConfig::BATTERY_LED_RED_PIN, HIGH);
    return;
  }

  switch (storedBatteryState) {
    case BatteryState::Good:
      digitalWrite(ProjectConfig::BATTERY_LED_GREEN_PIN, HIGH);
      digitalWrite(ProjectConfig::BATTERY_LED_RED_PIN, LOW);
      break;

    case BatteryState::Medium:
      digitalWrite(ProjectConfig::BATTERY_LED_RED_PIN, LOW);

      if (millis() - lastBlinkTime >= 500UL) {
        lastBlinkTime = millis();
        greenBlinkState = !greenBlinkState;
        digitalWrite(
          ProjectConfig::BATTERY_LED_GREEN_PIN,
          greenBlinkState ? HIGH : LOW
        );
      }
      break;

    case BatteryState::Low:
      digitalWrite(ProjectConfig::BATTERY_LED_GREEN_PIN, LOW);
      digitalWrite(ProjectConfig::BATTERY_LED_RED_PIN, HIGH);
      break;

    case BatteryState::Unknown:
    default:
      digitalWrite(ProjectConfig::BATTERY_LED_GREEN_PIN, LOW);
      digitalWrite(ProjectConfig::BATTERY_LED_RED_PIN, LOW);
      break;
  }
}

}  // namespace

void initHardware() {
  // Latch power immediately.
  pinMode(ProjectConfig::SYS_LATCH_PIN, OUTPUT);
  digitalWrite(ProjectConfig::SYS_LATCH_PIN, HIGH);

  pinMode(ProjectConfig::BATTERY_LED_RED_PIN, OUTPUT);
  pinMode(ProjectConfig::BATTERY_LED_GREEN_PIN, OUTPUT);

  pinMode(ProjectConfig::ESPNOW_NODE_LED_1_PIN, OUTPUT);
  pinMode(ProjectConfig::ESPNOW_NODE_LED_2_PIN, OUTPUT);
  pinMode(ProjectConfig::ESPNOW_NODE_LED_3_PIN, OUTPUT);
  pinMode(ProjectConfig::ESPNOW_NODE_LED_4_PIN, OUTPUT);
  pinMode(ProjectConfig::CENTER_SENSOR_LED_PIN, OUTPUT);

  pinMode(ProjectConfig::CHARGING_DETECT_PIN, INPUT_PULLDOWN);
  pinMode(ProjectConfig::BATTERY_ADC_PIN, INPUT);

  turnOffAllLeds();

  pinMode(ProjectConfig::RESET_BUTTON_PIN, INPUT_PULLUP);
  attachInterrupt(
    digitalPinToInterrupt(ProjectConfig::RESET_BUTTON_PIN),
    resetBoardISR,
    FALLING
  );

  // Polling is safer than doing button timing inside an ISR.
  pinMode(ProjectConfig::POWER_BUTTON_PIN, INPUT_PULLDOWN);

  const BaseType_t powerTaskResult =
      xTaskCreatePinnedToCore(
        powerButtonTask,
        "PowerButton",
        2048,
        nullptr,
        4,
        nullptr,
        1
      );

  if (powerTaskResult != pdPASS) {
    Serial.println("Failed to create power-button task.");
  }


}

void checkBatteryLevelOnceBeforeWireless() {
  Serial.println(
    "Checking center battery before Wi-Fi/ESP-NOW starts..."
  );

  // When charging is already active, the divided battery voltage can be
  // misleading. Show charging and leave the stored level unknown.
  if (digitalRead(ProjectConfig::CHARGING_DETECT_PIN) == HIGH) {
    storedBatteryState = BatteryState::Unknown;
    digitalWrite(ProjectConfig::BATTERY_LED_GREEN_PIN, LOW);
    digitalWrite(ProjectConfig::BATTERY_LED_RED_PIN, HIGH);

    Serial.println("Charging detected at startup.");
    Serial.println(
      "Battery ADC check skipped. Restart without the charger "
      "to measure the battery level."
    );
    return;
  }

  // GPIO12 is ADC2. It must be sampled before Wi-Fi/ESP-NOW starts.
  analogSetPinAttenuation(
    ProjectConfig::BATTERY_ADC_PIN,
    ADC_11db
  );

  delay(100);

  constexpr int sampleCount = 20;
  uint32_t adcSum = 0;

  for (int i = 0; i < sampleCount; i++) {
    adcSum += analogRead(ProjectConfig::BATTERY_ADC_PIN);
    delay(5);
  }

  const int analogValue = adcSum / sampleCount;
  const float pinVoltage =
      (static_cast<float>(analogValue) / 4095.0f) * 3.3f;

  // Keep this multiplier only if the PCB voltage-divider ratio is 2.45.
  currentBatteryVoltage = pinVoltage * 2.45f;

  Serial.print("Startup ADC value: ");
  Serial.print(analogValue);
  Serial.print(" | Pin voltage: ");
  Serial.print(pinVoltage, 3);
  Serial.print(" V | Battery voltage: ");
  Serial.print(currentBatteryVoltage, 3);
  Serial.println(" V");

  if (currentBatteryVoltage > 3.8f) {
    storedBatteryState = BatteryState::Good;
    Serial.println("Center battery status: GOOD");
  } else if (currentBatteryVoltage < 3.5f) {
    storedBatteryState = BatteryState::Low;
    Serial.println("Center battery status: LOW");
  } else {
    storedBatteryState = BatteryState::Medium;
    Serial.println("Center battery status: MEDIUM");
  }

  applyStoredBatteryState();

  Serial.println(
    "Battery ADC check complete. No ADC2 reads will occur after wireless starts."
  );
}

void handleStoredBatteryLedDisplay() {
  applyStoredBatteryState();
}

void setCenterEspNowNodeStatus(uint8_t activeNodeMask) {
  static uint8_t previousActiveNodeMask = 0xFF;

  // Only use the lowest four bits.
  activeNodeMask &= 0x0F;

  if (activeNodeMask == previousActiveNodeMask) {
    return;
  }

  previousActiveNodeMask = activeNodeMask;

  digitalWrite(
    ProjectConfig::ESPNOW_NODE_LED_1_PIN,
    (activeNodeMask & (1U << 0)) ? HIGH : LOW
  );

  digitalWrite(
    ProjectConfig::ESPNOW_NODE_LED_2_PIN,
    (activeNodeMask & (1U << 1)) ? HIGH : LOW
  );

  digitalWrite(
    ProjectConfig::ESPNOW_NODE_LED_3_PIN,
    (activeNodeMask & (1U << 2)) ? HIGH : LOW
  );

  digitalWrite(
    ProjectConfig::ESPNOW_NODE_LED_4_PIN,
    (activeNodeMask & (1U << 3)) ? HIGH : LOW
  );

  Serial.print("Node status | N1: ");
  Serial.print((activeNodeMask & (1U << 0)) ? "ON" : "OFF");

  Serial.print(" | N2: ");
  Serial.print((activeNodeMask & (1U << 1)) ? "ON" : "OFF");

  Serial.print(" | N3: ");
  Serial.print((activeNodeMask & (1U << 2)) ? "ON" : "OFF");

  Serial.print(" | N4: ");
  Serial.println((activeNodeMask & (1U << 3)) ? "ON" : "OFF");
}

void setCenterSensorWorking(bool working) {
  static bool previousState = false;

  if (working == previousState) {
    return;
  }

  previousState = working;

  digitalWrite(
    ProjectConfig::CENTER_SENSOR_LED_PIN,
    working ? HIGH : LOW
  );

  Serial.print("Center BNO085 LED: ");
  Serial.println(working ? "ON" : "OFF");
}

void handleHardwareTasks() {
  static unsigned long powerPressStart = 0;
  static bool powerButtonWasPressed = false;

  if (resetRequested) {
    delay(300);
    esp_restart();
  }

  // Ignore the power-on button state during the first two seconds.
  if (millis() < 2000UL) {
    return;
  }

  const bool powerButtonPressed =
      digitalRead(ProjectConfig::POWER_BUTTON_PIN) == HIGH;

  if (powerButtonPressed) {
    if (!powerButtonWasPressed) {
      powerButtonWasPressed = true;
      powerPressStart = millis();
    }

    if (millis() - powerPressStart >= 100UL) {
      Serial.println("Powering off...");

      turnOffAllLeds();
      delay(50);
      digitalWrite(ProjectConfig::SYS_LATCH_PIN, LOW);

      while (true) {
        delay(100);
      }
    }
  } else {
    powerButtonWasPressed = false;
    powerPressStart = 0;
  }
}