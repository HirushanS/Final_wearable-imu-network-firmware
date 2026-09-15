#include "hardware.h"

volatile bool resetRequest = false;
float currentVoltage = 0.0;

// ===============================
// BATTERY STATE
// ===============================
enum BatteryState {
  BATTERY_UNKNOWN,
  BATTERY_GOOD,
  BATTERY_MEDIUM,
  BATTERY_LOW,
  BATTERY_CHARGING
};

BatteryState storedBatteryState = BATTERY_UNKNOWN;

// ===============================
// RESET ISR ONLY
// ===============================
void IRAM_ATTR resetBoardISR() {
  resetRequest = true;
}

// ===============================
// INIT HARDWARE
// ===============================
void initHardware() {
  // 1. LATCH POWER ON IMMEDIATELY
  pinMode(SYS_LATCH, OUTPUT);
  digitalWrite(SYS_LATCH, HIGH);

  // 2. Setup Battery LEDs
  pinMode(BATT_LED_RED, OUTPUT);
  pinMode(BATT_LED_GREEN, OUTPUT);
  digitalWrite(BATT_LED_RED, LOW);
  digitalWrite(BATT_LED_GREEN, LOW);

  // 3. Charging detect pin
  pinMode(CHG_DT, INPUT_PULLDOWN);

  // 4. Reset button still uses interrupt
  pinMode(RESET_BUTTON, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(RESET_BUTTON), resetBoardISR, FALLING);

  // 5. Power button uses polling, NOT interrupt
  pinMode(POWER_BUTTON, INPUT_PULLDOWN);
}

// ===============================
// HANDLE POWER / RESET TASKS
// ===============================
void handleHardwareTasks() {
  static unsigned long powerPressStart = 0;
  static bool powerButtonWasPressed = false;

  // Reset button handling
  if (resetRequest) {
    delay(300);
    esp_restart();
  }

  // Ignore power button during first 2 seconds after power ON
  if (millis() < 2000) {
    return;
  }

  bool powerButtonState = digitalRead(POWER_BUTTON);

  // Your circuit: GPIO17 LOW normally, HIGH when button pressed
  if (powerButtonState == HIGH) {
    if (!powerButtonWasPressed) {
      powerButtonWasPressed = true;
      powerPressStart = millis();
    }

    // Hold power button 100 ms to turn OFF
    if (millis() - powerPressStart >= 100) {
      Serial.println("Powering off...");

      digitalWrite(BATT_LED_GREEN, LOW);
      digitalWrite(BATT_LED_RED, LOW);

      delay(50);

      // Turn OFF power latch
      digitalWrite(SYS_LATCH, LOW);

      while (1) {
        delay(100);
      }
    }
  } 
  else {
    powerButtonWasPressed = false;
    powerPressStart = 0;
  }
}

// ===============================
// BATTERY CHECK - ONLY ONCE BEFORE ESP-NOW
// ===============================
void checkBatteryLevelOnceBeforeEspNow() {
  Serial.println("Checking battery level before ESP-NOW starts...");

  // Charging detected at startup
  if (digitalRead(CHG_DT) == HIGH) {
    storedBatteryState = BATTERY_CHARGING;

    digitalWrite(BATT_LED_RED, HIGH);
    digitalWrite(BATT_LED_GREEN, LOW);

    Serial.println("Charging detected at startup");
    Serial.println("Battery ADC check skipped because charging is detected.");
    return;
  }

  // GPIO12 is ADC2, so read it only before Wi-Fi / ESP-NOW starts
  analogSetPinAttenuation(BATT_ADC_PIN, ADC_11db);

  delay(100);

  const int samples = 20;
  uint32_t adcSum = 0;

  for (int i = 0; i < samples; i++) {
    adcSum += analogRead(BATT_ADC_PIN);
    delay(5);
  }

  int analogVal = adcSum / samples;

  float pinVoltage = (analogVal / 4095.0) * 3.3;
  currentVoltage = pinVoltage * 2.45;

  Serial.print("Startup ADC Value: ");
  Serial.print(analogVal);
  Serial.print("\t| Pin Voltage: ");
  Serial.print(pinVoltage);
  Serial.print("\t| Real Battery Voltage: ");
  Serial.println(currentVoltage);

  if (currentVoltage > 3.8) {
    storedBatteryState = BATTERY_GOOD;

    digitalWrite(BATT_LED_GREEN, HIGH);
    digitalWrite(BATT_LED_RED, LOW);

    Serial.println("Battery Status: GOOD");
  } 
  else if (currentVoltage < 3.5) {
    storedBatteryState = BATTERY_LOW;

    digitalWrite(BATT_LED_GREEN, LOW);
    digitalWrite(BATT_LED_RED, HIGH);

    Serial.println("Battery Status: LOW");
  } 
  else {
    storedBatteryState = BATTERY_MEDIUM;

    digitalWrite(BATT_LED_GREEN, HIGH);
    digitalWrite(BATT_LED_RED, LOW);

    Serial.println("Battery Status: MEDIUM");
  }

  Serial.println("Battery ADC check finished.");
  Serial.println("ADC will NOT be read again after ESP-NOW starts.");
}

// ===============================
// LED DISPLAY ONLY - NO ADC READ
// ===============================
void handleStoredBatteryLedDisplay() {

  // If charging is connected anytime, show RED LED
  if (digitalRead(CHG_DT) == HIGH) {
    digitalWrite(BATT_LED_GREEN, LOW);
    digitalWrite(BATT_LED_RED, HIGH);
    return;
  }
  static unsigned long lastBlinkTime = 0;
  static bool greenBlinkState = false;

  // Good battery: green always ON
  if (storedBatteryState == BATTERY_GOOD) {
    digitalWrite(BATT_LED_GREEN, HIGH);
    digitalWrite(BATT_LED_RED, LOW);
  }

  // Low battery: red always ON
  else if (storedBatteryState == BATTERY_LOW) {
    digitalWrite(BATT_LED_GREEN, LOW);
    digitalWrite(BATT_LED_RED, HIGH);
  }

  // Charging detected at startup: red ON
  else if (storedBatteryState == BATTERY_CHARGING) {
    digitalWrite(BATT_LED_GREEN, LOW);
    digitalWrite(BATT_LED_RED, HIGH);
  }

  // Medium battery: green blinking, NO ADC reading
  else if (storedBatteryState == BATTERY_MEDIUM) {
    digitalWrite(BATT_LED_RED, LOW);

    if (millis() - lastBlinkTime >= 500) {
      lastBlinkTime = millis();
      greenBlinkState = !greenBlinkState;
      digitalWrite(BATT_LED_GREEN, greenBlinkState ? HIGH : LOW);
    }
  }

  // Unknown state: both OFF
  else {
    digitalWrite(BATT_LED_GREEN, LOW);
    digitalWrite(BATT_LED_RED, LOW);
  }
}

// ===============================
// OLD FUNCTION KEPT FOR COMPATIBILITY
// ===============================
// This function no longer reads ADC.
// It only displays the stored battery level.
void handleBatteryMonitor() {
  handleStoredBatteryLedDisplay();
}