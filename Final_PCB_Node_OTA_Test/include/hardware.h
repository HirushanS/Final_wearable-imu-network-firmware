#ifndef HARDWARE_H
#define HARDWARE_H

#include <Arduino.h>

#define SYS_LATCH 13
#define POWER_BUTTON 17
#define RESET_BUTTON 4
#define CHG_DT 2

// LED 3 Pins
#define LED3_RED 25
#define LED3_GREEN 26

// Battery & LED Pins
#define BATT_LED_RED 27
#define BATT_LED_GREEN 14
#define BATT_ADC_PIN 35  

void initHardware();
void handleHardwareTasks();

// OLD battery monitor function can stay, but we will not use it in main loop
void handleBatteryMonitor();

// NEW functions
void checkBatteryLevelOnceBeforeEspNow();
void handleStoredBatteryLedDisplay();

#endif