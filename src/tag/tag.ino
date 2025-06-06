#include <WiFi.h>
#include <ArduinoWebsockets.h>
#include <SPI.h>

#include "DW1000Ranging.h"
#include "DW1000.h"
#include "LED_controller.hpp"
#include "util.h"

#define TAG_ID_0

// SPI pins
#define SPI_SCK 18
#define SPI_MISO 19
#define SPI_MOSI 23
#define DW_CS 4

// DW1000 pins
#define PIN_RST 27 // reset pin
#define PIN_IRQ 34 // irq pin
#define PIN_SS 4   // spi select pin

#if defined(TAG_ID_0)

#define BATTERY_LED_RED_PIN 14
#define BATTERY_LED_GREEN_PIN 25
#define BATTERY_LED_BLUE_PIN -1
#define BATTERY_ADC_PIN 39
#define SENSOR_LED_RED_PIN 26
#define SENSOR_LED_GREEN_PIN 32
#define SENSOR_LED_BLUE_PIN -1

//calibrated Antenna Delay setting for this anchor
uint16_t Adelay = 16630;

// leftmost two bytes below will become the "short address"
char uwb_addr[] = "00:00:8D:99:3A:47:55:1C";

#elif defined(TAG_ID_2)

#define BATTERY_LED_RED_PIN 2
#define BATTERY_LED_GREEN_PIN 12
#define BATTERY_LED_BLUE_PIN -1
#define BATTERY_ADC_PIN 32
#define SENSOR_LED_RED_PIN 13
#define SENSOR_LED_GREEN_PIN 14
#define SENSOR_LED_BLUE_PIN -1

//calibrated Antenna Delay setting for this anchor
uint16_t Adelay = 16630;

// leftmost two bytes below will become the "short address"
char uwb_addr[] = "02:00:8D:99:3A:47:55:1C";

#else
  #error "Must define either TAG_ID_0 or TAG_ID_2"
#endif


LEDController ledController;

float minVoltage = 3.0; // TODO(yifan): To verify
float maxVoltage = 4.2; // TODO(yifan): To verify.

uint8_t tagState = TAG_STATE_SENSOR_NOT_CONNECTED;

unsigned int lastBatteryCheckTimeMillis = 0;
unsigned int batteryCheckIntervalMillis = 2000;

void setup() {
  // Start Serial for debugging
  unsigned long startTime = millis();
  unsigned long timeout = 1000; // 1 seconds timeout
                                //
  Serial.begin(115200);
  while (!Serial && (millis() - startTime < timeout)) {
    // Wait for Serial or timeout
  }

  setupLED();
  setupUWB();
  setupPinnedTask();
}

void setupPinnedTask() {
  // Create a task and pin it to core 0.
  // Parameters: task function, name, stack size, parameter, priority, task handle, core id.
  xTaskCreatePinnedToCore(
    nonUWBTask,   // Task function.
    "nonUWBTask", // Name of task.
    4096,         // Stack size in words.
    NULL,         // Task input parameter.
    1,            // Task priority.
    NULL,         // Task handle.
    0             // Core where the task should run (0 or 1).
  );
}

// Task function for non UWB computation that's not time critical.
void nonUWBTask(void * parameter) {
  while (true) {
    checkBattery();
    // Short delay to yield to other tasks; adjust as necessary.
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}

void loop() {
  DW1000Ranging.loop();

}

void setupLED() {
  ledController.initSensorLED(SENSOR_LED_RED_PIN, SENSOR_LED_GREEN_PIN, SENSOR_LED_BLUE_PIN);

#if defined(TAG_ID_0)
  ledController.initBatteryLED(BATTERY_LED_RED_PIN, BATTERY_LED_GREEN_PIN, BATTERY_LED_BLUE_PIN, BATTERY_ADC_PIN, minVoltage, maxVoltage);
#elif defined(TAG_ID_2)
  ledController.initBatteryLED(BATTERY_LED_RED_PIN, BATTERY_LED_GREEN_PIN, BATTERY_LED_BLUE_PIN, BATTERY_ADC_PIN, minVoltage, maxVoltage, 31000, 10000); // Measured battery ratio.
#else
  #error "Must define either TAG_ID_0 or TAG_ID_2"
#endif

  ledController.setLEDRed(SENSOR_LED_RED_PIN, SENSOR_LED_GREEN_PIN, SENSOR_LED_BLUE_PIN);
  ledController.updateBatteryLED();
}

void setupUWB() {
  //init the configuration
  SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI);
  DW1000Ranging.initCommunication(PIN_RST, PIN_SS, PIN_IRQ); //Reset, CS, IRQ pin

  // set antenna delay for anchors only. Tag is default (16384)
  DW1000.setAntennaDelay(Adelay);

  DW1000Ranging.attachNewRange(newRange);
  DW1000Ranging.attachNewDevice(newDevice);
  DW1000Ranging.attachInactiveDevice(inactiveDevice);

  // NOTE (yifan): The DW1000 "tag" is different from the Autocam "tag".
  // In fact, the naming conventions are reversed:
  //   - DW1000 "anchors" correspond to Autocam Remote and Autocam Tag.
  //   - DW1000 "tag" corresponds to sensor.
  // This reversal is due to the DW1000 library's limitation of supporting a single tag with multiple anchors,
  // whereas our system uses multiple anchors (Autocam Remote and Autocam Tag).
  DW1000Ranging.startAsAnchor(uwb_addr, DW1000.MODE_LONGDATA_RANGE_LOWPOWER, false);
}

void newRange() {
  //LOGF("Autocam Sensor address=%X, distance=%f(m)\n", DW1000Ranging.getDistantDevice()->getShortAddress(), DW1000Ranging.getDistantDevice()->getRange());
  updateState(TAG_STATE_SENSOR_CONNECTED);
}

void newDevice(DW1000Device *device) {
  LOGF("Autocam Sensor connected, address=%X\n", device->getShortAddress());
  updateState(TAG_STATE_SENSOR_CONNECTED);
}

void inactiveDevice(DW1000Device *device) {
  LOGF("Autocam Sensor disconnected, address=%X\n", device->getShortAddress());
  updateState(TAG_STATE_SENSOR_NOT_CONNECTED);
}

void checkBattery() {
  unsigned int now = millis();
  if (now - lastBatteryCheckTimeMillis > batteryCheckIntervalMillis) {
    lastBatteryCheckTimeMillis = now;
    ledController.updateBatteryLED();
  }
}

void updateState(uint8_t newState) {
  if (tagState == newState) {
    return;
  }
  tagState = newState;
  switch (tagState) {
    case TAG_STATE_SENSOR_NOT_CONNECTED:
      ledController.setLEDRed(SENSOR_LED_RED_PIN, SENSOR_LED_GREEN_PIN, SENSOR_LED_BLUE_PIN);
      break;
    case TAG_STATE_SENSOR_CONNECTED:
      ledController.setLEDGreen(SENSOR_LED_RED_PIN, SENSOR_LED_GREEN_PIN, SENSOR_LED_BLUE_PIN);
      break;
    default:
      LOGF("Unkown tag stata value: %d\n", tagState);
  }
}
