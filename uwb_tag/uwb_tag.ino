// currently tag is module #5
// The purpose of this code is to set the tag address and antenna delay to default.
// this tag will be used for calibrating the anchors.

#include <WiFi.h>
#include <ArduinoWebsockets.h>
#include <SPI.h>

#include "DW1000Ranging.h"
#include "DW1000.h"
#include "LED_controller.hpp"

#define SPI_SCK 18
#define SPI_MISO 19
#define SPI_MOSI 23
#define DW_CS 4

#define BATTERY_LED_RED_PIN 2
#define BATTERY_LED_GREEN_PIN 12
#define BATTERY_LED_BLUE_PIN -1
#define BATTERY_ADC_PIN 36
#define SENSOR_LED_RED_PIN 13
#define SENSOR_LED_GREEN_PIN 14
#define SENSOR_LED_BLUE_PIN -1

// connection pins
const uint8_t PIN_RST = 27; // reset pin
const uint8_t PIN_IRQ = 34; // irq pin
const uint8_t PIN_SS = 4;   // spi select pin

// TAG antenna delay defaults to 16384
uint16_t Adelay = 16384;

// leftmost two bytes below will become the "short address"
char tag_addr[] = "7D:00:22:EA:82:60:3B:9C";

LEDController ledController;

float minVoltage = 3.0; // TODO(yifan): To verify
float maxVoltage = 4.2; // TODO(yifan): To verify.

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
  setupUWBTag();
}

void loop() {
  DW1000Ranging.loop();
  checkBattery();
}

void setupLED() {
  ledController.initSensorLED(SENSOR_LED_RED_PIN, SENSOR_LED_GREEN_PIN, SENSOR_LED_BLUE_PIN);
  ledController.initBatteryLED(BATTERY_LED_RED_PIN, BATTERY_LED_GREEN_PIN, BATTERY_LED_BLUE_PIN, BATTERY_ADC_PIN, minVoltage, maxVoltage);
  ledController.setLEDRed(SENSOR_LED_RED_PIN, SENSOR_LED_GREEN_PIN, SENSOR_LED_BLUE_PIN);
  ledController.updateBatteryLED();
}

void setupUWBTag() {
  //init the configuration
  SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI);
  DW1000Ranging.initCommunication(PIN_RST, PIN_SS, PIN_IRQ); //Reset, CS, IRQ pin

  // set antenna delay for anchors only. Tag is default (16384)
  //DW1000.setAntennaDelay(Adelay);

  DW1000Ranging.attachNewRange(newRange);
  DW1000Ranging.attachNewDevice(newDevice);
  DW1000Ranging.attachInactiveDevice(inactiveDevice);

  //start the module as a tag, do not assign random short address
  DW1000Ranging.startAsTag(tag_addr, DW1000.MODE_LONGDATA_RANGE_LOWPOWER, false);
}

void newRange() {
  Serial.printf("Anchor address=%X, distance=%f(m)\n", DW1000Ranging.getDistantDevice()->getShortAddress(), DW1000Ranging.getDistantDevice()->getRange());
}

void newDevice(DW1000Device *device) {
  Serial.printf("Anchor connected, address=%X\n", device->getShortAddress());
  ledController.setLEDGreen(SENSOR_LED_RED_PIN, SENSOR_LED_GREEN_PIN, SENSOR_LED_BLUE_PIN);
}

void inactiveDevice(DW1000Device *device) {
  Serial.printf("Anchor disconnected, address=%X\n", device->getShortAddress());
  ledController.setLEDRed(SENSOR_LED_RED_PIN, SENSOR_LED_GREEN_PIN, SENSOR_LED_BLUE_PIN);
}

void checkBattery() {
  unsigned int now = millis();
  if (now - lastBatteryCheckTimeMillis > batteryCheckIntervalMillis) {
    lastBatteryCheckTimeMillis = now;
    ledController.updateBatteryLED();
  }
}
