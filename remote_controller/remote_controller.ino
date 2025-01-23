#include <SPI.h>

#include "DW1000Ranging.h"
#include "DW1000.h"
#include "BLE_setup.hpp"

#define DATA_RATE 100 // 100 Hz

// Available state.
#define STATE_NOT_READY 0
#define STATE_SENSOR_READY 1
#define STATE_REMOTE_CONTROLLER_READY 2

#define DRIVE_MODE_MANUAL 0
#define DRIVE_MODE_AUTO_FOLLOW 1

const int minThrottle = 1000, maxThrottle = 2000, midThrottle = 1500;
const int minSteering = 1000, maxSteering = 2000, midSteering = 1500;

// State indicators.
int state = STATE_NOT_READY;
int driveMode = DRIVE_MODE_MANUAL;

// Variable toggle the drive mode.
int driveModeTriggerValue = DRIVE_MODE_MANUAL;

// Variables to store values
int throttleValue = midThrottle, steeringValue = midSteering;

// Variables to store the gimbal controller data.
float yawTorqueValue = 0.0;
float pitchTorqueValue = 0.0;
int activeTrackToggledValue = 0;

struct ControllerData {
  int throttleValue;
  int steeringValue;
  int driveMode;
  int state;
  float yaw_torque;
  float pitch_torque;
  int active_track_toggled;
};

ControllerData receivedData;

// BLE Service and Characteristic
BLEService AutocamControllerService("f49f531a-9cba-4ada-905c-68699d400122");
BLECharacteristic AutocamControllerData("B330", BLERead | BLEWrite | BLENotify, sizeof(ControllerData), true);

unsigned int lastPingTime = 0;

// DW1000 constants.
#define SPI_SCK 18
#define SPI_MISO 19
#define SPI_MOSI 23
#define DW_CS 4

// connection pins
const uint8_t PIN_RST = 27; // reset pin
const uint8_t PIN_IRQ = 34; // irq pin
const uint8_t PIN_SS = 4;   // spi select pin

// TAG antenna delay defaults to 16384
uint16_t Adelay = 16384;

// leftmost two bytes below will become the "short address"
char tag_addr[] = "7D:00:22:EA:82:60:3B:9C";

void setup() {
  // Start Serial for debugging
  unsigned long startTime = millis();
  unsigned long timeout = 1000; // 1 seconds timeout
                                //
  Serial.begin(115200);
  while (!Serial && (millis() - startTime < timeout)) {
    // Wait for Serial or timeout
  }

  setupUWBTag();
  ControllerData data = {0, 0, 0, 0, 0, 0, 0};
  setupBLEPeripheral("Autocam Remote", "autocam-remote", AutocamControllerService, AutocamControllerData, (uint8_t *)&data, sizeof(data));
}

void loop() {
  readStatusData();
  DW1000Ranging.loop();
  readControllerData();
  sendControllerData(); // TODO(yifan): Send update only when there's a value change.
  delay(1000 / DATA_RATE); // Control the data rate.
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

  //start the module as an anchor, do not assign random short address
  DW1000Ranging.startAsTag(tag_addr, DW1000.MODE_LONGDATA_RANGE_LOWPOWER, false);
}

void updateState(int newState) {
  if (state == newState) {
    return;
  }

  state = newState;
  Serial.printf("Update state=%d\n", state);
  // TODO(yifan): Update LED.
}

void updateDriveMode(int newDriveMode) {
  if (driveMode == newDriveMode) {
    return;
  }

  driveMode = newDriveMode;
  Serial.printf("Update drive mode=%d\n", driveMode);
  // TODO(yifan): Update LED.
}

void readStatusData() {
  if (!(BLECentral && BLECentral.connected())) {
    updateState(state & ~STATE_REMOTE_CONTROLLER_READY);
    if (!connectToCentral()) {
      return;
    }
    updateState(state | STATE_REMOTE_CONTROLLER_READY);
  }

  if (AutocamControllerData.written()) {
    AutocamControllerData.readValue((uint8_t*)&receivedData, sizeof(receivedData));
    Serial.printf("Received throttle=%d, steering=%d, driveMode=%d, state=%d, yaw_torque=%f, pitch_torque=%f, active_track_toggled=%d\n", receivedData.throttleValue, receivedData.steeringValue, receivedData.driveMode, receivedData.state, receivedData.yaw_torque, receivedData.pitch_torque, receivedData.active_track_toggled);
    updateState(receivedData.state);
    updateDriveMode(receivedData.driveMode);

    //Serial.printf("Data interval: %d(ms)\n", millis() - lastPingTime);
    //lastPingTime = millis();
  }
}

void newRange() {
  Serial.printf("Anchor address=%X, distance=%d(m)\n", DW1000Ranging.getDistantDevice()->getShortAddress(), DW1000Ranging.getDistantDevice()->getRange());
}

void newDevice(DW1000Device *device) {
  Serial.printf("Anchor connected, address=%X\n", device->getShortAddress());
}

void inactiveDevice(DW1000Device *device) {
  Serial.printf("Anchor disconnected, address=%X\n", device->getShortAddress());
}

 // TODO(yifan): Fill me in.
void readControllerData() {
  static int throttleStep = 1;
  static int steeringStep = 1;
  static int lastUpdate = 0;

  if (throttleValue >= maxThrottle || throttleValue <= minThrottle) {
    throttleStep = -throttleStep;
  }
  if (steeringValue >= maxSteering || steeringValue <= minSteering) {
    steeringStep = -steeringStep;
  }

  throttleValue += throttleStep;
  steeringValue += steeringStep;


  if (millis() - lastUpdate >= 1000) {
    driveModeTriggerValue = driveMode == DRIVE_MODE_MANUAL ? DRIVE_MODE_AUTO_FOLLOW : DRIVE_MODE_MANUAL;
    lastUpdate = millis();
  }
}

void sendControllerData() {
  if (!(BLECentral && BLECentral.connected())) {
    updateState(state & ~STATE_REMOTE_CONTROLLER_READY);
    if (!connectToCentral()) {
      return;
    }
    updateState(state | STATE_REMOTE_CONTROLLER_READY);
  }

  ControllerData data = {throttleValue, steeringValue, driveModeTriggerValue, state, yawTorqueValue, pitchTorqueValue, activeTrackToggledValue}; // state doesn't matter because it will not be used by the server.
  AutocamControllerData.writeValue((uint8_t*)&data, sizeof(data));
  Serial.printf("Sent via BLE: throttle=%d, steering=%d, driveMode=%d, state=%d, yaw_torque=%f, pitch_torque=%f, active_track_toggled=%d\n", data.throttleValue, data.steeringValue, data.driveMode, data.state, data.yaw_torque, data.pitch_torque, data.active_track_toggled);
  //Serial.printf("Data interval: %d(ms)\n", millis() - lastPingTime);
  lastPingTime = millis();
}
