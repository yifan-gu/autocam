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
float yawSpeedValue = 0.0;
float pitchSpeedValue = 0.0;
int activeTrackToggledValue = 0;

ControllerData receivedData;

unsigned int lastPingTime = 0;

void setup() {
  // Start Serial for debugging
  unsigned long startTime = millis();
  unsigned long timeout = 1000; // 1 seconds timeout
                                //
  Serial.begin(115200);
  while (!Serial && (millis() - startTime < timeout)) {
    // Wait for Serial or timeout
  }

  ControllerData data = {0, 0, 0, 0, 0, 0, 0};
  setupBLEPeripheral("Autocam Remote", "autocam-remote", AutocamControllerService, AutocamControllerData, (uint8_t *)&data, sizeof(ControllerData));
}

void loop() {
  readStatusData();
  readControllerData();
  sendControllerData(); // TODO(yifan): Send update only when there's a value change.
  delay(1000 / DATA_RATE); // Control the data rate.
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
    AutocamControllerData.readValue((uint8_t*)&receivedData, sizeof(ControllerData));
    Serial.printf("Received throttle=%d, steering=%d, driveMode=%d, state=%d, yaw_speed=%f, pitch_speed=%f, active_track_toggled=%d\n", receivedData.throttleValue, receivedData.steeringValue, receivedData.driveMode, receivedData.state, receivedData.yaw_speed, receivedData.pitch_speed, receivedData.active_track_toggled);
    updateState(receivedData.state);
    updateDriveMode(receivedData.driveMode);

    //Serial.printf("Data interval: %d(ms)\n", millis() - lastPingTime);
    //lastPingTime = millis();
  }
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

  yawSpeedValue = 0.5;
  pitchSpeedValue = 0;

  driveModeTriggerValue = DRIVE_MODE_MANUAL;
  //if (millis() - lastUpdate >= 1000) {
  //  driveModeTriggerValue = driveMode == DRIVE_MODE_MANUAL ? DRIVE_MODE_AUTO_FOLLOW : DRIVE_MODE_MANUAL;
  //  lastUpdate = millis();
  //}
}

void sendControllerData() {
  if (!(BLECentral && BLECentral.connected())) {
    updateState(state & ~STATE_REMOTE_CONTROLLER_READY);
    if (!connectToCentral()) {
      return;
    }
    updateState(state | STATE_REMOTE_CONTROLLER_READY);
  }

  ControllerData data = {throttleValue, steeringValue, driveModeTriggerValue, state, yawSpeedValue, pitchSpeedValue, activeTrackToggledValue}; // state doesn't matter because it will not be used by the server.
  AutocamControllerData.writeValue((uint8_t*)&data, sizeof(ControllerData));
  //Serial.printf("Sent via BLE: throttle=%d, steering=%d, driveMode=%d, state=%d, yaw_speed=%f, pitch_speed=%f, active_track_toggled=%d\n", data.throttleValue, data.steeringValue, data.driveMode, data.state, data.yaw_speed, data.pitch_speed, data.active_track_toggled);
  //Serial.printf("Data interval: %d(ms)\n", millis() - lastPingTime);
  lastPingTime = millis();
}
