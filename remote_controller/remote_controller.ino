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

#define STEERING_STICK_X 2
#define THROTTLE_STICK_Y 12
#define GIMBAL_STICK_X 13
#define GIMBAL_STICK_Y 14

#define STICK_DEAD_ZONE 200
#define ADC_MAX 4095   // ESP32 ADC range
#define ADC_MID (ADC_MAX / 2) // Midpoint for dead zone calculation

#define SMOOTHING_FACTOR 0.2  // Smoothing factor (0.0 - 1.0, higher = more smoothing)

// Smoothed joystick values
float smoothedX = ADC_MID;
float smoothedY = ADC_MID;
float smoothedGX = ADC_MID;
float smoothedGY = ADC_MID;

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
  setupBLE();
}

void loop() {
  readStatusData();
  readControllerData();
  sendControllerData(); // TODO(yifan): Send update only when there's a value change.
  delay(1000 / DATA_RATE); // Control the data rate.
}

void setupBLE() {
  AutocamControllerService.addCharacteristic(AutocamControllerData);
  setupBLEPeripheral("Autocam Remote", "autocam-remote", AutocamControllerService);
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
    AutocamControllerData.readValue((uint8_t *)&receivedData, sizeof(ControllerData));
    Serial.printf("Received state=%d, driveMode=%d, activeTrackToggled=%d\n", receivedData.state, receivedData.driveMode, receivedData.active_track_toggled);
    updateState(receivedData.state);
    updateDriveMode(receivedData.driveMode);

    //Serial.printf("Data interval: %d(ms)\n", millis() - lastPingTime);
    //lastPingTime = millis();
  }
}

void readControllerData() {
  // Read raw joystick values
  int rawX = analogRead(STEERING_STICK_X);
  int rawY = analogRead(THROTTLE_STICK_Y);
  int rawGX = analogRead(GIMBAL_STICK_X);
  int rawGY = analogRead(GIMBAL_STICK_Y);

  // Apply dead zone filtering
  if (abs(rawX - ADC_MID) < STICK_DEAD_ZONE) rawX = ADC_MID;
  if (abs(rawY - ADC_MID) < STICK_DEAD_ZONE) rawY = ADC_MID;
  if (abs(rawGX - ADC_MID) < STICK_DEAD_ZONE) rawGX = ADC_MID;
  if (abs(rawGY - ADC_MID) < STICK_DEAD_ZONE) rawGY = ADC_MID;

  // Apply exponential moving average (smoothing)
  smoothedX = (SMOOTHING_FACTOR * rawX) + ((1 - SMOOTHING_FACTOR) * smoothedX);
  smoothedY = (SMOOTHING_FACTOR * rawY) + ((1 - SMOOTHING_FACTOR) * smoothedY);
  smoothedGX = (SMOOTHING_FACTOR * rawGX) + ((1 - SMOOTHING_FACTOR) * smoothedGX);
  smoothedGY = (SMOOTHING_FACTOR * rawGY) + ((1 - SMOOTHING_FACTOR) * smoothedGY);

  // Map throttle and steering using floating-point math:
  throttleValue = minThrottle + (int)round((maxThrottle - minThrottle) * ((float)smoothedY / (float)ADC_MAX));
  steeringValue = minSteering + (int)round((maxSteering - minSteering) * ((float)smoothedX / (float)ADC_MAX));

  // For yaw: linear mapping from [0, ADC_MAX] to [-1, 1]
  yawSpeedValue = ((float)smoothedGX / ADC_MAX) * 2.0 - 1.0;
  pitchSpeedValue = ((float)smoothedGY / ADC_MAX) * 2.0 - 1.0;

  // Ensure the midpoint results in exactly 1500 and 0.
  if (rawY == ADC_MID) throttleValue = midThrottle;
  if (rawX == ADC_MID) steeringValue = midSteering;
  if (rawGX == ADC_MID) yawSpeedValue = 0.0;
  if (rawGY == ADC_MID) pitchSpeedValue = 0.0;

  // Print all values, including gimbal joystick positions
  Serial.printf("Joystick X=%.1f, Y=%.1f | Throttle=%d, Steering=%d | Gimbal X=%.1f, Y=%.1f | Yaw=%.2f, Pitch=%.2f\n",
                smoothedX, smoothedY, throttleValue, steeringValue, smoothedGX, smoothedGY, yawSpeedValue, pitchSpeedValue);
}

void sendControllerData() {
  if (!(BLECentral && BLECentral.connected())) {
    updateState(state & ~STATE_REMOTE_CONTROLLER_READY);
    if (!connectToCentral()) {
      return;
    }
    updateState(state | STATE_REMOTE_CONTROLLER_READY);
  }

  ControllerData data = {.throttleValue = throttleValue, .steeringValue = steeringValue, .driveMode = driveModeTriggerValue, .yaw_speed = yawSpeedValue, .pitch_speed = pitchSpeedValue, .active_track_toggled = activeTrackToggledValue};
  AutocamControllerData.writeValue((uint8_t *)&data, sizeof(ControllerData));
  activeTrackToggledValue = 0;
  //Serial.printf("Sent via BLE: throttle=%d, steering=%d, driveMode=%d, state=%d, yaw_speed=%f, pitch_speed=%f, active_track_toggled=%d\n", data.throttleValue, data.steeringValue, data.driveMode, data.state, data.yaw_speed, data.pitch_speed, data.active_track_toggled);
  //Serial.printf("Data interval: %d(ms)\n", millis() - lastPingTime);
  lastPingTime = millis();
}
