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

#define MODE_SWITCH_PIN 3
#define ACTIVE_TRACK_TOGGLE_PIN 2
#define STEERING_STICK_X_PIN 12
#define THROTTLE_STICK_Y_PIN 13
#define GIMBAL_STICK_X_PIN 14
#define GIMBAL_STICK_Y_PIN 15

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
const float minPitch = -1, maxPitch = 1, midPitch = 0;
const float minYaw = -1, maxYaw = 1, midYaw = 0;

// State indicators.
int state = STATE_NOT_READY;
int driveMode = DRIVE_MODE_MANUAL;

// Variables toggle the drive mode.
int driveModeTriggerValue = DRIVE_MODE_MANUAL;

// Variables to store values
int throttleValue = midThrottle, steeringValue = midSteering;

// Variables to store the gimbal controller data.
float yawSpeedValue = 0.0;
float pitchSpeedValue = 0.0;
bool activeTrackToggledValue = false;

ControllerData receivedData;

// Global variable to hold last sent data.
// (Initialize with known defaults.)
ControllerData lastSentData = { midThrottle, midSteering, DRIVE_MODE_MANUAL, midPitch, midYaw, false };

unsigned int lastPingTime = 0;

// --- Button Debounce Helpers ---
struct ButtonDebounce {
  uint8_t pin;             // Button pin.
  int lastReading;         // Last raw reading.
  int stableState;         // Last stable (debounced) state.
  unsigned long lastDebounceTime; // Last time the reading changed.
};

// Instantiate debounce objects for each button.
ButtonDebounce modeSwitchDebounce = { MODE_SWITCH_PIN, HIGH, HIGH, DRIVE_MODE_MANUAL };
ButtonDebounce activeTrackDebounce = { ACTIVE_TRACK_TOGGLE_PIN, HIGH, HIGH, false };


void setup() {
  // Start Serial for debugging
  unsigned long startTime = millis();
  unsigned long timeout = 1000; // 1 seconds timeout
                                //
  Serial.begin(115200);
  while (!Serial && (millis() - startTime < timeout)) {
    // Wait for Serial or timeout
  }
  setupInput();
  setupBLE();
}

void loop() {
  readStatusData();
  readControllerData();
  sendControllerData();
  delay(1000 / DATA_RATE); // Control the data rate.
}

void setupInput() {
  // Set button input pins as input with internal pull-up.
  pinMode(MODE_SWITCH_PIN, INPUT_PULLUP);
  pinMode(ACTIVE_TRACK_TOGGLE_PIN, INPUT_PULLUP);
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
    Serial.printf("Received state=%d, driveMode=%d, activeTrackToggled=%d\n", receivedData.state, receivedData.driveMode, receivedData.activeTrackToggled);
    updateState(receivedData.state);
    updateDriveMode(receivedData.driveMode);

    //Serial.printf("Data interval: %d(ms)\n", millis() - lastPingTime);
    //lastPingTime = millis();
  }
}

//// Returns true if the button is detected as a new press (transition to LOW)
//// without using a blocking delay.
//bool buttonPressed(ButtonDebounce &button, unsigned long debounceDelay = 0) {
//  int reading = digitalRead(button.pin);
//  bool event = false;
//
//  // If the reading has changed, reset the debounce timer.
//  if (reading != button.lastReading) {
//    button.lastDebounceTime = millis();
//  }
//
//  // If the reading has been stable for longer than debounceDelay...
//  if ((millis() - button.lastDebounceTime) > debounceDelay) {
//    // If the stable state has changed, update it.
//    if (reading != button.stableState) {
//      button.stableState = reading;
//      // Detect a press (when the stable state goes LOW).
//      if (button.stableState == LOW) {
//        event = true;
//      }
//    }
//  }
//
//  button.lastReading = reading;
//  return event;
//}

bool buttonPressed(ButtonDebounce &button) {
  int previousState = button.stableState;
  button.stableState = digitalRead(button.pin);
  return previousState != button.stableState && button.stableState == LOW;
}

// Check both buttons using the shared debounce function.
void checkButtons() {
  if (buttonPressed(modeSwitchDebounce)) {
    // Toggle drive mode.
    driveModeTriggerValue = (driveModeTriggerValue == DRIVE_MODE_MANUAL)
                              ? DRIVE_MODE_AUTO_FOLLOW
                              : DRIVE_MODE_MANUAL;
    updateDriveMode(driveModeTriggerValue);
    Serial.printf("Drive mode toggled: %d\n", driveModeTriggerValue);
  }

  if (buttonPressed(activeTrackDebounce)) {
    // Toggle active track.
    activeTrackToggledValue = true;
    Serial.printf("Active track toggled: %d\n", activeTrackToggledValue);
  }
}

void readControllerData() {
  // Check buttons.
  checkButtons();

  // Read raw joystick values
  int rawX = analogRead(STEERING_STICK_X_PIN);
  int rawY = analogRead(THROTTLE_STICK_Y_PIN);
  int rawGX = analogRead(GIMBAL_STICK_X_PIN);
  int rawGY = analogRead(GIMBAL_STICK_Y_PIN);

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
  if (rawGX == ADC_MID) yawSpeedValue = midPitch;
  if (rawGY == ADC_MID) pitchSpeedValue = midYaw;

  // Print all values, including gimbal joystick positions
  //Serial.printf("ModeTrigger=%d, Joystick X=%.1f, Y=%.1f | Throttle=%d, Steering=%d | Gimbal X=%.1f, Y=%.1f | Yaw=%.2f, Pitch=%.2f\n",
  //driveModeTriggerValue, smoothedX, smoothedY, throttleValue, steeringValue, smoothedGX, smoothedGY, yawSpeedValue, pitchSpeedValue);
}

bool controllerDataChanged(const ControllerData &oldData, const ControllerData &newData) {
  if (oldData.throttleValue != newData.throttleValue) return true;
  if (oldData.steeringValue != newData.steeringValue) return true;
  if (oldData.driveMode != newData.driveMode) return true;
  if (fabs(oldData.yawSpeed - newData.yawSpeed) > 0.01) return true;
  if (fabs(oldData.pitchSpeed - newData.pitchSpeed) > 0.01) return true;
  if (oldData.activeTrackToggled != newData.activeTrackToggled) return true;
  return false;
}

void sendControllerData() {
  if (!(BLECentral && BLECentral.connected())) {
    updateState(state & ~STATE_REMOTE_CONTROLLER_READY);
    if (!connectToCentral()) {
      return;
    }
    updateState(state | STATE_REMOTE_CONTROLLER_READY);
  }

  ControllerData data = {
    .throttleValue = throttleValue,
    .steeringValue = steeringValue,
    .driveMode = driveModeTriggerValue,
    .yawSpeed = yawSpeedValue,
    .pitchSpeed = pitchSpeedValue,
    .activeTrackToggled = activeTrackToggledValue
  };

  // Send update only if there is a change.
  static bool firstUpdate = true;
  if (firstUpdate || controllerDataChanged(lastSentData, data)) {
    AutocamControllerData.writeValue((uint8_t *)&data, sizeof(ControllerData));
    activeTrackToggledValue = false;
    lastSentData = data;
    firstUpdate = false;
    Serial.printf("Sent via BLE: throttle=%d, steering=%d, driveMode=%d, state=%d, yawSpeed=%f, pitchSpeed=%f, activeTrackToggled=%d\n", data.throttleValue, data.steeringValue, data.driveMode, data.state, data.yawSpeed, data.pitchSpeed, data.activeTrackToggled);
  }
  //Serial.printf("Data interval: %d(ms)\n", millis() - lastPingTime);
  lastPingTime = millis();
}
