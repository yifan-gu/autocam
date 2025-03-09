#include <SPI.h>

#include "DW1000Ranging.h"
#include "DW1000.h"
#include "BLE_setup.hpp"
#include "LED_controller.hpp"

#define DATA_RATE 100 // 100 Hz

// Available state.
#define STATE_NOT_READY 0
#define STATE_SENSOR_READY 1
#define STATE_REMOTE_CONTROLLER_READY 2

#define DRIVE_MODE_MANUAL 0
#define DRIVE_MODE_AUTO_FOLLOW 1

// SPI pins
#define SPI_SCK 18
#define SPI_MISO 19
#define SPI_MOSI 23
#define DW_CS 4

// DW1000 pins
#define PIN_RST 27 // reset pin
#define PIN_IRQ 34 // irq pin
#define PIN_SS 4   // spi select pin

// Remote controller pins
#define UWB_SELECTOR_SWITCH_PIN 12
#define ACTIVE_TRACK_BUTTON_PIN 13
#define LOCK_SWITCH_PIN 14
#define MODE_BUTTON_PIN 15

#define BATTERY_LED_RED_PIN 16
#define BATTERY_LED_GREEN_PIN 5
#define BATTERY_LED_BLUE_PIN -1
#define UWB_SELECTOR_LED_RED_PIN -1
#define UWB_SELECTOR_LED_GREEN_PIN -1
#define UWB_SELECTOR_LED_BLUE_PIN 17
#define SENSOR_LED_RED_PIN 21
#define SENSOR_LED_GREEN_PIN 22
#define SENSOR_LED_BLUE_PIN -1
#define REMOTE_CONTROLLER_LED_RED_PIN 26
#define REMOTE_CONTROLLER_LED_GREEN_PIN 32
#define REMOTE_CONTROLLER_LED_BLUE_PIN -1
#define DRIVE_MODE_LED_RED_PIN -1
#define DRIVE_MODE_LED_GREEN_PIN -1
#define DRIVE_MODE_LED_BLUE_PIN 33

#define STEERING_STICK_X_PIN 2
#define THROTTLE_STICK_Y_PIN 36
#define GIMBAL_STICK_X_PIN 35
#define GIMBAL_STICK_Y_PIN 25
#define BATTERY_ADC_PIN 39  // ADC pin to measure battery voltage.

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
uint16_t uwbSelector = 0;
bool uwbStarted = false;

// Variables to store the drive mode button trigger events.
int driveModeTriggerValue = DRIVE_MODE_MANUAL;

// Variables toggle the UWB selector switch trigger events.
uint16_t uwbSelectorTriggerValue = 0;

// Variables to store values
int throttleValue = midThrottle, steeringValue = midSteering;

// Variables to store the gimbal controller data.
float yawSpeedValue = 0.0;
float pitchSpeedValue = 0.0;
bool activeTrackToggledValue = false;

ControllerData receivedData;

// Global variable to hold last sent data.
// (Initialize with known defaults.)
ControllerData lastSentData = { midThrottle, midSteering, DRIVE_MODE_MANUAL, midPitch, midYaw, false, uwbSelector };

unsigned int lastPingTime = 0;

//calibrated Antenna Delay setting for this anchor
uint16_t Adelay = 16630;

// leftmost two bytes below will become the "short address"
char uwb_addr[] = "01:00:BC:FE:41:2A:39:80";

// --- Button Debounce Helpers ---
struct ButtonDebounce {
  uint8_t pin;             // Button pin.
  int lastReading;         // Last raw reading.
  int stableState;         // Last stable (debounced) state.
  unsigned long lastDebounceTime; // Last time the reading changed.
};

// Instantiate debounce objects for each button.
ButtonDebounce modeSwitchDebounce = { MODE_BUTTON_PIN, HIGH, HIGH, DRIVE_MODE_MANUAL };
ButtonDebounce activeTrackDebounce = { ACTIVE_TRACK_BUTTON_PIN, HIGH, HIGH, false };

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
  setupInput();
  setupBLE();
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
    readStatusData();
    readControllerData();
    sendControllerData();
    checkBattery();

    // Short delay to yield to other tasks; adjust as necessary.
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}

void loop() {
  checkUWB();
}

void setupLED() {
  ledController.initSensorLED(SENSOR_LED_RED_PIN, SENSOR_LED_GREEN_PIN, SENSOR_LED_BLUE_PIN);
  ledController.initRemoteLED(REMOTE_CONTROLLER_LED_RED_PIN, REMOTE_CONTROLLER_LED_GREEN_PIN, REMOTE_CONTROLLER_LED_BLUE_PIN);
  ledController.initDriveLED(DRIVE_MODE_LED_RED_PIN, DRIVE_MODE_LED_GREEN_PIN, DRIVE_MODE_LED_BLUE_PIN);
  ledController.initUWBSelectorLED(UWB_SELECTOR_LED_RED_PIN, UWB_SELECTOR_LED_GREEN_PIN, UWB_SELECTOR_LED_BLUE_PIN);
  ledController.initBatteryLED(BATTERY_LED_RED_PIN, BATTERY_LED_GREEN_PIN, BATTERY_LED_BLUE_PIN, BATTERY_ADC_PIN, minVoltage, maxVoltage);

  ledController.updateStateLED(state);
  ledController.updateDriveModeLED(driveMode);
  ledController.updateBatteryLED();
  ledController.updateUWBSelectorLED(uwbSelector);
}

void setupInput() {
  // Set button input pins as input with internal pull-up.
  pinMode(MODE_BUTTON_PIN, INPUT_PULLUP);
  pinMode(ACTIVE_TRACK_BUTTON_PIN, INPUT_PULLUP);
  pinMode(LOCK_SWITCH_PIN, INPUT_PULLUP);
  pinMode(UWB_SELECTOR_SWITCH_PIN, INPUT_PULLUP);
}

void startUWB() {
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

void stopUWB() {
  DW1000.end();
}

void checkUWB() {
  if (uwbSelector == 0) {
    if (!uwbStarted) {
      return; // Do nothing.
    }
    stopUWB();
    uwbStarted = false;
    return;
  }

  if (!uwbStarted) {
    startUWB();
    uwbStarted = true;
  }
  DW1000Ranging.loop();
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
  ledController.updateStateLED(state);
  Serial.printf("Update state=%d\n", state);
}

void updateDriveMode(int newDriveMode) {
  if (driveMode == newDriveMode) {
    return;
  }
  driveMode = newDriveMode;
  ledController.updateDriveModeLED(driveMode);
  Serial.printf("Update drive mode=%d\n", driveMode);
}

void updateUWBSelector(uint16_t newUWBSelector) {
  if (uwbSelector == newUWBSelector) {
    return;
  }
  uwbSelector = newUWBSelector;
  ledController.updateUWBSelectorLED(uwbSelector);
}

void readStatusData() {
  if (!(BLECentral && BLECentral.connected())) {
    updateState(state & ~STATE_REMOTE_CONTROLLER_READY);
    driveModeTriggerValue = DRIVE_MODE_MANUAL;
    if (!connectToCentral()) {
      return;
    }
    updateState(state | STATE_REMOTE_CONTROLLER_READY);
  }

  if (AutocamControllerData.written()) {
    AutocamControllerData.readValue((uint8_t *)&receivedData, sizeof(ControllerData));
    Serial.printf("Received state=%d, driveMode=%d, activeTrackToggled=%d, uwbSelector=%d\n", receivedData.state, receivedData.driveMode, receivedData.activeTrackToggled, receivedData.uwbSelector);
    updateState(receivedData.state);
    updateDriveMode(receivedData.driveMode);
    updateUWBSelector(receivedData.uwbSelector);

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
    Serial.printf("Drive mode toggled: %d\n", driveModeTriggerValue);
  }

  activeTrackToggledValue = false;
  if (buttonPressed(activeTrackDebounce)) {
    // Toggle active track.
    activeTrackToggledValue = true;
    Serial.printf("Active track toggled: %d\n", activeTrackToggledValue);
  }
}

bool lockSwitchLocked() {
  return digitalRead(LOCK_SWITCH_PIN) == LOW;
}

uint16_t readUWBSelector() {
  return digitalRead(UWB_SELECTOR_SWITCH_PIN) == LOW ? 1 : 0;
}

void readControllerData() {
  if (lockSwitchLocked()) {
    // Check the lock switch first.
    // If locked then skip reading the inputs.
    // TODO(yifan): maybe reset all inputs??
    Serial.println("Lock switch in active, skip input reading");
    return;
  }

  // Check buttons.
  checkButtons();
  uwbSelectorTriggerValue = readUWBSelector();

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
  //              driveModeTriggerValue, smoothedX, smoothedY, throttleValue, steeringValue, smoothedGX, smoothedGY, yawSpeedValue, pitchSpeedValue);
}

bool controllerDataChanged(const ControllerData &oldData, const ControllerData &newData) {
  if (oldData.throttleValue != newData.throttleValue) return true;
  if (oldData.steeringValue != newData.steeringValue) return true;
  if (oldData.driveMode != newData.driveMode) return true;
  if (fabs(oldData.yawSpeed - newData.yawSpeed) > 0.01) return true;
  if (fabs(oldData.pitchSpeed - newData.pitchSpeed) > 0.01) return true;
  if (oldData.activeTrackToggled != newData.activeTrackToggled) return true;
  if (oldData.uwbSelector != newData.uwbSelector) return true;
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
    .activeTrackToggled = activeTrackToggledValue,
    .uwbSelector = uwbSelectorTriggerValue,
  };

  // Send update only if there is a change.
  static bool firstUpdate = true;
  if (firstUpdate || controllerDataChanged(lastSentData, data)) {
    AutocamControllerData.writeValue((uint8_t *)&data, sizeof(ControllerData));
    lastSentData = data;
    lastSentData.activeTrackToggled = false; // Do not resend "activeTrackToggled = false" because it's an no-op anyway.
    firstUpdate = false;

    //Serial.printf("Sent via BLE: throttle=%d, steering=%d, driveMode=%d, state=%d, yawSpeed=%f, pitchSpeed=%f, activeTrackToggled=%d, uwbSelector=%d\n", data.throttleValue, data.steeringValue, data.driveMode, data.state, data.yawSpeed, data.pitchSpeed, data.activeTrackToggled, data.uwbSelector);
  }
  //Serial.printf("Data interval: %d(ms)\n", millis() - lastPingTime);
  lastPingTime = millis();
}

void checkBattery() {
  unsigned int now = millis();
  if (now - lastBatteryCheckTimeMillis > batteryCheckIntervalMillis) {
    lastBatteryCheckTimeMillis = now;
    ledController.updateBatteryLED();
  }
}

void newRange() {
  Serial.printf("UWB Sensor address=%X, distance=%f(m)\n", DW1000Ranging.getDistantDevice()->getShortAddress(), DW1000Ranging.getDistantDevice()->getRange());
}

void newDevice(DW1000Device *device) {
  Serial.printf("UWB Sensor connected, address=%X\n", device->getShortAddress());
}

void inactiveDevice(DW1000Device *device) {
  Serial.printf("UWB Sensor disconnected, address=%X\n", device->getShortAddress());
}
