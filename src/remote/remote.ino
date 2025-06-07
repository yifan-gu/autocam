#include <SPI.h>

#include "DW1000Ranging.h"
#include "DW1000.h"
#include "BLE_setup.hpp"
#include "LED_controller.hpp"
#include "util.h"

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
#define GIMBAL_FUNCTION_BUTTON_PIN 13
#define LOCK_SWITCH_PIN 14
#define DRIVE_MODE_BUTTON_PIN 25

#define BATTERY_LED_RED_PIN 26
#define BATTERY_LED_GREEN_PIN 32
#define BATTERY_LED_BLUE_PIN -1
#define UWB_SELECTOR_LED_RED_PIN -1
#define UWB_SELECTOR_LED_GREEN_PIN -1
#define UWB_SELECTOR_LED_BLUE_PIN 22
#define SENSOR_LED_RED_PIN 5
#define SENSOR_LED_GREEN_PIN 16
#define SENSOR_LED_BLUE_PIN -1
#define REMOTE_LED_RED_PIN 21
#define REMOTE_LED_GREEN_PIN 17
#define REMOTE_LED_BLUE_PIN -1
#define DRIVE_MODE_LED_RED_PIN -1
#define DRIVE_MODE_LED_GREEN_PIN -1
#define DRIVE_MODE_LED_BLUE_PIN 33

#define STEERING_STICK_X_PIN 2
#define THROTTLE_STICK_Y_PIN 36
#define GIMBAL_STICK_X_PIN 15
#define GIMBAL_STICK_Y_PIN 35
#define BATTERY_ADC_PIN 39  // ADC pin to measure battery voltage.

const int16_t minThrottle = 1000, maxThrottle = 2000, midThrottle = 1500;
const int16_t minSteering = 1000, maxSteering = 2000, midSteering = 1500;
const float minPitch = -4, maxPitch = 4, midPitch = 0;
const float minYaw = -4, maxYaw = 4, midYaw = 0;

// State indicators.
uint8_t state = SERVER_STATE_NOT_READY;
uint8_t driveMode = DRIVE_MODE_MANUAL;
uint8_t uwbSelector = 1;
bool uwbStarted = false;

// Variables to store the drive mode button trigger events.
uint8_t driveModeTriggerValue = DRIVE_MODE_MANUAL;

// Variables toggle the UWB selector switch trigger events.
uint8_t uwbSelectorTriggerValue = 0;

// Variables to store values
int16_t throttleValue = midThrottle, steeringValue = midSteering;

// Variables to store the gimbal controller data.
float yawSpeedValue = 0.0;
float pitchSpeedValue = 0.0;
uint8_t toggleState = NO_GIMBAL_TOGGLE;

bool buttonIsPressed = false;

RemoteDataRecv receivedData;

// Global variable to hold last sent data.
// (Initialize with known defaults.)
RemoteDataSend lastSentData = { midThrottle, midSteering, DRIVE_MODE_MANUAL, midPitch, midYaw, false, uwbSelector };

unsigned int lastPingTime = 0;
unsigned int lastHeartBeatMillis = 0;
unsigned int heartBeatDuration = 200;

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
ButtonDebounce modeSwitchDebounce = { DRIVE_MODE_BUTTON_PIN, HIGH, HIGH, DRIVE_MODE_MANUAL };
ButtonDebounce gimbalFnDebounce = { GIMBAL_FUNCTION_BUTTON_PIN, HIGH, HIGH, 0 };


// Joystick Input Helpers
struct JoystickInput {
  uint8_t pin;           // Analog pin
  int adcMin;            // ADC minimum value
  int adcMax;            // ADC maximum value
  int adcMid;            // Midpoint
  int deadZone;          // Dead zone range around the center
  float smoothingFactor; // EMA smoothing factor
  float smoothedValue;   // Internal smoothed value
};

// Joystick Inputs
JoystickInput steeringX = {STEERING_STICK_X_PIN, 600, 3100, 1850, 50, 0.2f};
JoystickInput throttleY = {THROTTLE_STICK_Y_PIN, 580, 3080, 1810, 50, 0.2f};
JoystickInput gimbalX = {GIMBAL_STICK_X_PIN, 480, 2890, 1685, 50, 0.2f};
JoystickInput gimbalY = {GIMBAL_STICK_Y_PIN, 590, 3200, 1825, 50, 0.2f};

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
  // Set up the periodic timing
  TickType_t xLastWakeTime = xTaskGetTickCount();
  // Store previous iteration time
  TickType_t previousIteration = xLastWakeTime;

  while (true) {
    // Get the current tick count at the beginning of the iteration
    TickType_t currentIteration = xTaskGetTickCount();
    // Calculate the elapsed time in ticks and convert to milliseconds
    uint32_t intervalMs = (currentIteration - previousIteration) * portTICK_PERIOD_MS;
    previousIteration = currentIteration;
    //LOGF("Interval since last iteration: %u ms\n", intervalMs);

    readStatusData();
    readInput();
    sendInput();
    checkDriveModeLED();
    checkBattery();

    vTaskDelay(pdMS_TO_TICKS(LOOP_PERIOD_MS));
  }
}

void loop() {
  checkUWB();
}

void setupLED() {
  ledController.initSensorLED(SENSOR_LED_RED_PIN, SENSOR_LED_GREEN_PIN, SENSOR_LED_BLUE_PIN);
  ledController.initRemoteLED(REMOTE_LED_RED_PIN, REMOTE_LED_GREEN_PIN, REMOTE_LED_BLUE_PIN);
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
  pinMode(DRIVE_MODE_BUTTON_PIN, INPUT_PULLUP);
  pinMode(GIMBAL_FUNCTION_BUTTON_PIN, INPUT_PULLUP);
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
  AutocamRemoteService.addCharacteristic(AutocamRemoteDataSend);
  AutocamRemoteService.addCharacteristic(AutocamRemoteDataRecv);
  setupBLEPeripheral("Autocam Remote", "autocam-remote", AutocamRemoteService);
}

void updateState(uint8_t newState) {
  if ((newState & SERVER_STATE_SENSOR_READY) == 0 || (newState & SERVER_STATE_REMOTE_READY) == 0) {
    // Reset drive mode, because at this point the server must be in DRIVE_MODE_MANUAL.
    driveModeTriggerValue = DRIVE_MODE_MANUAL;
    lastSentData.driveMode = DRIVE_MODE_MANUAL;
  }

  if (state == newState) {
    return;
  }

  state = newState;
  ledController.updateStateLED(state);
  LOGF("Update state=%d\n", state);
}

void updateDriveMode(uint8_t newDriveMode) {
  if (driveMode == newDriveMode) {
    return;
  }
  driveMode = newDriveMode;
  ledController.updateDriveModeLED(driveMode);
  LOGF("Update drive mode=%d\n", driveMode);
}

void updateUWBSelector(uint8_t newUWBSelector) {
  if (uwbSelector == newUWBSelector) {
    return;
  }
  uwbSelector = newUWBSelector;
  ledController.updateUWBSelectorLED(uwbSelector);
}

void readStatusData() {
  if (!(BLECentral && BLECentral.connected())) {
    updateState(SERVER_STATE_NOT_READY);
    resetInputButtonValues();
    if (!connectToCentral()) {
      return;
    }
    updateState(state | SERVER_STATE_REMOTE_READY);
  }

  if (AutocamRemoteDataRecv.written()) {
    AutocamRemoteDataRecv.readValue((uint8_t *)&receivedData, sizeof(RemoteDataRecv));
    LOGF("Received state=%d, driveMode=%d, uwbSelector=%d\n", receivedData.state, receivedData.driveMode, receivedData.uwbSelector);
    updateState(receivedData.state);
    updateDriveMode(receivedData.driveMode);
    updateUWBSelector(receivedData.uwbSelector);

    //LOGF("Data interval: %d(ms)\n", millis() - lastPingTime);
    //lastPingTime = millis();
  }
}

bool buttonPressed(ButtonDebounce &button) {
  int previousState = button.stableState;
  button.stableState = digitalRead(button.pin);
  return previousState != button.stableState && button.stableState == LOW;
}

void checkGimbalFnButton() {
  // Use a static variable to track clicks and time.
  static unsigned long lastClickTime = 0;
  static int clickCount = 0;
  const unsigned long clickTimeout = 500; // Time window in ms for multi-click recognition

  // If the button is pressed (debounced)
  if (buttonPressed(gimbalFnDebounce)) {
    unsigned long now = millis();
    // If within the timeout window, increment the click counter,
    // otherwise start a new series.
    if (now - lastClickTime < clickTimeout) {
      clickCount++;
    } else {
      clickCount = 1;
    }
    lastClickTime = now;
  }

  // If the timeout has passed since the last detected click, process the clicks.
  if (clickCount > 0 && (millis() - lastClickTime > clickTimeout)) {
    if (clickCount == 1) {
      toggleState = ACTIVE_TRACK_TOGGLED;
    } else if (clickCount == 2) {
      toggleState = GIMBAL_RECENTER_TOGGLED;
    } else if (clickCount >= 3) {
      toggleState = CAMERA_RECORDING_TOGGLED;
    }
    //LOGF("Toggle state set to: %d after %d click(s)\n", toggleState, clickCount);
    clickCount = 0;
    buttonIsPressed = true;
  }
}

void checkDriveModeButton() {
  // Use a static variable to track clicks and time.
  static unsigned long lastClickTime = 0;
  static int clickCount = 0;
  const unsigned long clickTimeout = 500; // Time window in ms for multi-click recognition

  // If the button is pressed (debounced)
  if (buttonPressed(modeSwitchDebounce)) {
    unsigned long now = millis();
    // If within the timeout window, increment the click counter,
    // otherwise start a new series.
    if (now - lastClickTime < clickTimeout) {
      clickCount++;
    } else {
      clickCount = 1;
    }
    lastClickTime = now;
  }

  // If the timeout has passed since the last detected click, process the clicks.
  if (clickCount > 0 && (millis() - lastClickTime > clickTimeout)) {

    if (clickCount == 1) {
      driveModeTriggerValue = DRIVE_MODE_MANUAL;
    } else if (clickCount == 2) {
      driveModeTriggerValue = DRIVE_MODE_FOLLOW;
    } else if (clickCount >= 3) {
      driveModeTriggerValue = DRIVE_MODE_CINEMA;
    }
    //LOGF("Drive mode set to: %d after %d click(s)\n", driveModeTriggerValue, clickCount);
    // Optionally update the global drive mode immediately:
    // Reset the click counter for the next sequence.
    clickCount = 0;
    buttonIsPressed = true;
  }
}

// Check both buttons using the shared debounce function.
void checkButtons() {
  checkGimbalFnButton();
  checkDriveModeButton();
}

bool lockSwitchLocked() {
  return digitalRead(LOCK_SWITCH_PIN) == LOW;
}

uint8_t readUWBSelector() {
  return digitalRead(UWB_SELECTOR_SWITCH_PIN) == LOW ? 1 : 0;
}

float readJoystick(JoystickInput &joy) {
  int raw = analogRead(joy.pin);

  // Apply dead zone
  if (abs(raw - joy.adcMid) < joy.deadZone) {
    raw = joy.adcMid;
  }

  // Exponential Moving Average smoothing
  joy.smoothedValue = (joy.smoothingFactor * raw) +
                      ((1.0f - joy.smoothingFactor) * joy.smoothedValue);
  return joy.smoothedValue;
}

float mapJoystickCentered(float smoothedValue, int adcMin, int adcMid, int adcMax, int deadZone, float outA, float outMid, float outB) {
  if (abs((int)smoothedValue - adcMid) < deadZone) {
    return outMid;
  } else if (smoothedValue < adcMid) {
    return mapFloat(smoothedValue, adcMin, adcMid, outA, outMid);
  } else {
    return mapFloat(smoothedValue, adcMid, adcMax, outMid, outB);
  }
}

float mapFloat(float x, float in_min, float in_max, float outA, float outB) {
  float result = outA + (x - in_min) * (outB - outA) / (in_max - in_min);
  // Clamp result according to the ordering of outA and outB
  if (outA < outB) {
    if(result < outA) result = outA;
    if(result > outB) result = outB;
  } else {
    if(result > outA) result = outA;
    if(result < outB) result = outB;
  }
  return result;
}

void readJoysticks() {
  steeringValue = mapJoystickCentered(readJoystick(steeringX), steeringX.adcMin, steeringX.adcMid, steeringX.adcMax, steeringX.deadZone, minSteering, midSteering, maxSteering);
  throttleValue = mapJoystickCentered(readJoystick(throttleY), throttleY.adcMin, throttleY.adcMid, throttleY.adcMax, throttleY.deadZone, maxThrottle, midThrottle, minThrottle);
  yawSpeedValue = mapJoystickCentered(readJoystick(gimbalX), gimbalX.adcMin, gimbalX.adcMid, gimbalX.adcMax, gimbalX.deadZone, minYaw, midYaw, maxYaw);
  pitchSpeedValue = mapJoystickCentered(readJoystick(gimbalY), gimbalY.adcMin, gimbalY.adcMid, gimbalY.adcMax, gimbalY.deadZone, maxPitch, midPitch, minPitch);
}

void readInput() {
  if (lockSwitchLocked()) {
    // Check the lock switch first.
    // If locked then skip reading the inputs.
    // TODO(yifan): maybe reset all inputs??
    //LOGLN("Lock switch in active, skip input reading");
    return;
  }

  // Check buttons.
  checkButtons();
  uwbSelectorTriggerValue = readUWBSelector();
  readJoysticks();

  // Print all values, including gimbal joystick positions
  //LOGF("ModeTrigger=%d, Joystick X=%.1f, Y=%.1f | Steering=%d, Throttle=%d | Gimbal X=%.1f, Y=%.1f | Yaw=%.2f, Pitch=%.2f\n",
  //              driveModeTriggerValue,  steeringX.smoothedValue, throttleY.smoothedValue, steeringValue, throttleValue, gimbalX.smoothedValue, gimbalY.smoothedValue, yawSpeedValue, pitchSpeedValue);
}

bool dataChanged(const RemoteDataSend &oldData, const RemoteDataSend &newData) {
  if (oldData.throttleValue != newData.throttleValue) return true;
  if (oldData.steeringValue != newData.steeringValue) return true;
  if (oldData.yawSpeed !=  newData.yawSpeed) return true;
  if (oldData.pitchSpeed != newData.pitchSpeed) return true;
  if (oldData.uwbSelector != newData.uwbSelector) return true;
  return false;
}

void sendInput() {
  unsigned int now = millis();

  if (!(BLECentral && BLECentral.connected())) {
    updateState(SERVER_STATE_NOT_READY);
    resetInputButtonValues();
    if (!connectToCentral()) {
      return;
    }
    updateState(state | SERVER_STATE_REMOTE_READY);
  }

  RemoteDataSend data = {
    .throttleValue = throttleValue,
    .steeringValue = steeringValue,
    .yawSpeed = yawSpeedValue,
    .pitchSpeed = pitchSpeedValue,
    .driveMode = driveModeTriggerValue,
    .toggleState = toggleState,
    .uwbSelector = uwbSelectorTriggerValue,
  };

  // Send update only if there is a change.
  if (buttonIsPressed || dataChanged(lastSentData, data) || now - lastHeartBeatMillis > heartBeatDuration) {
    AutocamRemoteDataSend.writeValue((uint8_t *)&data, sizeof(RemoteDataSend));
    lastSentData = data;
    lastSentData.toggleState = NO_GIMBAL_TOGGLE; // Do not resend zero toggleState, because it's an no-op anyway.
    toggleState = NO_GIMBAL_TOGGLE;
    buttonIsPressed = false;

    lastHeartBeatMillis = now;
    //LOGF("Sent via BLE: throttle=%d, steering=%d, driveMode=%d, yawSpeed=%f, pitchSpeed=%f, toggleState=%d, uwbSelector=%d\n", data.throttleValue, data.steeringValue, data.driveMode, data.yawSpeed, data.pitchSpeed, data.toggleState, data.uwbSelector);
  }
  //LOGF("Data interval: %d(ms)\n", millis() - lastPingTime);
  lastPingTime = now;
}

void checkDriveModeLED() {
  ledController.updateDriveModeLED(driveMode);
}

void checkBattery() {
  unsigned int now = millis();
  if (now - lastBatteryCheckTimeMillis > batteryCheckIntervalMillis) {
    lastBatteryCheckTimeMillis = now;
    ledController.updateBatteryLED();
  }
}

void newRange() {
  //LOGF("Autocam Sensor address=%X, distance=%f(m)\n", DW1000Ranging.getDistantDevice()->getShortAddress(), DW1000Ranging.getDistantDevice()->getRange());
}

void newDevice(DW1000Device *device) {
  LOGF("Autocam Sensor connected, address=%X\n", device->getShortAddress());
}

void inactiveDevice(DW1000Device *device) {
  LOGF("Autocam Sensor disconnected, address=%X\n", device->getShortAddress());
}

void resetInputButtonValues() {
  driveModeTriggerValue = DRIVE_MODE_MANUAL;
  toggleState = NO_GIMBAL_TOGGLE;
}
