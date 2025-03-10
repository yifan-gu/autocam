#include <SPI.h>

#include "DW1000Ranging.h"
#include "DW1000.h"
#include "DJI_ronin_controller.hpp"
#include "BLE_setup.hpp"
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

// DJI Can pins
#define CAN_TX 12  // Connects to CTX
#define CAN_RX 2  // Connects to CRX
#define CAN_RATE 1000 // Speed rate in kbps

#define SERVER_LED_RED_PIN 25
#define SERVER_LED_GREEN_PIN 26
#define SERVER_LED_BLUE_PIN -1
#define TAG_LED_RED_PIN 32
#define TAG_LED_GREEN_PIN 33
#define TAG_LED_BLUE_PIN -1

// leftmost two bytes below will become the "short address"
char uwbAddr[] = "AA:AA:6F:BE:08:3D:4E:72";

struct Anchor {
  uint16_t address;
  bool connected;
};

Anchor uwbAnchors[] = {
  {0x0000, false}, // NOTE(yifan): Make the UWB short address same as the index, for quick looking up.
  {0x0001, false}
};

uint16_t uwbSelector = 0;

// TAG antenna delay defaults to 16384
uint16_t Adelay = 16384;

float distance = 0;
float heading = 0;
int state = SENSOR_STATE_TAG_CONNECTED;
float yawSpeed = 0;
float pitchSpeed = 0;

int toggleState = 0; // Bit 0 = activeTrackToggled, bit 1 = gimbalRecenterToggled, bit 2 = cameraRecordingToggled.

DJIRoninController djiRoninController(CAN_TX, CAN_RX, CAN_RATE);

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

  setupLED();
  setupUWB();
  //setupDJIRoninController();
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

    //getHeading();
    sendSensorDataSend();
    getSensorDataRecv();
    applyUWBSelector();
    //setGimbalPosition();
    checkActiveTrack();
    checkGimbalRecenter();
    checkCameraRecording();

    vTaskDelay(pdMS_TO_TICKS(LOOP_PERIOD_MS));
  }
}

void loop() {
  getDistance();
}

void setupLED() {
  pinMode(TAG_LED_RED_PIN, OUTPUT);
  pinMode(TAG_LED_GREEN_PIN, OUTPUT);
  pinMode(SERVER_LED_RED_PIN, OUTPUT);
  pinMode(SERVER_LED_GREEN_PIN, OUTPUT);
  digitalWrite(TAG_LED_RED_PIN, HIGH);
  digitalWrite(TAG_LED_GREEN_PIN, HIGH);
  digitalWrite(SERVER_LED_RED_PIN, HIGH);
  digitalWrite(SERVER_LED_GREEN_PIN, HIGH);

  updateLED(state);
}

void setupBLE() {
  AutocamSensorService.addCharacteristic(AutocamSensorDataSend);
  AutocamSensorService.addCharacteristic(AutocamSensorDataRecv);
  setupBLEPeripheral("Autocam Sensor", "autocam-sensor", AutocamSensorService);
}

void setupUWB() {
  //init the configuration
  SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI);
  DW1000Ranging.initCommunication(PIN_RST, PIN_SS, PIN_IRQ); //Reset, CS, IRQ pin

  // set antenna delay for anchors only. Tag is default (16384)
  //DW1000.setAntennaDelay(Adelay);

  DW1000Ranging.attachNewRange(newRange);
  DW1000Ranging.attachNewDevice(newDevice);
  DW1000Ranging.attachInactiveDevice(inactiveDevice);

  // NOTE (yifan): The DW1000 "tag" is different from the Autocam "tag".
  // In fact, the naming conventions are reversed:
  //   - DW1000 "anchors" correspond to Autocam Remote and Autocam Tag.
  //   - DW1000 "tag" corresponds to sensor.
  // This reversal is due to the DW1000 library's limitation of supporting a single tag with multiple anchors,
  // whereas our system uses multiple anchors (Autocam Remote and Autocam Tag).
  DW1000Ranging.startAsTag(uwbAddr, DW1000.MODE_LONGDATA_RANGE_LOWPOWER, false);
}

void getSensorDataRecv() {
  if (!(BLECentral && BLECentral.connected())) {
    if (!connectToCentral()) {
      resetGimbalControl();
      return;
    }
  }
  updateState(state | SENSOR_STATE_SERVER_CONNECTED);

  if (!AutocamSensorDataRecv.written()) {
    return;
  }

  // TODO(yifan): Maybe verify the input.
  SensorDataRecv data;
  AutocamSensorDataRecv.readValue((uint8_t *)&data, sizeof(SensorDataRecv));
  //LOGF("Received yawSpeed=%f, pitchSpeed=%f, toggleState=%d, uwbSelector=%d\n", data.yawSpeed, data.pitchSpeed, data.toggleState, data.uwbSelector);
  yawSpeed = data.yawSpeed;
  pitchSpeed = data.pitchSpeed;
  toggleState = data.toggleState;
  uwbSelector = data.uwbSelector;

  //LOGF("Data interval: %d(ms)\n", millis() - lastPingTime);
  lastPingTime = millis();
}

void setGimbalPosition() {
  float yaw = convert_gimbal_speed(yawSpeed);
  float pitch = convert_gimbal_speed(pitchSpeed);
  if (!djiRoninController.set_position(yaw, 0, pitch, 0, 100)) {
    LOGLN("Failed to set DJI Ronin position");
  }
}

void checkActiveTrack() {
  if (toggleState & ACTIVE_TRACK_TOGGLED) {
    if (!djiRoninController.gimbal_active_track()) {
      LOGLN("Failed to trigger DJI Ronin active track");
    }
    toggleState &= ~ACTIVE_TRACK_TOGGLED;
  }
}

void checkGimbalRecenter() {
  if (toggleState & GIMBAL_RECENTER_TOGGLED) {
    if (!djiRoninController.gimbal_recenter()) {
      LOGLN("Failed to trigger DJI Ronin gimbal recenter");
    }
    toggleState &= ~GIMBAL_RECENTER_TOGGLED;
  }
}

void checkCameraRecording() {
  if (!(toggleState & CAMERA_RECORDING_TOGGLED)) {
    return;
  }
  toggleState &= ~CAMERA_RECORDING_TOGGLED;

  bool isCameraRecording = false;
  if (!djiRoninController.check_camera_recording_state(&isCameraRecording)) {
    LOGLN("Failed to get DJI Ronin camera recording state");
    return;
  }

  if (!isCameraRecording) {
    if (!djiRoninController.start_camera_recording()) {
      LOGLN("Failed to trigger DJI Ronin camera recording start");
    }
    return;
  }
  // Camera is already recording, so stop it.
  if (!djiRoninController.stop_camera_recording()) {
    LOGLN("Failed to trigger DJI Ronin camera recording stop");
  }
}

void resetGimbalControl() {
  updateState(state & ~SENSOR_STATE_SERVER_CONNECTED);
  yawSpeed = 0;
  pitchSpeed = 0;
  toggleState = 0;
}

float convert_gimbal_speed(float speed) {
  // TODO(yifan): Convert yawSpeed and pitchSpeed;
  return speed;
}


void setupDJIRoninController() {
  uint8_t SDK_version[4];

  while (!djiRoninController.begin()) {
    LOGLN("Waiting for DJI Ronin to start...");
    delay(1000);
  }

  while (!djiRoninController.get_version(SDK_version)) {
    LOGLN("Reading DJI Ronin SDK version...");
    delay(1000);
  }

  LOGF("DJI R SDK Version=%d.%d.%d.%d\n", SDK_version[0], SDK_version[1], SDK_version[2], SDK_version[3]);
}

void getDistance() {
  DW1000Ranging.loop();
}

void getHeading() {
  float yaw, roll, pitch;
  if (!djiRoninController.get_position(&yaw, &roll, &pitch)) {
    LOGLN("Failed to get DJI Ronin position");
    return;
  }
  heading = -yaw;
  if (heading < 0) {
    heading += 360;
  }
}

void sendSensorDataSend() {
  if (!(BLECentral && BLECentral.connected())) {
    if (!connectToCentral()) {
      resetGimbalControl();
      return;
    }
  }
  updateState(state | SENSOR_STATE_SERVER_CONNECTED);

  SensorDataSend data = {.distance = distance, .heading = heading, .state = state};
  AutocamSensorDataSend.writeValue((uint8_t *)&data, sizeof(SensorDataSend));

  //LOGF("Sent via BLE: distance=%f, heading=%f, state=%d\n", distance, heading, state);
  //LOGF("Data interval: %d(ms)\n", millis() - lastPingTime);
  //lastPingTime = millis();
}

void newRange() {
  DW1000Device *device = DW1000Ranging.getDistantDevice();
  if (device->getShortAddress() == uwbSelector) {
    distance = device->getRange();
    //LOGF("UWB device address=%X, distance=%f(m), state=%d\n", uwbSelector, distance, state);
  }
}

void newDevice(DW1000Device *device) {
  uint16_t address = device->getShortAddress();
  uwbAnchors[address].connected = true;
  LOGF("UWB device connected, address=%X\n", address);
  if (address == uwbSelector) {
    updateState(state | SENSOR_STATE_TAG_CONNECTED);
  }
}

void inactiveDevice(DW1000Device *device) {
  uint16_t address = device->getShortAddress();
  uwbAnchors[address].connected = false;
  LOGF("UWB device disconnected, address=%X\n", address);
  if (address == uwbSelector) {
    updateState(state & ~SENSOR_STATE_TAG_CONNECTED);
  }
}

void applyUWBSelector() {
  if (uwbAnchors[uwbSelector].connected) {
    updateState(state | SENSOR_STATE_TAG_CONNECTED);
  } else {
    updateState(state & ~SENSOR_STATE_TAG_CONNECTED);
  }
}

void updateLED(int state) {
  if (state & SENSOR_STATE_TAG_CONNECTED) {
    LOGLN("tag, green");
    digitalWrite(TAG_LED_RED_PIN, HIGH);
    digitalWrite(TAG_LED_GREEN_PIN, LOW);
  } else {
    LOGLN("tag, red");
    digitalWrite(TAG_LED_RED_PIN, LOW);
    digitalWrite(TAG_LED_GREEN_PIN, HIGH);
  }

  if (state & SENSOR_STATE_SERVER_CONNECTED) {
    LOGLN("server, green");
    digitalWrite(SERVER_LED_RED_PIN, HIGH);
    digitalWrite(SERVER_LED_GREEN_PIN, LOW);
  } else {
    LOGLN("server, red");
    digitalWrite(SERVER_LED_RED_PIN, LOW);
    digitalWrite(SERVER_LED_GREEN_PIN, HIGH);
  }
}

void updateState(int newState) {
  if (state == newState) {
    return;
  }
  state = newState;
  updateLED(state);
}

/*
// Dummy data for testing the server
void sendSensorDataStraightVertical() {
  // Define the waypoints for the square path
  const float waypoints[][2] = {
      {10.0, 10.0},    // Point 1: Top-right
      {10.0, -10.0}   // Point 2: Bottom-right
  };
  sendSensorDataWayPoints(waypoints, sizeof(waypoints) / sizeof(waypoints[0]));
}

// Dummy data for testing the server
void sendSensorDataSquare() {
  // Define the waypoints for the square path
  const float waypoints[][2] = {
      {10.0, 10.0},    // Point 1: Top-right
      {-10.0, 10.0},   // Point 2: Top-left
      {-10.0, -10.0},  // Point 3: Bottom-left
      {10.0, -10.0}   // Point 4: Bottom-right
  };
  sendSensorDataWayPoints(waypoints, sizeof(waypoints) / sizeof(waypoints[0]));
}

void sendSensorDataWayPoints(const float waypoints[][2], int waypointCount) {
  static int currentWaypointIndex = 0;  // Index of the current waypoint

  static float posX = 0;  // Current X position
  static float posY = 0;  // Current Y position
  const float stepSize = float(1) / float(DATA_RATE);  // Movement per step

  // Get the target waypoint
  float targetX = waypoints[currentWaypointIndex][0];
  float targetY = waypoints[currentWaypointIndex][1];

  // Calculate the direction to the target waypoint
  float deltaX = targetX - posX;
  float deltaY = targetY - posY;
  float distanceToTarget = sqrt(deltaX * deltaX + deltaY * deltaY);

  // If close enough to the target, move to the next waypoint
  if (distanceToTarget < stepSize) {
    posX = targetX;
    posY = targetY;

    currentWaypointIndex++;
    if (currentWaypointIndex >= waypointCount) {
      currentWaypointIndex = 0;  // Loop back to the first waypoint
    }
  } else {
    // Move toward the target waypoint
    float stepFactor = stepSize / distanceToTarget;
    posX += deltaX * stepFactor;
    posY += deltaY * stepFactor;
  }

  // Calculate distance and heading from (0, 0) to the current position
  distance = sqrt(posX * posX + posY * posY);  // Distance from origin
  heading = atan2(posY, posX) * (180.0 / PI) - 90;  // Heading in degrees. -90 since the heading is calculated from the N direction.
  if (heading < 0) {
    heading += 360;
  }

  // Send `distance` and `heading` via BLE
  sendSensorData();

  // Print debugging information
  //LOGLN("Position: (" + String(posX, 2) + ", " + String(posY, 2) +
  //               ") | Distance: " + String(distance, 2) +
  //               " | Heading: " + String(heading, 2) +
  //               " | Target: (" + String(targetX, 2) + ", " + String(targetY, 2) + ")");
}
*/
