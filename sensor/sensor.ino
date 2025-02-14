#include <SPI.h>

#include "DW1000Ranging.h"
#include "DW1000.h"
#include "DJI_ronin_controller.hpp"
#include "BLE_setup.hpp"

#define PI 3.14159265359
#define DATA_RATE 100 // 100 Hz

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

// leftmost two bytes below will become the "short address"
char anchor_addr[] = "80:00:5B:D5:A9:9A:E2:9C";

//calibrated Antenna Delay setting for this anchor
uint16_t Adelay = 16630;

#define SENSOR_STATE_NOT_READY 0
#define SENSOR_STATE_READY 1

float distance = 0;
float heading = 0;
int state = SENSOR_STATE_NOT_READY;
float yaw_speed = 0;
float pitch_speed = 0;
int active_track_toggled = 0;

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

  setupUWBAnchor();
  setupDJIRoninController();
  setupBLE();
}

void loop() {
  getDistance();
  getHeading();
  sendSensorData();
  getGimbalControllerData();
  setGimbalPosition();
  delay(1000 / DATA_RATE); // Control the data rate.
}

void setupBLE() {
  UWBAnchorService.addCharacteristic(UWBAnchorSensorDataSend);
  UWBAnchorService.addCharacteristic(UWBAnchorSensorDataRecv);
  setupBLEPeripheral("UWB Anchor", "uwb-anchor", UWBAnchorService);
}

void setupUWBAnchor() {
  //init the configuration
  SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI);
  DW1000Ranging.initCommunication(PIN_RST, PIN_SS, PIN_IRQ); //Reset, CS, IRQ pin

  // set antenna delay for anchors only. Tag is default (16384)
  DW1000.setAntennaDelay(Adelay);

  DW1000Ranging.attachNewRange(newRange);
  DW1000Ranging.attachNewDevice(newDevice);
  DW1000Ranging.attachInactiveDevice(inactiveDevice);

  //start the module as an anchor, do not assign random short address
  DW1000Ranging.startAsAnchor(anchor_addr, DW1000.MODE_LONGDATA_RANGE_LOWPOWER, false);
}

void getGimbalControllerData() {
  if (!(BLECentral && BLECentral.connected())) {
    if (!connectToCentral()) {
      resetGimbalControl();
      return;
    }
  }

  if (!UWBAnchorSensorDataRecv.written()) {
    return;
  }

  SensorDataRecv data;
  UWBAnchorSensorDataRecv.readValue((uint8_t *)&data, sizeof(SensorDataRecv));
  Serial.printf("Received yaw_speed=%f, pitch_speed=%f, active_track_toggled=%d\n", data.yaw_speed, data.pitch_speed, data.active_track_toggled);
  yaw_speed = data.yaw_speed;
  pitch_speed = data.pitch_speed;
  active_track_toggled = data.active_track_toggled;

  Serial.printf("Data interval: %d(ms)\n", millis() - lastPingTime);
  lastPingTime = millis();
}

void setGimbalPosition() {
  float yaw = convert_gimbal_speed(yaw_speed);
  float pitch = convert_gimbal_speed(pitch_speed);
  if (!djiRoninController.set_position(yaw, 0, pitch, 0, 100)) {
    Serial.println("Failed to set DJI Ronin position");
  }
}

void resetGimbalControl() {
  yaw_speed = 0;
  pitch_speed = 0;
  active_track_toggled = false;
}

float convert_gimbal_speed(float speed) {
  // TODO(yifan): Convert yaw_speed and pitch_speed;
  return speed;
}


void setupDJIRoninController() {
  uint8_t SDK_version[4];

  while (!djiRoninController.begin()) {
    Serial.println("Waiting for DJI Ronin to start...");
    delay(1000);
  }

  while (!djiRoninController.get_version(SDK_version)) {
    Serial.println("Reading DJI Ronin SDK version...");
    delay(1000);
  }

  Serial.printf("DJI R SDK Version=%d.%d.%d.%d\n", SDK_version[0], SDK_version[1], SDK_version[2], SDK_version[3]);
}

void getDistance() {
  DW1000Ranging.loop();
}

void getHeading() {
  float yaw, roll, pitch;
  if (!djiRoninController.get_position(&yaw, &roll, &pitch)) {
    Serial.println("Failed to get DJI Ronin position");
    return;
  }
  heading = -yaw;
  if (heading < 0) {
    heading += 360;
  }
}

void sendSensorData() {
  if (!(BLECentral && BLECentral.connected())) {
    if (!connectToCentral()) {
      resetGimbalControl();
      return;
    }
  }

  SensorDataSend data = {.distance = distance, .heading = heading, .state = state};
  UWBAnchorSensorDataSend.writeValue((uint8_t *)&data, sizeof(SensorDataSend));

  //Serial.printf("Sent via BLE: distance=%f, heading=%f, state=%d\n", distance, heading, state);
  //Serial.printf("Data interval: %d(ms)\n", millis() - lastPingTime);
  //lastPingTime = millis();
}

void newRange() {
  distance = DW1000Ranging.getDistantDevice()->getRange();
}

void newDevice(DW1000Device *device) {
  Serial.printf("Tag connected, address=%X\n", device->getShortAddress());
  state = SENSOR_STATE_READY;
}

void inactiveDevice(DW1000Device *device) {
  Serial.printf("Anchor disconnected, address=%X\n", device->getShortAddress());
  state = SENSOR_STATE_NOT_READY;
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
  //Serial.println("Position: (" + String(posX, 2) + ", " + String(posY, 2) +
  //               ") | Distance: " + String(distance, 2) +
  //               " | Heading: " + String(heading, 2) +
  //               " | Target: (" + String(targetX, 2) + ", " + String(targetY, 2) + ")");
}
*/
