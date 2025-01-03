#include <ArduinoBLE.h>
#include <ESP32Servo.h>
#include <ESPAsyncWebServer.h>
#include <ESPmDNS.h>
#include <math.h>
#include <WiFi.h>

#include "index_html.h"

#define PI 3.14159265359

#define DRIVE_MODE_MANUAL 0
#define DRIVE_MODE_AUTO_FOLLOW 1

// Pin configuration
#define STEERING_PIN 2  // GPIO2 for steering
#define THROTTLE_PIN 3  // GPIO3 for throttle

// Access Point credentials
const char* ssid = "RC-Controller";
const char* password = "12345678"; // Minimum 8 characters
const char* hostname = "autocam"; // mDNS hostname

// Async WebSocket server
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

// Servo objects
Servo throttleServo;
Servo steeringServo;

// Define throttle and steering ranges
const int minThrottle = 1000, maxThrottle = 2000, midThrottle = 1500;
const int minSteering = 1000, maxSteering = 2000, midSteering = 1500;

// Variable for the drive mode.
int driveMode = DRIVE_MODE_MANUAL;

// Variables to store values
int throttleValue = midThrottle, steeringValue = midSteering;

// The reported distances and angle.

// Distance of the tag to the front anchors.
float distance = 0;
float targetDistance = 0;

// Heading of the anchor.
float heading = 0; // [0, 360)  N = 0/360, W = 90, S = 180, E = 270
float targetHeading = 0;

const float delta = 0.5; // The "play" margin.

// The calculated coordinates of the tag.
float currentX = 0, currentY = 0;

// The target coordinates to match.
float targetX = 0, targetY = 0;

// Heartbeat tracking
unsigned long lastPingTime = 0;           // Time of last received ping
const unsigned long heartbeatTimeout = 1000; // 1 second timeout

const int steeringChangeValue = 500;
const int steeringThrottleChangeValue = 200;

// PID Controller variables for throttle.
float Kp_t = 100;  // Proportional gain. Diff = Kp_t * distance
float Ki_t = 0.0;  // Integral gain. Diff = Ki_t * distance * 1000 * second.
float Kd_t = 0.0;  // Derivative gain. Diff = Kd_t / (speed m/s * 1000 ms)

float previousError_t = 0.0;  // Previous error for the derivative term
float integral_t = 0.0;       // Accumulated integral term

float minMoveThrottle = -300;    // Minimum throttle value in practice.
float maxMoveThrottle = 300;  // Maximum throttle value in practice.

// // PID Controller variables for steering.
float Kp_s = 10;  // Proportional gain. Diff = Kp_s * angle diff.
float Ki_s = 0.0;  // Integral gain. Diff = Ki_s * angle diff * 1000 * second.
float Kd_s = 0 ;  // Derivative gain. Diff = Kd_s / (anglur speed * 1000 ms)

float previousError_s = 0.0;  // Previous error for the derivative term
float integral_s = 0.0;       // Accumulated integral term

float minMoveSteering = -500;    // Minimum steering value in practice.
float maxMoveSteering = 500;  // Maximum steering value in practice.

float lastDeltaTimeMillis = 0; // Used to calculate ki, and Kd in the PID system.
float minDeltaTimeMillis = 10; // Control the rate of PID update.

struct SensorData {
  float distance;
  float heading;
};

SensorData sensorData;

struct ControllerData {
  int throttleValue;
  int steeringValue;
  int driveMode;
};

ControllerData controllerData;

// BLE variables
BLEDevice UWBAnchor, AutocamController;
BLEService UWBAnchorService, AutocamControllerService;

const char UWBAnchorServiceUUID[] = "e2b221c6-7a26-49f4-80cc-0cd58a53041d";
const char AutocamControllerServiceUUID[] = "f49f531a-9cba-4ada-905c-68699d400122";

const char UWBAnchorSensorDataCharacteristicUUID[] = "A319";
const char AutocamControllerControllerDataCharacteristicUUID[] = "B320";

BLECharacteristic UWBAnchorSensorData;
BLECharacteristic AutocamControllerData;

void setup() {
  // Start Serial for debugging
  unsigned long startTime = millis();
  unsigned long timeout = 1000; // 1 seconds timeout

  Serial.begin(115200);
  while (!Serial && (millis() - startTime < timeout)) {
    // Wait for Serial or timeout
  }

  setupESC();
  setupServer();
  setupBLECentral();
}

void loop() {
  //getUWBAnchorData();
  getAutocamControllerData();
  calculateCoordinates();
  calculateSteeringThrottle();
  runESCController();
  runHealthCheck();
}

void setupBLECentral() {
  if (!BLE.begin()) {
    Serial.println("Starting BLE failed!");
    while (1);
  }
  Serial.println("BLE Central initialized.");
  BLE.setConnectionInterval(6, 6);
}

void scanForPeripheral(BLEDevice &device, BLEService &service, BLECharacteristic &characteristic, const char *serviceUUID, const char *characteristicUUID) {
  Serial.printf("Scanning for BLE peripheral [%s]...\n", serviceUUID);
  BLE.scanForUuid(serviceUUID);

  device = BLE.available();
  if (device) {
    Serial.printf("Found BLE peripheral, local name: [%s]\n", device.localName());
    BLE.stopScan();
    connectToBLEDevice(device, service, characteristic, serviceUUID, characteristicUUID);
    return;
  }
  delay(1000);
}

void connectToBLEDevice(BLEDevice &device, BLEService &service, BLECharacteristic &characteristic, const char *serviceUUID, const char *characteristicUUID) {
  if (!device.connect()) {
    Serial.printf("Failed to connect to [%s].\n", device.localName());
    return;
  }

  Serial.printf("Connected to [%s]!\n", device.localName());
  if (!device.discoverAttributes()) {
    Serial.printf("Attribute discovery failed for [%s]! Disconnecting...\n", device.localName());
    device.disconnect();
    return;
  }
    
  service = device.service(serviceUUID);
  Serial.printf("BLE device name: [%s], advertised svc count: [%d], Real svc count: [%d], characteristics count: [%d]\n", device.deviceName(), device.advertisedServiceUuidCount(), device.serviceCount(), device.characteristicCount());
  characteristic = service.characteristic(characteristicUUID);

  if (!characteristic) {
    Serial.printf("Failed to find characteristic [%s] for device [%s]! Disconnecting...\n", characteristicUUID, device.deviceName());
    device.disconnect();
    return;
  }

  if (!characteristic.canSubscribe()) {
    Serial.printf("Cannot subscribe characteristic [%s] for device [%s]! Disconnecting...\n", characteristicUUID, device.deviceName());
    device.disconnect();
    return;
  }

  if (!characteristic.subscribe()) {
    Serial.printf("Failed to subscribe characteristic [%s] for device [%s]! Disconnecting...\n", characteristicUUID, device.deviceName());
    device.disconnect();
    return;
  }
  Serial.printf("Found characteristic [%s] on device [%s] and successfully subscribed!\n", characteristicUUID, device.deviceName());
}

void setupESC() {
  Serial.println("Initializing ESC...");
  delay(1000); // Wait 1 seconds to ensure the ESC ready to arm.

  // Attach servos
  throttleServo.attach(THROTTLE_PIN, minThrottle, maxThrottle);
  steeringServo.attach(STEERING_PIN, minSteering, maxSteering);

  // Initialize to neutral positions to arm the ESC.
  throttleServo.writeMicroseconds(midThrottle);
  steeringServo.writeMicroseconds(midSteering);

  Serial.println("ESC Initialized!");
}

// WiFi, WebServer, and WebSocket setup
void setupServer() {
  // Start Access Point
  WiFi.softAP(ssid, password);
  Serial.println("Access Point started!");
  Serial.print("SSID: ");
  Serial.println(ssid);
  Serial.print("IP Address: ");
  Serial.println(WiFi.softAPIP());

  // Start mDNS
  if (!MDNS.begin(hostname)) {
    Serial.println("Error starting mDNS");
    return;
  }
  Serial.println("mDNS responder started!");
  Serial.print("Access the device at http://");
  Serial.print(hostname);
  Serial.println(".local");

  // Configure WebSocket events
  ws.onEvent(onWebSocketEvent);
  server.addHandler(&ws);

  // Serve static files
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(200, "text/html", index_html);
  });

  // Start server
  server.begin();
}

// Handle WebSocket events
void onWebSocketEvent(AsyncWebSocket *server, AsyncWebSocketClient *client,
                      AwsEventType type, void *arg, uint8_t *data, size_t len) {
  switch (type) {
    case WS_EVT_CONNECT:
      Serial.println("WebSocket connected");
      break;

    case WS_EVT_DISCONNECT:
      Serial.println("WebSocket disconnected");
      setDriveMode(DRIVE_MODE_MANUAL);
      throttleValue = midThrottle;
      steeringValue = midSteering;
      Serial.println("Throttle and Steering reset to middle positions");
      break;

    case WS_EVT_DATA: {
      String message = String((char*)data).substring(0, len);
      lastPingTime = millis();

      if (message == "REQUEST_DATA") {
        // Respond with the current state as a JSON string
        String response = "{";
        response += "\"throttle\":" + String(throttleValue) + ",";
        response += "\"steering\":" + String(steeringValue) + ",";
        response += "\"distance\":" + String(distance, 2) + ",";
        response += "\"heading\":" + String(heading, 2) + ",";
        response += "\"currentX\":" + String(currentX, 2) + ",";
        response += "\"currentY\":" + String(currentY, 2 ) + ",";
        response += "\"targetX\":" + String(targetX, 2) + ",";
        response += "\"targetY\":" + String(targetY, 2);
        response += "}";

        client->text(response); // Send the JSON response to the client
        //Serial.println("Sent data: " + response);
      } else if (message.startsWith("mode=")) {
        if (message == "mode=manual") {
          setDriveMode(DRIVE_MODE_MANUAL);
        } else if (message == "mode=auto") {
          setDriveMode(DRIVE_MODE_AUTO_FOLLOW);
          throttleValue = midThrottle;
          steeringValue = midSteering;
        }
      } else if (message.startsWith("throttle=")) {
        throttleValue = map(message.substring(9).toInt(), 1000, 2000, minThrottle, maxThrottle);
      } else if (message.startsWith("steering=")) {
        steeringValue = map(message.substring(9).toInt(), 1000, 2000, minSteering, maxSteering);
      }
      break;
    }

    case WS_EVT_ERROR:
      Serial.println("WebSocket error");
      break;

    default:
      Serial.println("Unknown WebSocket event");
      break;
  }
}

void runESCController() {
  steeringServo.writeMicroseconds(steeringValue);
  throttleServo.writeMicroseconds(throttleValue);

   /*Serial.print("Throttle: ");
   Serial.print(throttleValue);
   Serial.print(" | ");
   Serial.print("Steering: ");
   Serial.println(steeringValue);*/
}

void runHealthCheck() {
  unsigned long currentMillis = millis();
  if (currentMillis > lastPingTime && currentMillis - lastPingTime > heartbeatTimeout) {
    Serial.printf("Heartbeat timeout, current: %lu, last: %lu, reset\n", currentMillis, lastPingTime);
    throttleValue = midThrottle; // Reset throttle and steering
    steeringValue = midSteering;
    lastPingTime = millis(); // Reset timer to avoid repeated timeouts
  }
}

// Get data from the UWB Anchor via BLE.
void getUWBAnchorData() {
  if (!UWBAnchor.connected()) {
    scanForPeripheral(UWBAnchor, UWBAnchorService, UWBAnchorSensorData, UWBAnchorServiceUUID, UWBAnchorSensorDataCharacteristicUUID);
    return;
  }

  if (!UWBAnchorSensorData.valueUpdated()) { // No available data.
    return;
  }

  UWBAnchorSensorData.readValue((uint8_t*)&sensorData, sizeof(sensorData));
  Serial.printf("Received distance=%f, heading=%f\n", sensorData.distance, sensorData.heading);
  distance = sensorData.distance;
  heading = sensorData.heading;
  //Serial.printf("Data interval: %d(ms)\n", millis() - lastPingTime);
  lastPingTime = millis();
}

void getAutocamControllerData() {
  if (!AutocamController.connected()) {
    scanForPeripheral(AutocamController, AutocamControllerService, AutocamControllerData, AutocamControllerServiceUUID, AutocamControllerControllerDataCharacteristicUUID);
    updateAutocamControllerStatus(); // Update the status after the initial connection;
    return;
  }

  if (!AutocamControllerData.valueUpdated()) { // No available data.
    return;
  }
  
  AutocamControllerData.readValue((uint8_t*)&controllerData, sizeof(controllerData));
  Serial.printf("Received throttle=%d, steering=%d, driveMode=%d\n", controllerData.throttleValue, controllerData.steeringValue, controllerData.driveMode);
  Serial.printf("Data interval: %d(ms)\n", millis() - lastPingTime);
  lastPingTime = millis();

  throttleValue = controllerData.throttleValue;
  steeringValue = controllerData.steeringValue;
  setDriveMode(controllerData.driveMode);
}

void setDriveMode(int value) {
  int prevValue = driveMode;
  driveMode = value;
  if (prevValue != value) {
    updateAutocamControllerStatus(); // Update the status only if there's a change.
  }
}

void updateAutocamControllerStatus() {
  if (!AutocamController.connected()) {
    Serial.println("Autocam controller is not connected, will not update status!");
    return;
  }

  ControllerData data = {throttleValue, steeringValue, driveMode};
  AutocamControllerData.writeValue((uint8_t*)&data, sizeof(data));
  Serial.printf("Sent status to [%s] via BLE: throttle=%d, steering=%d, driveMode=%d\n", AutocamController.deviceName(), throttleValue, steeringValue, driveMode);
  return;
}

void calculateSteeringThrottle() {
  if (driveMode != DRIVE_MODE_AUTO_FOLLOW) {
    return;
  }

  float deltaTimeMillis = millis() - lastDeltaTimeMillis;
  if (deltaTimeMillis < minDeltaTimeMillis) {
    return; // No change.
  }
  lastDeltaTimeMillis = deltaTimeMillis;

  float distanceDiff = abs(distance - targetDistance);

  if (distanceDiff <= delta) { // No distance change.
    throttleValue = midThrottle;
    steeringValue = midSteering;
    return;
  }

  // Calculate throttle.
  float throttleDiff = calculateThrottleDiff(distanceDiff, deltaTimeMillis);

  boolean moveForward = false;
  boolean moveBackward = false;
  if (currentY - targetY > delta) {
    setMoveForward(throttleDiff);
    moveForward = true;
  } else if (targetY - currentY > delta) {
    setMoveBackward(throttleDiff);
    moveBackward = true;
  } else if (currentY > 0) { // Parallel moving.
    setMoveForward(throttleDiff);
    moveForward = true;
  } else if (currentY < 0) {
    setMoveBackward(throttleDiff);
    moveBackward = true;
  }

  if (abs(currentX - targetX) <= delta) {
    steeringValue = midSteering;
    return;
  }

  // Calculate steering.
  float headingDiff = heading - targetHeading; // headingDiff < 0 means the target is turning clockwise; headingDiff > 0 means the target is turning counterclockwise.
  if (headingDiff > 180) {
    headingDiff -= 360;
  } else if (headingDiff < -180) {
    headingDiff += 360;
  }
  float steeringDiff = calculateSteeringDiff(headingDiff, deltaTimeMillis);
  if (moveForward) {
    setSteering(steeringDiff);
  } else if (moveBackward) {
    setSteering(-steeringDiff); // In reverse, we need to turn the other way around.
  }
}

// Function to calculate the new throttle diff using PID control.
float calculateThrottleDiff(float error_t, float deltaTimeMillis) {

  // Step 1: Calculate integral (accumulated error)
  integral_t += error_t * deltaTimeMillis;

  // Step 2: Calculate derivative (rate of error change)
  float derivative_t = (error_t - previousError_t) / deltaTimeMillis;

  // Step 3: Compute PID output
  float throttle = (Kp_t * error_t) + (Ki_t * integral_t) + (Kd_t * derivative_t);

  // Step 4: Constrain throttle within limits
  throttle = constrain(throttle, minMoveThrottle, maxMoveThrottle);

  // Step 5: Update previous error
  previousError_t = error_t;

  return throttle;
}

// Function to calculate the new steering diff using PID control.
float calculateSteeringDiff(float error_s, float deltaTimeMillis) {

  // Step 1: Calculate integral (accumulated error)
  integral_s += error_s * deltaTimeMillis;

  // Step 2: Calculate derivative (rate of error change)
  float derivative_s = (error_s - previousError_s) / deltaTimeMillis;

  // Step 3: Compute PID output
  float steering = (Kp_s * error_s) + (Ki_s * integral_s) + (Kd_s * derivative_s);

  // Step 4: Constrain throttle within limits
  steering = constrain(steering, minMoveSteering, maxMoveSteering);

  // Step 5: Update previous error
  previousError_s = error_s;

  return steering;
}

void calculateCoordinates() {
  // Calculate the heading angle in radians
  float headingRadians = radians(heading + 90); // +90 since we are always facing towards +Y axis.

  // Use trigonometry to calculate the coordinates
  currentX = distance * cos(headingRadians);
  currentY = distance * sin(headingRadians);

  // Serial output for debugging
  //Serial.print("Calculated coordinates (currentX, currentY): ");
  //Serial.print(currentX);
  //Serial.print(", ");
  //Serial.println(currentY)

  if (driveMode == DRIVE_MODE_MANUAL) { // Update the real time target coordinates if in manual mode.
    targetX = currentX;
    targetY = currentY;
    targetDistance = distance;
    targetHeading = heading;
    //Serial.print("Updated target coordinates (targetX, targetY): ");
    //Serial.print(targetX);
    //Serial.print(", ");
    //Serial.println(targetY);
  }

  return;
}

void setSteering(float steeringDiff) {
  steeringValue = midSteering + steeringDiff;
}

void setMoveForward(float throttleDiff) {
  throttleValue = midThrottle + throttleDiff;
}

void setMoveBackward(float throttleDiff) {
  throttleValue = midThrottle - throttleDiff;
}
