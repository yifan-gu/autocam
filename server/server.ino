#include <ESP32Servo.h>
#include <ESPAsyncWebServer.h>
#include <ESPmDNS.h>
#include <math.h>
#include <WiFi.h>

#include "html_pages.h"
#include "BLE_setup.hpp"
#include "LED_controller.hpp"

#define PI 3.14159265359

#define STATE_NOT_READY 0
#define STATE_SENSOR_READY 1
#define STATE_REMOTE_CONTROLLER_READY 2

#define DRIVE_MODE_MANUAL 0
#define DRIVE_MODE_AUTO_FOLLOW 1

// Pin configuration
#define STEERING_PIN 24  //  A7 for steering
#define THROTTLE_PIN 23  //  A6 for throttle

#define SENSOR_LED_RED_PIN 22 // A5
#define SENSOR_LED_GREEN_PIN 21 // A4
#define SENSOR_LED_BLUE_PIN -1

#define REMOTE_CONTROLLER_LED_RED_PIN 20 // A3
#define REMOTE_CONTROLLER_LED_GREEN_PIN 19 // A2
#define REMOTE_CONTROLLER_LED_BLUE_PIN -1

#define DRIVE_MODE_LED_RED_PIN 18 // A1
#define DRIVE_MODE_LED_GREEN_PIN 17 // A0
#define DRIVE_MODE_LED_BLUE_PIN -1

LEDController ledController;

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

// Heartbeat tracking
unsigned long lastPingTime = 0;           // Time of last received ping
const unsigned long heartbeatTimeout = 1000; // 1 second timeout

// Variables to store values
int throttleValue = midThrottle, steeringValue = midSteering;

// Variable for store the state.
int state = STATE_NOT_READY;

// Variable for the drive mode.
int driveMode = DRIVE_MODE_MANUAL;
// The reported distances and angle.

// Distance of the tag to the front anchors.
float distance = 0;
float targetDistance = 0;

// Heading of the anchor.
float heading = 0; // [0, 360)  N = 0/360, W = 90, S = 180, E = 270
float targetHeading = 0;

float yawSpeed = 0;
float pitchSpeed = 0;
bool activeTrackToggled = false;
uint16_t uwbSelector = 0;

bool gimbalControllerValueChanged = false;

// The calculated coordinates of the tag.
float currentX = 0, currentY = 0;

// The target coordinates to match.
float targetX = 0, targetY = 0;


//////////////////////////////////////////////////////////////////////////////////
//
// PID Controller variables for throttle. Tunable.
//
float delta = 0.5; // The "play" margin.

float Kp_t = 100.0;  // Proportional gain. Diff = Kp_t * distance
float Ki_t = 0.0;  // Integral gain. Diff = Ki_t * distance * 1000 * second.
float Kd_t = 0.0;  // Derivative gain. Diff = Kd_t / (speed m/s * 1000 ms)

float previousError_t = 0.0;  // Previous error for the derivative term
float integral_t = 0.0;       // Accumulated integral term

float maxMoveThrottle = 300;                 // Maximum throttle value in practice.
float minMoveThrottle = -maxMoveThrottle;    // Minimum throttle value in practice.

// // PID Controller variables for steering.
float Kp_s = 10.0;   // Proportional gain. Diff = Kp_s * angle diff.
float Ki_s = 0.0;  // Integral gain. Diff = Ki_s * angle diff * 1000 * second.
float Kd_s = 0.0;   // Derivative gain. Diff = Kd_s / (anglur speed * 1000 ms)

float previousError_s = 0.0;  // Previous error for the derivative term
float integral_s = 0.0;       // Accumulated integral term

float maxMoveSteering = 500;              // Maximum steering value in practice.
float minMoveSteering = -maxMoveSteering; // Minimum steering value in practice.

float lastDeltaTimeMillis = 0; // Used to calculate ki, and Kd in the PID system.
float minDeltaTimeMillis = 10; // Control the rate of PID update.
//
//
//////////////////////////////////////////////////////////////////////////////////

#define SENSOR_STATE_NOT_READY 0
#define SENSOR_STATE_READY 3 // SENSOR_STATE_READY means tag and server connected to the sensor.

void setup() {
  // Start Serial for debugging
  unsigned long startTime = millis();
  unsigned long timeout = 1000; // 1 seconds timeout

  Serial.begin(115200);
  while (!Serial && (millis() - startTime < timeout)) {
    // Wait for Serial or timeout
  }

  setupLED();
  setupESC();
  setupServer();
  setupBLECentral();
  establishUWBAnchorBLEConnection();
  establishAutocamControllerBLEConnection();
}

void loop() {
  getUWBAnchorData();
  getAutocamControllerData();
  sendGimbalControllerData();
  calculateCoordinates();
  calculateSteeringThrottle();
  runESCController();
  runHealthCheck();
}

void setupLED() {
  ledController.initSensorLED(SENSOR_LED_RED_PIN, SENSOR_LED_GREEN_PIN, SENSOR_LED_BLUE_PIN);
  ledController.initRemoteLED(REMOTE_CONTROLLER_LED_RED_PIN, REMOTE_CONTROLLER_LED_GREEN_PIN, REMOTE_CONTROLLER_LED_BLUE_PIN);
  ledController.initDriveLED(DRIVE_MODE_LED_RED_PIN, DRIVE_MODE_LED_GREEN_PIN, DRIVE_MODE_LED_BLUE_PIN);

  ledController.updateStateLED(state);
  ledController.updateDriveModeLED(driveMode);
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

  // Add a new handler for serving the parameters page
  server.on("/parameters", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(200, "text/html", parameters_page_html);
  });

  server.on("/get_parameters", HTTP_GET, [](AsyncWebServerRequest *request) {
    String json = "{";
    json += "\"delta\":" + String(delta, 2) + ",";
    json += "\"Kp_t\":" + String(Kp_t, 1) + ",";
    json += "\"Ki_t\":" + String(Ki_t, 1) + ",";
    json += "\"Kd_t\":" + String(Kd_t, 1) + ",";
    json += "\"Kp_s\":" + String(Kp_s, 1) + ",";
    json += "\"Ki_s\":" + String(Ki_s, 1) + ",";
    json += "\"Kd_s\":" + String(Kd_s, 1) + ",";
    json += "\"maxMoveThrottle\":" + String(maxMoveThrottle, 0) + ",";
    json += "\"minMoveThrottle\":" + String(minMoveThrottle, 0) + ",";
    json += "\"maxMoveSteering\":" + String(maxMoveSteering, 0) + ",";
    json += "\"minMoveSteering\":" + String(minMoveSteering, 0);
    json += "}";
    request->send(200, "application/json", json);
  });

  // Add a handler to process the parameters update
  server.on("/update_parameters", HTTP_POST, [](AsyncWebServerRequest *request) {
    if (request->hasParam("delta", true)) {
      delta = request->getParam("delta", true)->value().toFloat();
      Serial.printf("delta: %f\n", delta);
    }
    if (request->hasParam("Kp_t", true)) {
      Kp_t = request->getParam("Kp_t", true)->value().toFloat();
      Serial.printf("Kp_t: %f\n", Kp_t);
    }
    if (request->hasParam("Ki_t", true)) {
      Ki_t = request->getParam("Ki_t", true)->value().toFloat();
      Serial.printf("Ki_t: %f\n", Ki_t);
    }
    if (request->hasParam("Kd_t", true)) {
      Kd_t = request->getParam("Kd_t", true)->value().toFloat();
      Serial.printf("Kd_t: %f\n", Kd_t);
    }
    if (request->hasParam("Kp_s", true)) {
      Kp_s = request->getParam("Kp_s", true)->value().toFloat();
      Serial.printf("Kp_s: %f\n", Kp_s);
    }
    if (request->hasParam("Ki_s", true)) {
      Ki_s = request->getParam("Ki_s", true)->value().toFloat();
      Serial.printf("Ki_s: %f\n", Ki_s);
    }
    if (request->hasParam("Kd_s", true)) {
      Kd_s = request->getParam("Kd_s", true)->value().toFloat();
      Serial.printf("Kd_s: %f\n", Kd_s);
    }
    if (request->hasParam("maxMoveThrottle", true)) {
      maxMoveThrottle = request->getParam("maxMoveThrottle", true)->value().toFloat();
      Serial.printf("maxMoveThrottle: %f\n", maxMoveThrottle);
    }
    if (request->hasParam("minMoveThrottle", true)) {
      minMoveThrottle = request->getParam("minMoveThrottle", true)->value().toFloat();
      Serial.printf("minMoveThrottle: %f\n", minMoveThrottle);
    }
    if (request->hasParam("maxMoveSteering", true)) {
      maxMoveSteering = request->getParam("maxMoveSteering", true)->value().toFloat();
      Serial.printf("maxMoveSteering: %f\n", maxMoveSteering);
    }
    if (request->hasParam("minMoveSteering", true)) {
      minMoveSteering = request->getParam("minMoveSteering", true)->value().toFloat();
      Serial.printf("minMoveSteering: %f\n", minMoveSteering);
    }

    request->send(200, "text/plain", "Parameters updated");
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
        response += "\"targetY\":" + String(targetY, 2) + ",";
        response += "\"state\":" + String(state);
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
  if (!UWBAnchor.connected()) {
    Serial.println("UWBAnchor not connected, will reconnect...");
    emergencyStop();
    updateState(state & ~STATE_SENSOR_READY); // Clear the sensor ready indicator bit.
    establishUWBAnchorBLEConnection();
  }

  if (!AutocamController.connected()) {
    Serial.println("AutocamController not connected, will reconnect...");
    emergencyStop();
    sendGimbalControllerData();
    updateState(state & ~STATE_REMOTE_CONTROLLER_READY); // Clear the remote controller ready indicator bit.
    establishAutocamControllerBLEConnection();
  }

  unsigned long currentMillis = millis();
  if (currentMillis > lastPingTime && currentMillis - lastPingTime > heartbeatTimeout) {
    Serial.printf("Heartbeat timeout, current: %lu, last: %lu, reset\n", currentMillis, lastPingTime);
    emergencyStop();
    lastPingTime = millis(); // Reset timer to avoid repeated timeouts
  }
}

void establishAutocamControllerBLEConnection() {
  while (!scanForPeripheral(AutocamController, AutocamControllerService, AutocamControllerData, AutocamControllerServiceUUID, AutocamControllerControllerDataCharacteristicUUID)) {
    delay(1000);
  }
  updateState(state | STATE_REMOTE_CONTROLLER_READY);
}

void establishUWBAnchorBLEConnection() {
  while (!scanForPeripheral(UWBAnchor, UWBAnchorService, UWBAnchorSensorDataSend, UWBAnchorServiceUUID, UWBAnchorSensorDataSendCharacteristicUUID)) {
    delay(1000);
  };
  while (!scanForPeripheral(UWBAnchor, UWBAnchorService, UWBAnchorSensorDataRecv, UWBAnchorServiceUUID, UWBAnchorSensorDataRecvCharacteristicUUID)) {
    delay(1000);
  };
}

void sendGimbalControllerData() {
  if (!gimbalControllerValueChanged) {
    return;
  }

  if (!UWBAnchor.connected()) {
    Serial.println("UWB Anchor is not connected, will not update gimbal");
    return;
  }

  SensorDataRecv data = {.yawSpeed = yawSpeed, .pitchSpeed = pitchSpeed, .activeTrackToggled = activeTrackToggled, .uwbSelector = uwbSelector};
  UWBAnchorSensorDataRecv.writeValue((uint8_t *)&data, sizeof(SensorDataRecv));
  activeTrackToggled = false; // Reset active track toggle after triggering it.
  gimbalControllerValueChanged = false;

  Serial.printf("Sent via BLE: yawSpeed=%f, pitchSpeed=%f, activeTrackToggled=%d, uwbSelector=%d\n", data.yawSpeed, data.pitchSpeed, data.activeTrackToggled, data.uwbSelector);
  //Serial.printf("Data interval: %d(ms)\n", millis() - lastPingTime);
}

// Get data from the UWB Anchor via BLE.
void getUWBAnchorData() {
  if (!UWBAnchor.connected()) {
    if (driveMode != DRIVE_MODE_MANUAL) {
      emergencyStop();
    }

    updateState(state & ~STATE_SENSOR_READY); // Clear the sensor ready indicator bit.
    return;
  }

  if (!UWBAnchorSensorDataSend.valueUpdated()) { // No available data.
    return;
  }

  if (UWBAnchorSensorDataSend.valueLength() != sizeof(SensorDataSend)) { // Appearantly if read too fast, the valueLength() is invalid.
    return;
  }

  SensorDataSend data;

  UWBAnchorSensorDataSend.readValue((uint8_t *)&data, sizeof(SensorDataSend));
  Serial.printf("Received distance=%f, heading=%f, state=%d\n", data.distance, data.heading, data.state);
  distance = data.distance;
  heading = data.heading;
  if (data.state != SENSOR_STATE_READY) {
    if (driveMode != DRIVE_MODE_MANUAL) {
      emergencyStop();
    }
    updateState(state & ~STATE_SENSOR_READY);
  } else {
    updateState(state | STATE_SENSOR_READY);
  }

  //Serial.printf("Data interval: %d(ms)\n", millis() - lastPingTime);
  lastPingTime = millis();
}

void getAutocamControllerData() {
  if (!AutocamController.connected()) {
    emergencyStop();
    updateState(state & ~STATE_REMOTE_CONTROLLER_READY); // Clear the remote controller ready indicator bit.
    return;
  }

  if (!AutocamControllerData.valueUpdated()) { // No available data.
    return;
  }

  if (AutocamControllerData.valueLength() != sizeof(ControllerData)) { // Appearantly if read too fast, the valueLength() is invalid.
    return;
  }

  ControllerData data;
  AutocamControllerData.readValue((uint8_t *)&data, sizeof(ControllerData));
  Serial.printf("Received throttle=%d, steering=%d, driveMode=%d, yawSpeed=%f, pitchSpeed=%f, activeTrackToggled=%d, uwbSelector=%d\n", data.throttleValue, data.steeringValue, data.driveMode, data.yawSpeed, data.pitchSpeed, data.activeTrackToggled, data.uwbSelector);
  //Serial.printf("Data interval: %d(ms)\n", millis() - lastPingTime);
  lastPingTime = millis();

  gimbalControllerValueChanged = yawSpeed != data.yawSpeed || pitchSpeed != data.pitchSpeed || activeTrackToggled != data.activeTrackToggled || uwbSelector != data.uwbSelector;

  throttleValue = data.throttleValue;
  steeringValue = data.steeringValue;
  yawSpeed = data.yawSpeed;
  pitchSpeed = data.pitchSpeed;
  activeTrackToggled = data.activeTrackToggled;

  setDriveMode(data.driveMode);
  setUWBSelector(data.uwbSelector);
}

void updateState(int newState) {
  if (state == newState) {
    return;
  }
  state = newState;
  ledController.updateStateLED(state);
  updateAutocamControllerStatus();
}
void updateAutocamControllerStatus() {
  if (!AutocamController.connected()) {
    Serial.println("Autocam controller is not connected, will not update status!");
    return;
  }

  ControllerData data = {.driveMode = driveMode, .activeTrackToggled = activeTrackToggled, .uwbSelector = uwbSelector, .state = state};
  AutocamControllerData.writeValue((uint8_t *)&data, sizeof(ControllerData));
  Serial.printf("Sent status to remote via BLE: driveMode = %d, state=%d\n", driveMode, state);
  return;
}

void setDriveMode(int newDriveMode) {
  if (driveMode == newDriveMode) {
    return;
  }
  driveMode = newDriveMode;
  ledController.updateDriveModeLED(driveMode);
  updateAutocamControllerStatus();
}

void setUWBSelector(uint16_t newUWBSelector) {
  if (uwbSelector == newUWBSelector) {
    return;
  }
  uwbSelector = newUWBSelector;
  updateAutocamControllerStatus();
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
  steeringValue = midSteering + (int)steeringDiff;
}

void setMoveForward(float throttleDiff) {
  throttleValue = midThrottle + (int)throttleDiff;
}

void setMoveBackward(float throttleDiff) {
  throttleValue = midThrottle - (int)throttleDiff;
}

void emergencyStop() { //TODO(yifan): Refactor out this with a state machine.
  setDriveMode(DRIVE_MODE_MANUAL);
  distance = 0;
  heading = 0;
  yawSpeed = 0;
  pitchSpeed = 0;
  throttleValue = midThrottle;
  steeringValue = midSteering;
  gimbalControllerValueChanged = true; // Force gimbal to stop.
}
