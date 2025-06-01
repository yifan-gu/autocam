#include <ESP32Servo.h>
#include <ESPAsyncWebServer.h>
#include <ESPmDNS.h>
#include <math.h>
#include <WiFi.h>

#include "html_pages.h"
#include "BLE_setup.hpp"
#include "LED_controller.hpp"
#include "util.h"

// Pin configuration
#define STEERING_PIN D8  //  D8 for steering
#define THROTTLE_PIN D7  //  D7 for throttle

#define SENSOR_LED_RED_PIN D6 // D6
#define SENSOR_LED_GREEN_PIN D5 // D5
#define SENSOR_LED_BLUE_PIN -1

#define REMOTE_LED_RED_PIN D4 // D4
#define REMOTE_LED_GREEN_PIN D3 // D3
#define REMOTE_LED_BLUE_PIN -1

#define DRIVE_MODE_LED_RED_PIN -1
#define DRIVE_MODE_LED_GREEN_PIN -1
#define DRIVE_MODE_LED_BLUE_PIN D2 // D2

LEDController ledController;

// Access Point credentials
const char* ssid = "Autocam Server";
const char* password = "AbsoluteCinema!"; // Minimum 8 characters
const char* hostname = "autocam"; // mDNS hostname

// Async WebSocket server
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

// Servo objects
Servo throttleServo;
Servo steeringServo;

// Define throttle and steering ranges
const int minThrottle = 1000, maxThrottle = 2000, midThrottle = 1500;
const int minSteering = 1000, maxSteering = 2000, midSteering = 1500; // 1000 = max right turn, 2000 = max left turn.

const float fieldOfViewRatio = 28 / 20; // Car length / car width (measured from the wheel).

int minMoveThrottle = 1300, maxMoveThrottle = 1700;
int minMoveSteering = 1000, maxMoveSteering = 2000;

// Heartbeat tracking
unsigned long lastPingTime = 0;           // Time of last received ping
const unsigned long heartbeatTimeout = 1000; // 1 second timeout


struct GlobalState {
  int throttleValue;       // e.g., servo pulse width for throttle
  int steeringValue;       // e.g., servo pulse width for steering
  int state;               // overall state (e.g., SERVER_STATE_NOT_READY, etc.)
  int driveMode;           // e.g., DRIVE_MODE_MANUAL, DRIVE_MODE_FOLLOW, DRIVE_MODE_CINEMA
  float distance;          // distance from the Tag/Remote to the Sensor
  float targetDistance;    // desired distance
  float heading;           // measured heading in degrees [0, 360)  Front = 0/360, Left = 90, Back = 180, Right = 270
  float targetHeading;     // desired heading in degrees
  float yawSpeed;          // Yaw speed
  float pitchSpeed;        // Pitch speed
  int toggleState;         // Flag to store all the toggle button value (activeTrack, gimbalRecenter, cameraRecording)
  uint16_t uwbSelector;    // UWB selector value
  float currentX;          // The tag's current coordinates.
  float currentY;          // The tag's current coordinates.
  float targetX;           // The tag's target coordinates.
  float targetY;           // The tag's target coordinates.
};

GlobalState globalState = {
  midThrottle,           // throttleValue
  midSteering,           // steeringValue
  SERVER_STATE_NOT_READY,       // state
  DRIVE_MODE_MANUAL,     // driveMode
  0.0,                   // distance
  0.0,                   // targetDistance
  0.0,                   // heading
  0.0,                   // targetHeading
  0.0,                   // yawSpeed
  0.0,                   // pitchSpeed
  0,                     // toggleState
  1,                     // uwbSelector
  0.0,                   // currentX
  0.0,                   // currentY
  0.0,                   // targetX
  0.0,                   // targetY
};

void printGlobalState() {
  LOGF("Throttle:%d, Steering:%d, State:%d, Drive Mode:%d, Dist:%.2f, TargetDist:%.2f, Head:%.2f, TargetHead:%.2f, Yaw:%.2f, Pitch:%.2f, toggleState:%d, UWB Selector:%d, currentX: %.2f, currentY: %.2f, targetX: %.2f, targetY: %.2f\n",
    globalState.throttleValue,
    globalState.steeringValue,
    globalState.state,
    globalState.driveMode,
    globalState.distance,
    globalState.targetDistance,
    globalState.heading,
    globalState.targetHeading,
    globalState.yawSpeed,
    globalState.pitchSpeed,
    globalState.toggleState,
    globalState.uwbSelector,
    globalState.currentX,
    globalState.currentY,
    globalState.targetX,
    globalState.targetY
  );
}



bool autocamRemoteInputChanged = false;

//////////////////////////////////////////////////////////////////////////////////
//
// PID Controller variables for throttle. Tunable.
//
float distanceDelta = 0.05; // The distance tolerance in meters.
float headingDelta = 1; // The heading tolerance in degrees.
float throttleConstant = 100;
float steeringConstant = 20;

float Kp_t = 1.0;  // Proportional gain. Diff = Kp_t * distance
float Ki_t = 0.0;  // Integral gain. Diff = Ki_t * distance * 1000 * second.
float Kd_t = 0.0;  // Derivative gain. Diff = Kd_t / (speed m/s * 1000 ms)

float previousError_t = 0.0;  // Previous error for the derivative term
float integral_t = 0.0;       // Accumulated integral term

// // PID Controller variables for steering.
float Kp_s = 1.0;   // Proportional gain. Diff = Kp_s * angle diff.
float Ki_s = 0.0;  // Integral gain. Diff = Ki_s * angle diff * 1000 * second.
float Kd_s = 0.0;   // Derivative gain. Diff = Kd_s / (anglur speed * 1000 ms)

float previousError_s = 0.0;  // Previous error for the derivative term
float integral_s = 0.0;       // Accumulated integral term

float lastDeltaTimeMillis = 0; // Used to calculate ki, and Kd in the PID system.
float minDeltaTimeMillis = 10; // Control the rate of PID update.
float distanceSmoothFactor = 0.1; // Smoothing factor for distance readings (0 < alpha <= 1; lower values are smoother).

float leadingTurningCoefficient = 10;
float tailingTurningCoefficient = 1;

//
//
//////////////////////////////////////////////////////////////////////////////////

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
  establishAutocamSensorBLEConnection();
  establishAutocamRemoteBLEConnection();
}

void loop() {
  // Initialize the periodic timing. These static variables keep their value between iterations.
  static TickType_t xLastWakeTime = xTaskGetTickCount();
  static TickType_t previousIteration = xLastWakeTime;

  // Calculate and print the interval between iterations.
  TickType_t currentIteration = xTaskGetTickCount();
  uint32_t intervalMs = (currentIteration - previousIteration) * portTICK_PERIOD_MS;
  previousIteration = currentIteration;
  //LOGF("Interval since last iteration: %u ms\n", intervalMs);

  getAutocamSensorData();
  getAutocamRemoteData();
  //printGlobalState();
  sendGimbalControllerData();
  calculateCoordinates();
  calculateSteeringThrottle();
  runESCController();
  checkDriveModeLED();
  runHealthCheck();

  // Delay until the next period; this call ensures a steady 10 ms loop period.
  vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(LOOP_PERIOD_MS));
}

void setupLED() {
  ledController.initSensorLED(SENSOR_LED_RED_PIN, SENSOR_LED_GREEN_PIN, SENSOR_LED_BLUE_PIN);
  ledController.initRemoteLED(REMOTE_LED_RED_PIN, REMOTE_LED_GREEN_PIN, REMOTE_LED_BLUE_PIN);
  ledController.initDriveLED(DRIVE_MODE_LED_RED_PIN, DRIVE_MODE_LED_GREEN_PIN, DRIVE_MODE_LED_BLUE_PIN);

  ledController.updateStateLED(globalState.state);
  ledController.updateDriveModeLED(globalState.driveMode);
}

void setupESC() {
  LOGLN("Initializing ESC...");
  delay(1000); // Wait 1 seconds to ensure the ESC ready to arm.

  // Attach servos
  throttleServo.attach(THROTTLE_PIN, minThrottle, maxThrottle);
  steeringServo.attach(STEERING_PIN, minSteering, maxSteering);

  // Initialize to neutral positions to arm the ESC.
  throttleServo.writeMicroseconds(midThrottle);
  steeringServo.writeMicroseconds(midSteering);

  LOGLN("ESC Initialized!");
}

// WiFi, WebServer, and WebSocket setup
void setupServer() {
  // Start Access Point
  WiFi.softAP(ssid, password);
  LOGLN("Access Point started!");
  LOG("SSID: ");
  LOGLN(ssid);
  LOG("IP Address: ");
  LOGLN(WiFi.softAPIP());

  // Start mDNS
  if (!MDNS.begin(hostname)) {
    LOGLN("Error starting mDNS");
    return;
  }
  LOGLN("mDNS responder started!");
  LOG("Access the device at http://");
  LOG(hostname);
  LOGLN(".local");

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
    json += "\"distanceDelta\":" + String(distanceDelta, 2) + ",";
    json += "\"headingDelta\":" + String(headingDelta, 0) + ",";
    json += "\"Kp_t\":" + String(Kp_t, 2) + ",";
    json += "\"Ki_t\":" + String(Ki_t, 3) + ",";
    json += "\"Kd_t\":" + String(Kd_t, 2) + ",";
    json += "\"Kp_s\":" + String(Kp_s, 2) + ",";
    json += "\"Ki_s\":" + String(Ki_s, 3) + ",";
    json += "\"Kd_s\":" + String(Kd_s, 2) + ",";
    json += "\"maxMoveThrottle\":" + String(maxMoveThrottle - midThrottle) + ",";
    json += "\"minMoveThrottle\":" + String(minMoveThrottle - midThrottle) + ",";
    json += "\"maxMoveSteering\":" + String(maxMoveSteering - midSteering) + ",";
    json += "\"minMoveSteering\":" + String(minMoveSteering - midSteering) + ",";
    json += "\"distanceSmoothFactor\":" + String(distanceSmoothFactor, 2);
    json += "}";
    request->send(200, "application/json", json);
    LOGF("get_parameters: %s\n", json.c_str());
  });

  // Add a handler to process the parameters update
  server.on("/update_parameters", HTTP_POST, [](AsyncWebServerRequest *request) {
    if (request->hasParam("distanceDelta", true)) {
      distanceDelta = constrain(request->getParam("distanceDelta", true)->value().toFloat(), 0, 1);
      LOGF("distanceDelta: %f\n", distanceDelta);
    }
    if (request->hasParam("headingDelta", true)) {
      headingDelta = constrain(request->getParam("headingDelta", true)->value().toFloat(), 0, 30);
      LOGF("headingDelta: %f\n", headingDelta);
    }
    if (request->hasParam("Kp_t", true)) {
      Kp_t = constrain(request->getParam("Kp_t", true)->value().toFloat(), 0, 10);
      LOGF("Kp_t: %f\n", Kp_t);
    }
    if (request->hasParam("Ki_t", true)) {
      Ki_t = constrain(request->getParam("Ki_t", true)->value().toFloat(), 0, 0.1);
      LOGF("Ki_t: %f\n", Ki_t);
    }
    if (request->hasParam("Kd_t", true)) {
      Kd_t = constrain(request->getParam("Kd_t", true)->value().toFloat(), 0, 10);
      LOGF("Kd_t: %f\n", Kd_t);
    }
    if (request->hasParam("Kp_s", true)) {
      Kp_s = constrain(request->getParam("Kp_s", true)->value().toFloat(), 0, 10);
      LOGF("Kp_s: %f\n", Kp_s);
    }
    if (request->hasParam("Ki_s", true)) {
      Ki_s = constrain(request->getParam("Ki_s", true)->value().toFloat(), 0, 0.1);
      LOGF("Ki_s: %f\n", Ki_s);
    }
    if (request->hasParam("Kd_s", true)) {
      Kd_s = constrain(request->getParam("Kd_s", true)->value().toFloat(), 0, 10);
      LOGF("Kd_s: %f\n", Kd_s);
    }
    if (request->hasParam("maxMoveThrottle", true)) {
      maxMoveThrottle = midThrottle + constrain(request->getParam("maxMoveThrottle", true)->value().toFloat(), 0, 500);
      LOGF("maxMoveThrottle: %f\n", maxMoveThrottle);
    }
    if (request->hasParam("minMoveThrottle", true)) {
      minMoveThrottle = midThrottle + constrain(request->getParam("minMoveThrottle", true)->value().toFloat(), -500, 0);
      LOGF("minMoveThrottle: %f\n", minMoveThrottle);
    }
    if (request->hasParam("maxMoveSteering", true)) {
      maxMoveSteering = midSteering + constrain(request->getParam("maxMoveSteering", true)->value().toFloat(), 0, 500);
      LOGF("maxMoveSteering: %f\n", maxMoveSteering);
    }
    if (request->hasParam("minMoveSteering", true)) {
      minMoveSteering = midSteering + constrain(request->getParam("minMoveSteering", true)->value().toFloat(), -500, 0);
      LOGF("minMoveSteering: %f\n", minMoveSteering);
    }
    if (request->hasParam("distanceSmoothFactor", true)) {
      distanceSmoothFactor = constrain(request->getParam("distanceSmoothFactor", true)->value().toFloat(), 0.01, 1);
      LOGF("distanceSmoothFactor: %f\n", distanceSmoothFactor);
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
      LOGLN("WebSocket connected");
      break;

    case WS_EVT_DISCONNECT:
      LOGLN("WebSocket disconnected");
      setDriveMode(DRIVE_MODE_MANUAL);
      globalState.throttleValue = midThrottle;
      globalState.steeringValue = midSteering;
      LOGLN("Throttle and Steering reset to middle positions");
      break;

    case WS_EVT_DATA: {
      String message = String((char*)data).substring(0, len);
      lastPingTime = millis();

      if (message == "REQUEST_DATA") {
        // Respond with the current state as a JSON string
        String response = "{";
        response += "\"throttle\":" + String(globalState.throttleValue) + ",";
        response += "\"steering\":" + String(globalState.steeringValue) + ",";
        response += "\"distance\":" + String(globalState.distance, 2) + ",";
        response += "\"heading\":" + String(globalState.heading, 2) + ",";
        response += "\"currentX\":" + String(globalState.currentX, 2) + ",";
        response += "\"currentY\":" + String(globalState.currentY, 2 ) + ",";
        response += "\"targetX\":" + String(globalState.targetX, 2) + ",";
        response += "\"targetY\":" + String(globalState.targetY, 2) + ",";
        response += "\"state\":" + String(globalState.state) + ",";
        response += "\"driveMode\":" + String(globalState.driveMode);
        response += "}";

        client->text(response); // Send the JSON response to the client
        //LOGLN("Sent data: " + response);
        break;
      }

      if (message == "driveMode=0") {
        setDriveMode(DRIVE_MODE_MANUAL);
        break;
      }
      if (message == "driveMode=1") {
        setDriveMode(DRIVE_MODE_FOLLOW);
        break;
      }
      if (message == "driveMode=2") {
        setDriveMode(DRIVE_MODE_CINEMA);
        break;
      }
      if (message.startsWith("throttle=")) {
        globalState.throttleValue = map(message.substring(9).toInt(), 1000, 2000, minMoveThrottle, maxMoveThrottle);
        break;
      }
      if (message.startsWith("steering=")) {
        globalState.steeringValue = map(message.substring(9).toInt(), 1000, 2000, minMoveSteering, maxMoveSteering);
        break;
      }
      LOGF("Unknown Websocket message: %s\n", message);
      break;
    }

    case WS_EVT_ERROR:
      LOGLN("WebSocket error");
      break;

    default:
      LOGLN("Unknown WebSocket event");
      break;
  }
}

void runESCController() {
  steeringServo.writeMicroseconds(globalState.steeringValue);
  throttleServo.writeMicroseconds(globalState.throttleValue);

   /*LOG("Throttle: ");
   LOG(globalState.throttleValue);
   LOG(" | ");
   LOG("Steering: ");
   LOGLN(globalState.steeringValue);*/
}

void runHealthCheck() {
  if (!AutocamSensor.connected()) {
    LOGLN("AutocamSensor not connected, will reconnect...");
    emergencyStop();
    updateState(globalState.state & ~SERVER_STATE_SENSOR_READY); // Clear the sensor ready indicator bit.
    establishAutocamSensorBLEConnection();
  }

  if (!AutocamRemote.connected()) {
    LOGLN("AutocamRemote not connected, will reconnect...");
    emergencyStop();
    sendGimbalControllerData();
    updateState(globalState.state & ~SERVER_STATE_REMOTE_READY); // Clear the remote controller ready indicator bit.
    establishAutocamRemoteBLEConnection();
  }

  unsigned long currentMillis = millis();
  if (currentMillis > lastPingTime && currentMillis - lastPingTime > heartbeatTimeout) {
    LOGF("Heartbeat timeout, current: %lu, last: %lu, reset\n", currentMillis, lastPingTime);
    emergencyStop();
    lastPingTime = millis(); // Reset timer to avoid repeated timeouts
  }
}

void establishAutocamRemoteBLEConnection() {
  while (!scanForPeripheral(AutocamRemote, AutocamRemoteService, AutocamRemoteDataSend, AutocamRemoteServiceUUID, AutocamRemoteDataSendCharacteristicUUID)) {
    delay(1000);
  }
  while (!scanForPeripheral(AutocamRemote, AutocamRemoteService, AutocamRemoteDataRecv, AutocamRemoteServiceUUID, AutocamRemoteDataRecvCharacteristicUUID)) {
    delay(1000);
  }
  updateState(globalState.state | SERVER_STATE_REMOTE_READY);
}

void establishAutocamSensorBLEConnection() {
  while (!scanForPeripheral(AutocamSensor, AutocamSensorService, AutocamSensorDataSend, AutocamSensorServiceUUID, AutocamSensorDataSendCharacteristicUUID)) {
    delay(1000);
  };
  while (!scanForPeripheral(AutocamSensor, AutocamSensorService, AutocamSensorDataRecv, AutocamSensorServiceUUID, AutocamSensorDataRecvCharacteristicUUID)) {
    delay(1000);
  };
}

void sendGimbalControllerData() {
  if (!autocamRemoteInputChanged) {
    return;
  }

  if (!AutocamSensor.connected()) {
    LOGLN("UWB Anchor is not connected, will not update gimbal");
    return;
  }

  SensorDataRecv data = {.yawSpeed = globalState.yawSpeed, .pitchSpeed = globalState.pitchSpeed, .toggleState = globalState.toggleState, .uwbSelector = globalState.uwbSelector};
  AutocamSensorDataRecv.writeValue((uint8_t *)&data, sizeof(SensorDataRecv));
  globalState.toggleState = 0; // Reset toggle state after triggering it.
  autocamRemoteInputChanged = false;

  //LOGF("Sent via BLE: yawSpeed=%f, pitchSpeed=%f, toggleState=%d, uwbSelector=%d\n", data.yawSpeed, data.pitchSpeed, data.toggleState, data.uwbSelector);
  //LOGF("Data interval: %d(ms)\n", millis() - lastPingTime);
}

// Get data from the UWB Anchor via BLE.
void getAutocamSensorData() {
  if (!AutocamSensor.connected()) {
    if (globalState.driveMode != DRIVE_MODE_MANUAL) {
      emergencyStop();
    }

    updateState(globalState.state & ~SERVER_STATE_SENSOR_READY); // Clear the sensor ready indicator bit.
    return;
  }

  if (!AutocamSensorDataSend.valueUpdated()) { // No available data.
    return;
  }

  if (AutocamSensorDataSend.valueLength() != sizeof(SensorDataSend)) { // Appearantly if read too fast, the valueLength() is invalid.
    return;
  }

  SensorDataSend data;

  AutocamSensorDataSend.readValue((uint8_t *)&data, sizeof(SensorDataSend));
  //LOGF("Received distance=%f, heading=%f, state=%d\n", data.distance, data.heading, data.state);

  // Smooth the distance reading using an exponential moving average.
  // New smoothed distance = alpha * (new reading) + (1 - alpha) * (previous smoothed value)
  globalState.distance = distanceSmoothFactor * data.distance +
                         (1.0 - distanceSmoothFactor) * globalState.distance;

  globalState.heading = data.heading;
  if (data.state & SENSOR_STATE_TAG_CONNECTED && data.state & SENSOR_STATE_SERVER_CONNECTED) {
    updateState(globalState.state | SERVER_STATE_SENSOR_READY);
  } else {
    if (globalState.driveMode != DRIVE_MODE_MANUAL) {
      emergencyStop();
    }
    updateState(globalState.state & ~SERVER_STATE_SENSOR_READY);
  }

  //LOGF("Data interval: %d(ms)\n", millis() - lastPingTime);
  lastPingTime = millis();
}

void getAutocamRemoteData() {
  if (!AutocamRemote.connected()) {
    emergencyStop();
    updateState(globalState.state & ~SERVER_STATE_REMOTE_READY); // Clear the remote controller ready indicator bit.
    return;
  }

  if (!AutocamRemoteDataSend.valueUpdated()) { // No available data.
    return;
  }

  if (AutocamRemoteDataSend.valueLength() != sizeof(RemoteDataSend)) { // Appearantly if read too fast, the valueLength() is invalid.
    return;
  }

  RemoteDataSend data;
  AutocamRemoteDataSend.readValue((uint8_t *)&data, sizeof(RemoteDataSend));
  //LOGF("Received throttle=%d, steering=%d, driveMode=%d, yawSpeed=%f, pitchSpeed=%f, toggleState=%d, uwbSelector=%d\n", data.throttleValue, data.steeringValue, data.driveMode, data.yawSpeed, data.pitchSpeed, data.toggleState, data.uwbSelector);
  //LOGF("Data interval: %d(ms)\n", millis() - lastPingTime);

  lastPingTime = millis();

  autocamRemoteInputChanged = globalState.yawSpeed != data.yawSpeed || globalState.pitchSpeed != data.pitchSpeed || globalState.toggleState != data.toggleState || globalState.uwbSelector != data.uwbSelector;

  globalState.throttleValue = constrain(data.throttleValue, minMoveThrottle, maxMoveThrottle);
  globalState.steeringValue = constrain(data.steeringValue, minMoveSteering, maxMoveSteering);
  globalState.yawSpeed = data.yawSpeed;
  globalState.pitchSpeed = data.pitchSpeed;
  globalState.toggleState = data.toggleState;

  setDriveMode(data.driveMode);
  setUWBSelector(data.uwbSelector);
}

void updateState(int newState) {
  if (globalState.state == newState) {
    return;
  }
  globalState.state = newState;
  ledController.updateStateLED(globalState.state);
  updateAutocamRemoteStatus();
}
void updateAutocamRemoteStatus() {
  if (!AutocamRemote.connected()) {
    LOGLN("Autocam controller is not connected, will not update status!");
    return;
  }

  RemoteDataRecv data = {.driveMode = globalState.driveMode, .toggleState = globalState.toggleState, .uwbSelector = globalState.uwbSelector, .state = globalState.state};
  AutocamRemoteDataRecv.writeValue((uint8_t *)&data, sizeof(RemoteDataRecv));
  LOGF("Sent status to remote via BLE: driveMode = %d, state=%d\n", globalState.driveMode, globalState.state);
  return;
}

void setDriveMode(int newDriveMode) {
  if (globalState.driveMode == newDriveMode) {
    return;
  }
  globalState.driveMode = newDriveMode;
  ledController.updateDriveModeLED(globalState.driveMode);
  updateAutocamRemoteStatus();
}

void setUWBSelector(uint16_t newUWBSelector) {
  if (globalState.uwbSelector == newUWBSelector) {
    return;
  }

  globalState.uwbSelector = newUWBSelector;
  if (globalState.driveMode != DRIVE_MODE_MANUAL) { // Disable auto pilot when switching the target uwb tag.
    setDriveMode(DRIVE_MODE_MANUAL);
  } else {
    updateAutocamRemoteStatus();
  }
}

void calculateSteeringThrottle() {
  switch (globalState.driveMode) {
    case DRIVE_MODE_FOLLOW:
      calculateSteeringThrottleFollow();
      break;
    case DRIVE_MODE_CINEMA:
      calculateSteeringThrottleCinema();
      break;
    default: // DRIVE_MODE_MANUAL
      return;
  }
  return;
}

void calculateSteeringThrottleFollow() {
  float deltaTimeMillis = millis() - lastDeltaTimeMillis;
  if (deltaTimeMillis < minDeltaTimeMillis) {
    return; // No change.
  }
  lastDeltaTimeMillis = deltaTimeMillis;

  float distanceDiff = globalState.distance - globalState.targetDistance;
  if (distanceDiff <= distanceDelta) { // Stay still if the target is moving closer.
    globalState.throttleValue = midThrottle;
    globalState.steeringValue = midSteering;
    return;
  }

  // Calculate throttle.
  float distanceDiffError = 0;
  if (distanceDiff > 0) {
    distanceDiffError = distanceDiff - distanceDelta; // Compensate the tolerance.
  } else {
    distanceDiffError = distanceDiff + distanceDelta;
  }
  float throttleCoeff = calculateThrottleCoeff(distanceDiffError, deltaTimeMillis);

  boolean moveForward = false;
  boolean moveBackward = false;
  if (globalState.currentY > distanceDelta) {
    setMoveForward(throttleCoeff);
    moveForward = true;
  } else if (globalState.currentY < -distanceDelta) {
    setMoveBackward(throttleCoeff);
    moveBackward = true;
  }

  // Convert headingDiff from [0, 360) to  (-180, 180].
  // So now Positive value means it's currently heading left.
  // Negative value means it's heading right.
  float headingDiff = globalState.heading;
  if (moveForward) {
    if (headingDiff > 180) {
      headingDiff = headingDiff - 360;
    }
  } else { // moveBackward
    headingDiff = headingDiff - 180;
  }

  if (abs(headingDiff) < headingDelta) {
    globalState.steeringValue = midSteering; // Within tolerance, no steering.
    return;
  }

  // Calculate steering.
  float headingDiffError = 0;
  if (headingDiff > 0) {
    headingDiffError = headingDiff - headingDelta; // Compensate the tolerance.
  } else {
    headingDiffError = headingDiff + headingDelta;
  }
  float steeringCoeff = calculateSteeringCoeff(headingDiff, deltaTimeMillis);
  if (moveForward) {
    setSteering(steeringCoeff);
  } else if (moveBackward) {
    setSteering(-steeringCoeff); // In reverse, we need to turn the other way around.
  }
}

void calculateSteeringThrottleCinema() {
  float deltaTimeMillis = millis() - lastDeltaTimeMillis;
  if (deltaTimeMillis < minDeltaTimeMillis) {
    return; // No change.
  }

  lastDeltaTimeMillis = deltaTimeMillis;

  float yDiff = globalState.currentY - globalState.targetY;
  float xDiff = globalState.currentX - globalState.targetX;
  float headingDiff = globalState.heading - globalState.targetHeading;
  if (headingDiff > 180) {
    headingDiff -= 360;
  } else if (headingDiff <= -180) {
    headingDiff += 360;
  }

  if (abs(yDiff) <= distanceDelta) {
    globalState.throttleValue = midThrottle;
    globalState.steeringValue = midSteering;
    return;
  }

  boolean moveForward = false;
  boolean moveBackward = false;

  // Calculate throttle.
  float yDiffError = 0;
  if (yDiff > 0) {
    yDiffError = yDiff - distanceDelta; // Compensate the tolerance.
  } else {
    yDiffError = yDiff + distanceDelta;
  }
  float throttleCoeff = calculateThrottleCoeff(yDiffError, deltaTimeMillis);

  if (yDiff > 0) {
    setMoveForward(throttleCoeff);
    moveForward = true;
  } else {
    setMoveBackward(-throttleCoeff); // throttleCoeff < 0.
    moveBackward = true;
  }

  float steeringCoeff;
  if (isLeading(globalState.targetX, globalState.targetY, moveForward, moveBackward)) {
    // Leading.
    steeringCoeff = abs(xDiff) * leadingTurningCoefficient;
    if (pushingLeft(globalState.currentX, globalState.targetX, distanceDelta)) {
      setSteering(steeringCoeff);
    } else if (pushingRight(globalState.currentX, globalState.targetX, distanceDelta)) {
      setSteering(-steeringCoeff);
    }
  } else {
    // Tailing.
    steeringCoeff = headingDiff * tailingTurningCoefficient;
    if (moveForward) {
      setSteering(steeringCoeff);
    } else if (moveBackward) {
      setSteering(-steeringCoeff);
    }
  }
}

boolean pushingRight(float currentX, float targetX, float distanceDelta) {
  return currentX > targetX + distanceDelta;
}

boolean pushingLeft(float currentX, float targetX, float distanceDelta) {
  return currentX < targetX - distanceDelta;
}

boolean isInRearTriangle(float currentX, float currentY, float fieldOfViewRatio, boolean moveForward, boolean moveBackward) {
  if ((currentY >= 0 && moveForward) || (currentY <= 0 && moveBackward)) {
    // Not behind the car
    return false;
  }
  // Check if the point is inside the wedge
  return abs(currentY) >= fieldOfViewRatio * abs(currentX);
}

boolean isLeading(float currentX, float currentY, boolean moveForward, boolean moveBackward) {
  return !isInRearTriangle(currentX, currentY, fieldOfViewRatio, moveForward, moveBackward);
}

// Function to calculate the new distance diff using PID control.
float calculateThrottleCoeff(float error_t, float deltaTimeMillis) {

  // Step 1: Calculate integral (accumulated error)
  integral_t += error_t * deltaTimeMillis;

  // Step 2: Calculate derivative (rate of error change)
  float derivative_t = (error_t - previousError_t) / deltaTimeMillis;

  // Step 3: Compute PID output
  float throttleCoeff = (Kp_t * error_t) + (Ki_t * integral_t) + (Kd_t * derivative_t);

  // Step 4: Update previous error
  previousError_t = error_t;

  return throttleCoeff;
}

// Function to calculate the new heading diff using PID control.
float calculateSteeringCoeff(float error_s, float deltaTimeMillis) {

  // Step 1: Calculate integral (accumulated error)
  integral_s += error_s * deltaTimeMillis;

  // Step 2: Calculate derivative (rate of error change)
  float derivative_s = (error_s - previousError_s) / deltaTimeMillis;

  // Step 3: Compute PID output
  float steeringCoeff = (Kp_s * error_s) + (Ki_s * integral_s) + (Kd_s * derivative_s);

  // Step 4: Update previous error
  previousError_s = error_s;

  return steeringCoeff;
}

void calculateCoordinates() {
  // Calculate the heading angle in radians
  float headingRadians = radians(globalState.heading + 90); // +90 since we are always facing towards +Y axis.

  // Use trigonometry to calculate the coordinates
  globalState.currentX = globalState.distance * cos(headingRadians);
  globalState.currentY = globalState.distance * sin(headingRadians);

  // Serial output for debugging
  //LOG("Calculated coordinates (currentX, currentY): ");
  //LOG(globalState.currentX);
  //LOG(", ");
  //LOGLN(globalState.currentY)

  if (globalState.driveMode == DRIVE_MODE_MANUAL) { // Update the real time target coordinates if in manual mode.
    globalState.targetX = globalState.currentX;
    globalState.targetY = globalState.currentY;
    globalState.targetDistance = globalState.distance;
    globalState.targetHeading = globalState.heading;
    //LOG("Updated target coordinates (targetX, targetY): ");
    //LOG(globalState.targetX);
    //LOG(", ");
    //LOGLN(globalState.targetY);
  }
  return;
}

void setSteering(float steeringCoeff) {
  int steering = midSteering + (int) (steeringCoeff * steeringConstant);
  globalState.steeringValue = constrain(steering, minMoveSteering, maxMoveSteering);
}

void setMoveForward(float throttleCoeff) {
  int throttle = midThrottle + (int) (throttleCoeff * throttleConstant);
  globalState.throttleValue = constrain(throttle, minMoveThrottle, maxMoveThrottle);
}

void setMoveBackward(float throttleCoeff) {
  int throttle = midThrottle - (int) (throttleCoeff * throttleConstant);
  globalState.throttleValue = constrain(throttle, minMoveThrottle, maxMoveThrottle);
}

void emergencyStop() { //TODO(yifan): Refactor out this with a state machine.
  setDriveMode(DRIVE_MODE_MANUAL);
  globalState.distance = 0;
  globalState.targetDistance = 0;
  globalState.heading = 0;
  globalState.targetHeading = 0;
  globalState.yawSpeed = 0;
  globalState.pitchSpeed = 0;
  globalState.throttleValue = midThrottle;
  globalState.steeringValue = midSteering;
  autocamRemoteInputChanged = true; // Force gimbal to stop.
}

void checkDriveModeLED() {
  ledController.updateDriveModeLED(globalState.driveMode);
}
