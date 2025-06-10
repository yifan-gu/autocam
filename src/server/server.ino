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

// A small buffer to hold incoming fragments
static String wsBuffer;

// Servo objects
Servo throttleServo;
Servo steeringServo;

// Define throttle and steering ranges
const int16_t minThrottle = 1000, maxThrottle = 2000, midThrottle = 1500;
const int16_t minSteering = 1000, maxSteering = 2000, midSteering = 1500; // 1000 = max right turn, 2000 = max left turn.
const int16_t minSteeringThrottle = 50;

const float fieldOfViewRatio = 28 / 20; // Car length / car width (measured from the wheel).
const float SAFETY_DISTANCE = 10.0; // Maximum distance allowed, if current distance goes beyond this value, the car will stop.

int16_t minMoveThrottle = 1100, maxMoveThrottle = 1900;
int16_t minMoveSteering = 1000, maxMoveSteering = 2000;

// Heartbeat tracking
unsigned long lastWebSocketDataMillis = 0;

unsigned long lastSensorConnectedTime = 0;
unsigned long lastSensorDataMillis = 0;

unsigned long lastRemoteConnectedTime = 0;
unsigned long lastRemoteDataMillis = 0;
unsigned long lastRemoteStatusUpdatedMillis = 0;

const unsigned long sensorHeartbeatTimeout = 2000; // 2 seconds timeout to tolerate CAN communication failure.
const unsigned long remoteHeartbeatTimeout = 3000; // 3 seconds timeout
const unsigned long CONNECT_GRACE_MS = 2000; // 2000ms grace after BLE_CONNECTED
const unsigned long remoteStatusUpdateDuration = 400; // Update the remote's status every 400ms to keep the iOS app awake in the background even when the screen is locked.

struct GlobalState {
  int16_t throttleValue;       // e.g., servo pulse width for throttle
  int16_t steeringValue;       // e.g., servo pulse width for steering
  uint8_t state;               // overall state (e.g., SERVER_STATE_NOT_READY, etc.)
  uint8_t driveMode;           // e.g., DRIVE_MODE_MANUAL, DRIVE_MODE_FOLLOW, DRIVE_MODE_CINEMA
  float distance;          // distance from the Tag/Remote to the Sensor
  float targetDistance;    // desired distance
  float heading;           // measured heading in degrees [0, 360)  Front = 0/360, Left = 90, Back = 180, Right = 270
  float targetHeading;     // desired heading in degrees
  float yawSpeed;          // Yaw speed
  float pitchSpeed;        // Pitch speed
  uint8_t toggleState;         // Flag to store all the toggle button value (activeTrack, gimbalRecenter, cameraRecording)
  uint8_t uwbSelector;    // UWB selector value
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
  0,                     // uwbSelector
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

bool autocamRemoteDataUpdated = false;

enum BleConnectState {
  BLE_IDLE = 0,
  BLE_SCANNING,   // Actively trying to discover “Send” and/or “Recv”
  BLE_CONNECTED   // Both characteristics discovered
};

// For Remote:
BleConnectState remoteState       = BLE_IDLE;
bool            remoteSendReady   = false;
bool            remoteRecvReady   = false;

// For Sensor:
BleConnectState sensorState       = BLE_IDLE;
bool            sensorSendReady   = false;
bool            sensorRecvReady   = false;

// For Websocket:
enum WebSocketState {
  WEBSOCKET_DISCONNECTED = 0,
  WEBSOCKET_CONNECTED,
};

WebSocketState wsState = WEBSOCKET_DISCONNECTED;

//////////////////////////////////////////////////////////////////////////////////
//
// PID Controller variables for throttle. Tunable.
//
float distanceDelta = 0.0; // The distance tolerance in meters.
float headingDelta = 0; // The heading tolerance in degrees.
float cinemaLeadingHeadingDelta = 0; // The heading tolerance in cinema leading in meters (converts xDiff to headingDiff);
float throttleConstant = 100;
float steeringConstant = 10;

float Kp_t = 1.5;  // Proportional gain. Diff = Kp_t * distance
float Ki_t = 4;  // Integral gain. Diff = Ki_t * distance * 1000 * second, Ki_t * maxIntegralLimit_t * throttleConstant = IntegramThrottle
float Kd_t = 0.5;  // Derivative gain. Diff = Kd_t / (speed m/s * 1000 ms)

float previousError_t = 0.0;  // Previous error for the derivative term
float integral_t = 0.0;       // Accumulated integral term
float maxIntegralLimit_t = 0.25; // Ki_t * maxIntegralLimit_t * throttleConstant = IntegramThrottle
float maxIntegralLimit_s = 0;

// // PID Controller variables for steering.
float Kp_s = 1.0;   // Proportional gain. Diff = Kp_s * angle diff.
float Ki_s = 0.0;  // Integral gain. Diff = Ki_s * angle diff * 1000 * second.
float Kd_s = 0.0;   // Derivative gain. Diff = Kd_s / (anglur speed * 1000 ms)

float previousError_s = 0.0;  // Previous error for the derivative term
float integral_s = 0.0;       // Accumulated integral term

float lastDeltaTimeMillis = 0; // Used to calculate ki, and Kd in the PID system.
float minDeltaTimeMillis = 10; // Control the rate of PID update.
float deltaTimeThreshold = 1000; // If the delay is more than the threshold, skip the iteration.
float distanceSmoothFactor = 0.1; // Smoothing factor for distance readings (0 < alpha <= 1; lower values are smoother but slower to converge).

float leadingTurningCoefficient = 10;
float tailingTurningCoefficient = 0.5;


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

  establishAutocamRemoteBLEConnection();
  establishAutocamSensorBLEConnection();
  setupLED();
  setupESC();
  setupServer();
  setupBLECentral();
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

  attemptConnectRemote();
  attemptConnectSensor();
  //yield();

  getAutocamSensorData();
  getAutocamRemoteData();
  //yield();

  //printGlobalState();

  sendGimbalControllerData();
  updateAutocamRemoteStatus();
  //yield();

  calculateCoordinates();
  calculateSteeringThrottle();
  //yield();

  runESCController();
  checkDriveModeLED();
  runHealthCheck();
  //yield();

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

  // Send neutral signals
  setThrottle(midThrottle);
  setSteering(midSteering);

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
    json += "\"cinemaLeadingHeadingDelta\":" + String(cinemaLeadingHeadingDelta, 2) + ",";
    json += "\"maxIntegralLimit_t\":" + String(maxIntegralLimit_t, 2) + ",";
    json += "\"maxIntegralLimit_s\":" + String(maxIntegralLimit_s, 2) + ",";
    json += "\"Kp_t\":" + String(Kp_t, 2) + ",";
    json += "\"Ki_t\":" + String(Ki_t, 2) + ",";
    json += "\"Kd_t\":" + String(Kd_t, 2) + ",";
    json += "\"Kp_s\":" + String(Kp_s, 2) + ",";
    json += "\"Ki_s\":" + String(Ki_s, 2) + ",";
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
      distanceDelta = constrain(request->getParam("distanceDelta", true)->value().toFloat(), 0, 5);
      LOGF("distanceDelta: %f\n", distanceDelta);
    }
    if (request->hasParam("headingDelta", true)) {
      headingDelta = constrain(request->getParam("headingDelta", true)->value().toFloat(), 0, 30);
      LOGF("headingDelta: %f\n", headingDelta);
    }
    if (request->hasParam("cinemaLeadingHeadingDelta", true)) {
      cinemaLeadingHeadingDelta = constrain(request->getParam("cinemaLeadingHeadingDelta", true)->value().toFloat(), 0, 5);
      LOGF("cinemaLeadingHeadingDelta: %f\n", cinemaLeadingHeadingDelta);
    }
    if (request->hasParam("maxIntegralLimit_t", true)) {
      maxIntegralLimit_t = constrain(request->getParam("maxIntegralLimit_t", true)->value().toFloat(), 0, 100);
      LOGF("maxIntegralLimit_t: %f\n", maxIntegralLimit_t);
    }
    if (request->hasParam("maxIntegralLimit_s", true)) {
      maxIntegralLimit_s = constrain(request->getParam("maxIntegralLimit_s", true)->value().toFloat(), 0, 100);
      LOGF("maxIntegralLimit_s: %f\n", maxIntegralLimit_s);
    }
    if (request->hasParam("Kp_t", true)) {
      Kp_t = constrain(request->getParam("Kp_t", true)->value().toFloat(), 0, 10);
      LOGF("Kp_t: %f\n", Kp_t);
    }
    if (request->hasParam("Ki_t", true)) {
      Ki_t = constrain(request->getParam("Ki_t", true)->value().toFloat(), 0, 10);
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
      Ki_s = constrain(request->getParam("Ki_s", true)->value().toFloat(), 0, 10);
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

    setDriveMode(DRIVE_MODE_MANUAL);
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
      wsState = WEBSOCKET_CONNECTED;
      break;

    case WS_EVT_DISCONNECT:
      LOGLN("WebSocket disconnected");
      setDriveMode(DRIVE_MODE_MANUAL);
      LOGLN("Throttle and Steering reset to middle positions");
      wsState = WEBSOCKET_DISCONNECTED;
      break;

    case WS_EVT_DATA: {
      AwsFrameInfo *info = (AwsFrameInfo*)arg;

      // If this is the first fragment (index == 0), clear our buffer
      if (info->index == 0) {
        wsBuffer = "";
      }
      // Append this fragment’s payload
      wsBuffer += String((char*)data).substring(0, len);

      // If fin == true, this is the last fragment: process the full message
      if (info->final) {
        String message = wsBuffer;  // complete text

        // Reset the buffer for the next frame
        wsBuffer = "";

        // Now handle the full payload exactly once
        if (message == "REQUEST_DATA") {
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

          client->text(response);
        }
        else if (message == "driveMode=0") {
          setDriveMode(DRIVE_MODE_MANUAL);
        }
        else if (message == "driveMode=1") {
          setDriveMode(DRIVE_MODE_FOLLOW);
        }
        else if (message == "driveMode=2") {
          setDriveMode(DRIVE_MODE_CINEMA);
        }
        else if (message.startsWith("throttle=")) {
          globalState.throttleValue = map(
            message.substring(9).toInt(),
            1000, 2000,
            minMoveThrottle, maxMoveThrottle
          );
        }
        else if (message.startsWith("steering=")) {
          globalState.steeringValue = map(
            message.substring(9).toInt(),
            1000, 2000,
            minMoveSteering, maxMoveSteering
          );
        }
        else {
          LOGF("Unknown Websocket message: %s\n", message);
        }
      }
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

void setThrottle(int microseconds) {
  throttleServo.writeMicroseconds(globalState.throttleValue);
}

void setSteering(int microseconds) {
  throttleServo.writeMicroseconds(globalState.throttleValue);
}

void runESCController() {
  setThrottle(globalState.throttleValue);
  if (abs(globalState.throttleValue - midThrottle) > minSteeringThrottle) { // Prevent dry (stationary) steering: only drive steering when throttle exceeds a minimum value.
    setSteering(globalState.steeringValue);
  }

  /*LOG("Throttle: ");
  LOG(globalState.throttleValue);
  LOG(" | ");
  LOG("Steering: ");
  LOGLN(globalState.steeringValue);*/
}

void runHealthCheck() {
  unsigned long now;

  // ----- Sensor health -----
  now = millis();
  if ((AutocamSensor.connected() && sensorState == BLE_CONNECTED && now - lastSensorConnectedTime > CONNECT_GRACE_MS && now - lastSensorDataMillis > sensorHeartbeatTimeout) || (!AutocamSensor.connected() && sensorState != BLE_SCANNING)) {
    LOGF("Sensor disconnected → restarting non‐blocking scan, now: %ld, lastSensorConnectedTime: %ld, lastSensorDataMillis: %ld\n", now, lastSensorConnectedTime, lastSensorDataMillis);
    disconnectPeripheral(AutocamSensor, AutocamSensorServiceUUID);
    sensorState = BLE_IDLE;
    emergencyStop();
    updateState(globalState.state & ~SERVER_STATE_SENSOR_CONNECTED & ~SERVER_STATE_SENSOR_READY);
    establishAutocamSensorBLEConnection();
  }

  // ----- Remote health -----
  //LOGF("now: %ld, lastRemoteDataMillis: %ld\n", now, lastRemoteDataMillis);
  if ((AutocamRemote.connected() && remoteState == BLE_CONNECTED && now - lastRemoteConnectedTime > CONNECT_GRACE_MS && now - lastRemoteDataMillis > remoteHeartbeatTimeout) || (!AutocamRemote.connected() && remoteState != BLE_SCANNING)) {
    LOGF("Remote disconnected → restarting non‐blocking scan, now: %ld, lastRemoteConnectedTime: %ld, lastRemoteDataMillis: %ld\n", now, lastRemoteConnectedTime, lastRemoteDataMillis);
    disconnectPeripheral(AutocamRemote, AutocamRemoteServiceUUID);
    remoteState = BLE_IDLE;
    emergencyStop();
    updateState(globalState.state & ~SERVER_STATE_REMOTE_CONNECTED & ~SERVER_STATE_REMOTE_READY);
    establishAutocamRemoteBLEConnection();
  }

  // ----- WebSocket health -----
  //now = millis();
  //if (wsState == WEBSOCKET_CONNECTED && now - lastWebSocketDataMillis > heartbeatTimeout) {
  //  LOGLN("WebSocket inactive → closing all clients");
  //  ws.closeAll();                     // Close every WebSocket client
  //  wsState = WEBSOCKET_DISCONNECTED;
  //}
}

void runSafetyCheck() {
  if (globalState.distance >= SAFETY_DISTANCE) {
    emergencyStop();
  }
}

void establishAutocamRemoteBLEConnection() {
  // Reset the flags and enter SCANNING mode
  remoteSendReady  = false;
  remoteRecvReady  = false;
  remoteState      = BLE_SCANNING;
}

void establishAutocamSensorBLEConnection() {
  sensorSendReady  = false;
  sensorRecvReady  = false;
  sensorState      = BLE_SCANNING;
}

void attemptConnectRemote() {
  if (remoteState != BLE_SCANNING) {
    // Not in “scanning” mode, nothing to do.
    return;
  }

  // Step 1: If Send is not yet discovered, try it once per second
  if (!remoteSendReady) {
    bool okSend = scanForPeripheral(
      AutocamRemote,
      AutocamRemoteService,
      AutocamRemoteDataSend,
      AutocamRemoteServiceUUID,
      AutocamRemoteDataSendCharacteristicUUID
    );
    if (okSend) {
      // Successfully discovered “Send” characteristic:
      remoteSendReady = true;

    }
    return; // One characteristic at a time to avoid long blocking.
  }

  // Step 2: If Send is ready but Recv is not, try Recv immediately (no delay):
  if (!remoteRecvReady) {
    bool okRecv = scanForPeripheral(
      AutocamRemote,
      AutocamRemoteService,
      AutocamRemoteDataRecv,
      AutocamRemoteServiceUUID,
      AutocamRemoteDataRecvCharacteristicUUID
    );
    if (okRecv) {
      // Successfully discovered “Recv” characteristic:
      remoteRecvReady = true;
    }
  }

  if (remoteSendReady && remoteRecvReady && AutocamRemote.connected()) {
    remoteState = BLE_CONNECTED;
    lastRemoteConnectedTime = millis();
    updateState(globalState.state | SERVER_STATE_REMOTE_CONNECTED | SERVER_STATE_REMOTE_READY);
  }
}

void attemptConnectSensor() {
  if (sensorState != BLE_SCANNING) {
    return;
  }

  // Step 1: Try “Send” once per second
  if (!sensorSendReady) {
    bool okSend = scanForPeripheral(
      AutocamSensor,
      AutocamSensorService,
      AutocamSensorDataSend,
      AutocamSensorServiceUUID,
      AutocamSensorDataSendCharacteristicUUID
    );
    if (okSend) {
      sensorSendReady = true;
    }
    return; // One characteristic at a time to avoid long blocking.
  }

  // Step 2: If Send is ready but Recv not yet, attempt Recv immediately
  if (!sensorRecvReady) {
    bool okRecv = scanForPeripheral(
      AutocamSensor,
      AutocamSensorService,
      AutocamSensorDataRecv,
      AutocamSensorServiceUUID,
      AutocamSensorDataRecvCharacteristicUUID
    );
    if (okRecv) {
      sensorRecvReady = true;
    }
  }
  if (sensorSendReady && sensorRecvReady && AutocamSensor.connected()) {
    sensorState = BLE_CONNECTED;
    lastSensorConnectedTime = millis();
    updateState(globalState.state | SERVER_STATE_SENSOR_CONNECTED);
  }
}


void sendGimbalControllerData() {
  if (sensorState != BLE_CONNECTED) {
    return;
  }

  if (!autocamRemoteDataUpdated) {
    return;
  }

  SensorDataRecv data = {.yawSpeed = globalState.yawSpeed, .pitchSpeed = globalState.pitchSpeed, .toggleState = globalState.toggleState, .uwbSelector = globalState.uwbSelector};
  AutocamSensorDataRecv.writeValue((uint8_t *)&data, sizeof(SensorDataRecv));
  globalState.toggleState = 0; // Reset toggle state after triggering it.

  autocamRemoteDataUpdated = false;
  //LOGF("Sent via BLE: yawSpeed=%f, pitchSpeed=%f, toggleState=%d, uwbSelector=%d\n", data.yawSpeed, data.pitchSpeed, data.toggleState, data.uwbSelector);
  //LOGF("Data interval: %d(ms)\n", millis() - lastPingTime);
}

// Get data from the UWB Anchor via BLE.
void getAutocamSensorData() {
  if (sensorState != BLE_CONNECTED) {
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

  lastSensorDataMillis = millis();

  //LOGF("Received distance=%f, heading=%f, state=%d\n", data.distance, data.heading, data.state);

  // Smooth the distance reading using an exponential moving average.
  // New smoothed distance = alpha * (new reading) + (1 - alpha) * (previous smoothed value)
  globalState.distance = distanceSmoothFactor * data.distance +
                         (1.0 - distanceSmoothFactor) * globalState.distance;

  globalState.heading = data.heading;
  if (data.state & SENSOR_STATE_TAG_CONNECTED && data.state & SENSOR_STATE_SERVER_CONNECTED && data.state & SENSOR_STATE_CAN_CONNECTED) {
    updateState(globalState.state | SERVER_STATE_SENSOR_READY);
  } else {
    if (globalState.driveMode != DRIVE_MODE_MANUAL) {
      emergencyStop();
    }
    updateState(globalState.state & ~SERVER_STATE_SENSOR_READY);
  }

  //LOGF("Data interval: %d(ms)\n", millis() - lastPingTime);
  //lastPingTime = millis();
}

void getAutocamRemoteData() {
  if (remoteState != BLE_CONNECTED) {
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

  autocamRemoteDataUpdated = true;
  lastRemoteDataMillis = millis();
  //LOGF("Received throttle=%d, steering=%d, driveMode=%d, yawSpeed=%f, pitchSpeed=%f, toggleState=%d, uwbSelector=%d\n", data.throttleValue, data.steeringValue, data.driveMode, data.yawSpeed, data.pitchSpeed, data.toggleState, data.uwbSelector);
  //LOGF("Data interval: %d(ms)\n", millis() - lastPingTime);

  if (globalState.driveMode == DRIVE_MODE_MANUAL) {
    globalState.throttleValue = map(data.throttleValue, minThrottle, maxThrottle, minMoveThrottle, maxMoveThrottle);
    globalState.steeringValue = map(data.steeringValue, minSteering, maxSteering, minMoveSteering, maxMoveSteering);
  }

  globalState.yawSpeed = data.yawSpeed;
  globalState.pitchSpeed = data.pitchSpeed;
  globalState.toggleState = data.toggleState;

  setDriveMode(data.driveMode);
  setUWBSelector(data.uwbSelector);
}

void updateState(uint8_t newState) {
  if (globalState.state == newState) {
    return;
  }
  globalState.state = newState;
  ledController.updateStateLED(globalState.state);
}

void updateAutocamRemoteStatus() {
  if (remoteState != BLE_CONNECTED) {
    return;
  }

  unsigned long now = millis();
  if (lastRemoteStatusUpdatedMillis - now < remoteStatusUpdateDuration) {
    return;
  }

  RemoteDataRecv data = { .state = globalState.state, .driveMode = globalState.driveMode, .uwbSelector = globalState.uwbSelector };
  AutocamRemoteDataRecv.writeValue((uint8_t *)&data, sizeof(RemoteDataRecv));
  lastRemoteStatusUpdatedMillis = now;

  //LOGF("Sent status to remote via BLE: state=%d, driveMode=%d, uwbSelector=%d\n", globalState.state, globalState.driveMode, globalState.uwbSelector);
}

void setDriveMode(uint8_t newDriveMode) {
  if (globalState.driveMode == newDriveMode) {
    return;
  }

  if (sensorState != BLE_CONNECTED && newDriveMode != DRIVE_MODE_MANUAL) {
    return;
  }

  globalState.driveMode = newDriveMode;
  globalState.throttleValue = midThrottle;
  globalState.steeringValue = midSteering;
  resetPIDValues();
  ledController.updateDriveModeLED(globalState.driveMode);
}

void resetPIDValues() {
  previousError_t = 0;
  integral_t = 0;
  previousError_s = 0;
  integral_s = 0;
  lastDeltaTimeMillis = 0;
}

void setUWBSelector(uint8_t newUWBSelector) {
  if (globalState.uwbSelector == newUWBSelector) {
    return;
  }

  globalState.uwbSelector = newUWBSelector;
  if (globalState.driveMode != DRIVE_MODE_MANUAL) { // Disable auto pilot when switching the target uwb tag.
    setDriveMode(DRIVE_MODE_MANUAL);
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
  float now = millis();
  float deltaTimeMillis = now - lastDeltaTimeMillis;
  lastDeltaTimeMillis = now;

  if (deltaTimeMillis < minDeltaTimeMillis) { // Also handles deltaTimeMillis < 0 where the timestamps wraps around.
    return; // No change.
  }

  if (deltaTimeMillis == now) {
    return; // Skip the first iteration for a clean start.
  }

  if (deltaTimeMillis > deltaTimeThreshold) {
    LOGF("WARN: Throttle / steering calculation interval (%s ms) is larger than the threshold (%s ms), will reset the integral and previous error", deltaTimeMillis, deltaTimeThreshold);
    resetPIDValues();
    return;
  }

  float distanceDiff = globalState.distance - globalState.targetDistance;

  // Calculate the error and compensate for the tolerance.
  float distanceDiffError = 0;
  if (abs(distanceDiff) > distanceDelta) {
    if (distanceDiff > 0) {
      distanceDiffError = distanceDiff - distanceDelta;
    } else {
      distanceDiffError = distanceDiff + distanceDelta;
    }
  }

  float throttleCoeff = calculateThrottleCoeff(distanceDiffError, deltaTimeMillis);
  if (throttleCoeff < 0) {
    throttleCoeff = 0; // Prevent the car from backing off (If the throttleCoeff is 0 the car will slow down eventually).
  }

  bool moveForward = false;
  bool moveBackward = false;
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

  // Calculate steering.
  float headingDiffError = 0;
  if (abs(headingDiff) > headingDelta) {
    if (headingDiff > 0) {
      headingDiffError = headingDiff - headingDelta; // Compensate the tolerance.
    } else {
      headingDiffError = headingDiff + headingDelta;
    }
  }

  float steeringCoeff = calculateSteeringCoeff(headingDiff, deltaTimeMillis);
  if (moveForward) {
    setSteeringValue(steeringCoeff);
  } else if (moveBackward) {
    setSteeringValue(-steeringCoeff); // In reverse, we need to turn the other way around.
  }
  //LOGF("Throttle / steering coeff follow mode: %lf ; %lf\n", throttleCoeff, steeringCoeff);
}

void calculateSteeringThrottleCinema() {
  float now = millis();
  float deltaTimeMillis = now - lastDeltaTimeMillis;
  lastDeltaTimeMillis = now;

  if (deltaTimeMillis < minDeltaTimeMillis) { // Also handles deltaTimeMillis < 0 where the timestamps wraps around.
    return; // No change.
  }

  if (deltaTimeMillis == now) {
    return; // Skip the first iteration for a clean start.
  }

  if (deltaTimeMillis > deltaTimeThreshold) {
    LOGF("WARN: Throttle / steering calculation interval (%s ms) is larger than the threshold (%s ms), will reset the integral and previous error", deltaTimeMillis, deltaTimeThreshold);
    resetPIDValues();
    return;
  }

  float yDiff = globalState.currentY - globalState.targetY;
  float xDiff = globalState.currentX - globalState.targetX;
  float headingDiff = globalState.heading - globalState.targetHeading;
  if (headingDiff > 180) {
    headingDiff -= 360;
  } else if (headingDiff <= -180) {
    headingDiff += 360;
  }

  // Calculate throttle.
  float yDiffError = 0;
  if (abs(yDiff) > distanceDelta) {
    if (yDiff > 0) {
      yDiffError = yDiff - distanceDelta; // Compensate the tolerance.
    } else {
      yDiffError = yDiff + distanceDelta;
    }
  }

  float throttleCoeff = calculateThrottleCoeff(yDiffError, deltaTimeMillis);

  bool moveForward = false;
  bool moveBackward = false;
  if (yDiff > 0) {
    setMoveForward(throttleCoeff);
    moveForward = true;
  } else {
    setMoveBackward(-throttleCoeff); // throttleCoeff < 0.
    moveBackward = true;
  }

  float headingDiffError = 0;
  float steeringCoeff;
  if (isLeading(globalState.targetX, globalState.targetY, moveForward, moveBackward)) {
    // Leading.
    if (abs(xDiff) > cinemaLeadingHeadingDelta) {
      if (xDiff > 0) {
        headingDiffError = xDiff - cinemaLeadingHeadingDelta;
      } else {
        headingDiffError = xDiff + cinemaLeadingHeadingDelta;
      }
    }
    steeringCoeff = calculateSteeringCoeff(headingDiffError, deltaTimeMillis) * leadingTurningCoefficient;
    setSteeringValue(-steeringCoeff);
  } else {
    // Tailing.
    if (abs(headingDiff) > headingDelta) {
      if (headingDiff > 0) {
        headingDiffError = headingDiff - headingDelta;
      } else {
        headingDiffError = headingDiff + headingDelta;
      }
    }
    steeringCoeff = calculateSteeringCoeff(headingDiffError, deltaTimeMillis) * tailingTurningCoefficient;
    if (moveForward) {
      setSteeringValue(steeringCoeff);
    } else if (moveBackward) {
      setSteeringValue(-steeringCoeff);
    }
  }
  //LOGF("Throttle / steering coeff cinema mode: %lf ; %lf\n", throttleCoeff, steeringCoeff);
}

bool isInRearTriangle(float currentX, float currentY, float fieldOfViewRatio, bool moveForward, bool moveBackward) {
  if ((currentY >= 0 && moveForward) || (currentY <= 0 && moveBackward)) {
    // Not behind the car
    return false;
  }
  // Check if the point is inside the wedge
  return abs(currentY) >= fieldOfViewRatio * abs(currentX);
}

bool isLeading(float currentX, float currentY, bool moveForward, bool moveBackward) {
  return !isInRearTriangle(currentX, currentY, fieldOfViewRatio, moveForward, moveBackward);
}

// Function to calculate the new distance diff using PID control.
float calculateThrottleCoeff(float error_t, float deltaTimeMillis) {
  // Step 1: Calculate integral (accumulated error)
  integral_t += error_t * deltaTimeMillis / 1000;
  integral_t = constrain(integral_t, -maxIntegralLimit_t, maxIntegralLimit_t);

  // Step 2: Calculate derivative (rate of error change)
  float derivative_t = (error_t - previousError_t) * 1000 / deltaTimeMillis;

  // Step 3: Calculate all the terms.
  float Kp_term = Kp_t * error_t;

  float Ki_term = Ki_t * integral_t;

  float Kd_term = Kd_t * derivative_t;

  //LOGF("Kp_term: %.2f, Ki_term: %.2f, Kd_term: %.2f\n", Kp_term, Ki_term, Kd_term);

  // Step 4: Compute PID output
  float throttleCoeff = Kp_term + Ki_term + Kd_term;

  // Step 5: Update previous error
  previousError_t = error_t;

  return throttleCoeff;
}

// Function to calculate the new heading diff using PID control.
float calculateSteeringCoeff(float error_s, float deltaTimeMillis) {

  // Step 1: Calculate integral (accumulated error)
  integral_s += error_s * deltaTimeMillis / 1000;
  integral_s = constrain(integral_s, -maxIntegralLimit_s, maxIntegralLimit_s);

  // Step 2: Calculate derivative (rate of error change)
  float derivative_s = (error_s - previousError_s) * 1000 / deltaTimeMillis;

  // Step 3: Calculate all the terms.
  float Kp_term = Kp_s * error_s;

  float Ki_term = Ki_s * integral_s;

  float Kd_term = Kd_s * derivative_s;

  //LOGF("Kp_term: %.2f, Ki_term: %.2f, Kd_term: %.2f\n", Kp_term, Ki_term, Kd_term);

  // Step 4: Compute PID output
  float steeringCoeff = Kp_term + Ki_term + Kd_term;

  // Step 5: Update previous error
  previousError_s = error_s;

  return steeringCoeff;
}

void calculateCoordinates() {
  if (sensorState != BLE_CONNECTED) {
    resetGlobalStateDistanceValues();
    return;
  }

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

void setSteeringValue(float steeringCoeff) {
  int16_t steering = midSteering + (int16_t) (steeringCoeff * steeringConstant);
  globalState.steeringValue = constrain(steering, minMoveSteering, maxMoveSteering);
}

void setMoveForward(float throttleCoeff) {
  int16_t throttle = midThrottle + (int16_t) (throttleCoeff * throttleConstant);
  globalState.throttleValue = constrain(throttle, minMoveThrottle, maxMoveThrottle);
}

void setMoveBackward(float throttleCoeff) {
  int16_t throttle = midThrottle - (int16_t) (throttleCoeff * throttleConstant);
  globalState.throttleValue = constrain(throttle, minMoveThrottle, maxMoveThrottle);
}

void emergencyStop() { //TODO(yifan): Refactor out this with a state machine.
  stopCar();
  setDriveMode(DRIVE_MODE_MANUAL);
  //resetGlobalStateDistanceValues();
  //resetGlobalStateGimbalValues();
}

void stopCar() {
  globalState.throttleValue = midThrottle;
  globalState.steeringValue = midSteering;
  setThrottle(globalState.throttleValue);
  setSteering(globalState.steeringValue);
}

void resetGlobalStateDistanceValues() {
  globalState.distance = 0;
  globalState.targetDistance = 0;
  globalState.heading = 0;
  globalState.targetHeading = 0;
  globalState.currentX = 0;
  globalState.currentY = 0;
  globalState.targetX = 0;
  globalState.targetY = 0;
}

void resetGlobalStateGimbalValues() {
  globalState.yawSpeed = 0;
  globalState.pitchSpeed = 0;
  autocamRemoteDataUpdated = true; // Force gimbal to stop.
}

void checkDriveModeLED() {
  ledController.updateDriveModeLED(globalState.driveMode);
}
