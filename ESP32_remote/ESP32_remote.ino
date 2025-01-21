#include <ArduinoBLE.h>

#define DATA_RATE 100 // 100 Hz

// Available state.
#define STATE_NOT_READY 0
#define STATE_SENSOR_READY 1
#define STATE_REMOTE_CONTROLLER_READY 2

#define DRIVE_MODE_MANUAL 0
#define DRIVE_MODE_AUTO_FOLLOW 1

const int minThrottle = 1000, maxThrottle = 2000, midThrottle = 1500;
const int minSteering = 1000, maxSteering = 2000, midSteering = 1500;

// State indicators.
int state = STATE_NOT_READY;
int driveMode = DRIVE_MODE_MANUAL;

// Variable toggle the drive mode.
int driveModeTriggerValue = DRIVE_MODE_MANUAL;

// Variables to store values
int throttleValue = midThrottle, steeringValue = midSteering;

struct ControllerData {
  int throttleValue;
  int steeringValue;
  int driveMode;
  int state;
};

ControllerData receivedData;

// BLE Service and Characteristic
BLEService AutocamControllerService("f49f531a-9cba-4ada-905c-68699d400122");
BLECharacteristic AutocamControllerData("B320", BLERead | BLEWrite | BLENotify, sizeof(ControllerData), true);

BLEDevice BLECentral;

unsigned int lastPingTime = 0;

unsigned long lastScanMillis = 0;
unsigned long scanIntervalMillis = 1000;

void setup() {
  // Start Serial for debugging
  unsigned long startTime = millis();
  unsigned long timeout = 1000; // 1 seconds timeout
                                //
  Serial.begin(115200);
  while (!Serial && (millis() - startTime < timeout)) {
    // Wait for Serial or timeout
  }

  setupBLEPeripheral();
  connectToCentral();
}

void loop() {
  readStatusData();
  readControllerData();
  sendControllerData(); // TODO(yifan): Send update only when there's a value change.
  delay(1000 / DATA_RATE); // Control the data rate.
}

void setupBLEPeripheral() {
   // begin initialization
  if (!BLE.begin()) {
    Serial.println("Failed to start BLE module!");
    while (1);
  }
  BLE.setConnectionInterval(6, 6);
  BLE.setLocalName("Autocam Remote");
  BLE.setDeviceName("autocam-remote");
  BLE.setAdvertisedService(AutocamControllerService);

  // add the characteristic to the service
  AutocamControllerService.addCharacteristic(AutocamControllerData);

  // add service
  BLE.addService(AutocamControllerService);

  // set the initial value for the characeristic:
  ControllerData data = {0, 0, 0, 0};
  AutocamControllerData.writeValue((uint8_t*)&data, sizeof(data));

  // start advertising
  BLE.advertise();
  Serial.println("BLE Peripheral advertised!");
}

bool connectToCentral() {
  unsigned long currentMillis = millis();
  if (currentMillis - lastScanMillis < scanIntervalMillis) {
    return false;
  }
  lastScanMillis = currentMillis;

  Serial.println("Connecting to BLE central...");

  BLECentral = BLE.central();
  if (!(BLECentral && BLECentral.connected())) {
    return false;
  }

  Serial.printf("Connected to BLE central: %s\n", BLECentral.address());
  return true;
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
    AutocamControllerData.readValue((uint8_t*)&receivedData, sizeof(receivedData));
    Serial.printf("Received throttle=%d, steering=%d, driveMode=%d, state=%d\n", receivedData.throttleValue, receivedData.steeringValue, receivedData.driveMode, receivedData.state);
    updateState(receivedData.state);
    updateDriveMode(receivedData.driveMode);

    //Serial.printf("Data interval: %d(ms)\n", millis() - lastPingTime);
    //lastPingTime = millis();
  }
}

 // TODO(yifan): Fill me in.
void readControllerData() {
  static int throttleStep = 1;
  static int steeringStep = 1;
  static int lastUpdate = 0;

  if (throttleValue >= maxThrottle || throttleValue <= minThrottle) {
    throttleStep = -throttleStep;
  }
  if (steeringValue >= maxSteering || steeringValue <= minSteering) {
    steeringStep = -steeringStep;
  }

  throttleValue += throttleStep;
  steeringValue += steeringStep;


  if (millis() - lastUpdate >= 1000) {
    driveModeTriggerValue = driveMode == DRIVE_MODE_MANUAL ? DRIVE_MODE_AUTO_FOLLOW : DRIVE_MODE_MANUAL;
    lastUpdate = millis();
  }
}

void sendControllerData() {
  if (!(BLECentral && BLECentral.connected())) {
    updateState(state & ~STATE_REMOTE_CONTROLLER_READY);
    if (!connectToCentral()) {
      return;
    }
    updateState(state | STATE_REMOTE_CONTROLLER_READY);
  }

  ControllerData data = {throttleValue, steeringValue, driveModeTriggerValue, state}; // state doesn't matter because it will not be used by the server.
  AutocamControllerData.writeValue((uint8_t*)&data, sizeof(data));
  Serial.printf("Sent via BLE: throttle=%d, steering=%d, driveMode=%d, state=%d\n", data.throttleValue, data.steeringValue, data.driveMode, data.state);
  //Serial.printf("Data interval: %d(ms)\n", millis() - lastPingTime);
  lastPingTime = millis();
}
