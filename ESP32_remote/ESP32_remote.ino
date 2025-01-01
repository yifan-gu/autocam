#include <ArduinoBLE.h>

#define DATA_RATE 100 // 100 Hz

// Available state.
#define STATE_NOT_CONNECTED 0
#define STATE_CONNECTING 1
#define STATE_CONNECTED_FREE_MODE 2
#define STATE_CONNECTED_LOCK_MODE 3

const int minThrottle = 1000, maxThrottle = 2000, midThrottle = 1500;
const int minSteering = 1000, maxSteering = 2000, midSteering = 1500;

// Variable to toggle the free vs lock mode.
boolean isLockModeStateValue = false;
boolean isLockModeTriggerValue = false;

int state = STATE_NOT_CONNECTED;

// Variables to store values
int throttleValue = midThrottle, steeringValue = midSteering;

struct ControllerData {
  int throttleValue;
  int steeringValue;
  boolean isLockMode;
};

ControllerData receivedData;

// BLE Service and Characteristic
BLEService AutocamControllerService("f49f531a-9cba-4ada-905c-68699d400122");
BLECharacteristic AutocamControllerData("B320", BLERead | BLEWrite | BLENotify, sizeof(ControllerData), true);

BLEDevice BLECentral;

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

  setupBLEPeripheral();
}

void loop() {
  readStatusData();
  readControllerData();
  sendControllerData();
  delay(1000 / DATA_RATE); // Control the data rate.
}

void setupBLEPeripheral() {
  updateState(STATE_NOT_CONNECTED);

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
  ControllerData data = {0, 0, false};
  AutocamControllerData.writeValue((uint8_t*)&data, sizeof(data));

  // start advertising
  BLE.advertise();
  Serial.println("BLE Peripheral advertised!");

  connectToCentral();
}

void connectToCentral() {
  updateState(STATE_CONNECTING);
  while (!(BLECentral && BLECentral.connected())) {
    Serial.println("Connecting to BLE central...");
    BLECentral = BLE.central();
    delay(1000);
  }
  Serial.print("Connected to BLE central: ");
  // print the central's MAC address:
  Serial.println(BLECentral.address());
}

void sendControllerData() {
  if (state <= STATE_CONNECTING) {
    //Serial.println("Waiting for the initial state sync..."); No need to delay(), because apparently delay() will block BLE attributes discovery on the central device.
    return;
  }

  if (!(BLECentral && BLECentral.connected())) {
    Serial.println("Disconnected from central, reconnecting...");
    updateState(STATE_NOT_CONNECTED);
    connectToCentral();
  }

  ControllerData data = {throttleValue, steeringValue, isLockModeTriggerValue};
  AutocamControllerData.writeValue((uint8_t*)&data, sizeof(data));
  Serial.printf("Sent via BLE: throttle=%d, steering=%d, isLockMode=%d\n", data.throttleValue, data.steeringValue, data.isLockMode);
  //Serial.printf("Data interval: %d(ms)\n", millis() - lastPingTime);
  lastPingTime = millis();
}

void readStatusData() {
  if (!(BLECentral && BLECentral.connected())) {
    Serial.println("Disconnected from central, reconnecting...");
    updateState(STATE_NOT_CONNECTED);
    connectToCentral();
  }

  if (AutocamControllerData.written()) {
    AutocamControllerData.readValue((uint8_t*)&receivedData, sizeof(receivedData));
    Serial.printf("Received throttle=%d, steering=%d, isLockMode=%d\n", receivedData.throttleValue, receivedData.steeringValue, receivedData.isLockMode);
    isLockModeStateValue = receivedData.isLockMode;
    updateState(isLockModeStateValue ? STATE_CONNECTED_LOCK_MODE : STATE_CONNECTED_FREE_MODE);
    //Serial.printf("Data interval: %d(ms)\n", millis() - lastPingTime);
    //lastPingTime = millis();
  }
}

void updateState(int newState) {
  state = newState;
  Serial.printf("Update state=%d\n", state);
  // TODO(yifan): Update LED.
}

 // TOOD(yifan): Fill me in.
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
    isLockModeTriggerValue = !isLockModeTriggerValue;
    lastUpdate = millis();
  }
}
