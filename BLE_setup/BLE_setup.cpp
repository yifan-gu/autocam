#include "BLE_setup.hpp"

BLEDevice BLECentral;

BLEDevice UWBAnchor, AutocamController, GimbalController;

const char *UWBAnchorServiceUUID = "e2b221c6-7a26-49f4-80cc-0cd58a53041d";
const char *AutocamControllerServiceUUID = "f49f531a-9cba-4ada-905c-68699d400122";
const char *GimbalControllerServiceUUID = "53f884f3-b189-4e84-8a16-f4c24d9add7c";

const char *UWBAnchorSensorDataCharacteristicUUID = "B328";
const char *AutocamControllerControllerDataCharacteristicUUID = "B330";
const char *GimbalControllerDataCharacteristicUUID = "B335";

BLEService UWBAnchorService(UWBAnchorServiceUUID);
BLEService AutocamControllerService(AutocamControllerServiceUUID);
BLEService GimbalControllerService(GimbalControllerServiceUUID);

BLECharacteristic UWBAnchorSensorData(UWBAnchorSensorDataCharacteristicUUID, BLERead | BLENotify, sizeof(ControllerData), true);
BLECharacteristic AutocamControllerData(AutocamControllerControllerDataCharacteristicUUID, BLERead | BLEWrite | BLENotify, sizeof(ControllerData), true);
BLECharacteristic GimbalControllerData(GimbalControllerDataCharacteristicUUID, BLERead | BLENotify, sizeof(ControllerData), true);

unsigned long scanIntervalMillis = 1000;

void panic(const char *message) {
  Serial.println(message);
  while (1) {};
}

void setupBLECentral() {
  if (!BLE.begin()) {
    panic("Failed to start BLE as central");
  }
  Serial.println("BLE Central initialized.");
  BLE.setConnectionInterval(6, 6); // from 7.5ms to 7.5ms (6 * 1.5ms).
}

bool scanForPeripheral(BLEDevice &device, BLEService &service, BLECharacteristic &characteristic, const char *serviceUUID, const char *characteristicUUID) {
  static unsigned long lastScanMillis = 0;

  unsigned long currentMillis = millis();
  if (currentMillis - lastScanMillis < scanIntervalMillis ) {
    return false;
  }
  lastScanMillis = currentMillis;

  Serial.printf("Scanning for BLE peripheral [%s]...\n", serviceUUID);
  BLE.scanForUuid(serviceUUID);
  device = BLE.available();

  if (!device) {
    Serial.println("No BLE peripheral found.");
    return false;
  }
  Serial.printf("Found BLE peripheral, local name: [%s]\n", device.localName());
  BLE.stopScan();
  return connectToBLEDevice(device, service, characteristic, serviceUUID, characteristicUUID);
}

bool connectToBLEDevice(BLEDevice &device, BLEService &service, BLECharacteristic &characteristic, const char *serviceUUID, const char *characteristicUUID) {
  if (!device.connect()) {
    Serial.printf("Failed to connect to [%s].\n", device.localName());
    false;
  }

  Serial.printf("Connected to [%s]!\n", device.localName());
  if (!device.discoverAttributes()) {
    Serial.printf("Attribute discovery failed for [%s]! Disconnecting...\n", device.localName());
    device.disconnect();
    false;
  }

  service = device.service(serviceUUID);
  Serial.printf("BLE device name: [%s], advertised svc count: [%d], Real svc count: [%d], characteristics count: [%d]\n", device.deviceName(), device.advertisedServiceUuidCount(), device.serviceCount(), device.characteristicCount());
  characteristic = service.characteristic(characteristicUUID);

  if (!characteristic) {
    Serial.printf("Failed to find characteristic [%s] for device [%s]! Disconnecting...\n", characteristicUUID, device.deviceName());
    device.disconnect();
    false;
  }

  if (!characteristic.canSubscribe()) {
    Serial.printf("Cannot subscribe characteristic [%s] for device [%s]! Disconnecting...\n", characteristicUUID, device.deviceName());
    device.disconnect();
    false;
  }

  if (!characteristic.subscribe()) {
    Serial.printf("Failed to subscribe characteristic [%s] for device [%s]! Disconnecting...\n", characteristicUUID, device.deviceName());
    device.disconnect();
    false;
  }
  Serial.printf("Found characteristic [%s] on device [%s] and successfully subscribed!\n", characteristicUUID, device.deviceName());
  return true;
}

void setupBLEPeripheral(const char *localName, const char *deviceName, BLEService& service, BLECharacteristic& characteristic, uint8_t *data, int data_length) {
   // begin initialization
  if (!BLE.begin()) {
    panic("Failed to start BLE as peripheral!");
  }
  BLE.setConnectionInterval(6, 6);
  BLE.setLocalName(localName);
  BLE.setDeviceName(deviceName);
  BLE.setAdvertisedService(service);

  // add the characteristic to the service
  service.addCharacteristic(characteristic);

  // add service
  BLE.addService(service);

  // set the initial value for the characeristic:
  characteristic.writeValue(data, data_length);

  // start advertising
  BLE.advertise();
  Serial.println("BLE Peripheral advertised!");

  connectToCentral();
}

bool connectToCentral() {
  static unsigned long lastScanMillis = 0;

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
