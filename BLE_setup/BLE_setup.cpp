#include "BLE_setup.hpp"

BLEDevice BLECentral;

BLEDevice UWBAnchor, AutocamController;

const char *UWBAnchorServiceUUID = "e2b221c6-7a26-49f4-80cc-0cd58a53041d";
const char *AutocamControllerServiceUUID = "f49f531a-9cba-4ada-905c-68699d400122";

const char *UWBAnchorSensorDataSendCharacteristicUUID = "B328";
const char *UWBAnchorSensorDataRecvCharacteristicUUID = "B329";
const char *AutocamControllerControllerDataCharacteristicUUID = "B330";

BLEService UWBAnchorService(UWBAnchorServiceUUID);
BLEService AutocamControllerService(AutocamControllerServiceUUID);

BLECharacteristic UWBAnchorSensorDataSend(UWBAnchorSensorDataSendCharacteristicUUID, BLERead | BLENotify, sizeof(SensorDataSend));
BLECharacteristic UWBAnchorSensorDataRecv(UWBAnchorSensorDataRecvCharacteristicUUID, BLEWrite, sizeof(SensorDataRecv));
BLECharacteristic AutocamControllerData(AutocamControllerControllerDataCharacteristicUUID, BLERead | BLEWrite | BLENotify, sizeof(ControllerData));

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

  if (!device) {
    Serial.printf("Scanning for BLE peripheral [%s]...\n", serviceUUID);
    BLE.scanForUuid(serviceUUID);
    device = BLE.available();
    if (!device) {
      Serial.println("No BLE peripheral found.");
      return false;
    }
  }
  Serial.printf("Found BLE peripheral, local name: [%s]\n", device.localName());
  BLE.stopScan();
  return connectToBLEDevice(device, service, characteristic, serviceUUID, characteristicUUID);
}

bool connectToBLEDevice(BLEDevice &device, BLEService &service, BLECharacteristic &characteristic, const char *serviceUUID, const char *characteristicUUID) {
  if (!device.connected()) {
    if (!device.connect()) {
      Serial.printf("Failed to connect to [%s].\n", device.localName());
      return false;
    }
  }

  Serial.printf("Connected to [%s]!\n", device.localName());
  if (!device.discoverAttributes()) {
    Serial.printf("Attribute discovery failed for [%s]! Disconnecting...\n", device.localName());
    device.disconnect();
    return false;
  }

  service = device.service(serviceUUID);
  Serial.printf("BLE device name: [%s], advertised svc count: [%d], Real svc count: [%d], characteristics count: [%d]\n", device.deviceName(), device.advertisedServiceUuidCount(), device.serviceCount(), device.characteristicCount());
  characteristic = service.characteristic(characteristicUUID);

  if (!characteristic) {
    Serial.printf("Failed to find characteristic [%s] for device [%s]! Disconnecting...\n", characteristicUUID, device.deviceName());
    device.disconnect();
    return false;
  }

  if (characteristic.canSubscribe()) {
    if (!characteristic.subscribe()) {
      Serial.printf("Failed to subscribe characteristic [%s] for device [%s]! Disconnecting...\n", characteristicUUID, device.deviceName());
      device.disconnect();
      return false;
    }
  }
  Serial.printf("Found characteristic [%s] on device [%s] and successfully subscribed!\n", characteristicUUID, device.deviceName());
  return true;
}

void setupBLEPeripheral(const char *localName, const char *deviceName, BLEService& service) {
   // begin initialization
  if (!BLE.begin()) {
    panic("Failed to start BLE as peripheral!");
  }
  BLE.setConnectionInterval(6, 6);
  BLE.setLocalName(localName);
  BLE.setDeviceName(deviceName);
  BLE.setAdvertisedService(service);

  // add service
  BLE.addService(service);

  // start advertising
  BLE.advertise();
  Serial.println("BLE Peripheral advertised!");
}

bool connectToCentral() {
  static unsigned long lastScanMillis = 0;

  unsigned long currentMillis = millis();
  if (currentMillis - lastScanMillis < scanIntervalMillis) {
    return false;
  }
  lastScanMillis = currentMillis;

  Serial.println("Waiting for BLE central to connect...");

  BLECentral = BLE.central();
  if (!(BLECentral && BLECentral.connected())) {
    return false;
  }

  Serial.printf("BLE central[%s] connected!\n", BLECentral.address());
  return true;
}
