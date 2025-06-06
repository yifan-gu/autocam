#include "BLE_setup.hpp"
#include "util.h"
#include <map>

BLEDevice BLECentral;

BLEDevice AutocamSensor, AutocamRemote;

const char *AutocamSensorServiceUUID = "e2b221c6-7a26-49f4-80cc-0cd58a53041d";
const char *AutocamRemoteServiceUUID = "f49f531a-9cba-4ada-905c-68699d400122";

const char *AutocamSensorDataSendCharacteristicUUID = "B328";
const char *AutocamSensorDataRecvCharacteristicUUID = "B329";
const char *AutocamRemoteDataSendCharacteristicUUID = "B330";
const char *AutocamRemoteDataRecvCharacteristicUUID = "B331";

BLEService AutocamSensorService(AutocamSensorServiceUUID);
BLEService AutocamRemoteService(AutocamRemoteServiceUUID);

BLECharacteristic AutocamSensorDataSend(AutocamSensorDataSendCharacteristicUUID, BLERead | BLENotify, sizeof(SensorDataSend));
BLECharacteristic AutocamSensorDataRecv(AutocamSensorDataRecvCharacteristicUUID, BLEWrite, sizeof(SensorDataRecv));
BLECharacteristic AutocamRemoteDataSend(AutocamRemoteDataSendCharacteristicUUID, BLERead | BLENotify, sizeof(RemoteDataSend));
BLECharacteristic AutocamRemoteDataRecv(AutocamRemoteDataRecvCharacteristicUUID, BLEWrite, sizeof(RemoteDataRecv));

unsigned long scanIntervalMillis = 1000;
unsigned long bleBlockingLimitMillis = 500; // This ensures the BLE operation will not block the server.

// Map from service UUID (as String) to whether attributes have been discovered
static std::map<String, bool> attributesDiscovered;

void panic(const char *message) {
  LOGLN(message);
  while (1) {};
}

// Helper to mark a service UUID as “discovered” or not
void setAttributesDiscovered(const char* serviceUUID, bool discovered) {
  attributesDiscovered[String(serviceUUID)] = discovered;
}

// Helper to check if a service UUID has already been discovered
bool isAttributesDiscovered(const char* serviceUUID) {
  auto it = attributesDiscovered.find(String(serviceUUID));
  return (it != attributesDiscovered.end()) ? it->second : false;
}

void setupBLECentral() {
  if (!BLE.begin()) {
    panic("Failed to start BLE as central");
  }

  LOGLN("BLE Central initialized.");
  BLE.setConnectionInterval(6, 6); // from 7.5ms to 7.5ms (6 * 1.5ms).
}

bool scanForPeripheral(BLEDevice &device, BLEService &service, BLECharacteristic &characteristic, const char *serviceUUID, const char *characteristicUUID) {
  static unsigned long lastScanMillis = 0;

  unsigned long now = millis();
  if (now - lastScanMillis < scanIntervalMillis) {
    return false;
  }
  lastScanMillis = now;

    // If we already had a device but it is no longer connected, clear it:
  if (device && !device.connected()) {
    LOGLN("Previously connected device is gone—clearing reference.");
    device = BLEDevice();  // assign an empty object to force a rescan
  }

  if (!device) {
    LOGF("Scanning for BLE peripheral [%s]...\n", serviceUUID);
    BLE.scanForUuid(serviceUUID);
    device = BLE.available();
    if (!device) {
      LOGLN("No BLE peripheral found.");
      return false;
    }
  }
  LOGF("Found BLE peripheral, local name: [%s]\n", device.localName());
  BLE.stopScan();
  if (millis() - now > bleBlockingLimitMillis) {
    return false;
  }
  return connectToBLEDevice(device, service, characteristic, serviceUUID, characteristicUUID);
}

bool connectToBLEDevice(BLEDevice &device, BLEService &service, BLECharacteristic &characteristic, const char *serviceUUID, const char *characteristicUUID) {
  unsigned long now = millis();
  if (!device.connected()) {
    if (!device.connect()) {
      LOGF("Failed to connect to [%s].\n", device.localName());
      return false;
    }
  }

  LOGF("Connected to [%s]!\n", device.localName());
  if (millis() - now > bleBlockingLimitMillis) {
    return false;
  }

  if (!isAttributesDiscovered(serviceUUID) && !device.discoverAttributes()) {
    LOGF("Attribute discovery failed for [%s]! Disconnecting...\n", device.localName());
    disconnectPeripheral(device, serviceUUID);
    return false;
  }
  setAttributesDiscovered(serviceUUID, true);

  LOGF("BLE device name: [%s], advertised svc count: [%d], Real svc count: [%d], characteristics count: [%d]\n", device.deviceName(), device.advertisedServiceUuidCount(), device.serviceCount(), device.characteristicCount());
  if (millis() - now > bleBlockingLimitMillis) {
    return false;
  }

  service = device.service(serviceUUID);
  characteristic = service.characteristic(characteristicUUID);
  if (millis() - now > bleBlockingLimitMillis) {
    return false;
  }

  if (!characteristic) {
    LOGF("Failed to find characteristic [%s] for device [%s]! Disconnecting...\n", characteristicUUID, device.deviceName());
    disconnectPeripheral(device, serviceUUID);
    return false;
  }

  if (characteristic.canSubscribe()) {
    if (!characteristic.subscribe()) {
      LOGF("Failed to subscribe characteristic [%s] for device [%s]! Disconnecting...\n", characteristicUUID, device.deviceName());
      disconnectPeripheral(device, serviceUUID);
      return false;
    }
  }

  LOGF("Found characteristic [%s] on device [%s] and successfully subscribed!\n", characteristicUUID, device.deviceName());
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
  LOGLN("BLE Peripheral advertised!");
}

bool connectToCentral() {
  static unsigned long lastScanMillis = 0;

  unsigned long now = millis();
  if (now - lastScanMillis < scanIntervalMillis) {
    return false;
  }
  lastScanMillis = now;

  LOGLN("Waiting for BLE central to connect...");

  BLECentral = BLEDevice();
  BLECentral = BLE.central();
  if (!(BLECentral && BLECentral.connected())) {
    return false;
  }

  LOGF("BLE central[%s] connected!\n", BLECentral.address());
  return true;
}

void disconnectPeripheral(BLEDevice& device, const char *serviceUUID) {
  device.disconnect();
  // Once we disconnect, clear the “discovered” flag for that UUID:
  setAttributesDiscovered(serviceUUID, false);
}
