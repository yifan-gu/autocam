#include "BLE_setup.hpp"
#include "util.h"

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

void panic(const char *message) {
  LOGLN(message);
  while (1) {};
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

  unsigned long currentMillis = millis();
  if (currentMillis - lastScanMillis < scanIntervalMillis ) {
    return false;
  }
  lastScanMillis = currentMillis;

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
  return connectToBLEDevice(device, service, characteristic, serviceUUID, characteristicUUID);
}

bool connectToBLEDevice(BLEDevice &device, BLEService &service, BLECharacteristic &characteristic, const char *serviceUUID, const char *characteristicUUID) {
  if (!device.connected()) {
    if (!device.connect()) {
      LOGF("Failed to connect to [%s].\n", device.localName());
      return false;
    }
  }

  LOGF("Connected to [%s]!\n", device.localName());
  if (!device.discoverAttributes()) {
    LOGF("Attribute discovery failed for [%s]! Disconnecting...\n", device.localName());
    device.disconnect();
    return false;
  }

  service = device.service(serviceUUID);
  LOGF("BLE device name: [%s], advertised svc count: [%d], Real svc count: [%d], characteristics count: [%d]\n", device.deviceName(), device.advertisedServiceUuidCount(), device.serviceCount(), device.characteristicCount());
  characteristic = service.characteristic(characteristicUUID);

  if (!characteristic) {
    LOGF("Failed to find characteristic [%s] for device [%s]! Disconnecting...\n", characteristicUUID, device.deviceName());
    device.disconnect();
    return false;
  }

  if (characteristic.canSubscribe()) {
    if (!characteristic.subscribe()) {
      LOGF("Failed to subscribe characteristic [%s] for device [%s]! Disconnecting...\n", characteristicUUID, device.deviceName());
      device.disconnect();
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

  unsigned long currentMillis = millis();
  if (currentMillis - lastScanMillis < scanIntervalMillis) {
    return false;
  }
  lastScanMillis = currentMillis;

  LOGLN("Waiting for BLE central to connect...");

  BLECentral = BLE.central();
  if (!(BLECentral && BLECentral.connected())) {
    return false;
  }

  LOGF("BLE central[%s] connected!\n", BLECentral.address());
  return true;
}
