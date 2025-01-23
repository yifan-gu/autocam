#ifndef BLE_SETUP_HPP
#define BLE_SETUP_HPP

#include <ArduinoBLE.h>

extern BLEDevice BLECentral;

void setupBLECentral();
bool connectToBLEDevice(BLEDevice &device, BLEService &service, BLECharacteristic &characteristic, const char *serviceUUID, const char *characteristicUUID);
bool scanForPeripheral(BLEDevice &device, BLEService &service, BLECharacteristic &characteristic, const char *serviceUUID, const char *characteristicUUID);

void setupBLEPeripheral(const char *localName, const char *deviceName, BLEService& service, BLECharacteristic& characteristic, uint8_t *data, int data_length);
bool connectToCentral();

#endif
