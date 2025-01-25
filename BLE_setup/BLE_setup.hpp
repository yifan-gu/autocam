#ifndef BLE_SETUP_HPP
#define BLE_SETUP_HPP

#include <ArduinoBLE.h>

extern BLEDevice BLECentral;
extern BLEDevice UWBAnchor, AutocamController;
extern BLEService UWBAnchorService, AutocamControllerService;
extern BLECharacteristic UWBAnchorSensorData, AutocamControllerData;

extern const char *UWBAnchorServiceUUID;
extern const char *AutocamControllerServiceUUID;

extern const char *UWBAnchorSensorDataCharacteristicUUID;
extern const char *AutocamControllerControllerDataCharacteristicUUID;

struct SensorData {
  float distance;
  float heading;
  int state; // 1 = ready, 0 = not ready.

  // Values below are written by the central (server).
  float yaw_speed;
  float pitch_speed;
  int active_track_toggled;
};

struct ControllerData {
  int throttleValue;
  int steeringValue;
  int driveMode;
  float yaw_speed;
  float pitch_speed;
  int active_track_toggled;

  int state; // This value is written by the central (server).
};

void setupBLECentral();
bool connectToBLEDevice(BLEDevice &device, BLEService &service, BLECharacteristic &characteristic, const char *serviceUUID, const char *characteristicUUID);
bool scanForPeripheral(BLEDevice &device, BLEService &service, BLECharacteristic &characteristic, const char *serviceUUID, const char *characteristicUUID);

void setupBLEPeripheral(const char *localName, const char *deviceName, BLEService& service, BLECharacteristic& characteristic, uint8_t *data, int data_length);
bool connectToCentral();

#endif
