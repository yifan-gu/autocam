#ifndef BLE_SETUP_HPP
#define BLE_SETUP_HPP

#include <ArduinoBLE.h>

extern BLEDevice BLECentral;
extern BLEDevice UWBAnchor, AutocamController, GimbalController;
extern BLEService UWBAnchorService, AutocamControllerService, GimbalControllerService;
extern BLECharacteristic UWBAnchorSensorData, AutocamControllerData, GimbalControllerData;

extern const char *UWBAnchorServiceUUID;
extern const char *AutocamControllerServiceUUID;
extern const char *GimbalControllerServiceUUID;

extern const char *UWBAnchorSensorDataCharacteristicUUID;
extern const char *AutocamControllerControllerDataCharacteristicUUID;
extern const char *GimbalControllerDataCharacteristicUUID;

struct SensorData {
  float distance;
  float heading;
  int state; // 1 = ready, 0 = not ready.
};

struct ControllerData {
  int throttleValue;
  int steeringValue;
  int driveMode;
  int state;
  float yaw_speed;
  float pitch_speed;
  int active_track_toggled;
};

struct GimbalControllerDataData {
  float yaw_speed;
  float pitch_speed;
  bool active_track_toggled;
};

void setupBLECentral();
bool connectToBLEDevice(BLEDevice &device, BLEService &service, BLECharacteristic &characteristic, const char *serviceUUID, const char *characteristicUUID);
bool scanForPeripheral(BLEDevice &device, BLEService &service, BLECharacteristic &characteristic, const char *serviceUUID, const char *characteristicUUID);

void setupBLEPeripheral(const char *localName, const char *deviceName, BLEService& service, BLECharacteristic& characteristic, uint8_t *data, int data_length);
bool connectToCentral();

#endif
