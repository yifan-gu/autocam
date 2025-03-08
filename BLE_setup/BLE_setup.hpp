#ifndef BLE_SETUP_HPP
#define BLE_SETUP_HPP

#include <ArduinoBLE.h>

extern BLEDevice BLECentral;
extern BLEDevice UWBAnchor, AutocamController;
extern BLEService UWBAnchorService, AutocamControllerService;
extern BLECharacteristic UWBAnchorSensorDataSend, UWBAnchorSensorDataRecv, AutocamControllerData;

extern const char *UWBAnchorServiceUUID;
extern const char *AutocamControllerServiceUUID;

extern const char *UWBAnchorSensorDataSendCharacteristicUUID;
extern const char *UWBAnchorSensorDataRecvCharacteristicUUID;
extern const char *AutocamControllerControllerDataCharacteristicUUID;

struct SensorDataSend {
  float distance;
  float heading;
  int state; // 1 = ready, 0 = not ready.
};

struct SensorDataRecv {
  // Values below are written by the central (server).
  float yawSpeed;
  float pitchSpeed;
  bool activeTrackToggled;
  uint16_t uwbSelector;
};

struct ControllerData {
  int throttleValue;
  int steeringValue;
  int driveMode;
  float yawSpeed;
  float pitchSpeed;
  bool activeTrackToggled;
  uint16_t uwbSelector;

  int state; // This value is written by the central (server).
};

void setupBLECentral();
bool connectToBLEDevice(BLEDevice &device, BLEService &service, BLECharacteristic &characteristic, const char *serviceUUID, const char *characteristicUUID);
bool scanForPeripheral(BLEDevice &device, BLEService &service, BLECharacteristic &characteristic, const char *serviceUUID, const char *characteristicUUID);

void setupBLEPeripheral(const char *localName, const char *deviceName, BLEService& service);
bool connectToCentral();

#endif
