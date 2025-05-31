#ifndef BLE_SETUP_HPP
#define BLE_SETUP_HPP

#include <ArduinoBLE.h>

extern BLEDevice BLECentral;
extern BLEDevice AutocamSensor, AutocamRemote;
extern BLEService AutocamSensorService, AutocamRemoteService;
extern BLECharacteristic AutocamSensorDataSend, AutocamSensorDataRecv, AutocamRemoteDataSend, AutocamRemoteDataRecv;

extern const char *AutocamSensorServiceUUID;
extern const char *AutocamRemoteServiceUUID;

extern const char *AutocamSensorDataSendCharacteristicUUID;
extern const char *AutocamSensorDataRecvCharacteristicUUID;
extern const char *AutocamRemoteDataSendCharacteristicUUID;
extern const char *AutocamRemoteDataRecvCharacteristicUUID;

struct SensorDataSend {
  float distance;
  float heading;
  int state; // 1 = ready, 0 = not ready.
};

struct SensorDataRecv {
  // Values below are written by the central (server).
  float yawSpeed;
  float pitchSpeed;
  int toggleState; // Bit 0 = activeTrackToggled, bit 1 = gimbalRecenterToggled, bit 2 = cameraRecordingToggled.activeTrackToggled;
  uint16_t uwbSelector;
};

struct RemoteDataSend {
  int throttleValue;
  int steeringValue;
  int driveMode;
  float yawSpeed;
  float pitchSpeed;
  int toggleState; // Bit 0 = activeTrackToggled, bit 1 = gimbalRecenterToggled, bit 2 = cameraRecordingToggled.activeTrackToggled;
  uint16_t uwbSelector;
};

struct RemoteDataRecv {
  int driveMode;
  int toggleState; // Bit 0 = activeTrackToggled, bit 1 = gimbalRecenterToggled, bit 2 = cameraRecordingToggled.activeTrackToggled;
  uint16_t uwbSelector;
  int state;
};


void setupBLECentral();
bool connectToBLEDevice(BLEDevice &device, BLEService &service, BLECharacteristic &characteristic, const char *serviceUUID, const char *characteristicUUID);
bool scanForPeripheral(BLEDevice &device, BLEService &service, BLECharacteristic &characteristic, const char *serviceUUID, const char *characteristicUUID);

void setupBLEPeripheral(const char *localName, const char *deviceName, BLEService& service);
bool connectToCentral();

#endif
