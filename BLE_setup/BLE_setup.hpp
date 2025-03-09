#ifndef BLE_SETUP_HPP
#define BLE_SETUP_HPP

#include <ArduinoBLE.h>

extern BLEDevice BLECentral;
extern BLEDevice AutocamSensor, AutocamRemote;
extern BLEService AutocamSensorService, AutocamRemoteService;
extern BLECharacteristic AutocamSensorDataSend, AutocamSensorDataRecv, AutocamRemoteDataBidirection;

extern const char *AutocamSensorServiceUUID;
extern const char *AutocamRemoteServiceUUID;

extern const char *AutocamSensorDataSendCharacteristicUUID;
extern const char *AutocamSensorDataRecvCharacteristicUUID;
extern const char *AutocamRemoteDataBidirectionCharacteristicUUID;

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

struct RemoteDataBidirection {
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
