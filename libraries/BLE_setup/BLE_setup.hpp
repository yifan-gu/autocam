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

// Size = 12 bytes.
struct SensorDataSend {
  float distance;
  float heading;
  uint8_t state; // 1 = ready, 0 = not ready.
  uint8_t padding[3];
};

// Size = 12 bytes.
struct SensorDataRecv {
  // Values below are written by the central (server).
  float yawSpeed;
  float pitchSpeed;
  uint8_t toggleState; // Bit 0 = activeTrackToggled, bit 1 = gimbalRecenterToggled, bit 2 = cameraRecordingToggled.activeTrackToggled;
  uint8_t uwbSelector;
  uint8_t padding[2];
};

// Size = 16 bytes.
struct RemoteDataSend {
  int16_t throttleValue;
  int16_t steeringValue;
  float yawSpeed;
  float pitchSpeed;
  uint8_t driveMode;
  uint8_t toggleState;
  uint8_t uwbSelector;
  uint8_t padding;
};

// This ensures the iOS and ESP32 agree on byte order and length.
static_assert(sizeof(RemoteDataSend) == 16, "Size mismatch");

// Size = 4 bytes.
struct RemoteDataRecv {
  uint8_t state;
  uint8_t driveMode;
  uint8_t uwbSelector;
  uint8_t padding;
};

// This ensures the iOS and ESP32 agree on byte order and length.
static_assert(sizeof(RemoteDataRecv) == 4, "Size mismatch");

void setupBLECentral();
bool connectToBLEDevice(BLEDevice &device, BLEService &service, BLECharacteristic &characteristic, const char *serviceUUID, const char *characteristicUUID);
bool scanForPeripheral(BLEDevice &device, BLEService &service, BLECharacteristic &characteristic, const char *serviceUUID, const char *characteristicUUID);

void setupBLEPeripheral(const char *localName, const char *deviceName, BLEService& service);
bool connectToCentral();
void disconnectPeripheral(BLEDevice& service, const char *serviceUUID);

#endif
