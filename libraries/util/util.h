#ifndef UTIL_H_
#define UTIL_H_

// Uncomment the following line to enable debug prints
#define AUTOCAM_DEBUG

#ifdef AUTOCAM_DEBUG
  #define LOG(...) Serial.print(__VA_ARGS__)
  #define LOGF(...) Serial.printf(__VA_ARGS__)
  #define LOGLN(...) Serial.println(__VA_ARGS__)
#else
  #define LOG(...) ((void)0)
  #define LOGF(...) ((void)0)
  #define LOGLN(...) ((void)0)
#endif

// Constants share by different components.
#define PI 3.14159265359

#define DATA_RATE 30 // The magic number.
#define LOOP_PERIOD_MS (1000 / DATA_RATE) // Convert DATA_RATE to millis

#define UWB_TAG_COUNT 3

// Server state.
#define SERVER_STATE_NOT_READY 0
#define SERVER_STATE_SENSOR_READY 1
#define SERVER_STATE_REMOTE_READY 2

// Drive mode.
#define DRIVE_MODE_MANUAL 0
#define DRIVE_MODE_FOLLOW 1
#define DRIVE_MODE_CINEMA 2

// Gimbal fn button state.
#define NO_GIMBAL_TOGGLE 0
#define ACTIVE_TRACK_TOGGLED 1
#define GIMBAL_RECENTER_TOGGLED 2
#define CAMERA_RECORDING_TOGGLED 3

// Sensor specific state.
#define SENSOR_STATE_NOT_READY 0
#define SENSOR_STATE_TAG_CONNECTED 1
#define SENSOR_STATE_SERVER_CONNECTED 2

#define TAG_STATE_SENSOR_NOT_CONNECTED 0
#define TAG_STATE_SENSOR_CONNECTED 1

#endif // UTIL_H_
