#ifndef CONSTANTS_H_
#define CONSTANTS_H_

// Constants share by different components.
#define PI 3.14159265359

#define DATA_RATE 50 // The magic number.
#define LOOP_PERIOD_MS (1000 / DATA_RATE) // Convert DATA_RATE to millis

// Server state.
#define SERVER_STATE_NOT_READY 0
#define SERVER_STATE_SENSOR_READY 1
#define SERVER_STATE_REMOTE_READY 2

// Drive mode.
#define DRIVE_MODE_MANUAL 0
#define DRIVE_MODE_FOLLOW 1
#define DRIVE_MODE_CINEMA 2

// Gimbal fn button state.
#define ACTIVE_TRACK_TOGGLED 1
#define GIMBAL_RECENTER_TOGGLED 2
#define CAMERA_RECORDING_TOGGLED 4

// Sensor specific state.
#define SENSOR_STATE_NOT_READY 0
#define SENSOR_STATE_TAG_CONNECTED 1
#define SENSOR_STATE_SERVER_CONNECTED 2

#endif // CONSTANTS_H_
