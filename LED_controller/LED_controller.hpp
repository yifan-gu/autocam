#ifndef LED_CONTROLLER_HPP
#define LED_CONTROLLER_HPP

class LEDController {
private:
  // Sensor LED pins
  int sensorR = -1;
  int sensorG = -1;
  int sensorB = -1;
  // Remote Controller LED pins
  int remoteR = -1;
  int remoteG = -1;
  int remoteB = -1;
  // Drive Mode LED pins
  int driveR = -1;
  int driveG = -1;
  int driveB = -1;
  // Battery LED pins
  int batteryR = -1;
  int batteryG = -1;
  int batteryB = -1;

  int batteryADCPin = -1;
  float batteryMinVoltage = 0;
  float batteryMaxVoltage = 0;

  int batteryVoltageDividerR1 = 0;
  int batteryVoltageDividerR2 = 0;
  float batteryDividerFactor = 0;

  // Private helper functions (implemented in LEDController.cpp)
  void setupLEDPIN(int pin);
  int readBatteryPercentage();

public:
  LEDController() {};

  void setLEDRed(int red_pin, int green_pin, int blue_pin);
  void setLEDGreen(int red_pin, int green_pin, int blue_pin);
  void setLEDBlue(int red_pin, int green_pin, int blue_pin); // Optional, if needed
  void setLEDYellow(int red_pin, int green_pin, int blue_pin);

  void initSensorLED(int sensorRed, int sensorGreen, int sensorBlue);
  void initRemoteLED(int remoteRed, int remoteGreen, int remoteBlue);
  void initDriveLED(int driveRed, int driveGreen, int driveBlue);
  void initBatteryLED(int batteryRed, int batteryGreen, int batteryBlue, int battery_adc_pin, float battery_min_v, float battery_max_v, int R1 = 220000, int R2 = 100000);
  // Update LEDs based on state.
  void updateStateLED(int state);
  // Update LEDs based on drive mode.
  void updateDriveModeLED(int driveMode);
  // Update battery LED based on battery percentage thresholds.
  void updateBatteryLED();
};

#endif // LED_CONTROLLER_HPP
