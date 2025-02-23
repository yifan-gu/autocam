#ifndef LED_CONTROLLER_HPP
#define LED_CONTROLLER_HPP

class LEDController {
private:
  // Sensor LED pins
  int sensorR, sensorG, sensorB;
  // Remote Controller LED pins
  int remoteR, remoteG, remoteB;
  // Drive Mode LED pins
  int driveR, driveG, driveB;

  // Private helper functions (implemented in LEDController.cpp)
  void setupLEDPIN(int pin);
  void setLEDRed(int red_pin, int green_pin, int blue_pin);
  void setLEDGreen(int red_pin, int green_pin, int blue_pin);
  void setLEDBlue(int red_pin, int green_pin, int blue_pin); // Optional, if needed

public:
  // Constructor: initialize with RGB pin numbers for sensor, remote, and drive mode LEDs
  LEDController(int sensorRed, int sensorGreen, int sensorBlue,
                int remoteRed, int remoteGreen, int remoteBlue,
                int driveRed, int driveGreen, int driveBlue);

  // Setup all LED pins by configuring them as outputs and set the initial state.
  void setupLED(int state, int driveMode);
  // Update LEDs based on state.
  void updateStateLED(int state);
  // Update LEDs based on drive mode.
  void updateDriveModeLED(int driveMode);
};

#endif // LED_CONTROLLER_HPP
