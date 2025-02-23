#include <Arduino.h>

#include "LED_controller.hpp"

#define STATE_NOT_READY 0
#define STATE_SENSOR_READY 1
#define STATE_REMOTE_CONTROLLER_READY 2

#define DRIVE_MODE_MANUAL 0
#define DRIVE_MODE_AUTO_FOLLOW 1

// Private helper: setup a single pin
void LEDController::setupLEDPIN(int pin) {
  if (pin < 0) {
    return;
  }
  pinMode(pin, OUTPUT);
  digitalWrite(pin, HIGH);
}

// Private helper: set LED to red (common anode: LOW = on)
void LEDController::setLEDRed(int red_pin, int green_pin, int blue_pin) {
  if (red_pin > 0) {
    digitalWrite(red_pin, LOW);   // Red on
  }
  if (green_pin > 0) {
    digitalWrite(green_pin, HIGH);  // Green off
  }
  if (blue_pin > 0) {
    digitalWrite(blue_pin, HIGH);   // Blue off
  }
}

// Private helper: set LED to green (common anode: LOW = on)
void LEDController::setLEDGreen(int red_pin, int green_pin, int blue_pin) {
  if (red_pin > 0) {
    digitalWrite(red_pin, HIGH);    // Red off
  }
  if (green_pin > 0) {
    digitalWrite(green_pin, LOW);     // Green on
  }
  if (blue_pin > 0) {
    digitalWrite(blue_pin, HIGH);     // Blue off
  }
}

// Private helper: set LED to blue (optional)
void LEDController::setLEDBlue(int red_pin, int green_pin, int blue_pin) {
  if (red_pin > 0) {
    digitalWrite(red_pin, HIGH);
  }
  if (green_pin > 0) {
    digitalWrite(green_pin, HIGH);
  }
  if (blue_pin > 0) {
    digitalWrite(blue_pin, LOW);
  }
}

// Constructor
LEDController::LEDController(int sensorRed, int sensorGreen, int sensorBlue,
                             int remoteRed, int remoteGreen, int remoteBlue,
                             int driveRed, int driveGreen, int driveBlue)
  : sensorR(sensorRed), sensorG(sensorGreen), sensorB(sensorBlue),
    remoteR(remoteRed), remoteG(remoteGreen), remoteB(remoteBlue),
    driveR(driveRed), driveG(driveGreen), driveB(driveBlue)
{ }

// Setup all LED pins by configuring them as outputs and set the initial state.
void LEDController::setupLED(int state, int driveMode) {
  // Sensor LED pins
  setupLEDPIN(sensorR);
  setupLEDPIN(sensorG);
  setupLEDPIN(sensorB);
  // Remote Controller LED pins
  setupLEDPIN(remoteR);
  setupLEDPIN(remoteG);
  setupLEDPIN(remoteB);
  // Drive Mode LED pins
  setupLEDPIN(driveR);
  setupLEDPIN(driveG);
  setupLEDPIN(driveB);

  updateStateLED(state);
  updateDriveModeLED(driveMode);
}

// Update LEDs based on state.
void LEDController::updateStateLED(int state) {
  // Sensor LED update
  if (state & STATE_SENSOR_READY) {
    Serial.println("sensor, green");
    setLEDGreen(sensorR, sensorG, sensorB);
  } else {
    Serial.println("sensor, red");
    setLEDRed(sensorR, sensorG, sensorB);
  }

  // Remote Controller LED update
  if (state & STATE_REMOTE_CONTROLLER_READY) {
    Serial.println("remote, green");
    setLEDGreen(remoteR, remoteG, remoteB);
  } else {
    Serial.println("remote, red");
    setLEDRed(remoteR, remoteG, remoteB);
  }
}

// Update LEDs based on drive mode.
void LEDController::updateDriveModeLED(int driveMode) {
  if (driveMode == DRIVE_MODE_MANUAL) {
    Serial.println("drive mode, red");
    setLEDRed(driveR, driveG, driveB);
  } else {
    Serial.println("drive mode, green");
    setLEDGreen(driveR, driveG, driveB);
  }
}
