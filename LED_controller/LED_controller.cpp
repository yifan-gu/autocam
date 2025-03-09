#include <Arduino.h>

#include "LED_controller.hpp"

#define STATE_NOT_READY 0
#define STATE_SENSOR_READY 1
#define STATE_REMOTE_CONTROLLER_READY 2

#define DRIVE_MODE_MANUAL 0
#define DRIVE_MODE_AUTO_FOLLOW 1

// Calculate the divider factor (inverse of the ratio)

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

// Private helper: set LED to yellow (red and green on, blue off)
void LEDController::setLEDYellow(int red_pin, int green_pin, int blue_pin) {
  if (red_pin > 0) {
    digitalWrite(red_pin, LOW);   // Red on
  }
  if (green_pin > 0) {
    digitalWrite(green_pin, LOW);   // Green on
  }
  if (blue_pin > 0) {
    digitalWrite(blue_pin, HIGH);   // Blue off
  }
}

// Private helper: set LED to yellow (all LEDs off).
void LEDController::setLEDOff(int red_pin, int green_pin, int blue_pin) {
  if (red_pin > 0) {
    digitalWrite(red_pin, HIGH);   // Red off
  }
  if (green_pin > 0) {
    digitalWrite(green_pin, HIGH);   // Green off
  }
  if (blue_pin > 0) {
    digitalWrite(blue_pin, HIGH);   // Blue off
  }
}

// Private helper: read the battery percentage from the ESP32 ADC
int LEDController::readBatteryPercentage() {
  int measuredMilliVoltage = analogReadMilliVolts(batteryADCPin);
  // Scale measured voltage back up to actual battery voltage
  float voltage = measuredMilliVoltage * batteryDividerFactor / 1000;
  // Debug: print the measured voltage
  Serial.printf("Measured battery voltage: %f\n", voltage);

  // Map voltage to battery percentage.
  if (voltage >= batteryMaxVoltage) return 100;
  if (voltage <= batteryMinVoltage) return 0;
  return (int)(((voltage - batteryMinVoltage) / (batteryMaxVoltage - batteryMinVoltage)) * 100);
}

// Sensor LED initialization
void LEDController::initSensorLED(int sensorRed, int sensorGreen, int sensorBlue) {
  sensorR = sensorRed;
  sensorG = sensorGreen;
  sensorB = sensorBlue;

  setupLEDPIN(sensorR);
  setupLEDPIN(sensorG);
  setupLEDPIN(sensorB);
}

// Remote LED initialization
void LEDController::initRemoteLED(int remoteRed, int remoteGreen, int remoteBlue) {
  remoteR = remoteRed;
  remoteG = remoteGreen;
  remoteB = remoteBlue;

  setupLEDPIN(remoteR);
  setupLEDPIN(remoteG);
  setupLEDPIN(remoteB);
}

// Drive LED initialization
void LEDController::initDriveLED(int driveRed, int driveGreen, int driveBlue) {
  driveR = driveRed;
  driveG = driveGreen;
  driveB = driveBlue;

  setupLEDPIN(driveR);
  setupLEDPIN(driveG);
  setupLEDPIN(driveB);
}

// UWB selector LED initialization
void LEDController::initUWBSelectorLED(int uwbSelectorRed, int uwbSelectorGreen, int uwbSelectorBlue) {
  uwbSelectorR = uwbSelectorRed;
  uwbSelectorG = uwbSelectorGreen;
  uwbSelectorB = uwbSelectorBlue;

  setupLEDPIN(uwbSelectorR);
  setupLEDPIN(uwbSelectorG);
  setupLEDPIN(uwbSelectorB);
}

// Battery LED initialization
void LEDController::initBatteryLED(int batteryRed, int batteryGreen, int batteryBlue, int battery_adc_pin, float battery_min_v, float battery_max_v, int R1, int R2) {
  batteryR = batteryRed;
  batteryG = batteryGreen;
  batteryB = batteryBlue;

  batteryADCPin = battery_adc_pin;
  batteryMinVoltage = battery_min_v;
  batteryMaxVoltage = battery_max_v;
  batteryVoltageDividerR1 = R1;
  batteryVoltageDividerR2 = R2;
  if (R2 != 0 ) {
    batteryDividerFactor = (float)(R1 + R2) / R2;  // (100k+220k)/100k = 3.2
  }

  setupLEDPIN(batteryR);
  setupLEDPIN(batteryG);
  setupLEDPIN(batteryB);
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
    Serial.println("drive mode, off");
    setLEDRed(driveR, driveG, driveB);
  } else {
    Serial.println("drive mode, blue");
    setLEDBlue(driveR, driveG, driveB);
  }
}

// Update LEDs based on UWB Selector
void LEDController::updateUWBSelectorLED(uint16_t uwbSelector) {
  if (uwbSelector == 1) {
    Serial.println("UWB selector blue");
    setLEDBlue(uwbSelectorR, uwbSelectorG, uwbSelectorB);
  } else {
    Serial.println("UWB selector off");
    setLEDOff(uwbSelectorR, uwbSelectorG, uwbSelectorB);
  }
}

// Update battery LED based on battery percentage thresholds.
void LEDController::updateBatteryLED() {
  int batteryPercentage = readBatteryPercentage();
  Serial.print("Battery percentage: ");
  Serial.println(batteryPercentage);

  if (batteryPercentage >= 50) {
    Serial.println("battery, green");
    setLEDGreen(batteryR, batteryG, batteryB);
  }
  else if (batteryPercentage >= 10) {  // 10% to 24%
    Serial.println("battery, yellow");
    setLEDYellow(batteryR, batteryG, batteryB);
  }
  else {  // Less than 10%
    Serial.println("battery, red");
    setLEDRed(batteryR, batteryG, batteryB);
  }
}
