#include <Arduino.h>

#include "LED_controller.hpp"
#include "util.h"

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
  LOGF("Measured Voltage: %d mV\n", measuredMilliVoltage);
  // Scale measured voltage back up to actual battery voltage
  float voltage = measuredMilliVoltage * batteryDividerFactor / 1000;
  // Debug: print the measured voltage
  LOGF("Calculated Voltage: %f V\n", voltage);

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
    batteryDividerFactor = (float)(R1 + R2) / R2;  // (220k+100k)/100k = 3.2
  }

  setupLEDPIN(batteryR);
  setupLEDPIN(batteryG);
  setupLEDPIN(batteryB);
}

// Update LEDs based on state.
void LEDController::updateStateLED(uint8_t state) {
  // Sensor LED update
  if (state & SERVER_STATE_SENSOR_READY) {
    LOGLN("sensor, green");
    setLEDGreen(sensorR, sensorG, sensorB);
  } else {
    LOGLN("sensor, red");
    setLEDRed(sensorR, sensorG, sensorB);
  }

  // Remote Controller LED update
  if (state & SERVER_STATE_REMOTE_READY) {
    LOGLN("remote, green");
    setLEDGreen(remoteR, remoteG, remoteB);
  } else {
    LOGLN("remote, red");
    setLEDRed(remoteR, remoteG, remoteB);
  }
}

// This needs to be called in every loop to have the "blinking" effect.
void LEDController::updateDriveModeLED(uint8_t driveMode) {
  static unsigned long lastToggleTime = millis();
  static int stateIndex = 0;  // state machine index for blinking pattern
  static int prevDriveMode = -1;  // to detect mode changes
  unsigned long now = millis();

  // Reset the state machine if the drive mode changes.
  if (driveMode != prevDriveMode) {
    stateIndex = 0;
    lastToggleTime = now;
    prevDriveMode = driveMode;
  }

  switch (driveMode) {
    case DRIVE_MODE_MANUAL: {
      // Steady
      setLEDBlue(driveR, driveG, driveB);
      break;
    }
    case DRIVE_MODE_FOLLOW: {
      // 2 long blinks pattern:
      // State 0: LED ON for 1500ms
      // State 1: LED OFF for 1500ms
      const unsigned long longOn = 1500;
      const unsigned long longOff = 1500;
      if (stateIndex == 0) {
        setLEDBlue(driveR, driveG, driveB);
        if (now - lastToggleTime >= longOn) {
          stateIndex = 1;
          lastToggleTime = now;
        }
      } else if (stateIndex == 1) {
        setLEDOff(driveR, driveG, driveB);
        if (now - lastToggleTime >= longOff) {
          stateIndex = 0;
          lastToggleTime = now;
        }
      }
      break;
    }

    case DRIVE_MODE_CINEMA: {
      // 2 short blinks pattern:
      // State 0: LED ON for 500ms
      // State 1: LED OFF for 500ms
      // State 2: LED ON for 500ms
      // State 3: LED OFF for 1500ms (pause)
      const unsigned long shortOn = 500;
      const unsigned long shortOff = 500;
      const unsigned long pause = 1500;
      if (stateIndex == 0) {
        setLEDBlue(driveR, driveG, driveB);
        if (now - lastToggleTime >= shortOn) {
          stateIndex = 1;
          lastToggleTime = now;
        }
      } else if (stateIndex == 1) {
        setLEDOff(driveR, driveG, driveB);
        if (now - lastToggleTime >= shortOff) {
          stateIndex = 2;
          lastToggleTime = now;
        }
      } else if (stateIndex == 2) {
        setLEDBlue(driveR, driveG, driveB);
        if (now - lastToggleTime >= shortOn) {
          stateIndex = 3;
          lastToggleTime = now;
        }
      } else if (stateIndex == 3) {
        setLEDOff(driveR, driveG, driveB);
        if (now - lastToggleTime >= pause) {
          stateIndex = 0;
          lastToggleTime = now;
        }
      }
      break;
    }

    default:
      // For any unrecognized drive mode, turn off the LED.
      setLEDOff(driveR, driveG, driveB);
      stateIndex = 0;
      break;
  }
}

// This needs to be called in every loop to have the "blinking" effect.
void LEDController::updateUWBSelectorLED(uint8_t uwbSelector) {
  static unsigned long lastToggleTime = millis();
  static int stateIndex = 0;  // state machine index for blinking pattern
  static int prevUwbSelector = -1;  // to detect uwb selector changes
  unsigned long now = millis();

  // Reset the state machine if the uwb selector changes.
  if (uwbSelector != prevUwbSelector) {
    stateIndex = 0;
    lastToggleTime = now;
    prevUwbSelector = uwbSelector;
  }

  switch (uwbSelector) {
    case 0: {
      // Steady
      setLEDBlue(uwbSelectorR, uwbSelectorG, uwbSelectorB);
      break;
    }
    case 1: {
      // 1 long blinks pattern:
      // State 0: LED ON for 1500ms
      // State 1: LED OFF for 1500ms
      const unsigned long longOn = 1500;
      const unsigned long longOff = 1500;
      if (stateIndex == 0) {
        setLEDBlue(uwbSelectorR, uwbSelectorG, uwbSelectorB);
        if (now - lastToggleTime >= longOn) {
          stateIndex = 1;
          lastToggleTime = now;
        }
      } else if (stateIndex == 1) {
        setLEDOff(uwbSelectorR, uwbSelectorG, uwbSelectorB);
        if (now - lastToggleTime >= longOff) {
          stateIndex = 0;
          lastToggleTime = now;
        }
      }
      break;
    }

    case 2: {
      // 2 short blinks pattern:
      // State 0: LED ON for 500ms
      // State 1: LED OFF for 500ms
      // State 2: LED ON for 500ms
      // State 3: LED OFF for 1500ms (pause)
      const unsigned long shortOn = 500;
      const unsigned long shortOff = 500;
      const unsigned long pause = 1500;
      if (stateIndex == 0) {
        setLEDBlue(uwbSelectorR, uwbSelectorG, uwbSelectorB);
        if (now - lastToggleTime >= shortOn) {
          stateIndex = 1;
          lastToggleTime = now;
        }
      } else if (stateIndex == 1) {
        setLEDOff(uwbSelectorR, uwbSelectorG, uwbSelectorB);
        if (now - lastToggleTime >= shortOff) {
          stateIndex = 2;
          lastToggleTime = now;
        }
      } else if (stateIndex == 2) {
        setLEDBlue(uwbSelectorR, uwbSelectorG, uwbSelectorB);
        if (now - lastToggleTime >= shortOn) {
          stateIndex = 3;
          lastToggleTime = now;
        }
      } else if (stateIndex == 3) {
        setLEDOff(uwbSelectorR, uwbSelectorG, uwbSelectorB);
        if (now - lastToggleTime >= pause) {
          stateIndex = 0;
          lastToggleTime = now;
        }
      }
      break;
    }

    default:
      // For any unrecognized uwbSelector, turn off the LED.
      setLEDOff(uwbSelectorR, uwbSelectorG, uwbSelectorB);
      stateIndex = 0;
      break;
  }
}

// Update battery LED based on battery percentage thresholds.
void LEDController::updateBatteryLED() {
  int batteryPercentage = readBatteryPercentage();
  //LOGF("Battery percentage: %d\n", batteryPercentage);

  if (batteryPercentage >= 50) {
    //LOGLN("battery, green");
    setLEDGreen(batteryR, batteryG, batteryB);
  }
  else if (batteryPercentage >= 10) {  // 10% to 24%
    //LOGLN("battery, yellow");
    setLEDYellow(batteryR, batteryG, batteryB);
  }
  else {  // Less than 10%
    //LOGLN("battery, red");
    setLEDRed(batteryR, batteryG, batteryB);
  }
}
