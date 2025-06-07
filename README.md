Arduino files for my auto-following camera car!


## TODO
- [x] Add dji remote control and receiver
- [x] <s>Add PID encoder</s> Add a web page for parameters
- [x] Add BLE central for the anchor
- [x] Read joystick values for the remote controller
- [x] Read mode switch button for remote controller
- [x] Add state indicator in the web UI
- [x] Add LED state indicators
- [x] Add battery state LED (tag, remote)
- [x] Add LED logic for tag
- [x] Add lock mode (remote)
- [x] Combine tag and remote
  - [x] Add tag selector (sensor, remote)
- [x] Control the BLE data rate
- [x] Fix UWB Anchor naming, fix AutocamController naming
- [x] Add PCB boards
  - [x] Sensor
  - [x] Verify LED resistor values
  - [x] Verify IO0/IO1 port on the ESP32 UWB is usable 
  - [x] Tag
  - [x] Server
  - [x] Remote
- [x] Smooth the distance measurement.
- [x] Added a different mode (follow).
- [x] Added gimbal recenter and camera recording.
- [x] Assemble together
- [x] Design and make the enclosure
- [ ] Calibrate the UWB tag and anchor
- [ ] Measure battery voltage.
- [ ] (optimize) Store settings in external storage (SD card)
- [x] (optimize) Refactor the server code with a state machine, (Almost a state machine now).
- [ ] (optimize) Refactor the sensor LED control with LED controller
- [ ] (optimize) Reorganize shared code into libraries, add external dependencies 

- [x] heartbeat for the remote controller.
- [ ] <s> Nice to have, detect active track failure</s> Turns out there's no API support for that
- [x] Smart Phone APP for the remote (BLE version).

## KNOWN ISSUES
- [ ] Webserver stabability.
- [x] Steering thread issue (Do not steer before throttle is applied)
- [x] BLE connection issue
- [ ] ESC init issue
<s>- [x] Start/stop recording issue</s> Not actually an issue—I plugged the USB-C cable into the wrong gimbal port. However, since the camera drains the gimbal’s battery, I still remove the cable to conserve power.
- [x] Every time the remote buttons are pressed, send values to the server.

## 🚗 Simulator

👉 [Run the Simulator](https://yifan-gu.github.io/autocam/simulator.html)
