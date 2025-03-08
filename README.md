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
- [x] Add tag selector (sensor, remote)
- [ ] Control the BLE data rate
- [ ] Fix UWB Anchor naming, fix GimbalController naming
- [x] Add PCB boards
  - [x] Sensor
  - [x] Verify LED resistor values
  - [x] Verify IO0/IO1 port on the ESP32 UWB is usable 
  - [x] Tag
  - [x] Server
  - [x] Remote
- [ ] Assemble together
- [ ] Calibrate the UWB tag and anchor
- [ ] Measure battery voltage.
- [ ] Smooth the distance measurement.
- [ ] (optimize) Store settings in external storage (SD card)
- [ ] (optimize) Refactor the server code with a state machine
- [ ] (optimize) Refactor the sensor LED control with LED controller
- [ ] (optimize) Reorganize shared code into libraries, add external dependencies 

- [ ] <s> Nice to have, detect active track failure</s> Turns out there's no API support for that
- [ ] <s> Combine tag and remote (discarded)</s>
