Arduino files for my auto-following camera car!


## TODO
- [x] Add dji remote control and receiver
- [x] <s>Add PID encoder</s> Add a web page for parameters
- [x] Add BLE central for the anchor
- [x] Read joystick values for the remote controller
- [x] Read mode switch button for remote controller
- [x] Add state indicator in the web UI
- [x] Add LED state indicators
- [ ] Add battery state LED (tag, remote)
- [ ] Add lock mode (remote)
- [ ] Add tag selector (tag, remote)
- [ ] Add PCB boards
  - [x] Sensor
  - [x] Verify LED resistor values
  - [ ] Verify IO0/IO1 port on the ESP32 UWB is usable.
  - [x] Tag
  - [x] Server
  - [ ] Remote
- [ ] Assemble together
- [ ] Calibrate the UWB tag and anchor
- [ ] (optimize) Store settings in external storage (SD card)
- [ ] (optimize) Refactor the server code with a state machine
- [ ] (optimize) Reorganize shared code into libraries, add external dependencies 

- [ ] <s> Nice to have, detect active track failure</s> Turns out there's no API support for that
- [ ] <s> Combine tag and remote (discarded)</s>
