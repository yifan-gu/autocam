#include "DJI_ronin_controller.hpp"

#define CAN_TX   12  // Connects to CTX
#define CAN_RX   2  // Connects to CRX
#define CAN_RATE 1000 // Speed rate in kbps

DJIRoninController djiRoninController(CAN_TX, CAN_RX, CAN_RATE);

void setup() {
  // Set up serial for debugging
  Serial.begin(115200);
  delay(1000);

  if (djiRoninController.begin()) { //CAN_TX, CAN_RX, CAN_RATE)) {
    Serial.println("CAN bus started!");
  }
}

void loop() {
  if (!djiRoninController.set_position(1, 0, 0, 0, 100)) {
    Serial.println("Failed to move incrementally");
  }
}
