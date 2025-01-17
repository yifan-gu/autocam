#include "DJI_ronin_controller.hpp"

#define CAN_TX   12  // Connects to CTX
#define CAN_RX   2  // Connects to CRX
#define CAN_RATE 1000 // Speed rate in kbps

float yaw = 0;
float roll = 0;
float pitch = 0;

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
  //if (!djiRoninController.set_position(1, 0, 0, 0, 100)) {
  //  Serial.println("Failed to move incrementally");
  //}

  if (!djiRoninController.get_position(&yaw, &roll, &pitch)) {
    Serial.println("Failed to get position");
  }
  Serial.printf("yaw=%.1f, roll=%.1f, pitch=%.1f\n", yaw, roll, pitch);
  //delay(100);
}
