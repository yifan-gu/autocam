#include <WiFi.h>
#include <SPI.h>

#include "DW1000Ranging.h"
#include "DW1000.h"

#define PI 3.14159265359

// leftmost two bytes below will become the "short address"
char anchor_addr[] = "80:00:5B:D5:A9:9A:E2:9C";

//calibrated Antenna Delay setting for this anchor
uint16_t Adelay = 16630;

#define SPI_SCK 18
#define SPI_MISO 19
#define SPI_MOSI 23
#define DW_CS 4

// connection pins
const uint8_t PIN_RST = 27; // reset pin
const uint8_t PIN_IRQ = 34; // irq pin
const uint8_t PIN_SS = 4;   // spi select pin

// Replace with your WiFi credentials
const char* ssid = "RC-Controller";
const char* password = "12345678";

// Replace with your server's address and port
const char* serverAddress = "ws://autocam.local:80/ws";
IPAddress serverIP, udpServerIP;

// UDP Configuration
WiFiUDP udpClient;
const char* udpServerDomain = "autocam.local"; // Replace with the receiver's domain
const unsigned int udpServerPort = 12345;

float distance = 0;
float heading = 0;

unsigned int lastPingTime = 0;

void setup() {
  // Start Serial for debugging
  unsigned long startTime = millis();
  unsigned long timeout = 1000; // 1 seconds timeout
                                //
  Serial.begin(115200);
  while (!Serial && (millis() - startTime < timeout)) {
    // Wait for Serial or timeout
  }

  setupUWBAnchor();
  setupWiFi();
}

void loop() {
  DW1000Ranging.loop();
  sendSensorDataSquare();
}

void setupUWBAnchor() {
  //init the configuration
  SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI);
  DW1000Ranging.initCommunication(PIN_RST, PIN_SS, PIN_IRQ); //Reset, CS, IRQ pin

  // set antenna delay for anchors only. Tag is default (16384)
  DW1000.setAntennaDelay(Adelay);

  DW1000Ranging.attachNewRange(newRange);
  DW1000Ranging.attachNewDevice(newDevice);
  DW1000Ranging.attachInactiveDevice(inactiveDevice);

  //start the module as an anchor, do not assign random short address
  DW1000Ranging.startAsAnchor(anchor_addr, DW1000.MODE_LONGDATA_RANGE_LOWPOWER, false);
}

void setupWiFi() {
  // Connect to WiFi
  Serial.printf("Connecting to WiFi: %s\n", ssid);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected!");
  serverIP = WiFi.localIP();
  Serial.printf("IP Address: %s\n", serverIP.toString().c_str());

  if (WiFi.hostByName(udpServerDomain, udpServerIP)) {
    Serial.print("Resolved UDP IP: ");
    Serial.println(udpServerIP);
  } else {
    Serial.println("Failed to resolve domain!");
  }
}

void sendSensorData() {
  // Send `distance` data
  String message = "distance=" + String(distance, 2) + "&heading=" + String(heading, 2);
  sendUDP(message);
}

void sendUDP(String data) {
  udpClient.beginPacket(udpServerIP, udpServerPort);
  udpClient.print(data);
  udpClient.endPacket();
  //Serial.println("Sent via UDP: " + data);
  delay(10); // 100Hz.
  Serial.printf("Data interval: %d(ms)\n", millis() - lastPingTime);
  lastPingTime = millis();
}

void newRange() {
  distance = DW1000Ranging.getDistantDevice()->getRange();
}

void newDevice(DW1000Device *device) {
  Serial.print("Device added: ");
  Serial.println(device->getShortAddress(), HEX);
}

void inactiveDevice(DW1000Device *device) {
  Serial.print("Delete inactive device: ");
  Serial.println(device->getShortAddress(), HEX);
}

/*------------ Dummy data for testing the server */
void sendSensorDataStraightVertical() {
  // Define the waypoints for the square path
  const float waypoints[][2] = {
      {10.0, 10.0},    // Point 1: Top-right
      {10.0, -10.0}   // Point 2: Bottom-right
  };
  sendSensorDataWayPoints(waypoints, sizeof(waypoints) / sizeof(waypoints[0]));
}

/*------------ Dummy data for testing the server */
void sendSensorDataSquare() {
  // Define the waypoints for the square path
  const float waypoints[][2] = {
      {10.0, 10.0},    // Point 1: Top-right
      {-10.0, 10.0},   // Point 2: Top-left
      {-10.0, -10.0},  // Point 3: Bottom-left
      {10.0, -10.0}   // Point 4: Bottom-right
  };
  sendSensorDataWayPoints(waypoints, sizeof(waypoints) / sizeof(waypoints[0]));
}

void sendSensorDataWayPoints(const float waypoints[][2], int waypointCount) {
  static int currentWaypointIndex = 0;  // Index of the current waypoint

  static float posX = 0;  // Current X position
  static float posY = 0;  // Current Y position
  const float stepSize = 0.1;  // Movement per step

  // Get the target waypoint
  float targetX = waypoints[currentWaypointIndex][0];
  float targetY = waypoints[currentWaypointIndex][1];

  // Calculate the direction to the target waypoint
  float deltaX = targetX - posX;
  float deltaY = targetY - posY;
  float distanceToTarget = sqrt(deltaX * deltaX + deltaY * deltaY);

  // If close enough to the target, move to the next waypoint
  if (distanceToTarget < stepSize) {
    posX = targetX;
    posY = targetY;

    currentWaypointIndex++;
    if (currentWaypointIndex >= waypointCount) {
      currentWaypointIndex = 0;  // Loop back to the first waypoint
    }
  } else {
    // Move toward the target waypoint
    float stepFactor = stepSize / distanceToTarget;
    posX += deltaX * stepFactor;
    posY += deltaY * stepFactor;
  }

  // Calculate distance and heading from (0, 0) to the current position
  float distance = sqrt(posX * posX + posY * posY);  // Distance from origin
  float heading = atan2(posY, posX) * (180.0 / PI) - 90;  // Heading in degrees. -90 since the heading is calculated from the N direction.
  if (heading < 0) {
    heading += 360;
  }

  // Send `distance` and `heading` via UDP
  String message = "distance=" + String(distance, 2) + "&heading=" + String(heading, 2);
  sendUDP(message);

  // Print debugging information
  /*Serial.println("Position: (" + String(posX, 2) + ", " + String(posY, 2) +
                 ") | Distance: " + String(distance, 2) +
                 " | Heading: " + String(heading, 2) +
                 " | Target: (" + String(targetX, 2) + ", " + String(targetY, 2) + ")");*/
}

