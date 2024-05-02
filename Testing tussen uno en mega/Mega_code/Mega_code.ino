#include <Arduino.h>

struct Coordinates {
  int x;
  int y;
};

void setup() {
  Serial.begin(9600);        // Start communication with the computer
  Serial1.begin(9600);       // Start communication on Serial1 with Raspberry Pi

  // Wait for a stable connection
  delay(2000);               // Adjust based on your setup needs

  // Send initial coordinates to Raspberry Pi as a string
  sendCoordinates(0, 0);
  Serial.println("Initial coordinates sent to Raspberry Pi as string.");
}

void loop() {
  // Check if data is available from Raspberry Pi
  if (Serial1.available()) {
    Coordinates receivedCoords = receiveCoordinatesFromPi();  // Receive and parse coordinates
    Serial.print("Received from Pi: X=");
    Serial.print(receivedCoords.x);
    Serial.print(", Y=");
    Serial.println(receivedCoords.y);
    // Additional processing can be done here
  }
}

void sendCoordinates(int x, int y) {
  // Create a string from coordinates and send it
  String coordString = "X=" + String(x) + ",Y=" + String(y) + "\n";
  Serial1.print(coordString);
}

Coordinates receiveCoordinatesFromPi() {
  String data = Serial1.readStringUntil('\n');  // Read the string until a newline
  Coordinates coords;
  int separatorIndex = data.indexOf(',');
  if (separatorIndex != -1) {
    String xPart = data.substring(2, separatorIndex);  // Assumes format "X=123"
    String yPart = data.substring(separatorIndex + 3); // Assumes format ",Y=456"
    coords.x = xPart.toInt();  // Convert x part of string to integer
    coords.y = yPart.toInt();  // Convert y part of string to integer
  }
  return coords;
}
