/*
 Name:		Test_COM_RPI_5.ino
 Created:	2/13/2024 1:15:25 PM
 Author:	Iwanv
*/

void setup() {
  // Start serial communication at a baud rate of 9600
  // Begin the Serial1 at 9600 Baud for Raspberry Pi communication
  Serial1.begin(9600);
  // Initialize Serial for debugging purposes
  Serial.begin(9600);
}

void loop() {
  // Check if data is available to read
  if (Serial1.available() > 0) {
    String receivedMessage = Serial1.readStringUntil('\n');
    // Print the received message to the Serial Monitor
    Serial.println("Received from Raspberry Pi: " + receivedMessage);
    // Send a response back to Raspberry Pi
    Serial1.println("Hi Raspberry Pi!");
  }
}


