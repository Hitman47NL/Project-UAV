void setup() {
  Serial.begin(9600);
}

void loop() {
  // Send coordinates to the Mega
  Serial.println("X20,Y2");

  // Listen for a response from the Mega
  if (Serial.available()) {
    String received = Serial.readStringUntil('\n');
    Serial.print("Received from Mega: ");
    Serial.println(received);
    // Clear the buffer to ensure no leftovers
    while (Serial.available()) Serial.read();
  }

  delay(2000); // Wait for 2 seconds to ensure messages don't overlap
}
