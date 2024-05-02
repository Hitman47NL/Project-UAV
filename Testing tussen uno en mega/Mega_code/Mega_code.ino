void setup() {
  Serial.begin(9600);
  Serial3.begin(9600);
}

void loop() {
  if (Serial3.available()) {
    String data = Serial3.readStringUntil('\n');
    Serial.print("Received from Uno: ");
    Serial.println(data);

    // Acknowledge and send new coordinates back
    String response = "Received: " + data + " - Sending new coordinates: X10,Y1";
    Serial3.println(response);

    // Clear the buffer to ensure no leftovers
    while (Serial3.available()) Serial3.read();
  }

  delay(2000); // Wait for 2 seconds to prevent overflow and ensure complete data transmission
}
