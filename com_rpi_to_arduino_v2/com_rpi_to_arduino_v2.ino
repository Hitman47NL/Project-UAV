// Initial coordinates
float x = 0.0;
float y = 0.0;
float angle = 0.0;

// Structure to hold new coordinates
struct Coordinates {
  float x_new;
  float y_new;
  float angle_new;
};

Coordinates coordinates = {x, y, angle};

void setup() {
  Serial.begin(9600); // For the Serial Monitor (debugging)
  Serial1.begin(9600); // Communication between Raspberry Pi and Arduino
  Serial.println("Setup complete. Waiting for commands.");
}

void parseCoordinates(String data) {
  int firstComma = data.indexOf(',');
  int secondComma = data.indexOf(',', firstComma + 1);

  if (firstComma != -1 && secondComma != -1) {
    // Extract substrings and convert them to float
    coordinates.x_new = data.substring(0, firstComma).toFloat();
    coordinates.y_new = data.substring(firstComma + 1, secondComma).toFloat();
    coordinates.angle_new = data.substring(secondComma + 1).toFloat();

    Serial.print("Parsed values count: 3\n");
    Serial.print("Updated Coordinates: x=");
    Serial.print(coordinates.x_new, 2);
    Serial.print(", y=");
    Serial.print(coordinates.y_new, 2);
    Serial.print(", angle=");
    Serial.println(coordinates.angle_new, 2);
  } else {
    Serial.print("Invalid data received, unable to parse: ");
    Serial.println(data);
  }
}

void loop() {
  // Check for commands from the Raspberry Pi
  if (Serial1.available() > 0) {
    String command = Serial1.readStringUntil('\n');
    command.trim();

    Serial.print("Received command: "); // Debugging output
    Serial.println(command);

    if (command == "Send") {
      // Send coordinates to the Raspberry Pi
      String message = String(coordinates.x_new, 2) + "," + String(coordinates.y_new, 2) + "," + String(coordinates.angle_new, 2) + "\n";
      Serial1.print(message);
      Serial.print("Sent to Raspberry Pi: ");
      Serial.println(message);
    } else {
      // Parse the coordinates differences from the Raspberry Pi
      parseCoordinates(command);
    }
  }

  delay(1000); // Adjust delay as necessary
}
