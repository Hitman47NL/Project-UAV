// Initial coordinates as individual variables
float initial_x = 0.0;
float initial_y = 0.0;
float initial_angle = 0.0;

// Structure to hold new coordinates
struct UpdatedCoordinates {
  float x_new;
  float y_new;
  float angle_new;
};

// Initialize the new coordinates with initial values
UpdatedCoordinates updated = {initial_x, initial_y, initial_angle};

// Timing variable to control loop execution frequency without delay()
unsigned long previousMillis = 0;
const long interval = 500;  // Interval at which to run (milliseconds)

void setup() {
  Serial.begin(9600); // For the Serial Monitor (debugging)
  Serial1.begin(9600); // Communication between Raspberry Pi and Arduino
  Serial.println("Setup complete. Waiting for commands.");
  Serial.print("Initial Coordinates: x=");
  Serial.print(initial_x, 2);
  Serial.print(", y=");
  Serial.print(initial_y, 2);
  Serial.print(", angle=");
  Serial.println(initial_angle, 2);
}

void parseCoordinates(String data) {
  int firstComma = data.indexOf(',');
  int secondComma = data.indexOf(',', firstComma + 1);

  if (firstComma != -1 && secondComma != -1) {
    // Extract substrings and convert them to float
    updated.x_new = data.substring(0, firstComma).toFloat();
    updated.y_new = data.substring(firstComma + 1, secondComma).toFloat();
    updated.angle_new = data.substring(secondComma + 1).toFloat();

    Serial.println("Parsed values count: 3");
    Serial.print("Updated Coordinates: x_new=");
    Serial.print(updated.x_new, 2);
    Serial.print(", y_new=");
    Serial.print(updated.y_new, 2);
    Serial.print(", angle_new=");
    Serial.println(updated.angle_new, 2);
  } else {
    Serial.print("Invalid data received, unable to parse: ");
    Serial.println(data);
  }
}

void loop() {
  unsigned long currentMillis = millis();
  
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
  
    // Check for commands from the Raspberry Pi
    if (Serial1.available() > 0) {
      String command = Serial1.readStringUntil('\n');
      command.trim();

      Serial.print("Received command: "); // Debugging output
      Serial.println(command);

      if (command == "Send") {
        // Send coordinates to the Raspberry Pi
        String message = String(updated.x_new, 2) + "," + String(updated.y_new, 2) + "," + String(updated.angle_new, 2) + "\n";
        Serial1.print(message);
        Serial.print("Sent to Raspberry Pi: ");
        Serial.println(message);
      } else {
        // Parse the coordinates differences from the Raspberry Pi
        parseCoordinates(command);
      }
    }
  }
}
