unsigned long previousMillis = 0; // Stores the last time the trigger was sent
const long interval = 5000;       // Interval at which to send the trigger (5000 milliseconds or 5 seconds)
bool triggerSwitch = false;       // Switch state for sending the trigger

// Variables to store received coordinates
float X_coord = 0.0;
float Y_coord = 0.0;
float Angle_coord = 0.0;

void setup() {
  Serial.begin(9600);    // Start communication with the computer (for debugging)
  Serial1.begin(9600);   // Start communication on serial port 1

  while (!Serial) {
    ; // Wait for the connection to open
  }

  Serial.println("Arduino Mega is ready.");
}

// Function to handle coordinate transformation
void transformCoordinates() {
  int new_x_coords = X_coord - 10;
  int new_y_coords = Y_coord - 1;
  int new_angle_coords = Angle_coord - 45;
  
  Serial.print("new_x: ");
  Serial.println(new_x_coords);
  
  Serial.print("new_y: ");
  Serial.println(new_y_coords);
  
  Serial.print("new_angle: ");
  Serial.println(new_angle_coords);
}

void loop() {
  unsigned long currentMillis = millis();
  
  // Check if it's time to send the trigger again and the switch is on
  if (triggerSwitch && currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    
    // Send the trigger
    Serial1.println("Send");
    Serial.println("Trigger 'Send' sent to Python script.");
  }
  
  // Check if there is data available to read from Python
  if (Serial1.available() > 0) {
    String received = Serial1.readStringUntil('\n');
    Serial.print("Received coordinates: ");
    Serial.println(received);
    
    // Parse the received string into coordinates
    int firstCommaIndex = received.indexOf(',');
    int secondCommaIndex = received.indexOf(',', firstCommaIndex + 1);
    
    // Convert the substrings to float
    X_coord = received.substring(0, firstCommaIndex).toFloat();
    Y_coord = received.substring(firstCommaIndex + 1, secondCommaIndex).toFloat();
    Angle_coord = received.substring(secondCommaIndex + 1).toFloat();
    
    // Output the parsed coordinates for debugging
    Serial.print("X: ");
    Serial.print(X_coord);
    Serial.print(", Y: ");
    Serial.print(Y_coord);
    Serial.print(", Angle: ");
    Serial.println(Angle_coord);
    
    // Call the function to transform coordinates
    transformCoordinates();
  }
}

// Function to handle switch state change
void switchStateChange() {
  // Example code for changing switch state, replace with actual code reading the switch
  triggerSwitch = !triggerSwitch;
}
