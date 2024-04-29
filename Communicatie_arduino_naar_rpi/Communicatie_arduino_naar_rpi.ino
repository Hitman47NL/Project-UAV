void setup() {
  // Initialize serial communication
  Serial.begin(9600);
}

void loop() {
  // Check if there are incoming bytes
  if (Serial.available() >= 12) { // Ensure enough bytes for 3 floats
    byte buffer[12];
    Serial.readBytes(buffer, 12);
    
    // Extract the floats from the buffer
    int32_t X_int = (buffer[0] << 0) | (buffer[1] << 8) | (buffer[2] << 16) | (buffer[3] << 24);
    int32_t Y_int = (buffer[4] << 0) | (buffer[5] << 8) | (buffer[6] << 16) | (buffer[7] << 24);
    int32_t Angle_int = (buffer[8] << 0) | (buffer[9] << 8) | (buffer[10] << 16) | (buffer[11] << 24);
    
    // Convert integers to float values
    float X_coordinaat = static_cast<float>(X_int) / 1000.0;  // Scale factor 1000
    float Y_coordinaat = static_cast<float>(Y_int) / 1000.0;
    float Angle = static_cast<float>(Angle_int) / 1000.0;
    
    // Print received coordinates
    Serial.print("Received X coordinate: ");
    Serial.println(X_coordinaat);
    Serial.print("Received Y coordinate: ");
    Serial.println(Y_coordinaat);
    Serial.print("Received Angle: ");
    Serial.println(Angle);
    
    // Send confirmation message to Raspberry Pi
    Serial.println("Coordinates received successfully!");
  }
}
