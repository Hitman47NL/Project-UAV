void communicatie() {
  int reading = digitalRead(buttonPin);

  if (reading != lastButtonState) {
    lastDebounceTime = millis();
  }

  if ((millis() - lastDebounceTime) > debounceDelay) {
    if (reading != buttonState) {
      buttonState = reading;
      if (buttonState == LOW) {
        mySerial.println("0,0");  // Send coordinates to Raspberry Pi
        Serial.println("Sent coordinates: X=0, Y=0");
      }
    }
  }

  lastButtonState = reading;

  if (mySerial.available()) {
    String coordinates = mySerial.readStringUntil('\n');
    coordinates.trim();
    if (coordinates.length() > 0) {
      Serial.print("Received modified coordinates: ");
      Serial.println(coordinates);

      // Parsing the coordinates
      int commaIndex = coordinates.indexOf(',');
      if (commaIndex != -1) {
        receivedX = coordinates.substring(0, commaIndex).toInt();
        receivedY = coordinates.substring(commaIndex + 1).toInt();

        Serial.print("Parsed X: ");
        Serial.print(receivedX);
        Serial.print(", Parsed Y: ");
        Serial.println(receivedY);
      } else {
        Serial.println("Error: Received string is not in expected format.");
      }
    } else {
      Serial.println("Error: No data received.");
    }
  }
}