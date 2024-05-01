#include <SoftwareSerial.h>

// RX and TX pins
const int RXPin = 19;
const int TXPin = 18;
SoftwareSerial mySerial(RXPin, TXPin);

const int buttonPin = 2;
int buttonState = 0;
int lastButtonState = 0;

unsigned long lastDebounceTime = 0;
unsigned long debounceDelay = 50;

void setup() {
  pinMode(buttonPin, INPUT);
  mySerial.begin(9600);
  Serial.begin(9600);
}

void loop() {
  int reading = digitalRead(buttonPin);

  // If the button state has changed,
  if (reading != lastButtonState) {
    lastDebounceTime = millis();
  }

  if ((millis() - lastDebounceTime) > debounceDelay) {
    // If the button state has been changed:
    if (reading != buttonState) {
      buttonState = reading;

      // Only send the coordinates if the new button state is HIGH
      if (buttonState == LOW) {
        mySerial.println("0,0");  // Sending coordinates to Raspberry Pi
        Serial.println("Sent coordinates: X=0, Y=0");
      }
    }
  }

  lastButtonState = reading;

  // Check if Raspberry Pi has sent back modified coordinates
  if (mySerial.available()) {
    String coordinates = mySerial.readStringUntil('\n');
    coordinates.trim();  // Remove any whitespace or newline characters

    if (coordinates.length() > 0) {
      Serial.print("Received modified coordinates: ");
      Serial.println(coordinates);

      // Optional: Parsing the coordinates
      int commaIndex = coordinates.indexOf(',');
      if (commaIndex != -1) {
        String xStr = coordinates.substring(0, commaIndex);
        String yStr = coordinates.substring(commaIndex + 1);
        
        int x = xStr.toInt();
        int y = yStr.toInt();
        
        Serial.print("X: ");
        Serial.print(x);
        Serial.print(", Y: ");
        Serial.println(y);
      } else {
        Serial.println("Error: Received string is not in expected format.");
      }
    } else {
      Serial.println("Error: No data received.");
    }
  }
}
