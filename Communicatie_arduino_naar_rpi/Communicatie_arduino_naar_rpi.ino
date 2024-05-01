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

int receivedX = 0;  // Variable to store the received X coordinate
int receivedY = 0;  // Variable to store the received Y coordinate

void setup() {
  pinMode(buttonPin, INPUT);
  mySerial.begin(9600);
  Serial.begin(9600);
}

void loop() {
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
