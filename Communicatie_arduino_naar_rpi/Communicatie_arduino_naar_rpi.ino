#include <SoftwareSerial.h>

SoftwareSerial mySerial(19, 18); // RX, TX

const int buttonPin = 2;
int buttonState = 0;
bool isWaitingForResponse = false;

void setup() {
  pinMode(buttonPin, INPUT);
  mySerial.begin(9600);
  Serial.begin(9600);
}

void loop() {
  buttonState = digitalRead(buttonPin);
  
  if (buttonState == LOW && !isWaitingForResponse) {
    mySerial.println("0.0,0.0");
    isWaitingForResponse = true; // Avoid sending more data until we get a response
    Serial.println("Sent: 0.0,0.0");
    delay(1000);
  }

  if (isWaitingForResponse && mySerial.available() > 0) {
    String modifiedCoordinates = mySerial.readStringUntil('\n');
    if (modifiedCoordinates.length() > 0) {
      Serial.println("Received: " + modifiedCoordinates);
      isWaitingForResponse = false;
    }
  }
}
