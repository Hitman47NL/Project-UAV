#include <SoftwareSerial.h>

SoftwareSerial mySerial(19, 18); // RX, TX
float x, y;

void setup() {
  mySerial.begin(9600);
  pinMode(2, INPUT_PULLUP); // Button connected to pin 2
}

void loop() {
  if (digitalRead(2) == LOW) {
    mySerial.println("READY");
    while (mySerial.available() > 0) {
      String data = mySerial.readStringUntil('\n');
      if (data.length() > 0) {
        int commaIndex = data.indexOf(',');
        x = data.substring(0, commaIndex).toFloat();
        y = data.substring(commaIndex + 1).toFloat();
      }
    }
  }
}