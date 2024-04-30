void setup() {
  Serial.begin(9600);
}

void loop() {
  if (Serial.available() > 0) {
    String incomingData = Serial.readStringUntil('\n');
    int x_coord, y_coord;
    if (incomingData.startsWith("X") && incomingData.endsWith("Y")) {
      x_coord = incomingData.substring(1, incomingData.indexOf("Y")).toInt();
      y_coord = incomingData.substring(incomingData.indexOf("Y") + 1).toInt();
      Serial.print("Received coordinates: X=");
      Serial.print(x_coord);
      Serial.print(", Y=");
      Serial.println(y_coord);
    }
  }
}