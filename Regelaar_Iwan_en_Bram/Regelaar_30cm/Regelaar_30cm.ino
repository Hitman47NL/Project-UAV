#include <Wire.h>
#include <Adafruit_VL53L0X.h>  // Library for the TOF sensor

// Define constants and variables
const float m = 1.560;  // Weight of the object in kg
const float desiredDistance = 0.3;  // Desired distance from the wall in meters
const float startDistance = 0.8;  // Distance at which the regulator should start working in meters
const float settleTime = 5.0;  // Time within which the regulator must execute in seconds

Adafruit_VL53L0X sensor = Adafruit_VL53L0X();  // Create a TOF sensor object

float omega = (2 * PI) / settleTime;
float Im = omega;
float Re = omega / 2;
float Kp = (Re * Re + Im * Im) * m;
float Kd = 2 * Re * m;

// PD variables
float lastError = 0;
float lastTime = 0;

void setup() {
  Serial.begin(9600);
  Wire.begin();

  // Initialize the TOF sensor
  if (!sensor.begin()) {
    Serial.println(F("Failed to boot VL53L0X"));
    while (1);
  }

  // Initialize motors (assuming motor control pins are defined)
  pinMode(9, OUTPUT);  // Example motor control pin
}

void loop() {
  regulateDistance();
}

void regulateDistance() {
  float currentDistance = readSensor();
  if (currentDistance < startDistance) {
    float error = desiredDistance - currentDistance;
    float currentTime = millis() / 1000.0;  // Convert to seconds
    float deltaTime = currentTime - lastTime;
    
    // PD control calculations
    float P = Kp * error;
    float D = Kd * (error - lastError) / deltaTime;

    float controlSignal = P + D;
    applyControl(controlSignal);

    // Update for the next iteration
    lastError = error;
    lastTime = currentTime;
  }

  delay(50);  // Sample time delay
}

float readSensor() {
  VL53L0X_RangingMeasurementData_t measure;
  sensor.rangingTest(&measure, false);

  if (measure.RangeStatus != 4) {  // Check if the sensor is valid
    return measure.RangeMilliMeter / 1000.0;  // Convert to meters
  } else {
    Serial.println("Out of range");
    return startDistance + 1;  // Return a value out of start distance range to avoid false starts
  }
}

void applyControl(float controlSignal) {
  // Apply control signal to the motors
  // Here we assume a simple implementation where the motor speed is proportional to the control signal
  int motorSpeed = constrain(controlSignal, 0, 255);
  analogWrite(9, motorSpeed);
}
