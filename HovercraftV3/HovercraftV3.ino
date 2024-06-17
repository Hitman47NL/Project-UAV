// Gemaakt door Jelle
// Hiermee kunnen de MPU9250 en de TOF sensoren worden uitgelezen

#include <Wire.h>
#include <Adafruit_VL53L0X.h>
#include <LiquidCrystal_I2C.h>
#include <math.h> // Include the math library for atan2 function
#include <MPU9250_WE.h>
#include <SoftwareSerial.h>

// RX and TX pins for SoftwareSerial
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

#define TOF1_ADDRESS 0x30
#define TOF2_ADDRESS 0x31
#define TOF3_ADDRESS 0x32
#define BUZZER_PIN 6
#define MPU9250_ADDR 0x68

#define L9110S_B_IA 9   //  Motor B Input A --> MOTOR B +
#define L9110S_B_IB 8   //  Motor B Input B --> MOTOR B -
#define L9110S_A_IA 11  //  Motor A Input A --> MOTOR A +
#define L9110S_A_IB 10  //  Motor A Input B --> MOTOR A -
#define MOTOR_B_PWM L9110S_B_IA  // Motor B PWM Speed
#define MOTOR_B_DIR L9110S_B_IB  // Motor B Direction
#define MOTOR_A_PWM L9110S_A_IA  // Motor A PWM Speed
#define MOTOR_A_DIR L9110S_A_IB  // Motor A Direction

#define PWM_SLOW 50     // arbitrary slow speed PWM duty cycle
#define PWM_FAST 200    // arbitrary fast speed PWM duty cycle
#define DIR_DELAY 1000  // brief delay for abrupt motor changes

#define RELAY1_PIN 12
#define RELAY2_PIN 13

#define SHT_TOF1_PIN 38
#define SHT_TOF2_PIN 36
#define SHT_TOF3_PIN 34

#define OUT_STBY_PIN 41
#define OUT_A_PWM_PIN 2
#define OUT_A_IN2_PIN 39
#define OUT_A_IN1_PIN 37

Adafruit_VL53L0X tof1 = Adafruit_VL53L0X();
Adafruit_VL53L0X tof2 = Adafruit_VL53L0X();
Adafruit_VL53L0X tof3 = Adafruit_VL53L0X();

VL53L0X_RangingMeasurementData_t measure1;
VL53L0X_RangingMeasurementData_t measure2;
VL53L0X_RangingMeasurementData_t measure3;

int sensor1 = 0;
int sensor2 = 0;
int sensor3 = 0;

#define LCD_ADDRESS 0x27
#define LCD_COLUMNS 16
#define LCD_ROWS 2
LiquidCrystal_I2C lcd(LCD_ADDRESS, LCD_COLUMNS, LCD_ROWS);
MPU9250_WE myMPU9250 = MPU9250_WE(MPU9250_ADDR);

#define ACCU_SAFETY_PIN A8
#define LOW_BATTERY_THRESHOLD 3.3 // Example threshold voltage

void playTone(int tone, int duration) {
  for (long i = 0; i < duration * 100L; i += tone * 2) {
    digitalWrite(BUZZER_PIN, HIGH);
    delayMicroseconds(tone);
    digitalWrite(BUZZER_PIN, LOW);
    delayMicroseconds(tone);
  }
}

void playSuccessTune() {
  int melody[] = {220, 196, 165, 131};
  int noteDuration = 150;

  for (int note : melody) {
    playTone(note, noteDuration);
    delay(150);
  }
}

void playFailTune() {
  int melody[] = {294, 392, 600, 1600};
  int noteDuration = 150;

  for (int note : melody) {
    playTone(note, noteDuration);
    delay(150);
  }
}

void initializeSensors() {
  digitalWrite(SHT_TOF1_PIN, LOW);
  digitalWrite(SHT_TOF2_PIN, LOW);
  digitalWrite(SHT_TOF3_PIN, LOW);
  delay(10);
  digitalWrite(SHT_TOF1_PIN, HIGH);
  digitalWrite(SHT_TOF2_PIN, HIGH);
  digitalWrite(SHT_TOF3_PIN, HIGH);
  delay(10);

  digitalWrite(SHT_TOF1_PIN, HIGH);
  digitalWrite(SHT_TOF2_PIN, LOW);
  digitalWrite(SHT_TOF3_PIN, LOW);
  if (!tof1.begin(TOF1_ADDRESS)) {
    Serial.println(F("Failed to boot first VL53L0X"));
  }
  delay(10);

  digitalWrite(SHT_TOF2_PIN, HIGH);
  if (!tof2.begin(TOF2_ADDRESS)) {
    Serial.println(F("Failed to boot second VL53L0X"));
  }
  delay(10);

  digitalWrite(SHT_TOF3_PIN, HIGH);
  if (!tof3.begin(TOF3_ADDRESS)) {
    Serial.println(F("Failed to boot third VL53L0X"));
  }
}

void readDualSensors() {
  tof1.rangingTest(&measure1, false);
  tof2.rangingTest(&measure2, false);
  tof3.rangingTest(&measure3, false);
  sensor1 = measure1.RangeMilliMeter;
  sensor2 = measure2.RangeMilliMeter;
  sensor3 = measure3.RangeMilliMeter;
}

float calculateAngle() {
  // Calculate angle based on the furthest sensor
  float rawAngle = (sensor1 - sensor2) / 10.0; // Calculate raw angle difference
  float angle = atan(rawAngle) * 180 / M_PI; // Convert to degrees
  lcd.setCursor(5, 0);
  lcd.print(angle);
  return angle;
}

void setup() {
  pinMode(buttonPin, INPUT);
  mySerial.begin(9600);
  Serial.begin(9600);

  Wire.begin();
  Serial.begin(9600);

  Serial.println("Starting...");

  pinMode(RELAY1_PIN, OUTPUT);
  pinMode(RELAY2_PIN, OUTPUT);
  digitalWrite(RELAY1_PIN, HIGH);
  digitalWrite(RELAY2_PIN, HIGH);
  delay(100);

  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(SHT_TOF1_PIN, OUTPUT);
  pinMode(SHT_TOF2_PIN, OUTPUT);
  pinMode(SHT_TOF3_PIN, OUTPUT);
  digitalWrite(SHT_TOF1_PIN, LOW);
  digitalWrite(SHT_TOF2_PIN, LOW);
  digitalWrite(SHT_TOF3_PIN, LOW);

  initializeSensors();

  pinMode(MOTOR_B_DIR, OUTPUT);
  pinMode(MOTOR_B_PWM, OUTPUT);
  digitalWrite(MOTOR_B_DIR, LOW);
  digitalWrite(MOTOR_B_PWM, LOW);
  delay(2000);

  pinMode(MOTOR_A_DIR, OUTPUT);
  pinMode(MOTOR_A_PWM, OUTPUT);
  digitalWrite(MOTOR_A_DIR, LOW);
  digitalWrite(MOTOR_A_PWM, LOW);

  if (!myMPU9250.init()) {
    Serial.println("MPU9250 does not respond");
  } else {
    Serial.println("MPU9250 is connected");
  }

  if (!myMPU9250.initMagnetometer()) {
    Serial.println("Magnetometer does not respond");
  } else {
    Serial.println("Magnetometer is connected");
  }

  Serial.println("Position you MPU9250 flat and don't move it - calibrating...");
  delay(1000);
  myMPU9250.autoOffsets();
  Serial.println("Done!");

  myMPU9250.setAccOffsets(-14240.0, 18220.0, -17280.0, 15590.0, -20930.0, 12080.0);
  myMPU9250.setGyrOffsets(45.0, 145.0, -105.0);
  myMPU9250.enableGyrDLPF();
  myMPU9250.setGyrDLPF(MPU9250_DLPF_6);
  myMPU9250.setSampleRateDivider(5);
  myMPU9250.setGyrRange(MPU9250_GYRO_RANGE_250);
  myMPU9250.setAccRange(MPU9250_ACC_RANGE_2G);
  myMPU9250.enableAccDLPF(true);
  myMPU9250.setAccDLPF(MPU9250_DLPF_6);
  myMPU9250.setMagOpMode(AK8963_CONT_MODE_100HZ);
  delay(200);

  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("Starting up");

  playSuccessTune();
  delay(2000);
  playFailTune();
}

void controlMotors(char command) {
  digitalWrite(MOTOR_B_DIR, LOW);
  digitalWrite(MOTOR_B_PWM, LOW);
  delay(DIR_DELAY);
  digitalWrite(MOTOR_A_DIR, LOW);
  digitalWrite(MOTOR_A_PWM, LOW);
  delay(DIR_DELAY);

  switch (command) {
    case '1':  // Fast forward
      Serial.println("Fast forward...");
      digitalWrite(MOTOR_B_DIR, HIGH);           // direction = forward
      analogWrite(MOTOR_B_PWM, 255 - PWM_FAST);  // PWM speed = fast
      digitalWrite(MOTOR_A_DIR, HIGH);           // direction = forward
      analogWrite(MOTOR_A_PWM, 255 - PWM_FAST);  // PWM speed = fast
      break;
    case '2':  // Forward
      Serial.println("Forward...");
      digitalWrite(MOTOR_B_DIR, HIGH);           // direction = forward
      analogWrite(MOTOR_B_PWM, 255 - PWM_SLOW);  // PWM speed = slow
      digitalWrite(MOTOR_A_DIR, HIGH);           // direction = forward
      analogWrite(MOTOR_A_PWM, 255 - PWM_SLOW);  // PWM speed = slow
      break;
    case '3':  // Soft stop (coast)
      Serial.println("Soft stop (coast)...");
      break;
    case '4':  // Reverse
      Serial.println("Reverse...");
      digitalWrite(MOTOR_B_DIR, LOW);      // direction = reverse
      analogWrite(MOTOR_B_PWM, PWM_SLOW);  // PWM speed = slow
      digitalWrite(MOTOR_A_DIR, LOW);      // direction = reverse
      analogWrite(MOTOR_A_PWM, PWM_SLOW);  // PWM speed = slow
      break;
    case '5':  // Fast reverse
      Serial.println("Fast reverse...");
      digitalWrite(MOTOR_B_DIR, LOW);      // direction = reverse
      analogWrite(MOTOR_B_PWM, PWM_FAST);  // PWM speed = fast
      digitalWrite(MOTOR_A_DIR, LOW);      // direction = reverse
      analogWrite(MOTOR_A_PWM, PWM_FAST);  // PWM speed = fast
      break;
    case '6':  // Hard stop (brake)
      Serial.println("Hard stop (brake)...");
      digitalWrite(MOTOR_B_DIR, HIGH);
      digitalWrite(MOTOR_B_PWM, HIGH);
      digitalWrite(MOTOR_A_DIR, HIGH);
      digitalWrite(MOTOR_A_PWM, HIGH);
      break;
    case '7':
      Serial.println("Kleine motor...");
      digitalWrite(OUT_A_IN2_PIN, LOW);
      digitalWrite(OUT_A_IN1_PIN, HIGH);
      digitalWrite(RELAY1_PIN, HIGH);
      digitalWrite(RELAY2_PIN, HIGH);
      break;
    case '8':
      Serial.println("Alleen hover...");
      digitalWrite(RELAY1_PIN, HIGH);
      digitalWrite(RELAY2_PIN, HIGH);
      break;
    default:
      break;
  }
}

void checkBattery() {
  int sensorValue = analogRead(ACCU_SAFETY_PIN);
  float voltage = sensorValue * (5.0 / 1023.0);

  if (voltage < LOW_BATTERY_THRESHOLD) {
    Serial.println("Low battery! Voltage: " + String(voltage));
    playFailTune();
    // Perform any additional actions like stopping the motors
    controlMotors('6');  // Hard stop
  }
}

void loop() {
  xyzFloat gValue = myMPU9250.getGValues();
  xyzFloat gyr = myMPU9250.getGyrValues();
  xyzFloat magValue = myMPU9250.getMagValues();
  xyzFloat gAngle = myMPU9250.getAngles();
  float temp = myMPU9250.getTemperature();
  float resultantG = myMPU9250.getResultantG(gValue);

  Serial.println("Acceleration in g (x,y,z):");
  Serial.print(gValue.x);
  Serial.print("   ");
  Serial.print(gValue.y);
  Serial.print("   ");
  Serial.println(gValue.z);
  Serial.print("Resultant g: ");
  Serial.println(resultantG);

  Serial.println("Gyroscope data in degrees/s: ");
  Serial.print(gyr.x);
  Serial.print("   ");
  Serial.print(gyr.y);
  Serial.print("   ");
  Serial.println(gyr.z);

  Serial.println("Angle Data in degrees: ");
  Serial.print(gAngle.x);
  Serial.print("   ");
  Serial.print(gAngle.y);
  Serial.print("   ");
  Serial.println(gAngle.z);

  readDualSensors();
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(sensor1);
  lcd.setCursor(0, 1);
  lcd.print(sensor2);
  float ang = calculateAngle();
  lcd.setCursor(5, 1);
  lcd.print(ang);

  checkBattery();

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

  if (Serial.available()) {
    char command = Serial.read();
    controlMotors(command);
  }

  delay(50);
}
