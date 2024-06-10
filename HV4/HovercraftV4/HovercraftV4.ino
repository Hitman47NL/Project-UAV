// Gemaakt door Jelle en Iwan

//Libraries
#include "Grove_Motor_Driver_TB6612FNG.h"
#include <Wire.h>
#include <Adafruit_VL53L0X.h>
#include <LiquidCrystal_I2C.h>
#include <math.h>  // Include the math library for atan2 function
#include <MPU9250_WE.h>
#include <SoftwareSerial.h>
#include <avr/io.h>
#include <avr/wdt.h>
#include "ACS712.h"

//Define pinnen voor Arduino
//PWM
#define motorZijkantPWM 2
#define BUZZER_PINP 7
#define BUZZER_PIN 5
#define BUZZER_PINM 6
#define motorLinksPWM 11  //  Motor B Input A
#define motorLinks 10     //  Motor B Input B
#define motorRechtsPWM 9  //  Motor A Input A
#define motorRechts 8     //  Motor A Input B

//Digitale pinnen
#define BLOWRELAY 12
#define NOODSTOPRELAY 13

#define SHT_TOF1_PIN 34
#define SHT_TOF2_PIN 36
#define SHT_TOF3_PIN 38

#define motorZijkant 39
#define motorZijkantIn2 35
#define motorZijkantIn1 37

//Analoog
#define ACCU_SAFETY_PIN A0

//Define adressen voor i2c
#define TOF1_ADDR 0x30
#define TOF2_ADDR 0x31
#define TOF3_ADDR 0x32
#define MPU9250_ADDR 0x68
#define LCD_ADDR 0x27

// RX and TX pins for SoftwareSerial
const int RXPin = 19;
const int TXPin = 18;
SoftwareSerial mySerial(RXPin, TXPin);

// Benodigde functies
void Poolplaatsing_PD(float &Kp_PD, float &Kd_PD, float m, float Re_PD, float Im_PD);
void Poolplaatsing_PID(float &K_PID, float &Kd_PID, float &Ki_PID, float Iz, float Re_PID, float Im_PID, float pool3);
void Regeling_PD(float &Fx_PD, float &Fy_PD, float Kp_PD, float Kd_PD, float sp_PD, float sx, float dt);
void Regeling_PID(float &Fx_PID, float &Fy_PID, float Kp_PID, float Kd_PID, float Ki_PID, float sp_PID, float theta, float dt);
void motoraansturing_Iwan(float Fx_PD, float Fx_PID, float Fy_PD, float Fy_PID);
void motoraansturing_Bram(float Fx_PID);

//Vaste waardes
#define MAX_PWM 255  //Voor regelaar
#define MIN_PWM 0    // Voor regelaar

#define PWM_SLOW 50                // arbitrary slow speed PWM duty cycle
#define PWM_FAST 200               // arbitrary fast speed PWM duty cycle
#define DIR_DELAY 1000             // brief delay for abrupt motor changes
#define LOW_BATTERY_THRESHOLD 1.5  // threshold amperage

// Misc voor sensore en LCD
Adafruit_VL53L0X tof1 = Adafruit_VL53L0X();
Adafruit_VL53L0X tof2 = Adafruit_VL53L0X();
Adafruit_VL53L0X tof3 = Adafruit_VL53L0X();

VL53L0X_RangingMeasurementData_t measure1;
VL53L0X_RangingMeasurementData_t measure2;
VL53L0X_RangingMeasurementData_t measure3;

#define LCD_COLUMNS 16
#define LCD_ROWS 2
LiquidCrystal_I2C lcd(LCD_ADDR, LCD_COLUMNS, LCD_ROWS);
MPU9250_WE myMPU9250 = MPU9250_WE(MPU9250_ADDR);

ACS712 stroommeter = ACS712(ACS712_30A, ACCU_SAFETY_PIN);
//Global variable
const int buttonPin = 2;
int buttonState = 0;
int lastButtonState = 0;
unsigned long lastDebounceTime = 0;
unsigned long debounceDelay = 50;

int receivedX = 0;  // Variable to store the received X coordinate
int receivedY = 0;  // Variable to store the received Y coordinate

//Voor TOFsensoren
float TOFsensor1 = 0.0;  //mm
float TOFsensor2 = 0.0;  //mm
float TOFsensor3 = 0.0;  //mm
float TOFsensor1M = 0.0;
float TOFsensor2M = 0.0;
float degAngle = 0.0;
float radAngle = 0.0;

void setup() {
  //Communicatie
  Serial.begin(9600);
  mySerial.begin(9600);
  // analogReference(INTERNAL1V1);
  Wire.begin();
  //LCD
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("Starting up");  //Zodat de user weet dat er iets gebeurd
  //Serial
  Serial.println("Starting...");  //Zodat de programmeur weet dat er iets gebeurd
  checkBattery();                 // Om de baterij bij opstarten te checken
  setPins();                      //Om de pinnen juist op in en output te zetten
  initWrites();                   // Om de motoren niet gelijk te laten draaien
  delay(100);
  initTOFsensors();  // Om de TOF sensoren te initialiseren
  initGyroSensor();  // Om de gyro te initialiseren

  bootupCheck();
  delay(100);
  digitalWrite(NOODSTOPRELAY, HIGH);
  delay(500);
  digitalWrite(BLOWRELAY, HIGH);
  delay(100);
}

// Functie voor het instellen van de adressen van de TOF sensoren. Code kan worden uitgebreid om acties te ondernemen bij het niet opstarten van de TOF senosren
void initTOFsensors() {
  //Zet alle pinnen laag
  digitalWrite(SHT_TOF1_PIN, LOW);
  digitalWrite(SHT_TOF2_PIN, LOW);
  digitalWrite(SHT_TOF3_PIN, LOW);
  delay(10);  // Stabiiteit
  //Zet alle pinnen hoog
  digitalWrite(SHT_TOF1_PIN, HIGH);
  digitalWrite(SHT_TOF2_PIN, HIGH);
  digitalWrite(SHT_TOF3_PIN, HIGH);
  delay(10);  // Stabiliteit
  //Zet de pin van TOF1 hoog om adres te schrijven.
  digitalWrite(SHT_TOF1_PIN, HIGH);
  digitalWrite(SHT_TOF2_PIN, LOW);
  digitalWrite(SHT_TOF3_PIN, LOW);

  //Schrijf het adres naar TOF1
  if (!tof1.begin(TOF1_ADDR)) {
    Serial.println(F("Failed to boot first VL53L0X"));  //Foutmelding
    //Eventuele mogelijkheid om fout code uit te breiden
  }
  delay(10);  // Stabiliteit

  //Zet pin van TOF2 hoog om adres te schrijven
  digitalWrite(SHT_TOF2_PIN, HIGH);
  //Schrijf het adres naar TOF2
  if (!tof2.begin(TOF2_ADDR)) {
    Serial.println(F("Failed to boot second VL53L0X"));
    //Eventuele mogelijkheid om fout code uit te breiden
  }
  delay(10);  // Stabiliteit

  //Zet pin van TOF23 hoog om adres te schrijven
  digitalWrite(SHT_TOF3_PIN, HIGH);
  //Schrijf het adres naar TOF3
  if (!tof3.begin(TOF3_ADDR)) {
    Serial.println(F("Failed to boot third VL53L0X"));
    //Eventuele mogelijkheid om fout code uit te breiden
  }
}
void Motor_Rechts(float Fx_PD, float Fx_PID) {  // Dit is voor Maxon motor 2
  if (Fx_PD > 0) {                              // dit is voor wanneer de hovercraft achterwaarts moet bewegen
    Serial.print("PWM Maxon 2 pos: ");
    float Fx_nega = Fx_PD * -1;
    Serial.println(-0.00282 * Fx_nega * Fx_nega - 1.70725 * Fx_nega);
    digitalWrite(motorLinks, LOW);
    analogWrite(motorLinksPWM, -0.00282 * Fx_nega * Fx_nega - 1.70725 * Fx_nega);
  } else {  // dit is voor wanneer de hovercraft voorwaarts moet bewegen
    float Fx = Fx_PD, Fx_PID;
    Serial.print("PWM Maxon 2 nega: ");
    Serial.println(-0.00425 * Fx * Fx + 2.077594 * Fx);
    digitalWrite(motorLinks, HIGH);
    analogWrite(motorLinksPWM, -0.00425 * Fx * Fx + 2.077594 * Fx);
  }
}

void Motor_Links(float Fx_PD, float Fx_PID) {  // Dit is voor Maxon motor 1
  if (Fx_PD > 0) {                             // hierdoor gaat de uav achteruit
    Serial.print("PWM Maxon 1 pos: ");
    float Fx_nega = Fx_PD * -1;
    Fx_nega = Fx_PID * -1;
    Serial.println(-0.00298 * Fx_nega * Fx_nega - 1.75115 * Fx_nega);
    digitalWrite(motorRechts, LOW);  // dit is voor wanneer de hovercraft achterwaarts moet bewegen
    analogWrite(motorRechtsPWM, -0.00298 * Fx_nega * Fx_nega - 1.75115 * Fx_nega);
  } else {  // dit is voor wanneer de hovercraft voorwaartswaarts moet bewegen
    float Fx = Fx_PD, Fx_PID;
    Serial.print("PWM Maxon 1 nega: ");
    Serial.println(-0.00505 * Fx * Fx + 2.25550 * Fx);
    digitalWrite(motorRechts, HIGH);
    analogWrite(motorRechtsPWM, -0.00505 * Fx * Fx + 2.25550 * Fx);
  }
}

void Motor_midden(float Fy_PID, float Fy_PD) {  //Dit is voor de kleine motor
  if (Fy_PID < 0) {                             // dit is voor wanneer de hovercraft links moet bewegen
    float Fy = Fy_PD, Fy_PID;
    digitalWrite(motorRechts, LOW);
    analogWrite(motorRechtsPWM, 0.00519 * Fy * Fy - 0.67075 * Fy);
  } else {  // dit is voor wanneer de hovercraft rechts moet bewegen
    float Fy = Fy_PD, Fy_PID;
    digitalWrite(motorRechts, HIGH);
    analogWrite(motorRechtsPWM, 0.01060 * Fy * Fy + 1.12248 * Fy);
  }
}
// Functie voor het instellen voor de Gyroscoop,
void initGyroSensor() {
  //Check of de gyro is verbonden
  if (!myMPU9250.init()) {
    Serial.println("MPU9250 does not respond");
    //Mogelijkheid tot uitbreiden voor niet werkende gyro
  } else {
    Serial.println("MPU9250 is connected");
  }
  //Check of de magneetmeter werkt
  if (!myMPU9250.initMagnetometer()) {
    Serial.println("Magnetometer does not respond");
    //Mogelijkheid tot uitbreiden voor niet werkende magneetmeter
  } else {
    Serial.println("Magnetometer is connected");
  }

  Serial.println("Position you MPU9250 flat and don't move it - calibrating...");
  delay(100);
  myMPU9250.autoOffsets();
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
  delay(100);
}
//Deze functie zorgt voor het uitlezen van alle 3 TOF sensoren.
void readDualSensors() {
  tof1.rangingTest(&measure1, false);
  tof2.rangingTest(&measure2, false);
  tof3.rangingTest(&measure3, false);
  //Decleratie van de variable DMV functie. Alle afstanden zijn in mm
  TOFsensor1 = measure1.RangeMilliMeter;  //Sensor 1 afstand in mm
  TOFsensor2 = measure2.RangeMilliMeter;  //Sensor 2 afstand in mm
  TOFsensor3 = measure3.RangeMilliMeter;  //Sensor 3 afstand in mm
}
// Deze functie berekend de hoek op basis van de verste TOF sensor. De richting van de hoek wordt aangegeven door positief of negatief
float calculateAngle() {
  //Maakt van mm -> cm
  float TOFsensor1M = measure1.RangeMilliMeter / 10.0;
  float TOFsensor2M = measure2.RangeMilliMeter / 10.0;

  // Bepaald welke sensor verderweg staat
  if (TOFsensor1 > TOFsensor2) {
    //Berekeningen
    float radAngle = (TOFsensor2M - TOFsensor1M) / 25.0;  // Berekend het verschil en deelt dit door de onderlinge afstand van de TOF sensoren
    float degAngle = radAngle * (180.0 / 3.14);           // Convert to degrees
    Serial.println(TOFsensor1M);
    Serial.println(TOFsensor2M);
    Serial.println(degAngle);  //Printen in terminal
    lcd.setCursor(5, 0);       //Waar de lcd print 1ste rij 6de kolom
    lcd.print(degAngle);       //Print de hoek in graden
    return degAngle;           //Geeft de hoek in graden terug om te gebruiken voor de regelaar

  } else {
    //Berekeningen
    float radAngle = (TOFsensor1M - TOFsensor2M) / 25.0;  // Berekend het verschil en deelt dit door de onderlinge afstand van de TOF sensoren
    float degAngle = radAngle * (180.0 / 3.14);           // Convert to degrees
    Serial.println(TOFsensor1M);
    Serial.println(TOFsensor2M);
    Serial.println(-degAngle);  //Printen in terminal
    lcd.setCursor(5, 0);        //Waar de lcd print 1ste rij 6de kolom
    lcd.print(-degAngle);       //Print de negatieve hoek in graden
    return -radAngle;           //Geeft de negatieve hoek in graden terug om te gebruiken voor de regelaar
  }
}
//Om de motor handmatig aan te sturen
void controlMotors(char command) {
  switch (command) {
    case '1':  // Fast forward
      Serial.println("Fast forward...");

      // set the motor speed and direction
      Regelaar_Iwan();
      /*
      digitalWrite(motorLinks, HIGH);               // direction = forward
      analogWrite(motorLinksPWM, 255 - PWM_FAST);   // PWM speed = fast
      digitalWrite(motorRechts, HIGH);              // direction = forward
      analogWrite(motorRechtsPWM, 255 - PWM_FAST);  // PWM speed = fast
      digitalWrite(motorZijkantPWM, HIGH);
      digitalWrite(motorZijkant, HIGH);
      digitalWrite(motorZijkantIn1, LOW);
      digitalWrite(motorZijkantIn2, HIGH);
      */
      break;
    case '2':  // Forward
      /*
      Serial.println("Forward...");
      digitalWrite(motorLinks, LOW);
      digitalWrite(motorRechts, LOW);
      digitalWrite(motorLinksPWM, HIGH);
      digitalWrite(motorRechtsPWM, HIGH);

      digitalWrite(motorLinks, LOW);
      digitalWrite(motorRechts, LOW);
      digitalWrite(motorLinksPWM, HIGH);
      digitalWrite(motorRechtsPWM, HIGH);
      delay(DIR_DELAY);
      // set the motor speed and direction
      digitalWrite(motorLinks, LOW);                // direction = forward
      analogWrite(motorLinksPWM, 255 - PWM_SLOW);   // PWM speed = slow
      digitalWrite(motorRechts, LOW);               // direction = forward
      analogWrite(motorRechtsPWM, 255 - PWM_SLOW);  // PWM speed = slow
      */
      digitalWrite(NOODSTOPRELAY, HIGH);
      delay(500);
      digitalWrite(BLOWRELAY, HIGH);
      delay(500);
      Regelaar_Iwan();
      break;
    case '3':  // Soft stop (coast)
      Serial.println("Soft stop (coast)...");
      digitalWrite(motorLinks, LOW);
      digitalWrite(motorLinksPWM, LOW);
      digitalWrite(motorRechts, LOW);
      digitalWrite(motorRechtsPWM, LOW);

      digitalWrite(motorZijkant, LOW);
      digitalWrite(BLOWRELAY, LOW);
      break;
    case '4':  // Reverse
      Serial.println("Reverse...");
      digitalWrite(motorLinks, LOW);          // direction = reverse
      analogWrite(motorLinksPWM, PWM_SLOW);   // PWM speed = slow
      digitalWrite(motorRechts, LOW);         // direction = reverse
      analogWrite(motorRechtsPWM, PWM_SLOW);  // PWM speed = slow
      break;
    case '5':  // Fast reverse
      Serial.println("Fast reverse...");
      digitalWrite(motorLinks, LOW);          // direction = reverse
      analogWrite(motorLinksPWM, PWM_FAST);   // PWM speed = fast
      digitalWrite(motorRechts, LOW);         // direction = reverse
      analogWrite(motorRechtsPWM, PWM_FAST);  // PWM speed = fast
      break;
    case '6':  // Hard stop (brake)
      Serial.println("Hard stop (brake)...");
      digitalWrite(motorLinks, HIGH);
      digitalWrite(motorLinksPWM, HIGH);
      digitalWrite(motorRechts, HIGH);
      digitalWrite(motorRechtsPWM, HIGH);
      break;
    case '7':
      Serial.println("Kleine motor...");
      digitalWrite(motorZijkantIn2, LOW);
      digitalWrite(motorZijkantIn1, HIGH);
      digitalWrite(BLOWRELAY, HIGH);
      digitalWrite(NOODSTOPRELAY, HIGH);
      break;
    case '8':
      Serial.println("Alleen hover...");
      digitalWrite(BLOWRELAY, HIGH);
      digitalWrite(NOODSTOPRELAY, HIGH);
      break;
    case '9':
      digitalWrite(NOODSTOPRELAY, HIGH);
      digitalWrite(motorLinks, HIGH);              // direction = forward
      analogWrite(motorLinksPWM, 255 - PWM_FAST);  // PWM speed = fast
      digitalWrite(motorZijkantPWM, HIGH);
      digitalWrite(motorZijkant, HIGH);
      digitalWrite(motorZijkantIn1, HIGH);
      digitalWrite(motorZijkantIn2, LOW);
    default:
      break;
  }
}
//Functie om de batterij uit te lezen
void checkBattery() {

  int aDC = -1023 + analogRead(ACCU_SAFETY_PIN);  //Uitlezen sensor
  float voltage = abs(aDC * 5 / 1023.);           // Omrekenen naar spanning
  float ampere = (voltage - 2.47) / 0.066;        // Omrekenen naar stroom

  Serial.println("Battery raw: " + String(aDC));
  Serial.println("Voltage: " + String(voltage));
  Serial.println("Battery Ampere: " + String(ampere));
  float ACSDC = stroommeter.getCurrentDC();

  Serial.println("Battery VOLT: " + String(ACSDC));

  if (ampere < LOW_BATTERY_THRESHOLD) {
    Serial.println("Low battery! Ampere: " + String(ampere));
    playFailTune();
    // Perform any additional actions like stopping the motors
    // digitalWrite(NOODSTOPRELAY, LOW);  //Zodat de noodstop wordt geactiveerd
    //controlMotors('6');                // Hard stop
  }
}
//Deze functie ken de in en output aan de pinnen toe
void setPins() {
  pinMode(buttonPin, INPUT);  // Denk dat dit niet nodig is?
  pinMode(BLOWRELAY, OUTPUT);
  pinMode(NOODSTOPRELAY, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(SHT_TOF1_PIN, OUTPUT);
  pinMode(SHT_TOF2_PIN, OUTPUT);
  pinMode(SHT_TOF3_PIN, OUTPUT);
  pinMode(motorZijkantIn1, OUTPUT);
  pinMode(motorZijkantIn2, OUTPUT);
  pinMode(motorZijkant, OUTPUT);
  pinMode(motorZijkantPWM, OUTPUT);
  pinMode(motorLinks, OUTPUT);
  pinMode(motorLinksPWM, OUTPUT);
  pinMode(motorRechts, OUTPUT);
  pinMode(motorRechtsPWM, OUTPUT);
  pinMode(BUZZER_PINP, OUTPUT);
  pinMode(BUZZER_PINM, OUTPUT);
}
//Deze functie zorgt dat de motoren niet gelijk aan gaan
void initWrites() {
  digitalWrite(BUZZER_PINM, LOW);
  digitalWrite(BUZZER_PINP, HIGH);
  digitalWrite(BUZZER_PIN, LOW);
  digitalWrite(motorZijkantIn1, LOW);
  digitalWrite(motorZijkantIn2, LOW);
  digitalWrite(motorZijkant, LOW);

  digitalWrite(motorLinks, LOW);
  digitalWrite(motorLinksPWM, LOW);

  digitalWrite(motorRechts, LOW);
  digitalWrite(motorRechtsPWM, LOW);
}
//Functie voor waardes uit gyro the halen.
void readGyro() {
  xyzFloat gValue = myMPU9250.getGValues();
  xyzFloat gyr = myMPU9250.getGyrValues();
  xyzFloat magValue = myMPU9250.getMagValues();
  xyzFloat gAngle = myMPU9250.getAngles();
  float temp = myMPU9250.getTemperature();
  float resultantG = myMPU9250.getResultantG(gValue);
}

//Functie om data te printen in serial
void dataPrinten() {
  //Voor gyroscoop
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
}
//Functie om de Arduino te resetten
void softwareReset() {
  // Disable interrupts
  cli();

  // Perform software reset
  wdt_enable(WDTO_15MS);
  while (1) {}  // Wait for watchdog timer to trigger reset
}
//Functie om te controleren of alle sensoren goed zijn geinit
void bootupCheck() {
  //Voor gyroscoop
  xyzFloat gValue = myMPU9250.getGValues();
  xyzFloat gyr = myMPU9250.getGyrValues();
  xyzFloat magValue = myMPU9250.getMagValues();
  xyzFloat gAngle = myMPU9250.getAngles();
  float temp = myMPU9250.getTemperature();
  float resultantG = myMPU9250.getResultantG(gValue);

  // Example error handling in reading accelerometer values
  int vX = gValue.x;  // Check een X waarde
  int gY = gyr.y;     // Check een Y waarde
  int rZ = gAngle.z;  // Check een Z waarde
  bool successX = vX;
  bool succesY = gY;
  bool succesZ = rZ;
  if (!successX && !succesY && !succesZ) {  // Check of ze allemaal niet zijn opgestart dan reboot
    playFailTune();
    Serial.println("Error reading GYRO values!");
    lcd.setCursor(0, 1);
    lcd.print("ERROR REBOOT");
    delay(500);
    softwareReset();
  } else {  // Bij goed opstarten wordt er een geluid afgespeeld
    playSuccessTune();
    lcd.setCursor(5, 1);
    lcd.print("Lets fly");
    delay(50);
  }
}



void Poolplaatsing_PD(float &Kp_PD, float &Kd_PD, float m, float Re_PD, float Im_PD) {
  Kp_PD = (Re_PD * Re_PD + Im_PD * Im_PD) * m;
  Kd_PD = 2 * Re_PD * m;
}
void Poolplaatsing_PID(float &Kp_PID, float &Kd_PID, float &Ki_PID, float Iz, float Re_PID, float Im_PID, float pool3) {
  Kp_PID = (Re_PID * Re_PID + Im_PID * Im_PID + 2 * Re_PID * pool3) * Iz;
  Kd_PID = (2 * Re_PID + pool3) * Iz;
  Ki_PID = (Re_PID * Re_PID + Im_PID * Im_PID) * pool3 * Iz;
}

void Regeling_PD(float &Fx_PD, float &Fy_PD, float Kp_PD, float Kd_PD, float sp_PD, float sx, float dt) {
  static float error_oud = 0;  // Initialize previous error
  float error = sp_PD - sx;
  float d_error = (error - error_oud) / dt;  // Calculate derivative of error
  error_oud = error;                         // Update previous error
  Fx_PD = Kp_PD * error + Kd_PD * d_error;
  Fy_PD = Fx_PD;  // Assuming Fy should be the same as Fx for this example

  Serial.print("Error: ");
  Serial.println(error);
  Serial.print("d_error: ");
  Serial.println(d_error);
  Serial.print("Fx: ");
  Serial.println(Fx_PD);
}
void Regeling_PID(float &Fx_PID, float &Fy_PID, float Kp_PID, float Kd_PID, float Ki_PID, float sp_PID, float theta, float dt) {
  static float error_oud = 0;  // Initialize previous error
  float error = sp_PID - theta;
  error_oud = error;
  float d_error = error - error_oud;
  float errorSom = errorSom + error * dt;
  Fx_PID = Kp_PID * error + Kd_PID * d_error / dt + Ki_PID * errorSom;
}
void motoraansturing_Iwan(float Fx_PD, float Fx_PID, float Fy_PD, float Fy_PID) {
  Motor_Rechts(Fx_PD, Fx_PID);
  Motor_Links(Fx_PD, Fx_PID);
  Motor_Midden(Fy_PD, Fy_PID);
}
void motoraansturing_Bram(float Fx_PID) {
  Motor_Rechts(Fx_PID);
  Motor_Links(Fx_PID);
}

void loop() {
  if (Serial.available()) {
    char command = Serial.read();
    controlMotors(command);
  }
  xyzFloat gValue = myMPU9250.getGValues();
  float resultantG = myMPU9250.getResultantG(gValue);
  xyzFloat gAngle = myMPU9250.getAngles() / (180.0 / 3.14);

  checkBattery();
  readGyro();
  dataPrinten();
  readDualSensors();
  communicatie();
  Regelaar_Iwan();
  //Regelaar_Bram();
  Regelaar_Jari();
  Regelaar_Jelle();
  Regelaar_Teun();
  Regelaar_Maurits();
  digitalWrite(NOODSTOPRELAY, HIGH);
  digitalWrite(BLOWRELAY, HIGH);
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(TOFsensor1);
  lcd.setCursor(0, 1);
  lcd.print(TOFsensor2);
  lcd.setCursor(5, 1);
  lcd.print(gAngle.z);
  calculateAngle();
  communicatie();
}