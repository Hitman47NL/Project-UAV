#include "Grove_Motor_Driver_TB6612FNG.h"
#include <Wire.h>
#include <Adafruit_VL53L0X.h>
#include <LiquidCrystal_I2C.h>
#include <math.h>  // Include the math library for atan2 function
#include <avr/io.h>
#include <avr/wdt.h>

// Hier worden de pin gedefined
#define motorLinksPWM 11    //  Motor B Input A
#define motorLinks 10       //  Motor B Input B
#define motorRechtsPWM 9  //  Motor A Input A
#define motorRechts 8     //  Motor A Input

#define BLOWRELAY 12
#define NOODSTOPRELAY 13

#define SHT_TOF1_PIN 34
#define SHT_TOF2_PIN 36
#define SHT_TOF3_PIN 38

// I2C adressen aangemaakt
#define TOF1_ADDR 0x30
#define TOF2_ADDR 0x31
#define TOF3_ADDR 0x32

// INT van de sensoren
float TOFsensor1;
float TOFsensor2;
float TOFsensor3;
float TOFsensor1M;
float TOFsensor2M;
float degAngle;
float radAngle;

Adafruit_VL53L0X tof1 = Adafruit_VL53L0X();
Adafruit_VL53L0X tof2 = Adafruit_VL53L0X();
Adafruit_VL53L0X tof3 = Adafruit_VL53L0X();

VL53L0X_RangingMeasurementData_t measure1;
VL53L0X_RangingMeasurementData_t measure2;
VL53L0X_RangingMeasurementData_t measure3;

// Hier worden de functies aangemaakt
void Regelaar_Iwan();
void Poolplaatsing_PD(float &Kp, float &Kd, float m, float Re, float Im);
void Regeling_PD(float &Fx, float &Fy, float Kp, float Kd, float sp, float sx, float dt);
void motoraansturing_Iwan(float Fx);

unsigned long previousMillis = 0;  // Store last time delay was updated

void setPins() {
  pinMode(BLOWRELAY, OUTPUT);
  pinMode(NOODSTOPRELAY, OUTPUT);
  pinMode(SHT_TOF1_PIN, OUTPUT);
  pinMode(SHT_TOF2_PIN, OUTPUT);
  pinMode(SHT_TOF3_PIN, OUTPUT);
  pinMode(motorLinks, OUTPUT);
  pinMode(motorLinksPWM, OUTPUT);
  pinMode(motorRechts, OUTPUT);
  pinMode(motorRechtsPWM, OUTPUT);
}

// Uitlezen sensoren
void readDualSensors() {
  tof1.rangingTest(&measure1, false);
  tof2.rangingTest(&measure2, false);
  tof3.rangingTest(&measure3, false);
  // Decleratie van de variable DMV functie. Alle afstanden zijn in mm
  TOFsensor1 = measure1.RangeMilliMeter;  // Sensor 1 afstand in mm
  TOFsensor2 = measure2.RangeMilliMeter;  // Sensor 2 afstand in mm
  TOFsensor3 = measure3.RangeMilliMeter;  // Sensor 3 afstand in mm
}

void initTOFsensors() {
  digitalWrite(SHT_TOF1_PIN, LOW);
  digitalWrite(SHT_TOF2_PIN, LOW);
  digitalWrite(SHT_TOF3_PIN, LOW);
  delay(10); // Short delay to ensure proper reset
  digitalWrite(SHT_TOF1_PIN, HIGH);
  digitalWrite(SHT_TOF2_PIN, HIGH);
  digitalWrite(SHT_TOF3_PIN, HIGH);
  delay(10); // Short delay to ensure proper initialization

  // Zet de pin van TOF1 hoog om adres te schrijven.
  digitalWrite(SHT_TOF1_PIN, HIGH);
  digitalWrite(SHT_TOF2_PIN, LOW);
  digitalWrite(SHT_TOF3_PIN, LOW);
  delay(10);

  // Schrijf het adres naar TOF1
  if (!tof1.begin(TOF1_ADDR)) {
    Serial.println(F("Failed to boot first VL53L0X"));  // Foutmelding
    // Eventuele mogelijkheid om fout code uit te breiden
  }

  // Zet pin van TOF2 hoog om adres te schrijven
  digitalWrite(SHT_TOF2_PIN, HIGH);
  delay(10);

  // Schrijf het adres naar TOF2
  if (!tof2.begin(TOF2_ADDR)) {
    Serial.println(F("Failed to boot second VL53L0X"));
    // Eventuele mogelijkheid om fout code uit te breiden
  }

  // Zet pin van TOF23 hoog om adres te schrijven
  digitalWrite(SHT_TOF3_PIN, HIGH);
  delay(10);

  // Schrijf het adres naar TOF3
  if (!tof3.begin(TOF3_ADDR)) {
    Serial.println(F("Failed to boot third VL53L0X"));
    // Eventuele mogelijkheid om fout code uit te breiden
  }
}

void Regelaar_Iwan() {
  readDualSensors();
  const long cyclustijd = 10;  // Cyclustijd in ms
  static long t_oud = 0;       // Initialize t_oud to 0 at the beginning
  long t_nw = millis();        // Get the current time in ms
  const float Re = 0.75, Im = 1.0;
  const float m = 1560;       // In kilo gram
  float dt = 1;                // Nodig voor de d_error / dt
  float Fx, Fy;        // Initialize Fx and Fy
  float sx = TOFsensor3/1000;       // Begin voor waarde van de regelaar in m
  float Kp, Kd;                // Paramateres voor de regelaar
  const float sp = 0.3;        // Setpoint voor het stoppen van de regelaar m

  Serial.print("TOFsensor3 (sx): ");
  Serial.println(sx, 4);

  Poolplaatsing_PD(Kp, Kd, m, Re, Im);
  Serial.print("Kp: ");
  Serial.println(Kp);
  Serial.print("Kd: ");
  Serial.println(Kd);

  if (t_nw - t_oud > cyclustijd) {  // Check if the cyclustijd has passed
    dt = (t_nw - t_oud) * 0.001;    // Calculate dt in seconds
    t_oud = t_nw;                   // Update t_oud to the current time

    Regeling_PD(Fx, Fy, Kp, Kd, sp, sx, dt);
    Serial.print("PD Output Fx: ");
    Serial.println(Fx);
    Serial.print("PD Output Fy: ");
    Serial.println(Fy);

    motoraansturing_Iwan(Fx / 2);   // Control the motors with half of Fx
  } 
}

void motoraansturing_Iwan(float Fx) {
  Motor_Rechts(Fx);
  Motor_Links(Fx);
}

void Poolplaatsing_PD(float &Kp, float &Kd, float m, float Re, float Im) {
  Kp = (Re * Re + Im * Im) * m;
  Kd = 2 * Re * m;
}

void Regeling_PD(float &Fx, float &Fy, float Kp, float Kd, float sp, float sx, float dt) {
  static float error_oud = 0;  // Initialize previous error
  float error = sp - sx;
  float d_error = (error - error_oud) / dt;  // Calculate derivative of error
  error_oud = error;  // Update previous error
  Fx = (Kp + 1.5) * error + (Kd - 0.5) * d_error;
  Fy = Fx;  // Assuming Fy should be the same as Fx for this example

  Serial.print("Error: ");
  Serial.println(error);
  Serial.print("d_error: ");
  Serial.println(d_error);
  Serial.print("Fx: ");
  Serial.println(Fx);
}

void Motor_Links(float Fx) {  // Dit is voor Maxon motor 1
  if (Fx > 0) {   // hierdoor gaat de uav achteruit
    Serial.print("PWM Maxon 1 pos: ");             
    float Fx_nega = Fx * -1;
    Serial.println(-0.00298 * Fx_nega * Fx_nega - 1.75115 * Fx_nega);
    digitalWrite(motorRechts, LOW); // dit is voor wanneer de hovercraft achterwaarts moet bewegen
    analogWrite(motorRechtsPWM, -0.00298 * Fx_nega * Fx_nega - 1.75115 * Fx_nega);
  } else {  // dit is voor wanneer de hovercraft voorwaartswaarts moet bewegen
    Serial.print("PWM Maxon 1 nega: ");             
    Serial.println(-0.00505 * Fx * Fx + 2.25550 * Fx);
    digitalWrite(motorRechts, HIGH);
    analogWrite(motorRechtsPWM, -0.00505 * Fx * Fx + 2.25550 * Fx);
  }
}

void Motor_Rechts(float Fx) {  // Dit is voor Maxon motor 2
  if (Fx > 0) {               // dit is voor wanneer de hovercraft achterwaarts moet bewegen
    Serial.print("PWM Maxon 2 pos: ");
    float Fx_nega = Fx * -1;             
    Serial.println(-0.00282 * Fx * Fx - 1.70725 * Fx);
    digitalWrite(motorLinks, LOW);
    analogWrite(motorLinksPWM, -0.00282 * Fx_nega * Fx_nega - 1.70725 * Fx_nega);
  } else {  // dit is voor wanneer de hovercraft voorwaarts moet bewegen
    Serial.print("PWM Maxon 2 nega: ");             
    Serial.println(-0.00425 * Fx * Fx + 2.077594 * Fx);
    digitalWrite(motorLinks, HIGH);
    analogWrite(motorLinksPWM, -0.00425 * Fx * Fx + 2.077594 * Fx);
  }
}

void setup() {
  Serial.begin(9600);

  digitalWrite(BLOWRELAY, LOW);       // Verander naar HIGH om de blowers gelijk aan te zetten
  digitalWrite(NOODSTOPRELAY, HIGH);  // Zorgt dat overal stroom naar toe kan

  setPins();       // Om de pinnen juist op in en output te zetten
  initTOFsensors();  // Om de TOF sensoren te initialiseren

  digitalWrite(motorLinks, LOW);
  digitalWrite(motorLinksPWM, LOW);

  digitalWrite(motorRechts, LOW);
  digitalWrite(motorRechtsPWM, LOW);
}

void loop() {
  digitalWrite(BLOWRELAY, HIGH);
  digitalWrite(NOODSTOPRELAY, HIGH);
  Regelaar_Iwan();
  //digitalWrite(motorLinks, LOW);
  //analogWrite(motorLinksPWM, 255);
}
