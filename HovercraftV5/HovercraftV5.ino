// Gemaakt door Jelle en Iwan

//Libraries
#include "Wire.h"
#include <MPU6050_light.h>
#include <Wire.h>
#include <Adafruit_VL53L0X.h>
#include <LiquidCrystal_I2C.h>
#include <math.h>  // Include the math library for atan2 function
#include <SoftwareSerial.h>
#include <avr/io.h>
#include <avr/wdt.h>
#include "ACS712.h"

//Define pinnen voor Arduino
//PWM
#define BUZZER_PINP 6
#define BUZZER_PIN 7
#define BUZZER_PINM 5
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


#define motorZijkantPWM 4
#define motorZijkant 3

//Analoog
#define ACCU_SAFETY_PIN A0
#define ACCU_SHUT_OFF 45

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

#define LOW_BATTERY_THRESHOLD 6.95 // threshold amperage
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
MPU6050 mpu(Wire);

ACS712 stroommeter = ACS712(ACS712_30A, ACCU_SAFETY_PIN);
int receivedX = 0;  // Variable to store the received X coordinate
int receivedY = 0;  // Variable to store the received Y coordinate
  int consecutiveCount = 0;
long timer = 0;
 float Kp, Kd, Ki;
 float errorSom = 0.0; // Initieer som van errors
unsigned long startTime = 0;
//Voor TOFsensoren
int TOFsensor1 = 0.0;  //mm
int TOFsensor2 = 0.0;  //mm
int TOFsensor3 = 0.0;  //mm
float TOFsensor1M = 0.0;
float TOFsensor2M = 0.0;
float degAngle = 0.0;
float radAngle = 0.0;
int aDC = -1023 + analogRead(ACCU_SAFETY_PIN);  //Uitlezen sensor
float voltage = abs(aDC * 5 / 1023.);           // Omrekenen naar spanning
float ampere = (voltage - 2.47) / 0.066;        // Omrekenen naar stroom

bool Regelaar_Teun_Succes = false;
bool Regelaar_Jelle_Succes = false;
bool Regelaar_Bram_Succes  = false;
bool Regelaar_Iwan_Succes  = false;
bool Regelaar_Jari_Succes = false;
bool Regelaar_Maurits_Succes = false;
bool isInRange = false;
// Benodigde functies
void Motor_Rechts(float Fx);
void Motor_Links(float Fx);
void Motor_midden(float Fy);

void shutOFF(){
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("ERROR!");
    playFailTune();
    digitalWrite(NOODSTOPRELAY, LOW);
    digitalWrite(BLOWRELAY, LOW);
}
void softwareReset() {
  // Disable interrupts
  cli();

  // Perform software reset
  wdt_enable(WDTO_15MS);
  while (1) {}  // Wait for watchdog timer to trigger reset
}
//Functie om de batterij uit te lezen
void checkBattery() {
  Serial.println("Battery raw: " + String(aDC));
  Serial.println("Voltage: " + String(voltage));
  Serial.println("Battery Ampere: " + String(ampere));
  float ACSDC = stroommeter.getCurrentDC();

  Serial.println("Battery VOLT: " + String(ACSDC));

  if(ampere >= LOW_BATTERY_THRESHOLD || ACCU_SHUT_OFF == 1) {
    Serial.println("LOW battery! Ampere: " + String(ampere));
    playFailTune();
    digitalWrite(NOODSTOPRELAY, LOW);
    digitalWrite(BLOWRELAY, LOW);
    while(1);
  }
}
void bootupCheck() {
  // Example error handling in reading accelerometer values
  bool successX = mpu.getAccAngleX();
  bool succesY = mpu.getAngleZ(); 
   if (!successX && !succesY) {  // Check of ze allemaal niet zijn opgestart dan reboot
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
    delay(500);
  }
}
//Functie om geluid af te spelen
void playTone(int tone, int duration) {
  for (long i = 0; i < duration * 100L; i += tone * 2) {
    digitalWrite(BUZZER_PIN, HIGH);
    delayMicroseconds(tone);
    digitalWrite(BUZZER_PIN, LOW);
    delayMicroseconds(tone);
  }
}
//Geluid voor succes
void playSuccessTune() {
  int melody[] = { 220, 196, 165, 131 };
  int noteDuration = 150;

  for (int note : melody) {
    playTone(note, noteDuration);
    delay(150);
  }
}
//Geluid voor vaal
void playFailTune() {
  int melody[] = { 294, 392, 600, 1600 };
  int noteDuration = 150;

  for (int note : melody) {
    playTone(note, noteDuration);
    delay(150);
  }
}
void initGyro(){
  
  byte status = mpu.begin();
  Serial.print(F("MPU6050 status: "));
  Serial.println(status);
  while(status!=0){ } // stop everything if could not connect to MPU6050
  
  Serial.println(F("Calculating offsets, do not move MPU6050"));
  delay(1000);
  mpu.calcOffsets(true,true); // gyro and accelero
  Serial.println("Done!\n");
  delay(10);
}
//Deze functie ken de in en output aan de pinnen toe
void setPins() {
 
  pinMode(ACCU_SHUT_OFF, INPUT);
  pinMode(BLOWRELAY, OUTPUT);
  pinMode(NOODSTOPRELAY, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(SHT_TOF1_PIN, OUTPUT);
  pinMode(SHT_TOF2_PIN, OUTPUT);
  pinMode(SHT_TOF3_PIN, OUTPUT);
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
  digitalWrite(NOODSTOPRELAY, HIGH);
  digitalWrite(BUZZER_PINM, LOW);
  digitalWrite(BUZZER_PINP, HIGH);
  digitalWrite(BUZZER_PIN, LOW);
  digitalWrite(motorZijkantPWM, LOW);
  digitalWrite(motorZijkant, LOW);

  digitalWrite(motorLinks, LOW);
  digitalWrite(motorLinksPWM, LOW);

  digitalWrite(motorRechts, LOW);
  digitalWrite(motorRechtsPWM, LOW);
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
   // lcd.print(degAngle);       //Print de hoek in graden
    return degAngle;           //Geeft de hoek in graden terug om te gebruiken voor de regelaar

  } else {
    //Berekeningen
    float radAngle = (TOFsensor1M - TOFsensor2M) / 25.0;  // Berekend het verschil en deelt dit door de onderlinge afstand van de TOF sensoren
    float degAngle = radAngle * (180.0 / 3.14);           // Convert to degrees
    Serial.println(TOFsensor1M);
    Serial.println(TOFsensor2M);
    Serial.println(-degAngle);  //Printen in terminal
    lcd.setCursor(5, 0);        //Waar de lcd print 1ste rij 6de kolom
   // lcd.print(-degAngle);       //Print de negatieve hoek in graden
    return -radAngle;           //Geeft de negatieve hoek in graden terug om te gebruiken voor de regelaar
  }
}

// Aansturing van de motoren, graag niet aanpassen
void Motor_Links(float Fx) {  // Dit is voor Maxon motor 1
  if (Fx > 0) {   // hierdoor gaat de uav achteruit
    Serial.print("PWM Maxon 1 pos: ");             
    float Fx_nega = Fx * -1;
    Serial.println(-0.00298 * Fx_nega * Fx_nega - 1.75115 * Fx_nega);
    digitalWrite(motorRechts, LOW); // dit is voor wanneer de hovercraft achterwaarts moet bewegen
    analogWrite(motorRechtsPWM, constrain(-0.00298 * Fx_nega * Fx_nega - 1.75115 * Fx_nega, -255, -50));
  } else {  // dit is voor wanneer de hovercraft voorwaartswaarts moet bewegen
    Serial.print("PWM Maxon 1 nega: ");             
    Serial.println(-0.00505 * Fx * Fx + 2.25550 * Fx);
    digitalWrite(motorRechts, HIGH);
    analogWrite(motorRechtsPWM, constrain(-0.00505 * Fx * Fx + 2.25550 * Fx, 50, 255) );
  }
}

void Motor_Rechts(float Fx) {  // Dit is voor Maxon motor 2
  if (Fx > 0) {               // dit is voor wanneer de hovercraft achterwaarts moet bewegen
    Serial.print("PWM Maxon 2 pos: ");
    float Fx_nega = Fx * -1;             
    Serial.println(-0.00282 * Fx * Fx - 1.70725 * Fx);
    digitalWrite(motorLinks, LOW);
    analogWrite(motorLinksPWM, constrain(-0.00282 * Fx_nega * Fx_nega - 1.70725 * Fx_nega, -255, -50));
  } else {  // dit is voor wanneer de hovercraft voorwaarts moet bewegen
    Serial.print("PWM Maxon 2 nega: ");             
    Serial.println(-0.00425 * Fx * Fx + 2.077594 * Fx);
    digitalWrite(motorLinks, HIGH);
    analogWrite(motorLinksPWM, constrain(-0.00425 * Fx * Fx + 2.077594 * Fx, 50, 255));
  }
}

void Motor_midden(float Fy) {  // For the small motor
  if (Fy < 0) {                // When hovercraft needs to move left
    Serial.print("PWM Midden links beweging ");
    Serial.print(0.00519 * Fy * Fy - 0.67075 * Fy); 
    digitalWrite(motorZijkant, HIGH);
    analogWrite(motorZijkantPWM, constrain(0.01060 * Fy * Fy + 1.12248 * Fy, -255, -100));
  } else {  // When hovercraft needs to move right
    Serial.print("PWM Midden rechts beweging ");
    Serial.println(0.01060 * Fy * Fy + 1.12248 * Fy);
    digitalWrite(motorZijkant, LOW);
    analogWrite(motorZijkantPWM, constrain(0.00519 * Fy * Fy - 0.67075 * Fy, 100, 255));
  }
}
// Controller states
enum ControllerState {
  Teun, Jelle, Bram, Iwan, Jari, Maurits
};

ControllerState currentState = Teun;
void Regelaars(){
    switch (currentState) {
    case Teun:
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Regelaar 1");
    
      Regelaar_Teun();
      if(Regelaar_Teun_Succes){
        currentState = Jelle;
      }
      break;

    case Jelle:
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Regelaar 2");
      Regelaar_Jelle();
      if(Regelaar_Jelle_Succes){
        currentState = Iwan;
      }
      break;

    case Bram:
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Regelaar 3");
      Regelaar_Bram();
      if(Regelaar_Bram_Succes){
        currentState = Iwan;
      }      
      break;

    case Iwan:
        lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Regelaar 4");
      Regelaar_Iwan();
            if(Regelaar_Iwan_Succes){
        currentState = Jari;
      }  
      break;

    case Jari:
            lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Regelaar 5");
      Regelaar_Jari();
                  if(Regelaar_Jari_Succes){
        currentState = Maurits;
      }  
      break;

    case Maurits:
                lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Regelaar 6");
    shutOFF();
    digitalWrite(BLOWRELAY, LOW);
      //Regelaar_Maurits();
      delay(500);
      softwareReset();
      break;

    default:
      // Handle unexpected states
      break;
  }
}
void setup() {
  Serial.begin(9600);
  mySerial.begin(9600);
  Wire.begin();
  //LCD
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("Starting up");  //Zodat de user weet dat er iets gebeurd
  //Serial
  Serial.println("Starting...");  //Zodat de programmeur weet dat er iets gebeurd
  digitalWrite(NOODSTOPRELAY, HIGH);
  digitalWrite(BLOWRELAY, LOW);
  initGyro();
  setPins();       //Om de pinnen juist op in en output te zetten
  initWrites();    // Om de motoren niet gelijk te laten draaien
  initTOFsensors();
  bootupCheck();

 Serial.println("Done!");  //Zodat de programmeur weet dat er iets gebeurd
 lcd.setCursor(0, 0); lcd.print("Done!");  //Zodat de user weet dat er iets gebeurd
 lcd.clear();
}

void loop() {
    mpu.update();
    Serial.println(F("=====================================================\n"));
    Serial.print(F("ANGLE"));    Serial.print("\tZ: ");Serial.println(mpu.getAngleZ());
    Serial.println(F("=====================================================\n"));
    //checkBattery();
    digitalWrite(BLOWRELAY, HIGH);
    readDualSensors();
    calculateAngle();
    lcd.setCursor(0, 0);
    lcd.print(TOFsensor1);
    lcd.setCursor(0, 1);
    lcd.print(TOFsensor2);
    lcd.setCursor(5, 1);
    lcd.print(mpu.getAngleZ());
    attachInterrupt(digitalPinToInterrupt(ACCU_SHUT_OFF), shutOFF, LOW);
    attachInterrupt(ACCU_SAFETY_PIN, shutOFF, LOW);
    //Regelaars();
    Regelaar_Teun();
}
