//Gemaakt door Jelle en Iwan

//Benodige libraries
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

//definitie van de pinnen
//PWM Pinnen
#define motorZijkantPWM 4  //Regelt de snelheid voor de y-as motor
#define BUZZER_PINM 5
#define BUZZER_PINP 6
#define BUZZER_PIN 7
#define motorRechtsPWM 9  //Regelt de snelheid voor de x-as motor 2
#define motorLinksPWM 11  //Regelt de snelheid voor de x-as motor 1

//Digitale pinnen
#define LCD_ROWS 2
#define motorZijkant 3  //Zorgt er voor welke kant de motor op draait
#define motorRechts 8   //Zorgt er voor welke kant de motor op draait
#define motorLinks 10   //Zorgt er voor welke kant de motor op draait
#define BLOWRELAY 12
#define NOODSTOPRELAY 13
#define LCD_COLUMNS 16
const int TXPin = 18;
const int RXPin = 19;
#define SHT_TOF1_PIN 34
#define SHT_TOF2_PIN 36
#define SHT_TOF3_PIN 38
#define ACCU_SHUT_OFF 45

//Analoge pinnen
#define ACCU_SAFETY_PIN A0 //Zorgt ervoor dat stroommeter uitgelezen kan worden

//I2C adressen
#define LCD_ADDR 0x27
#define TOF1_ADDR 0x30
#define TOF2_ADDR 0x31
#define TOF3_ADDR 0x32
#define MPU9250_ADDR 0x68

//constante definineren
#define LOW_BATTERY_THRESHOLD 6.95 // threshold amperage
int TOFsensor1 = 0.0;  //mm
int TOFsensor2 = 0.0;  //mm
int TOFsensor3 = 0.0;  //mm
float TOFsensor1M = 0.0;
float TOFsensor2M = 0.0;
float degAngle = 0.0; // In graden
float radAngle = 0.0; // In radialen
int receivedX = 0;  // Variable to store the received X coordinate
int receivedY = 0;  // Variable to store the received Y coordinate
int aDC = -1023 + analogRead(ACCU_SAFETY_PIN);  //Uitlezen sensor
float voltage = abs(aDC * 5 / 1023.);           // Omrekenen naar spanning
float ampere = (voltage - 2.47) / 0.066;        // Omrekenen naar stroom
int consecutiveCount = 0;
long timer = 0;
unsigned long startTime = 0;
ACS712 stroommeter = ACS712(ACS712_30A, ACCU_SAFETY_PIN);

//Voor het aanzetten van de regelaars
bool Regelaar_Teun_Succes = false;
bool Regelaar_Jelle_Succes = false;
bool Regelaar_Bram_Succes  = false;
bool Regelaar_Iwan_Succes  = false;
bool Regelaar_Jari_Succes = false;
bool Regelaar_Maurits_Succes = false;
bool isInRange = false;

//RPI 4 uitlezing
SoftwareSerial mySerial(RXPin, TXPin);

//Uitlezen ToF sensoren
Adafruit_VL53L0X tof1 = Adafruit_VL53L0X();
Adafruit_VL53L0X tof2 = Adafruit_VL53L0X();
Adafruit_VL53L0X tof3 = Adafruit_VL53L0X();
VL53L0X_RangingMeasurementData_t measure1;
VL53L0X_RangingMeasurementData_t measure2;
VL53L0X_RangingMeasurementData_t measure3;

//LCD aanzetten
LiquidCrystal_I2C lcd(LCD_ADDR, LCD_COLUMNS, LCD_ROWS);
MPU6050 mpu(Wire);

//Reset na opstart arduino
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


// Controller states
enum ControllerState {
  Teun, Jelle, Bram, Iwan, Jari, Maurits
};

ControllerState currentState = Teun;

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
    Regelaars();
}
