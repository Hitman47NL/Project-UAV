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
void bootupCheck() {
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


void setup() {
  // put your setup code here, to run once:
}

void loop() {
  // put your main code here, to run repeatedly:
}
