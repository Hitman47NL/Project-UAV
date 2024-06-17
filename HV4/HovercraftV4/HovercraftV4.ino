// Gemaakt door Jelle en Iwan

//Libraries
#include <Wire.h>
#include <Adafruit_VL53L0X.h>
#include <LiquidCrystal_I2C.h>
#include <math.h>  // Include the math library for atan2 function
#include <MPU9250_WE.h>
#include <SoftwareSerial.h>
#include <avr/io.h>
#include <avr/wdt.h>
#include "ACS712.h"
#include <SPI.h>
#include <SD.h>

//Define pinnen voor Arduino
//PWM
#define BUZZER_PINP 7
#define BUZZER_PIN 5
#define BUZZER_PINM 6
#define motorLinksPWM 11  //  Motor B Input A
#define motorLinks 10     //  Motor B Input B
#define motorRechtsPWM 9  //  Motor A Input A
#define motorRechts 8     //  Motor A Input B
#define chipSelect 53  // CS pin voor de SD-kaartmodule - pas deze aan naar de CS pin van jouw bordje!

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
int TOFsensor1 = 0.0;  //mm
int TOFsensor2 = 0.0;  //mm
int TOFsensor3 = 0.0;  //mm
float TOFsensor1M = 0.0;
float TOFsensor2M = 0.0;
float degAngle = 0.0;
float radAngle = 0.0;

// Benodigde functies
void Motor_Rechts(float Fx);
void Motor_Links(float Fx);
void Motor_midden(float Fy);

void batterij_control(){
    if (ACCU_SHUT_OFF == 1){
    Emergency();
  }
}

void Emergency() {
  digitalWrite(NOODSTOPRELAY, LOW);
  digitalWrite(BLOWRELAY, LOW);
  while(1){
    Serial.println("PANIEK!!!");
  }
}
struct SensorData {
  long time;
  float value1;
  float value2;
  float value3;
};

// Definieer maximale grootte van buffer
#define maxBufferSize 4096

void setup() {
  Serial.begin(115200);
  
  // Initialisatie van de SD-kaart
  if (!SD.begin(chipSelect)) {
    Serial.println("\nInitialisatie van de SD-kaart mislukt!");
    return;
  }
  Serial.println("\nInitialisatie van de SD-kaart voltooid.");
}

void writeDataToFile(byte *buffer, unsigned bufferSize) {
  unsigned fileCounter = 0;
  String filename;
  do {
    filename = "data" + String(fileCounter) + ".bin";
    fileCounter++;
  } while (SD.exists(filename)); // Controleer of het bestand al bestaat
  File dataFile = SD.open(filename, FILE_WRITE);
  
  if (dataFile) {
    Serial.println("Schrijven naar " + filename + "...");
    // Schrijven van de buffer naar het bestand
    dataFile.write(buffer, bufferSize);
    
    // Sluiten van het bestand
    dataFile.close();
    Serial.println("Schrijven naar " + filename + " voltooid");
  } else {
    Serial.println("Fout bij het openen van het bestand");
  }
}

void data() {
  static const unsigned numElements = maxBufferSize / sizeof(SensorData); // Bereken hoeveel elementen (structs) er weggeschreven kunnen worden in de buffer.
  static SensorData buffer[numElements];
  static boolean stopWriting = false;
  static const long samplingTimeDataLogging = 100;  // ms, bepaalt frequentie waarmee sensordata wordt weggeschreven.
  static const long timeStopWriting = 60000;  // ms, bepaalt frequentie waarmee sensordata wordt weggeschreven.
  static long t_lastDataLog = 0, t_new;  // ms, ms;
  static int currentIndex = 0;

  t_new = millis();    // Lees de tijd
  if (t_new - t_lastDataLog > samplingTimeDataLogging) {
    t_lastDataLog = t_new;

    if (!stopWriting){
      buffer[currentIndex].time = t_new;

      // Vul de struct met je meetwaardes. 
      buffer[currentIndex].value1 = currentIndex * 1.5;  
      buffer[currentIndex].value2 = currentIndex * 2.5;
      buffer[currentIndex].value3 = currentIndex * 3.5;
      
      currentIndex++;

      // Schrijf volle buffer weg naar een bestand
      if (currentIndex == numElements)
      {
        writeDataToFile((byte*)buffer, numElements * sizeof(SensorData));
        currentIndex = 0;
      }

      // Stop na een bepaalde tijd met het wegschrijven van bestanden
      if(t_new > timeStopWriting)
      {
        Serial.println("Gestopt met meetdata wegschrijven.");
        stopWriting = true;
      }
    }
  }
}


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
  pinMode(ACCU_SHUT_OFF, INPUT);
  //attachInterrupt(digitalPinToInterrupt(ACCU_SHUT_OFF), Emergency, CHANGE);
  batterij_control();
  setPins();       //Om de pinnen juist op in en output te zetten
  initWrites();    // Om de motoren niet gelijk te laten draaien
  delay(100);
  initTOFsensors();  // Om de TOF sensoren te initialiseren
  initGyroSensor();  // Om de gyro te initialiseren
  digitalWrite(motorZijkant, HIGH);

  bootupCheck();
  delay(100);
  /*
  digitalWrite(NOODSTOPRELAY, HIGH);
  delay(500);
  digitalWrite(BLOWRELAY, HIGH);
  delay(100);
  */
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

void Poolplaatsing_PD(float &Kp, float &Kd, float m, float Re, float Im) {
  Kp = (Re * Re + Im * Im) * m - 1.5;
  Kd = 2 * Re * m + 1.0;
}

void Regeling_PD(float &Fy, float Kp, float Kd, float sp, float sx, float dt) {
  static float error_oud = 0;  // Initialize previous error
  float error = sp - sx;
  float d_error = (error - error_oud) / dt;  // Calculate derivative of error
  error_oud = error;                         // Update previous error
  Fy = Kp * error + Kd * d_error;
  //Fy = Fx;  // Assuming Fy should be the same as Fx for this example

  Serial.print("Error: ");
  Serial.println(error);
  Serial.print("d_error: ");
  Serial.println(d_error);
  Serial.print("Fx: ");
  Serial.println(Fy);
}

// Aansturing van de motoren, graag niet aanpassen
void Motor_Links(float Fx) {  // Dit is voor Maxon motor 1
  if (Fx > 0) {               // hierdoor gaat de uav achteruit
    Serial.print("PWM Maxon 1 pos: ");
    //float Fx_nega = Fx * -1;
    Serial.println(-0.00298 * Fx * Fx - 1.75115 * Fx);
    digitalWrite(motorRechts, LOW);  // dit is voor wanneer de hovercraft achterwaarts moet bewegen
    analogWrite(motorRechtsPWM, -0.00298 * Fx * Fx - 1.75115 * Fx);
  } else {  // dit is voor wanneer de hovercraft voorwaartswaarts moet bewegen
    Serial.print("PWM Maxon 1 nega: ");
    Serial.println(-0.00505 * Fx * Fx + 2.25550 * Fx);
    digitalWrite(motorRechts, HIGH);
    analogWrite(motorRechtsPWM, -0.00505 * Fx * Fx + 2.25550 * Fx);
  }
}

void Motor_Rechts(float Fx) {  // Dit is voor Maxon motor 2
  if (Fx > 0) {                // dit is voor wanneer de hovercraft achterwaarts moet bewegen
    Serial.print("PWM Maxon 2 pos: ");
    //float Fx_nega = Fx * -1;
    Serial.println(-0.00282 * Fx * Fx - 1.70725 * Fx);
    digitalWrite(motorLinks, LOW);
    analogWrite(motorLinksPWM, -0.00282 * Fx * Fx - 1.70725 * Fx);
  } else {  // dit is voor wanneer de hovercraft voorwaarts moet bewegen
    Serial.print("PWM Maxon 2 nega: ");
    Serial.println(-0.00425 * Fx * Fx + 2.077594 * Fx);
    digitalWrite(motorLinks, HIGH);
    analogWrite(motorLinksPWM, -0.00425 * Fx * Fx + 2.077594 * Fx);
  }
}

void Motor_midden(float Fy) {  // For the small motor
  if (Fy < 0) {                // When hovercraft needs to move left
    Serial.print("PWM Midden links beweging ");
    Serial.print(0.00519 * Fy * Fy - 0.67075 * Fy); 
    digitalWrite(motorZijkant, HIGH);
    analogWrite(motorZijkantPWM, 0.01060 * Fy * Fy + 1.12248 * Fy);
  } else {  // When hovercraft needs to move right
    Serial.print("PWM Midden rechts beweging ");
    Serial.println(0.01060 * Fy * Fy + 1.12248 * Fy);
    digitalWrite(motorZijkant, LOW);
    analogWrite(motorZijkantPWM, 0.00519 * Fy * Fy - 0.67075 * Fy);
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

  if (ampere >= LOW_BATTERY_THRESHOLD) {
    Serial.println("High battery! Ampere: " + String(ampere));
    playFailTune();
    Emergency();
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
  digitalWrite(motorZijkantPWM, LOW);
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

void loop() {
  /*
  if (Serial.available()) {
    char command = Serial.read();
    controlMotors(command);
  }
  */
  xyzFloat gValue = myMPU9250.getGValues();
  float resultantG = myMPU9250.getResultantG(gValue);
  xyzFloat gAngle = myMPU9250.getAngles() / (180.0 / 3.14);
  checkBattery();
  batterij_control();
  readGyro();
  dataPrinten();
  readDualSensors();
  communicatie();
  //Regelaar_Iwan();
  //Regelaar_Bram();
  //Regelaar_Jari();
  //Regelaar_Jelle();
  Regelaar_Teun();
  //Regelaar_Maurits();
  digitalWrite(NOODSTOPRELAY, HIGH);
  digitalWrite(BLOWRELAY, LOW);
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