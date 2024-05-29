//Gemaakt door Jelle
//Hiermee kunnen de MPU9250 en de TOF sensoren worden uitgelezen

#include "Grove_Motor_Driver_TB6612FNG.h"
#include <Wire.h>
#include <Adafruit_VL53L0X.h>
#include <LiquidCrystal_I2C.h>
#include <math.h>  // Include the math library for atan2 function
#include <MPU9250_WE.h>
#include <Motor_PID.h>

#define lampje 33
#define TOF1_ADDRESS 0x30
#define TOF2_ADDRESS 0x31
#define TOF3_ADDRESS 0x32

#define MPU9250_ADDR 0x68
// wired connections
#define L9110S_B_IA 9   //  Motor B Input A --> MOTOR B +
#define L9110S_B_IB 8   //  Motor B Input B --> MOTOR B -
#define L9110S_A_IA 11  //  Motor A Input A --> MOTOR A +
#define L9110S_A_IB 10  //  Motor A Input B --> MOTOR A -
#define Buzzer 6        // buzzer voor geluid
#define relay1 12       // pin waarop relay1 is aangesloten
#define relay2 13       // pin waarop relay2 is aangesloten
// functional connections
#define MOTOR_B_PWM L9110S_B_IA  // Motor B PWM Speed
#define MOTOR_B_DIR L9110S_B_IB  // Motor B Direction

#define MOTOR_A_PWM L9110S_A_IA  // Motor A PWM Speed
#define MOTOR_A_DIR L9110S_A_IB  // Motor A Direction

// the actual values for "fast" and "slow" depend on the motor
#define PWM_SLOW 50     // arbitrary slow speed PWM duty cycle
#define PWM_FAST 200    // arbitrary fast speed PWM duty cycle
#define DIR_DELAY 1000  // brief delay for abrupt motor changes

#define SHT_TOF1 51
#define SHT_TOF2 53
#define SHT_TOF3 52

#define LCD_ADDRESS 0x4F
#define LCD_COLUMNS 16
#define LCD_ROWS 2

Adafruit_VL53L0X tof1 = Adafruit_VL53L0X();
Adafruit_VL53L0X tof2 = Adafruit_VL53L0X();
Adafruit_VL53L0X tof3 = Adafruit_VL53L0X();

#define out_STBY 41
#define out_A_PWM 2
#define out_A_IN2 39
#define out_A_IN1 37
//#define motor_A 23

#define accuSafety A8
int accu = digitalRead(accuSafety);
VL53L0X_RangingMeasurementData_t measure1;
VL53L0X_RangingMeasurementData_t measure2;
VL53L0X_RangingMeasurementData_t measure3;

int sensor1 = 0;
int sensor2 = 0;
int sensor3 = 0;

unsigned long previousMillis = 0;  // Stores the last time the trigger was sent
const long interval = 5000;        // Interval at which to send the trigger (5000 milliseconds or 5 seconds)
bool triggerSwitch = false;        // Switch state for sending the trigger

// Variables to store received coordinates
float X_coord = 0.0;
float Y_coord = 0.0;
float Angle_coord = 0.0;

LiquidCrystal_I2C lcd(LCD_ADDRESS, LCD_COLUMNS, LCD_ROWS);
MPU9250_WE myMPU9250 = MPU9250_WE(MPU9250_ADDR);
#define LCDPin 48

//MPU 9250
xyzFloat gValue = myMPU9250.getGValues();
xyzFloat gyr = myMPU9250.getGyrValues();
xyzFloat magValue = myMPU9250.getMagValues();
xyzFloat gAngle = myMPU9250.getAngles();
float temp = myMPU9250.getTemperature();
float resultantG = myMPU9250.getResultantG(gValue);


void playTone(int tone, int duration) {
  for (long i = 0; i < duration * 100L; i += tone * 2) {
    digitalWrite(Buzzer, HIGH);
    delayMicroseconds(tone);
    digitalWrite(Buzzer, LOW);
    delayMicroseconds(tone);
  }
}

void playSuccessTune() {
  int melody[] = { 220, 196, 165, 131 };
  int noteDuration = 150;

  for (int note : melody) {
    playTone(note, noteDuration);
    delay(150);
  }
}

void playFailTune() {
  int melody[] = { 294, 392, 600, 1600 };
  int noteDuration = 150;

  for (int note : melody) {
    playTone(note, noteDuration);
    delay(150);
  }
}

void initializeSensors() {
  //Alle sht pinnen laag
  digitalWrite(SHT_TOF1, LOW);
  digitalWrite(SHT_TOF2, LOW);
  digitalWrite(SHT_TOF3, LOW);
  delay(10);
  //alle sht pinnen hoog
  digitalWrite(SHT_TOF1, HIGH);
  digitalWrite(SHT_TOF2, HIGH);
  digitalWrite(SHT_TOF3, HIGH);
  delay(10);
  //Stel adres van tof 1 in
  digitalWrite(SHT_TOF1, HIGH);
  digitalWrite(SHT_TOF2, LOW);
  digitalWrite(SHT_TOF3, LOW);
  if (!tof1.begin(TOF1_ADDRESS)) {
    Serial.println(F("Failed to boot first VL53L0X"));
    // while(1);
  }
  delay(10);
  //stel adres tof 2 in
  digitalWrite(SHT_TOF2, HIGH);
  if (!tof2.begin(TOF2_ADDRESS)) {
    Serial.println(F("Failed to boot second VL53L0X"));
    //  while(1);
  }
  delay(10);
  //Stel adres in tof 3
  digitalWrite(SHT_TOF3, HIGH);
  if (!tof3.begin(TOF3_ADDRESS)) {
    Serial.println(F("Failed to boot third VL53L0X"));
    // while(1);
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
  if (sensor1 < sensor2) {
    float rawAngle = (sensor1 - sensor2) / 10;  // Calculate raw angle difference
    float ang = atan(rawAngle) * 180 / 3.14;    // Convert to degrees
    lcd.setCursor(5, 0);
    lcd.print(ang);
    return ang;
  } else {
    float rawAngle = (sensor2 - sensor1) / 10;  // Calculate raw angle difference
    float ang = -atan(rawAngle) * 180 / 3.14;   // Convert to degrees
    lcd.setCursor(5, 0);
    lcd.print(ang);
    return ang;
  }
}
// Function to handle coordinate transformation
void transformCoordinates() {
  int new_x_coords = X_coord - 10;  // Waarom hier deze getallen eraf halen?
  int new_y_coords = Y_coord - 1;
  int new_angle_coords = Angle_coord - 45;

  Serial.print("new_x: ");
  Serial.println(new_x_coords);

  Serial.print("new_y: ");
  Serial.println(new_y_coords);

  Serial.print("new_angle: ");
  Serial.println(new_angle_coords);
}
void errorCheck() {
}
/*
void i2cscan(){
  
  byte error, address;
  int nDevices;
 
  Serial.println("Scanning...");
 
  nDevices = 0;
  for(address = 1; address < 127; address++ )
  {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
 
    if (error == 0)
    {
      Serial.print("I2C device found at address 0x");
      if (address<16)
        Serial.print("0");
      Serial.print(address,HEX);
      Serial.println("  !");
 
      nDevices++;
    }
    else if (error==4)
    {
      Serial.print("Unknown error at address 0x");
      if (address<16)
        Serial.print("0");
      Serial.println(address,HEX);
    }    
  }
  if (nDevices == 0)
    Serial.println("No I2C devices found\n");
  else
    Serial.println("done\n");
 
  delay(500);           // wait 5 seconds for next scan

}*/
void setup() {
  Serial.begin(9600);   // Start communication with the computer (for debugging)
  Serial1.begin(9600);  // Start communication on serial port 1
  // LCD start
  lcd.init();
  lcd.backlight();
  Serial.println("Starting...");
  lcd.setCursor(0, 0);
  lcd.print("Starting up");

  // Communicatie
  Wire.begin();

  //Instellen van de pinnen output
  pinMode(LCDPin, OUTPUT);
  pinMode(lampje, OUTPUT);
  pinMode(out_STBY, OUTPUT);
  pinMode(out_A_PWM, OUTPUT);
  pinMode(out_A_IN2, OUTPUT);
  pinMode(out_A_IN1, OUTPUT);
  // pinMode( motor_A, OUTPUT );
  //instellen van pinnen input
  pinMode(accuSafety, INPUT);

  digitalWrite(out_STBY, HIGH);
  digitalWrite(out_A_PWM, HIGH);
  digitalWrite(out_A_IN2, LOW);
  digitalWrite(out_A_IN1, LOW);
  //digitalWrite( motor_A, LOW );
  digitalWrite(LCDPin, HIGH);

  digitalWrite(lampje, HIGH);

  pinMode(MOTOR_B_DIR, OUTPUT);
  pinMode(MOTOR_B_PWM, OUTPUT);
  digitalWrite(MOTOR_B_DIR, LOW);
  digitalWrite(MOTOR_B_PWM, LOW);
  pinMode(relay1, OUTPUT);
  pinMode(relay2, OUTPUT);

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
  Serial.println("Done!");
  pinMode(Buzzer, OUTPUT);
  pinMode(SHT_TOF1, OUTPUT);
  pinMode(SHT_TOF2, OUTPUT);
  pinMode(SHT_TOF3, OUTPUT);
  digitalWrite(SHT_TOF1, LOW);
  digitalWrite(SHT_TOF2, LOW);
  digitalWrite(SHT_TOF3, LOW);
  initializeSensors();
  playSuccessTune();
  delay(2000);
  playFailTune();
  delay(50);
}
void lcdScherm() {
  lcd.setCursor(0, 0);
  lcd.print("A1");
  lcd.setCursor(2, 0);
  lcd.print(sensor1);

  lcd.setCursor(5, 0);
  lcd.print("A2");
  lcd.setCursor(7, 0);
  lcd.print(sensor2);

  lcd.setCursor(12, 0);
  lcd.print("A2");
  lcd.setCursor(14, 0);
  lcd.print(sensor2);

  float ang = calculateAngle();
  lcd.setCursor(0, 1);
  lcd.print("Ang");
  lcd.setCursor(3, 1);
  lcd.print(ang);

  lcd.setCursor(7, 1);
  lcd.print("XYZ");
  lcd.setCursor(10, 1);
  lcd.print(gAngle.x);
  lcd.print(gAngle.y);
  lcd.print(gAngle.z);

  delay(50);
}

void serialDataPrint() {
  Serial.print("X: ");
  Serial.print(X_coord);
  Serial.print(", Y: ");
  Serial.print(Y_coord);
  Serial.print(", Angle: ");
  Serial.println(Angle_coord);
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

  Serial.println("Angle Data in µTesla: ");
  Serial.print(gAngle.x);
  Serial.print("   ");
  Serial.print(gAngle.y);
  Serial.print("   ");
  Serial.println(gAngle.z);
  Serial.println(sensor1);
  Serial.println(sensor2);
  Serial.println(sensor3);
}
/*
void errorCheck(){
  // Example error handling in reading accelerometer values
  bool success = sensor1;
  if (!success) {
    Serial.println("Error reading accelerometer values!");
    lcd.setCursor(0, 1);
    lcd.print("ERROR REBOOT");
    //softwareReset();
  }
}
void softwareReset() {
  // Disable interrupts
  cli();

  // Perform software reset
  wdt_enable(WDTO_15MS);
  while (1) {}  // Wait for watchdog timer to trigger reset
}*/
void loop() {
  unsigned long currentMillis = millis();
  Serial.print("test");
  // Convert the substrings to float
  //X_coord = received.substring(0, firstCommaIndex).toFloat();
  // Y_coord = received.substring(firstCommaIndex + 1, secondCommaIndex).toFloat();
  // Angle_coord = received.substring(secondCommaIndex + 1).toFloat();

  serialDataPrint();
  // Call the function to transform coordinates
  transformCoordinates();
  //errorCheck();
  //lcdScherm();
  readDualSensors();
  //i2cscan();
  if () {
    boolean isValidInput;
    // draw a menu on the serial port
    Serial.println("-----------------------------");
    Serial.println("MENU:");
    Serial.println("1) Fast forward");
    Serial.println("2) Forward");
    Serial.println("3) Soft stop (coast)");
    Serial.println("4) Reverse");
    Serial.println("5) Fast reverse");
    Serial.println("6) Hard stop (brake)");
    Serial.println("-----------------------------");
    do {
      byte c;
      // get the next character from the serial port
      Serial.print("?");
      while (!Serial.available())
        ;  // LOOP...
      c = Serial.read();
      // execute the menu option based on the character recieved
      switch (c) {
        case '1':  // 1) Fast forward
          Serial.println("Fast forward...");
          // always stop motors briefly before abrupt changes
          digitalWrite(MOTOR_B_DIR, LOW);
          digitalWrite(MOTOR_B_PWM, LOW);
          digitalWrite(MOTOR_A_DIR, LOW);
          digitalWrite(MOTOR_A_PWM, LOW);
          delay(DIR_DELAY);
          // set the motor speed and direction
          digitalWrite(relay1, HIGH);
          digitalWrite(relay2, HIGH);
          digitalWrite(MOTOR_B_DIR, HIGH);           // direction = forward
          analogWrite(MOTOR_B_PWM, 255 - PWM_FAST);  // PWM speed = fast
          digitalWrite(MOTOR_A_DIR, HIGH);           // direction = forward
          analogWrite(MOTOR_A_PWM, 255 - PWM_FAST);  // PWM speed = fast
          digitalWrite(out_A_IN2, LOW);
          digitalWrite(out_A_IN1, LOW);
          //digitalWrite( motor_A, HIGH );

          isValidInput = true;
          break;


        case '2':  // 2) Forward
          Serial.println("Forward...");
          // always stop motors briefly before abrupt changes
          digitalWrite(MOTOR_B_DIR, LOW);
          digitalWrite(MOTOR_B_PWM, LOW);
          digitalWrite(MOTOR_A_DIR, LOW);
          digitalWrite(MOTOR_A_PWM, LOW);
          delay(DIR_DELAY);
          // set the motor speed and direction
          digitalWrite(MOTOR_B_DIR, HIGH);           // direction = forward
          analogWrite(MOTOR_B_PWM, 255 - PWM_SLOW);  // PWM speed = slow
          digitalWrite(MOTOR_A_DIR, HIGH);           // direction = forward
          analogWrite(MOTOR_A_PWM, 255 - PWM_SLOW);  // PWM speed = slow
          isValidInput = true;
          break;

        case '3':  // 3) Soft stop (preferred)
          Serial.println("Soft stop (coast)...");
          digitalWrite(MOTOR_B_DIR, LOW);
          digitalWrite(MOTOR_B_PWM, LOW);
          digitalWrite(MOTOR_A_DIR, LOW);
          digitalWrite(MOTOR_A_PWM, LOW);
          digitalWrite(out_A_IN2, LOW);
          digitalWrite(out_A_IN1, LOW);
          //digitalWrite( motor_A, LOW );
          isValidInput = true;
          break;

        case '4':  // 4) Reverse
          Serial.println("Fast forward...");
          // always stop motors briefly before abrupt changes
          digitalWrite(MOTOR_B_DIR, LOW);
          digitalWrite(MOTOR_B_PWM, LOW);
          digitalWrite(MOTOR_A_DIR, LOW);
          digitalWrite(MOTOR_A_PWM, LOW);
          delay(DIR_DELAY);
          // set the motor speed and direction
          digitalWrite(MOTOR_B_DIR, LOW);      // direction = reverse
          analogWrite(MOTOR_B_PWM, PWM_SLOW);  // PWM speed = slow
          digitalWrite(MOTOR_A_DIR, LOW);      // direction = reverse
          analogWrite(MOTOR_A_PWM, PWM_SLOW);  // PWM speed = slow
          isValidInput = true;
          break;

        case '5':  // 5) Fast reverse
          Serial.println("Fast forward...");
          // always stop motors briefly before abrupt changes
          digitalWrite(MOTOR_B_DIR, LOW);
          digitalWrite(MOTOR_B_PWM, LOW);
          digitalWrite(MOTOR_A_DIR, LOW);
          digitalWrite(MOTOR_A_PWM, LOW);
          delay(DIR_DELAY);
          // set the motor speed and direction
          digitalWrite(MOTOR_B_DIR, LOW);      // direction = reverse
          analogWrite(MOTOR_B_PWM, PWM_FAST);  // PWM speed = fast
          digitalWrite(MOTOR_A_DIR, LOW);      // direction = reverse
          analogWrite(MOTOR_A_PWM, PWM_FAST);  // PWM speed = fast
          isValidInput = true;
          break;

        case '6':  // 6) Hard stop (use with caution)
          Serial.println("Hard stop (brake)...");
          digitalWrite(MOTOR_B_DIR, HIGH);
          digitalWrite(MOTOR_B_PWM, HIGH);
          digitalWrite(MOTOR_A_DIR, HIGH);
          digitalWrite(MOTOR_A_PWM, HIGH);
          isValidInput = true;
          break;
        case '7':
          Serial.println("kleine motor");
          digitalWrite(out_A_IN2, LOW);
          digitalWrite(out_A_IN1, HIGH);
          digitalWrite(relay1, HIGH);
          digitalWrite(relay2, HIGH);
          break;
        case '8':
          Serial.println("Alleen hover");
          digitalWrite(relay1, HIGH);
          digitalWrite(relay2, HIGH);

        default:
          // wrong character! display the menu again!
          isValidInput = false;
          break;
      }
    } while (isValidInput == true);
  } else {
    digitalWrite(MOTOR_B_DIR, LOW);
    digitalWrite(MOTOR_B_PWM, LOW);
    digitalWrite(MOTOR_A_DIR, LOW);
    digitalWrite(MOTOR_A_PWM, LOW);
    digitalWrite(out_A_IN2, LOW);
    digitalWrite(out_A_IN1, LOW);
    //digitalWrite( motor_A, LOW );
  }
}