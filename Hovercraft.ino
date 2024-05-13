//Gemaakt door Jelle
//Hiermee kunnen de MPU9250 en de TOF sensoren worden uitgelezen


#include <Wire.h>
#include <Adafruit_VL53L0X.h>
#include <LiquidCrystal_I2C.h>
#include <math.h> // Include the math library for atan2 function
#include <MPU9250_WE.h>

#define TOF1_ADDRESS 0x30
#define TOF2_ADDRESS 0x31
#define TOF3_ADDRESS 0x32
#define Buzzer 3
#define MPU9250_ADDR 0x68

#define SHT_TOF1 7
#define SHT_TOF2 6
#define SHT_TOF3 5

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


void playTone(int tone, int duration) {
  for (long i = 0; i < duration * 100L; i += tone * 2) {
    digitalWrite(Buzzer, HIGH);
    delayMicroseconds(tone);
    digitalWrite(Buzzer, LOW);
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
  digitalWrite(SHT_TOF1, LOW);
  digitalWrite(SHT_TOF2, LOW);
  digitalWrite(SHT_TOF3, LOW);
  delay(10);
  digitalWrite(SHT_TOF1, HIGH);
  digitalWrite(SHT_TOF2, HIGH);
  digitalWrite(SHT_TOF3, HIGH);
  delay(10);

  digitalWrite(SHT_TOF1, HIGH);
  digitalWrite(SHT_TOF2, LOW);
  digitalWrite(SHT_TOF3, LOW);
  if(!tof1.begin(TOF1_ADDRESS)) {
    Serial.println(F("Failed to boot first VL53L0X"));
   // while(1);
  }
  delay(10);

  digitalWrite(SHT_TOF2, HIGH);
  if(!tof2.begin(TOF2_ADDRESS)) {
    Serial.println(F("Failed to boot second VL53L0X"));
  //  while(1);
  }
  delay(10);

  digitalWrite(SHT_TOF3, HIGH);
  if(!tof3.begin(TOF3_ADDRESS)) {
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
float calculateAngle(){
    // Calculate angle based on the furthest sensor
    if (sensor1 < sensor2) {
      float rawAngle = (sensor1 - sensor2) / 10; // Calculate raw angle difference
      float ang = atan(rawAngle)* 180/3.14; // Convert to degrees
      lcd.setCursor(5, 0);
      lcd.print(ang);
        return ang;
    } else {
      float rawAngle = (sensor2 - sensor1) / 10; // Calculate raw angle difference
      float ang = -atan(rawAngle)* 180/3.14; // Convert to degrees
      lcd.setCursor(5, 0);
      lcd.print(ang);
        return ang;
    }

}

void setup() {
  Serial.begin(9600);
  lcd.init();
  lcd.backlight();  
  Serial.println("Starting...");
  lcd.setCursor(0, 0);
  lcd.print("Starting up");
  Wire.begin();
    if(!myMPU9250.init()){
    Serial.println("MPU9250 does not respond");
  }
  else{
    Serial.println("MPU9250 is connected");
  }
  if(!myMPU9250.initMagnetometer()){
    Serial.println("Magnetometer does not respond");
  }
  else{
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

  Serial.println("Angle Data in µTesla: ");
  Serial.print(gAngle.x);
  Serial.print("   ");
  Serial.print(gAngle.y);
  Serial.print("   ");
  Serial.println(gAngle.z);

readDualSensors();

delay(1000);
  if (sensor1 != -1 && sensor2 != -1) {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print(sensor1);
    lcd.setCursor(0, 1);
    lcd.print(sensor2);
    float ang = calculateAngle();
    lcd.setCursor(5, 1);
    lcd.print(ang);
  delay(50);
}
}