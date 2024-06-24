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

