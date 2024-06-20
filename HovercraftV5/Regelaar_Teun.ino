void Regeling_PD_Teun(float &Fy, float Kp, float Kd, float sp, float sx, float dt);
void Poolplaatsing_PD_Teun(float &Kp, float &Kd, float m, float Re, float Im);
void motoraansturing_Teun(float Fy);
//void Motor_Rechts(float Fx);
//void Motor_Links(float Fx);
void Motor_midden(float Fy);

void waitForSensorValue1() {
    unsigned long startTime = millis();
    unsigned long endTime = startTime + 1000 * 1000; // Convert seconds to milliseconds

        int sensorValue = TOFsensor1;  // Replace with actual sensor reading function

        if (sensorValue > 190 && sensorValue < 210) {
            consecutiveCount++;
            lcd.setCursor(8, 1);
            lcd.print(consecutiveCount);
            lcd.setCursor(12, 1);
            lcd.print(sensorValue);
            
        } else {
            consecutiveCount = 0;
        }

        if (consecutiveCount >= 6) { // Check for the required duration
        Regelaar_Teun_Succes = true;
        consecutiveCount = 0;
}}

void Regelaar_Teun() {
  readDualSensors();
  const long cyclustijd = 10;  // Cyclustijd in ms
  static long t_oud = 0;       // Initialize t_oud to 0 at the beginning
  long t_nw = millis();        // Get the current time in ms
  const float Re = 0.6/4, Im = 1.2/4;
  const float m = 1560;  // In kilo gram
  float dt = 1;           // Nodig voor de d_error / dt
  float Fx, Fy;               // Initialize Fx and Fy
  int sx = TOFsensor1;    // Begin voor waarde van de regelaar in m
  float Kp, Kd;           // Paramateres voor de regelaar
  const float sp = 205;   // Setpoint voor het stoppen van de regelaar m

  Serial.print("TOFsensor1 (sx): ");
  Serial.println(sx);

  Poolplaatsing_PD_Teun(Kp, Kd, m, Re, Im);
  Serial.print("Kp: ");
  Serial.println(Kp);
  Serial.print("Kd: ");
  Serial.println(Kd);

  if (t_nw - t_oud > cyclustijd) {  // Check if the cyclustijd has passed
    dt = (t_nw - t_oud) * 0.001;    // Calculate dt in seconds
    t_oud = t_nw;                   // Update t_oud to the current time

    Regeling_PD_Teun(Fy, Kp, Kd, sp, sx, dt);
    Serial.print("PD Output Fy Teun: ");
    Serial.println(Fy);
    lcd.setCursor(1, 1);
    lcd.print(sx);
    motoraansturing_Teun(Fy / 2);  // Control the motors with half of Fx
    waitForSensorValue1();
  }
}

void Poolplaatsing_PD_Teun(float &Kp, float &Kd, float m, float Re, float Im) {
  Kp = ((Re * Re + Im * Im) * m) + 0.0;
  Kd = (2 * Re * m) + 0.0;
}

void Regeling_PD_Teun(float &Fy, float Kp, float Kd, float sp, float sx, float dt) {
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
  Serial.print("Fy Teun: ");
  Serial.println(Fy);
}

void motoraansturing_Teun(float Fy) {
  //Motor_Rechts(Fx);
  //Motor_Links(Fx);
  Motor_midden(Fy);
}
