void Regeling_PID_Jari(float &Mz, float Kp, float Kd, float Ki, float sp, float theta, float dt);
void Motoraansturing_Jari(float Mz);
void Motor_Links_Jari(float Mz);
void Motor_Rechts_Jari(float Mz);
void Poolplaasting_PID_Jari(float &Kp, float &Kd, float &Ki, float Iz, float Re, float Im, float Pool3);

void waitForSensorValue5() {
  int consecutiveCount = 0;
    unsigned long startTime = millis();
    unsigned long endTime = startTime + 1000 * 1000; // Convert seconds to milliseconds

        float theta = (mpu.getAngleZ() / 180);  // Replace with actual sensor reading function
            lcd.setCursor(12, 1);
            lcd.print(theta);
            
       if (theta < -0.3) {
                Regelaar_Jari_Succes = true;
            consecutiveCount++;
            lcd.setCursor(8, 1);
            lcd.print(consecutiveCount);

        } else {
            consecutiveCount = 0;
        }

        if (consecutiveCount >= 6) { // Check for the required duration
        Regelaar_Jari_Succes = true;
        consecutiveCount = 0;
}}
void Regelaar_Jari(){
  const long cyclustijd = 10;  // Cyclustijd in ms
  static long t_oud = 0;       // Initialize t_oud to 0 at the beginning
  long t_nw = millis();        // Get the current time in ms
  const float Re = 0.18, Im = 0.36, Pool3 = 1;
  const float Iz = 0.115405;  // In kilo gram
  float dt = 1;           // Nodig voor de d_error / dt
  float Mz;               // Initialize Fx and Fy
  float theta = (mpu.getAngleZ() / 180);    // Begin voor waarde van de regelaar in rad
  float Kp, Kd, Ki;           // Paramateres voor de regelaar
  const float sp = -0.5;   // Setpoint voor het stoppen van de regelaar m

  Serial.print("Gyro (theta): ");
  Serial.println(theta);

  Poolplaatsing_PID(Kp, Kd, Ki, Iz, Re, Im, Pool3);
  Serial.print("Kp: ");
  Serial.println(Kp);
  Serial.print("Kd: ");
  Serial.println(Kd);
  Serial.print("Ki: ");
  Serial.println(Ki);

  if (t_nw - t_oud > cyclustijd) {  // Check if the cyclustijd has passed
    dt = (t_nw - t_oud) * 0.001;    // Calculate dt in seconds
    t_oud = t_nw;                   // Update t_oud to the current time

    Regeling_PID_Jari(Mz, Kp, Kd, Ki, sp, theta, dt);
    Serial.print("PD Output Mz Jari: ");
    Serial.println(Mz);
    Motoraansturing_Jari(Mz / 2);  // Control the motors with half of Fx
    waitForSensorValue5();
  }
}

void Regeling_PID_Jari(float &Mz, float Kp, float Kd, float Ki, float sp, float theta, float dt) {
  static float error_oud = 0;
  float error = sp - theta;
  float d_error = (error - error_oud) / dt;  // Calculate derivative of error
  float errorSom = errorSom + error * dt;
  error_oud = error;                         // Update previous error
  Mz = (Kp + 0.0) *error + (Kd - 0.0) *d_error / dt + (Ki - 0.0) *errorSom;

  Serial.print("Error: ");
  Serial.println(error);
  Serial.print("d_error: ");
  Serial.println(d_error);
  Serial.print("Mz: ");
  Serial.println(Mz);
}

void Motoraansturing_Jari(float Mz){
  Motor_Links_Jari(Mz);
  Motor_Rechts_Jari(Mz);
}

void Motor_Links_Jari(float Mz) {  // Dit is voor Maxon motor 1
  if (Mz > 0) {               // hierdoor gaat de uav achteruit
    Serial.print("PWM Maxon 1 pos: ");
    //float Fx_nega = Fx * -1;
    Serial.println(-0.00298 * Mz * Mz - 1.75115 * Mz);
    digitalWrite(motorRechts, LOW);  // dit is voor wanneer de hovercraft achterwaarts moet bewegen
    analogWrite(motorRechtsPWM, -0.00298 * Mz * Mz - 1.75115 * Mz);
  } else {  // dit is voor wanneer de hovercraft voorwaartswaarts moet bewegen
    Serial.print("PWM Maxon 1 nega: ");
    Serial.println(-0.00505 * Mz * Mz + 2.25550 * Mz);
    digitalWrite(motorRechts, HIGH);
    analogWrite(motorRechtsPWM, -0.00505 * Mz * Mz + 2.25550 * Mz);
  }
}

void Motor_Rechts_Jari(float Mz) {  // Dit is voor Maxon motor 2
  if (Mz < 0) {                // dit is voor wanneer de hovercraft achterwaarts moet bewegen
    Serial.print("PWM Maxon 2 pos: ");
    //float Fx_nega = Fx * -1;
    Serial.println(-0.00282 * Mz * Mz - 1.70725 * Mz);
    digitalWrite(motorLinks, LOW);
    analogWrite(motorLinksPWM, -0.00282 * Mz * Mz - 1.70725 * Mz);
  } else {  // dit is voor wanneer de hovercraft voorwaarts moet bewegen
    Serial.print("PWM Maxon 2 nega: ");
    Serial.println(-0.00425 * Mz * Mz + 2.077594 * Mz);
    digitalWrite(motorLinks, HIGH);
    analogWrite(motorLinksPWM, -0.00425 * Mz * Mz + 2.077594 * Mz);
  }
}