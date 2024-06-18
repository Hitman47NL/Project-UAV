void Regeling_PID_Bram(float &Mz, float Kp, float Kd, float Ki, float sp, float theta, float dt);
//void Poolplaatsing_PID_Bram(float &Kp, float &Kd, float &Ki, float Iz, float Re, float Im, float Pool3);
void Motoraansturing_Bram(float Mz);
void Motor_Links_Bram(float Mz);
void Motor_Rechts_Bram(float Mz);

void waitForSensorValue3() {
  int consecutiveCount = 0;
    unsigned long startTime = millis();
    unsigned long endTime = startTime + 1000 * 1000; // Convert seconds to milliseconds

        float theta = mpu.getAccZ();  // Replace with actual sensor reading function
                        lcd.setCursor(12, 1);
            lcd.print(theta);
            
        if (theta > -10 && theta < 10) {
            consecutiveCount++;
            lcd.setCursor(8, 1);
            lcd.print(consecutiveCount);

        } else {
            consecutiveCount = 0;
        }

        if (consecutiveCount >= 10) { // Check for the required duration
        Regelaar_Bram_Succes = true;
        consecutiveCount = 0;
}}
void Regelaar_Bram() {
  const long cyclustijd = 10;  // Cyclustijd in ms
  static long t_oud = 0;       // Initialize t_oud to 0 at the beginning
  long t_nw = millis();        // Get the current time in ms
  const float Re = 2.5, Im = 4, pool3 = 0.005;
  const float m = 1.560;
  const float Iz = 0.115405;
  float dt = 1;           // Nodig voor de d_error / dt
  float Mz;
  float theta = mpu.getAngleZ();
  float sp = 0.0;

  Serial.print("theta: ");
  Serial.println(theta);

  float Kp = 1.0, Kd = 0.1, Ki = 0.05; // Declare PID constants
  //Poolplaatsing_PID_Bram(Kp, Kd, Ki, Iz, Re, Im, pool3); // Initialize PID constants
  Serial.print("Kp: ");
  Serial.println(Kp);
  Serial.print("Kd: ");
  Serial.println(Kd);
  Serial.print("Ki: ");
  Serial.println(Ki);

  t_nw = millis();
  if (t_nw - t_oud > cyclustijd) {
    dt = (t_nw - t_oud) * 0.001;
    t_oud = t_nw;

    Regeling_PID_Bram(Mz, Kp, Kd, Ki, sp, theta, dt); // Call PID controller

    Serial.print("PID_Bram: ");
    Serial.print(Mz);
    Motoraansturing_Bram(Mz / 2);  // Control the motors with half of Mz
    waitForSensorValue3();
  }
}

void Regeling_PID_Bram(float &Mz, float Kp, float Kd, float Ki, float sp, float theta, float dt) {
  static float error_oud = 0;  // Initialize previous error
  static float errorSom = 0;   // Initialize integral of error
  float error = sp - theta;
  float d_error = (error - error_oud) / dt;  // Calculate derivative of error
  errorSom += error * dt;  // Update integral of error
  error_oud = error;                         // Update previous error
  Mz = Kp * error + Kd * d_error + Ki * errorSom;

  Serial.print("Error: ");
  Serial.println(error);
  Serial.print("d_error: ");
  Serial.println(d_error);
  Serial.print("Mz: ");
  Serial.println(Mz);
}

void Motoraansturing_Bram(float Mz){
  Motor_Links_Bram(Mz);
  Motor_Rechts_Bram(Mz);
}

void Motor_Links_Bram(float Mz) {  // Dit is voor Maxon motor 1
  if (Mz < 0) {
    Serial.print("PWM Maxon 1 pos: ");
    Serial.println(-0.00298 * Mz * Mz - 1.75115 * Mz);
    digitalWrite(motorRechts, LOW);
    analogWrite(motorRechtsPWM, -0.00298 * Mz * Mz - 1.75115 * Mz);
  } else {
    Serial.print("PWM Maxon 1 nega: ");
    Serial.println(-0.00505 * Mz * Mz + 2.25550 * Mz);
    digitalWrite(motorRechts, HIGH);
    analogWrite(motorRechtsPWM, -0.00505 * Mz * Mz + 2.25550 * Mz);
  }
}

void Motor_Rechts_Bram(float Mz) {  // Dit is voor Maxon motor 2
  if (Mz > 0) {
    Serial.print("PWM Maxon 2 pos: ");
    Serial.println(-0.00282 * Mz * Mz - 1.70725 * Mz);
    digitalWrite(motorLinks, LOW);
    analogWrite(motorLinksPWM, -0.00282 * Mz * Mz - 1.70725 * Mz);
  } else {
    Serial.print("PWM Maxon 2 nega: ");
    Serial.println(-0.00425 * Mz * Mz + 2.077594 * Mz);
    digitalWrite(motorLinks, HIGH);
    analogWrite(motorLinksPWM, -0.00425 * Mz * Mz + 2.077594 * Mz);
  }
}

void Poolplaatsing_PID(float &Kp, float &Kd, float &Ki, float Iz, float Re, float Im, float Pool3) {
  // Implement your PID tuning logic here to assign values to Kp, Kd, and Ki
  Kp = 1.0;
  Kd = 0.1;
  Ki = 0.05;
}
