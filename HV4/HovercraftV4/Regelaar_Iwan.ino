void Poolplaatsing_PD(float &Kp_PD, float &Kd_PD, float m, float Re_PD, float Im_PD);
void Poolplaatsing_PID(float &Kp_PID, float &Kd_PID, float &Ki_PID, float Iz, float Re_PID, float Im_PID, float pool3);
void Regeling_PD(float &Fx_PD, float &Fy_PD, float Kp_PD, float Kd_PD, float sp_PD, float sx, float dt);
void Regeling_PID(float &Fx_PID, float &Fy_PID, float Kp_PID, float Kd_PID, float Ki_PID, float sp_PID, float theta, float dt);
void motoraansturing_Iwan(float Fx_PD, float Fx_PID, float Fy_PD, float Fy_PID);
void Motor_Rechts(float Fx_PD, float Fx_PID);
void Motor_Links(float Fx_PD, float Fx_PID);
void Motor_midden(float Fy_PD, float Fy_PID);

void Regelaar_Iwan() {
  readDualSensors();
  const long cyclustijd = 10;  // Cycle time in ms
  static long t_oud = 0;       // Initialize t_oud to 0 at the beginning
  long t_nw = millis();        // Get the current time in ms
  const float Re_PD = 0.75, Im_PD = 1.0;
  const float Re_PID = 0.75, Im_PID = 1.0, pool3 = 0.001;
  const float m = 1560;          // In kilograms
  const float Iz = 0.115405;     // Moment of inertia
  float dt = 1;                  // Needed for the d_error / dt calculation
  float Fx_PD, Fy_PD, Fx_PID, Fy_PID;  // Initialize Fx and Fy
  float sx = TOFsensor3 / 1000;  // Initial value for the controller in meters
  float theta = 0.0;             // Initial value for theta
  float Kp_PD, Kd_PD, Kp_PID, Kd_PID, Ki_PID;  // Parameters for the controllers
  const float sp_PD = 0.3, sp_PID = 0.0;  // Setpoints for the controllers

  Serial.print("TOFsensor3 (sx): ");
  Serial.println(sx, 4);
  Serial.print("Hoek gyro (Theta): ");
  Serial.println(theta);

  Poolplaatsing_PD(Kp_PD, Kd_PD, m, Re_PD, Im_PD);
  //Poolplaatsing_PID(Kp_PID, Kd_PID, Ki_PID, Iz, Re_PID, Im_PID, pool3);
  Serial.print("Kp_PD: ");
  Serial.println(Kp_PD);
  Serial.print("Kd_PD: ");
  Serial.println(Kd_PD);
  Serial.print("Kp_PID: ");
  Serial.println(Kp_PID);
  Serial.print("Kd_PID: ");
  Serial.println(Kd_PID);
  Serial.print("Ki_PID: ");
  Serial.println(Ki_PID);

  if (t_nw - t_oud > cyclustijd) {  // Check if the cycle time has passed
    dt = (t_nw - t_oud) * 0.001;    // Calculate dt in seconds
    t_oud = t_nw;                   // Update t_oud to the current time

    Regeling_PD(Fx_PD, Fy_PD, Kp_PD, Kd_PD, sp_PD, sx, dt);
    Regeling_PID(Fx_PID, Fy_PID, Kp_PID, Kd_PID, Ki_PID, sp_PID, theta, dt);
    Serial.print("PD Output Fx_PD: ");
    Serial.println(Fx_PD);
    Serial.print("PID Output Fx_PID: ");
    Serial.println(Fx_PID);

    motoraansturing_Iwan(Fx_PD / 2, Fx_PID / 2, Fy_PD, Fy_PID / 2);  // Control the motors with combined outputs
  }
}

void Motor_Rechts(float Fx_PD, float Fx_PID) {  // For Maxon motor 2
  if (Fx_PD > 0) {  // When hovercraft needs to move backward
    Serial.print("PWM Maxon 2 pos: ");
    float Fx_nega = Fx_PD * -1;
    Serial.println(-0.00282 * Fx_nega * Fx_nega - 1.70725 * Fx_nega);
    digitalWrite(motorLinks, LOW);
    analogWrite(motorLinksPWM, -0.00282 * Fx_nega * Fx_nega - 1.70725 * Fx_nega);
  } else {  // When hovercraft needs to move forward
    float Fx = Fx_PD;
    Serial.print("PWM Maxon 2 nega: ");
    Serial.println(-0.00425 * Fx * Fx + 2.077594 * Fx);
    digitalWrite(motorLinks, HIGH);
    analogWrite(motorLinksPWM, -0.00425 * Fx * Fx + 2.077594 * Fx);
  }
}

void Motor_Links(float Fx_PD, float Fx_PID) {  // For Maxon motor 1
  if (Fx_PD > 0) {  // When hovercraft needs to move backward
    Serial.print("PWM Maxon 1 pos: ");
    float Fx_nega = Fx_PD * -1;
    Serial.println(-0.00298 * Fx_nega * Fx_nega - 1.75115 * Fx_nega);
    digitalWrite(motorRechts, LOW);
    analogWrite(motorRechtsPWM, -0.00298 * Fx_nega * Fx_nega - 1.75115 * Fx_nega);
  } else {  // When hovercraft needs to move forward
    float Fx = Fx_PD;
    Serial.print("PWM Maxon 1 nega: ");
    Serial.println(-0.00505 * Fx * Fx + 2.25550 * Fx);
    digitalWrite(motorRechts, HIGH);
    analogWrite(motorRechtsPWM, -0.00505 * Fx * Fx + 2.25550 * Fx);
  }
}

void Motor_midden(float Fy_PD, float Fy_PID) {  // For the small motor
  if (Fy_PD < 0) {  // When hovercraft needs to move left
    float Fy = Fy_PD;
    digitalWrite(motorZijkantIn1, HIGH);
    digitalWrite(motorZijkantIn2, LOW);
    analogWrite(motorZijkantPWM, 0.00519 * Fy * Fy - 0.67075 * Fy);
  } else {  // When hovercraft needs to move right
    float Fy = Fy_PD;
    digitalWrite(motorZijkantIn1, LOW);
    digitalWrite(motorZijkantIn2, HIGH);
    analogWrite(motorZijkantPWM, 0.01060 * Fy * Fy + 1.12248 * Fy);
  }
}

void Poolplaatsing_PD(float &Kp_PD, float &Kd_PD, float m, float Re_PD, float Im_PD) {
  Kp_PD = (Re_PD * Re_PD + Im_PD * Im_PD) * m;
  Kd_PD = 2 * Re_PD * m;
}

void Poolplaatsing_PID(float &Kp_PID, float &Kd_PID, float &Ki_PID, float Iz, float Re_PID, float Im_PID, float pool3) {
  Kp_PID = (Re_PID * Re_PID + Im_PID * Im_PID + 2 * Re_PID * pool3) * Iz;
  Kd_PID = (2 * Re_PID + pool3) * Iz;
  Ki_PID = (Re_PID * Re_PID + Im_PID * Im_PID) * pool3 * Iz;
}

void Regeling_PD(float &Fx_PD, float &Fy_PD, float Kp_PD, float Kd_PD, float sp_PD, float sx, float dt) {
  static float error_oud = 0;  // Initialize previous error
  float error = sp_PD - sx;
  float d_error = (error - error_oud) / dt;  // Calculate derivative of error
  error_oud = error;                         // Update previous error
  Fx_PD = Kp_PD * error + Kd_PD * d_error;
  Fy_PD = Fx_PD;  // Assuming Fy should be the same as Fx for this example

  Serial.print("Error (PD): ");
  Serial.println(error);
  Serial.print("d_error (PD): ");
  Serial.println(d_error);
  Serial.print("Fx_PD: ");
  Serial.println(Fx_PD);
}

void Regeling_PID(float &Fx_PID, float &Fy_PID, float Kp_PID, float Kd_PID, float Ki_PID, float sp_PID, float theta, float dt) {
  static float error_oud = 0;  // Initialize previous error
  static float errorSom = 0;   // Initialize error sum
  float error = sp_PID - theta;
  float d_error = (error - error_oud) / dt;  // Calculate derivative of error
  errorSom += error * dt;                    // Accumulate the error
  error_oud = error;                         // Update previous error
  Fx_PID = Kp_PID * error + Kd_PID * d_error + Ki_PID * errorSom;
  Fy_PID = Fx_PID;  // Assuming Fy should be the same as Fx for this example

  Serial.print("Error (PID): ");
  Serial.println(error);
  Serial.print("d_error (PID): ");
  Serial.println(d_error);
  Serial.print("Fx_PID: ");
  Serial.println(Fx_PID);
}

void motoraansturing_Iwan(float Fx_PD, float Fx_PID, float Fy_PD, float Fy_PID) {
  Motor_Rechts(Fx_PD, Fx_PID);
  Motor_Links(Fx_PD, Fx_PID);
  Motor_midden(Fy_PD, Fy_PID);
}
