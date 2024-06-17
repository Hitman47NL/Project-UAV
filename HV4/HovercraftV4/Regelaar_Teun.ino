void Regeling_PD_Teun(float &Fy, float Kp, float Kd, float sp, float sx, float dt);
void Poolplaatsing_PD(float &Kp, float &Kd, float m, float Re, float Im);
void motoraansturing_Teun(float Fy);
//void Motor_Rechts(float Fx);
//void Motor_Links(float Fx);
void Motor_midden(float Fy);

void Regelaar_Teun() {
  readDualSensors();
  const long cyclustijd = 10;  // Cyclustijd in ms
  static long t_oud = 0;       // Initialize t_oud to 0 at the beginning
  long t_nw = millis();        // Get the current time in ms
  const float Re = 0.6, Im = 1.2;
  const float m = 1.560;  // In kilo gram
  float dt = 1;           // Nodig voor de d_error / dt
  float Fy;               // Initialize Fx and Fy
  int sx = TOFsensor1;    // Begin voor waarde van de regelaar in m
  float Kp, Kd;           // Paramateres voor de regelaar
  const float sp = 200;   // Setpoint voor het stoppen van de regelaar m

  Serial.print("TOFsensor1 (sx): ");
  Serial.println(sx);

  Poolplaatsing_PD(Kp, Kd, m, Re, Im);
  Serial.print("Kp: ");
  Serial.println(Kp);
  Serial.print("Kd: ");
  Serial.println(Kd);

  if (t_nw - t_oud > cyclustijd) {  // Check if the cyclustijd has passed
    dt = (t_nw - t_oud) * 0.001;    // Calculate dt in seconds
    t_oud = t_nw;                   // Update t_oud to the current time

    Regeling_PD(Fy, Kp, Kd, sp, sx, dt);
    Serial.print("PD Output Fy: ");
    Serial.println(Fy);

    motoraansturing_Teun(Fy / 2);  // Control the motors with half of Fx
  }
}

void Regeling_PD_Teun(float &Fy, float Kp, float Kd, float sp, float sx, float dt) {
  static float error_oud = 0;  // Initialize previous error
  float error = sp - sx;
  float d_error = (error - error_oud) / dt;  // Calculate derivative of error
  error_oud = error;                         // Update previous error
  Fy = (Kp + 0.0) * error + (Kd - 0.0) * d_error;
  //Fy = Fx;  // Assuming Fy should be the same as Fx for this example

  Serial.print("Error: ");
  Serial.println(error);
  Serial.print("d_error: ");
  Serial.println(d_error);
  Serial.print("Fx: ");
  Serial.println(Fy);
}

void motoraansturing_Teun(float Fy) {
  //Motor_Rechts(Fx);
  //Motor_Links(Fx);
  Motor_midden(Fy);
}
