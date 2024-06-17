void Regeling_PD_Iwan(float &Fx, float Kp, float Kd, float sp, float sx, float dt);
void Poolplaatsing_PD(float &Kp, float &Kd, float m, float Re, float Im);
void motoraansturing_Iwan(float Fx);


void Regelaar_Iwan() {
  readDualSensors();
  const long cyclustijd = 10;  // Cyclustijd in ms
  static long t_oud = 0;       // Initialize t_oud to 0 at the beginning
  long t_nw = millis();        // Get the current time in ms
  const float Re = 0.628318531, Im = 1.256637061;
  const float m = 1.560;  // In kilo gram
  float dt = 1;           // Nodig voor de d_error / dt
  float Fx;               // Initialize Fx and Fy
  int sx = TOFsensor3;    // Begin voor waarde van de regelaar in m
  float Kp, Kd;           // Paramateres voor de regelaar
  const float sp = 300;   // Setpoint voor het stoppen van de regelaar m

  Serial.print("TOFsensor3 (sx): ");
  Serial.println(sx);

  Poolplaatsing_PD(Kp, Kd, m, Re, Im);
  Serial.print("Kp: ");
  Serial.println(Kp);
  Serial.print("Kd: ");
  Serial.println(Kd);

  if (t_nw - t_oud > cyclustijd) {  // Check if the cyclustijd has passed
    dt = (t_nw - t_oud) * 0.001;    // Calculate dt in seconds
    t_oud = t_nw;                   // Update t_oud to the current time

    Regeling_PD(Fx, Kp, Kd, sp, sx, dt);
    Serial.print("PD Output Fx: ");
    Serial.println(Fx);
    motoraansturing_Iwan(Fx / 2);  // Control the motors with half of Fx
  }
}

void Regeling_PD_Iwan(float &Fy, float Kp, float Kd, float sp, float sx, float dt) {
  static float error_oud = 0;  // Initialize previous error
  float error = sp - sx;
  float d_error = (error - error_oud) / dt;  // Calculate derivative of error
  error_oud = error;                         // Update previous error
  Fy = (Kp + 2.5) * error + (Kd - 0.5) * d_error;
  //Fy = Fx;  // Assuming Fy should be the same as Fx for this example

  Serial.print("Error: ");
  Serial.println(error);
  Serial.print("d_error: ");
  Serial.println(d_error);
  Serial.print("Fx: ");
  Serial.println(Fy);
}


void motoraansturing_Iwan(float Fx) {
  Motor_Rechts(Fx);
  Motor_Links(Fx);
}
