void Regelaar_Iwan() {
  readDualSensors();
  const long cyclustijd = 10;  // Cyclustijd in ms
  static long t_oud = 0;       // Initialize t_oud to 0 at the beginning
  long t_nw = millis();        // Get the current time in ms
  const float Re_PD = 0.75, Im_PD = 1.0;
  const float Re_PID = 0.75, Im_PID = 1.0, pool3 = 0.001;
  const float m = 1560;          // In kilo gram
  const float Iz = 0.115405;     // Massa traagheid
  float dt = 1;                  // Nodig voor de d_error / dt
  float Fx_PD, Fy_PD, Fx_PID, Fy_PID;                  // Initialize Fx and Fy
  float sx = TOFsensor3 / 1000;  // Begin voor waarde van de regelaar in cm
  float theta;
  float Kp_PD, Kd_PD, Kp_PID, Kd_PID, Ki_PID, pool3;                  // Paramateres voor de regelaar
  const float sp_PD = 0.3, sp_PID = 0.0;          // Setpoint voor het stoppen van de regelaar cm

  Serial.print("TOFsensor3 (sx): ");
  Serial.println(sx, 4);
  Serial.print("Hoek gyro (Theta" );
  Serial.println(theta);

  Poolplaatsing_PD(Kp_PD, Kd_PD, m, Re_PD, Im_PD);
  Poolplaatsing_PID(Kp_PID, Kd_PID, Ki_PID, Iz, Re_PID, Im_PID, pool3);
  Serial.print("Kp_PD: ");
  Serial.println(Kp_PD);
  Serial.print("Kd_PD: ");
  Serial.println(Kd_PD);
  Serial.print("Kp_PD: ");
  Serial.println(Kp_PD);
  Serial.print("Kd_PD: ");
  Serial.println(Kd_PD);
  Serial.print("Kd_PD: ");
  Serial.println(Kd_PD);

  if (t_nw - t_oud > cyclustijd) {  // Check if the cyclustijd has passed
    dt = (t_nw - t_oud) * 0.001;    // Calculate dt in seconds
    t_oud = t_nw;                   // Update t_oud to the current time

    Regeling_PD(Fx_PD, Fy_PD, Kp_PD, Kd_PD, sp_PD, sx, dt);
    Regeling_PID(Fx_PID, Fy_PID, Kp_PID, Kd_PID, Ki_PID, sp_PID, theta, dt);
    Serial.print("PD Output Fx_PD: ");
    Serial.println(Fx_PD);
    Serial.print("PD Output Fy_PID: ");
    Serial.println(Fy_PID);

    motoraansturing_Iwan(Fx_PD / 2, Fx_PID / 2, Fy_PID, Fy_PID / 2);  // Control the motors with half of Fx
  }
}
