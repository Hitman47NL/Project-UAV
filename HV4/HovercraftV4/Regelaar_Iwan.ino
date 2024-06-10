void Regelaar_Iwan() {
  readDualSensors();
  const long cyclustijd = 10;  // Cyclustijd in ms
  static long t_oud = 0;       // Initialize t_oud to 0 at the beginning
  long t_nw = millis();        // Get the current time in ms
  const float Re = 0.75, Im = 1.0;
  const float m = 1560;          // In kilo gram
  float dt = 1;                  // Nodig voor de d_error / dt
  float Fx, Fy;                  // Initialize Fx and Fy
  float sx = TOFsensor3 / 1000;  // Begin voor waarde van de regelaar in cm
  float Kp, Kd;                  // Paramateres voor de regelaar
  const float sp = 0.3;          // Setpoint voor het stoppen van de regelaar cm

  Serial.print("TOFsensor3 (sx): ");
  Serial.println(sx, 4);

  Poolplaatsing_PD(Kp, Kd, m, Re, Im);
  Serial.print("Kp: ");
  Serial.println(Kp);
  Serial.print("Kd: ");
  Serial.println(Kd);

  if (t_nw - t_oud > cyclustijd) {  // Check if the cyclustijd has passed
    dt = (t_nw - t_oud) * 0.001;    // Calculate dt in seconds
    t_oud = t_nw;                   // Update t_oud to the current time

    Regeling_PD(Fx, Fy, Kp, Kd, sp, sx, dt);
    Serial.print("PD Output Fx: ");
    Serial.println(Fx);
    Serial.print("PD Output Fy: ");
    Serial.println(Fy);

    motoraansturing_Iwan(Fx / 2);  // Control the motors with half of Fx
  }
}
