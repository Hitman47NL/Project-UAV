void Regelaar_Bram() {
  const long cyclustijd = 10;
  long t_oud, t_nw;
  float dt;
  const float Re = 2.5, Im = 4, pool3 = 0.005;
  const float m = 1.560;
  const float Iz = 0.115405;
  float theta = calculateAngle();
  float Kp, Kd, Ki;
  float Fx;
  const float sp = 1.0;

  Serial.print("theta: ");
  Serial.println(theta);

  Poolplaatsing_PID(Kp, Kd, Ki, Iz, Re, Im, pool3);
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

    Regeling_PID(Fx, Kp, Kd, Ki, sp, theta, dt);
    Serial.print("PD_Bram: ");
    Serial.print(Fx);

    motoraansturing_Bram(Fx / 2);
  }
}