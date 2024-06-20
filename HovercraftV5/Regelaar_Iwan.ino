void Poolplaatsing_PD_Iwan(float &Kp, float &Kd, float m, float Re, float Im);
void Regeling_PD_Iwan(float &Fx, float &Fy, float Kp, float Kd, float sp, float sx, float dt);
void motoraansturing_Iwan(float Fx);
void Motor_Links_Iwan(float Fx);
void Motor_Rechts_Iwan(float Fx);

/*
void waitForSensorValue4() {
  
    unsigned long startTime = millis();
    unsigned long endTime = startTime + 1000 * 1000; // Convert seconds to milliseconds

        float theta = TOFsensor3;  // Replace with actual sensor reading function
                        lcd.setCursor(12, 1);
            lcd.print(theta);
            
        if (theta > 250 && theta < 300) {
            consecutiveCount++;
            lcd.setCursor(8, 1);
            lcd.print(consecutiveCount);

        } else {
            consecutiveCount = 0;
        }

        if (consecutiveCount >= 10) { // Check for the required duration
        Regelaar_Iwan_Succes = true;
        consecutiveCount = 0;
}}
*/
void Regelaar_Iwan() {
  readDualSensors();
  //waitForSensorValue4();
  const long cyclustijd = 10;  // Cyclustijd in ms
  static long t_oud = 0;       // Initialize t_oud to 0 at the beginning
  long t_nw = millis();        // Get the current time in ms
  const float Re = 0.628318531, Im = 1.256637061;
  const float m = 1.560;  // In kilo gram
  float dt = 1;           // Nodig voor de d_error / dt
  float Fx, Fy;               // Initialize Fx and Fy
  int sx = TOFsensor3;    // Begin voor waarde van de regelaar in m
  float Kp, Kd;           // Paramateres voor de regelaar
  const float sp = 300;   // Setpoint voor het stoppen van de regelaar m

  Serial.print("TOFsensor3 (sx): ");
  Serial.println(sx);

  Poolplaatsing_PD_Iwan(Kp, Kd, m, Re, Im);
  Serial.print("Kp: ");
  Serial.println(Kp);
  Serial.print("Kd: ");
  Serial.println(Kd);

  if (t_nw - t_oud > cyclustijd) {  // Check if the cyclustijd has passed
    dt = (t_nw - t_oud) * 0.001;    // Calculate dt in seconds
    t_oud = t_nw;                   // Update t_oud to the current time

    Regeling_PD_Iwan(Fx, Fy, Kp, Kd, sp, sx, dt);
    Serial.print("PD Output Fx Iwan: ");
    Serial.println(Fx);
    motoraansturing_Iwan(Fx / 2);  // Control the motors with half of Fx
  }
}

void Poolplaatsing_PD_Iwan(float &Kp, float &Kd, float m, float Re, float Im) {
  Kp = (Re * Re + Im * Im) * m;
  Kd = 2 * Re * m;
}

void Regeling_PD_Iwan(float &Fx, float &Fy, float Kp, float Kd, float sp, float sx, float dt) {
  static float error_oud = 0;  // Initialize previous error
  float error = sp - sx;
  float d_error = (error - error_oud) / dt;  // Calculate derivative of error
  error_oud = error;                         // Update previous error
  Fx = (Kp - 1.5) * error + (Kd + 0.5) * d_error;
  Fx = Fy;

  Serial.print("Error: ");
  Serial.println(error, 4);
  Serial.print("Fx Iwan: ");
  Serial.println(Fx, 4);
}

void Motor_Links_Iwan(float Fx) {  // Dit is voor Maxon motor 1
  if (Fx > 0) {   // hierdoor gaat de uav achteruit
    Serial.print("PWM Maxon 1 pos: ");             
    float Fx_nega = Fx * -1;
    Serial.println(-0.00298 * Fx_nega * Fx_nega - 1.75115 * Fx_nega);
    digitalWrite(motorRechts, LOW); // dit is voor wanneer de hovercraft achterwaarts moet bewegen
    analogWrite(motorRechtsPWM, constrain(-0.00298 * Fx_nega * Fx_nega - 1.75115 * Fx_nega, -255, -50));
  } else {  // dit is voor wanneer de hovercraft voorwaartswaarts moet bewegen
    Serial.print("PWM Maxon 1 nega: ");             
    Serial.println(-0.00505 * Fx * Fx + 2.25550 * Fx);
    digitalWrite(motorRechts, HIGH);
    analogWrite(motorRechtsPWM, constrain(-0.00505 * Fx * Fx + 2.25550 * Fx, 50, 255) );
  }
}

void Motor_Rechts_Iwan(float Fx) {  // Dit is voor Maxon motor 2
  if (Fx > 0) {               // dit is voor wanneer de hovercraft achterwaarts moet bewegen
    Serial.print("PWM Maxon 2 pos: ");
    float Fx_nega = Fx * -1;             
    Serial.println(-0.00282 * Fx * Fx - 1.70725 * Fx);
    digitalWrite(motorLinks, LOW);
    analogWrite(motorLinksPWM, constrain(-0.00282 * Fx_nega * Fx_nega - 1.70725 * Fx_nega, -255, -50));
  } else {  // dit is voor wanneer de hovercraft voorwaarts moet bewegen
    Serial.print("PWM Maxon 2 nega: ");             
    Serial.println(-0.00425 * Fx * Fx + 2.077594 * Fx);
    digitalWrite(motorLinks, HIGH);
    analogWrite(motorLinksPWM, constrain(-0.00425 * Fx * Fx + 2.077594 * Fx, 50, 255));
  }
}

void motoraansturing_Iwan(float Fx) {
  Motor_Rechts_Iwan(Fx);
  Motor_Links_Iwan(Fx);
}
