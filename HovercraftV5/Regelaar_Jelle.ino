
// Functie prototypes
void Regeling_PID_Jelle(float &Mz, float &Fy, float Kp, float Kd, float Ki, float sp, float theta, float dt);
void Poolplaatsing_PID_Jelle(float &Kp, float &Kd, float &Ki, float Iz, float Re, float Im, float Pool3);
void Motoraansturing_Jelle(float Mz);
void Motor_Links_Jelle(float Mz);
void Motor_Rechts_Jelle(float Mz);
    int consecutiveCount = 0;
void waitForSensorValue2() {
  readDualSensors();
    unsigned long startTime = millis();
    unsigned long endTime = startTime + 1000 * 1000; // Convert seconds to milliseconds

        float theta = calculateAngle();  // Replace with actual sensor reading function
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
        Regelaar_Jelle_Succes = true;
        consecutiveCount = 0;
}}
void Regelaar_Jelle() {
  readDualSensors();
    const long cyclustijd = 10;  // Cyclustijd in ms
    static long t_oud = 0;       // Initialize t_oud to 0 at the beginning
    long t_nw = millis();        // Get the current time in ms
    const float Re = 2.5, Im = 4, pool3 = 0.005;
    const float m = 1.560;
    const float Iz = 0.115405;
    float dt = 1.0;           // Nodig voor de d_error / dt
    float Mz = 0.0;
    float Fy = 0.0;
    float theta = calculateAngle(); // Vervang calculateAngle door je eigen functie voor het bepalen van de hoek theta
    
    const float sp = 0.0;

    Serial.print("theta: ");
    Serial.println(theta);

    Poolplaatsing_PID_Jelle(Kp, Kd, Ki, Iz, Re, Im, pool3);
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

        Regeling_PID_Jelle(Mz, Fy, Kp, Kd, Ki, sp, theta, dt);
        Serial.print("PID_Jelle: ");
        Serial.print(Mz);
        Motoraansturing_Jelle(Mz / 2);  // Control the motors with half of Mz
    waitForSensorValue2();
    }
}

void Regeling_PID_Jelle(float &Mz, float &Fy, float Kp, float Kd, float Ki, float sp, float theta, float dt) {
    static float error_oud = 0;  // Initialize previous error
    float error = sp - theta;
    float d_error = (error - error_oud) / dt;  // Calculate derivative of error
    errorSom = errorSom + error * dt;
    error_oud = error;                         // Update previous error
    Mz = (Kp * error) + (Kd * d_error / dt) + (Ki * errorSom);

    Serial.print("Error: ");
    Serial.println(error);
    Serial.print("d_error: ");
    Serial.println(d_error);
    Serial.print("Mz: ");
    Serial.println(Mz);
}

void Poolplaatsing_PID_Jelle(float &Kp, float &Kd, float &Ki, float Iz, float Re, float Im, float Pool3) {
    // Implementeer hier je logica voor Poolplaatsing_PID
    // Zorg ervoor dat je Kp, Kd en Ki instelt op de juiste waarden
    // Afhankelijk van de parameters Iz, Re, Im en Pool3
    Kp = 1.0; // Voorbeeldwaarde, vervang door de juiste berekening
    Kd = 0.1; // Voorbeeldwaarde, vervang door de juiste berekening
    Ki = 0.01; // Voorbeeldwaarde, vervang door de juiste berekening

    // Simulatie van instellingen
    Serial.println("Poolplaatsing_PID instellingen toegepast");
}

void Motoraansturing_Jelle(float Mz) {
    Motor_Links_Jelle(Mz);
    Motor_Rechts_Jelle(Mz);
}

void Motor_Links_Jelle(float Mz) {  // Dit is voor Maxon motor 1
    if (Mz > 0) {
        Serial.print("PWM Maxon 1 pos: ");
        Serial.println(-0.00298 * Mz * Mz - 1.75115 * Mz);
        digitalWrite(motorRechts, LOW);  // Achteruit bewegen
        analogWrite(motorRechtsPWM, -0.00298 * Mz * Mz - 1.75115 * Mz);
    } else {
        Serial.print("PWM Maxon 1 nega: ");
        Serial.println(-0.00505 * Mz * Mz + 2.25550 * Mz);
        digitalWrite(motorRechts, HIGH);  // Vooruit bewegen
        analogWrite(motorRechtsPWM, -0.00505 * Mz * Mz + 2.25550 * Mz);
    }
}

void Motor_Rechts_Jelle(float Mz) {  // Dit is voor Maxon motor 2
    if (Mz < 0) {
        Serial.print("PWM Maxon 2 pos: ");
        Serial.println(-0.00282 * Mz * Mz - 1.70725 * Mz);
        digitalWrite(motorLinks, LOW);  // Achteruit bewegen
        analogWrite(motorLinksPWM, -0.00282 * Mz * Mz - 1.70725 * Mz);
    } else {
        Serial.print("PWM Maxon 2 nega: ");
        Serial.println(-0.00425 * Mz * Mz + 2.077594 * Mz);
        digitalWrite(motorLinks, HIGH);  // Vooruit bewegen
        analogWrite(motorLinksPWM, -0.00425 * Mz * Mz + 2.077594 * Mz);
    }
}
