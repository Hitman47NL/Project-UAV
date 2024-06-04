/*
    Simulatie van plaats x en snelheid v van een massa m en een stuwkracht F
    Bewegingsvergelijking:
       m.a = F

    Integratie obv
       a = dv/dt
       v = dx/dt

    => dv = a.dt => v_nw - v_oud = a.dt => v_nw = v_oud + a.dt =>
       v = v + a*dt
       x = x + v*dt

  setup
    meet t0
    of simulatie: gebruik beginwaarden van x en v
    of realisatie: meet x
    bereken error
    error_oud = error
    bereken F, a mbv regelaar
    of sim: plot F, x, v, a
    of rea: stuur motoren aan
  loop
    meet t totdat tijd voor nieuwe regelstap is aangebroken (om de 10 ms)
      of sim: bereken x en v
      of rea: meet x
      error_oud = error
      bereken error
      bereken F, a mbv regelaar
      of sim: plot F, x, v, a
      of rea: stuur motoren aan

  Poolplaatsing s-domein
  Im(pool) = 2 x Re(pool) geeft Doorschot van 20%. Goed meetbaar!
  Im(pool) = Re(pool) geeft Doorschot van 4%. Niet goed meetbaar!

*/
const bool simulatie = true;

const int pwmPenL = 5;      // Pen Motor links  Mega: pen 4  980 Hz
const int pwmPenR = 6;      // Pen Motor rechts Mega: pen 13 980 Hz
const int pwmPenM = 3;      // Pen Motor midden Mega: pen 3  490 Hz

const long cyclustijd = 10;      // ms;       Regelaar wordt 100x per seconde ververst.
long t_oud, t_nw;                // ms, ms;
float dt = 1;                    // s;        Nodig vanwege d_error / dt in de regelaar
const float m = 0.8;             // kg;       Massa van de UAV
float Fx = 0.1, Fy, Mz;          // N, N, Nm; Stuurkrachten en stuurmoment
const float Fmin = -5, Fmax = 5; // Maximering van stuwkracht voor meer realistische simulatie
float ax, ay, alfa;              // m/s2;     Versnellingen en hoekversnelling
float vx    = 0.0, sx    = 0.0;  // m/s, m;   Beginwaarden tbv simulatie
float vy    = 0.0, sy    = 0.0;  // m/s, m;   Beginwaarden tbv simulatie
float omega = 0.0, theta = 0.0;  // m/s, m;   Beginwaarden tbv simulatie
float Kp = 1, Kd = 1, Ki = 1;    // regelaarparameters PID-regelaar
float error, error_oud, d_error, errorSom; // De errors voor een PID-regelaar
const float sp = 1.0;            // Setpoint, nodig om sprongresponsie van 0 naar 1 te simuleren

// Zorg dat de plot op Ã©Ã©n pagina past
const long simulatietijd_in_s = long(10); // s; De simulatietijd is instelbaar
const long simulatietijd = simulatietijd_in_s * long(1000); // ms
const int pixels = 499;          // aantal weer te geven punten in de Serial plotter
int pixel = 0;                   // teller van het aantal reeds weergegeven punten
const float tijdPerPixel = simulatietijd / pixels;
double tijdVoorNwePixelPlot;

float ToF() {
  float afstand;
  return (afstand);
}
int pwm_links() {
  const int aL = 300, bL = 100;
  analogWrite(pwmPenL, constrain(aL * Fx / 2 + bL, 0, 0));
}
int pwm_rechts() {
  const int aR = 302, bR = 105;
  analogWrite(pwmPenR, constrain(aR * Fx / 2 + bR, 0, 0));
}
int pwm_midden() {
  const int aM = 302, bM = 105;
  analogWrite(pwmPenM, constrain(aM * Fy + bM, 0, 0));
}
void motoraansturing() {
  pwm_links();     // Bereken en schrijf alle uitgangen
  pwm_rechts();
  pwm_midden();
}

void plot() {
  tijdVoorNwePixelPlot = tijdVoorNwePixelPlot + tijdPerPixel;
  Serial.print(Fx * 1);
  Serial.print(" ");
  Serial.print(ax * 1);
  Serial.print(" ");
  Serial.print(vx * 1);
  Serial.print(" ");
  Serial.println(sx * 1);
  pixel = pixel + 1;
}

void Poolplaatsing() {
  const float Re = 1, Im = 2, pool3 = 0.5; // Poolplaatsing Im(pool) = 2 x Re(pool): Doorschot = 20%
  // Berekening PD-parameters adhv de poolplaatsing
   Kp = (Re * Re + Im * Im) * m;
   Kd = 2 * Re * m;

  // Berekening PID-parameters adhv de poolplaatsing
  //  Kp = (Re * Re + Im * Im + 2 * Re * pool3) * m;
  //  Kd = (2 * Re + pool3) * m;
  //  Ki = (Re * Re + Im * Im) * pool3 * m;
}

void Regelaar() {
  error_oud = error;
  error = sp - sx;
  d_error = error - error_oud;
  errorSom = errorSom + error * dt;
  Fx = Kp * error + Kd * d_error / dt; // + Ki * errorSom;
  constrain(Fx, Fmin, Fmax);
  ax = Fx / m;     // bereken de versnelling
}

void setup() {
  if (simulatie) {
    Serial.begin(57600);
    Serial.print("F a v x");  // De legenda
    if (tijdPerPixel < cyclustijd) Serial.println("___XXXXXX____tijdPerPixel.<.cyclustijd____XXXXXX");
    else Serial.println();
  }
  t_oud = millis();
  if (simulatie) tijdVoorNwePixelPlot = t_oud;
  if (not simulatie) sx = ToF();
  error = sp - sx;
  Poolplaatsing();
  Regelaar();
  if (simulatie) plot(); 
  else motoraansturing();
}

void loop() {
  t_nw = millis();    // Lees de tijd
  if (t_nw - t_oud > cyclustijd) {
    dt = (t_nw - t_oud) * .001;
    t_oud = t_nw;

    if (simulatie) {
      sx = sx + vx * dt; // Bereken x (gebruik v_oud)
      vx = vx + ax * dt; // Bereken v
    } else {
      sx = ToF();      // Meet x
    }
    Regelaar();

    if (simulatie) {
      if (pixel == pixels) while (true);       // Vanglus. Stop met plotten als scherm vol is
      if (t_nw > tijdVoorNwePixelPlot) plot(); // Alleen plotten als tijdPerPixel is verlopen
    }
    else {
      motoraansturing();
    }
  }
}
