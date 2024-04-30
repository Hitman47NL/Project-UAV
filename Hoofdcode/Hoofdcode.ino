// Motor A
int enmotorA = 9;
int in1_rechtsom = 4;
int in2_linksom = 5;

// Motor B
int enmotorB = 10;
int in3_rechtsom = 6;
int in4_linksom = 7;

// Motor C
int enmotorC = 3;
int in5_rechtsom = 2;
int in6_linksom = 8;

// Noodstop relay
int noodstop = 22;

// blower relay
int blower = 23;

// tof sensoren
int tofA_1 = ;
int tofA_2 = ;

int tofB_1 = ;
int tofB_2 = ;

int tofC_1 = ;
int tofC_2 = ;

// Stroommeter
int stroommeter = A1;

// Gyro

// Batterij monitoren
int batcel_1 = A2;
int batcel_2 = A3;
int batcel_3 = A4;

void setup() {
  pinMode(enmotorA, OUTPUT);
  pinMode(enmotorB, OUTPUT);
  pinMode(enmotorC, OUTPUT);
  pinMode(in1_rechtsom, OUTPUT);
  pinMode(in2_linksom, OUTPUT);
  pinMode(in3_rechtsom, OUTPUT);
  pinMode(in4_linksom, OUTPUT);
  pinMode(in5_rechtsom, OUTPUT);
  pinMode(in6_linksom, OUTPUT);
  pinMode(noodstop, OUTPUT);
  pinMode(blower, OUTPUT);
  pinMode(stroommeter, INPUT);
  pinMode(batcel_1, INPUT);
  pinMode(batcel_2, INPUT);
  pinMode(batcel_3, INPUT);

  // Noodstop activatie
  digitalWrite(noodstop, HIGH);

  // Set motor direction
  digitalWrite(in1_rechtsom, HIGH);
  digitalWrite(in2_linksom, LOW);
  digitalWrite(in3_rechtsom, HIGH);
  digitalWrite(in4_linksom, LOW);
  digitalWrite(in5_rechtsom, HIGH);
  digitalWrite(in6_linksom, LOW);

  analogWrite(enmotorA, 128); // 50% duty cycle
  analogWrite(enmotorB, 128); // 50% duty cycle
  analogWrite(enmotorC, 128); // 50% duty cycle
  delay(1000); // spin for 1 second
  analogWrite(enmotorA, 0); // stop motor 1
  analogWrite(enmotorB, 0); // stop motor 2
  analogWrite(enmotorC, 0); // stop motor 3
  delay(500); // waiting 0.5 seconds for the motors to stop before take off


  // Blowers acivatie
  digitalWrite(blower, HIGH)
}

void loop() {



}
