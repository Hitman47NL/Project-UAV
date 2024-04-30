// Motor A
int enA = 9;
int in1 = 4;
int in2 = 5;

// Motor B
int enB = 10;
int in3 = 6;
int in4 = 7;

// Motor C
int enC = 3;
int in5 = 2;
int in6 = 8;

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


void setup() {
  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(enC, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  pinMode(in5, OUTPUT);
  pinMode(in6, OUTPUT);
  pinMode(noodstop, OUTPUT);
  pinMode(blower, OUTPUT);
  pinMode(stroommeter, INPUT);

  // Noodstop activatie
  digitalWrite(noodstop, HIGH);

  // Set motor direction
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  digitalWrite(in5, HIGH);
  digitalWrite(in6, LOW);

  analogWrite(enA, 128); // 50% duty cycle
  analogWrite(enB, 128); // 50% duty cycle
  analogWrite(enC, 128); // 50% duty cycle
  delay(1000); // spin for 1 second
  analogWrite(enA, 0); // stop motor 1
  analogWrite(enB, 0); // stop motor 2
  analogWrite(enC, 0); // stop motor 3
  delay(500); // waiting 0.5 seconds for the motors to stop before take off


  // Blowers acivatie
  digitalWrite(blower, HIGH)
}

void loop() {



}
