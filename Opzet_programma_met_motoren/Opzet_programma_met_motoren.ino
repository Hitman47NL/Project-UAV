const int Relais_stroom = 22;
const int Relais_blowers = 23;
int PWMAPin = 11; // PWM output for motor speed control
int AIN1Pin = 28; // Motor direction pin 1
int AIN2Pin = 29; // Motor direction pin 2
int standBy = 24;
int stroommeter = A0;


void setup() {
  // declaratie serial communicatie
  Serial1.begin(9600);
  Serial.begin(9600);
  
  // declaratie van de gebruikte pinnen
  pinMode(Relais_stroom, OUTPUT);
  pinMode(Relais_blowers, OUTPUT);

  pinMode(PWMAPin, OUTPUT);
  pinMode(AIN1Pin, OUTPUT);
  pinMode(AIN2Pin, OUTPUT);
  pinMode(standBy, OUTPUT);
  
}

void loop() {
  // put your main code here, to run repeatedly:
  relais_schakelen();
  /*
  if(Serial.available() > 0){
    String command = Serial.readString();
    Serial.println(command);
    Serial.flush();

    if(command = 'f') klein_DC_motor_forward();
    else if(command = 'r') klein_DC_motor_backward();
    else if(command = 's') stop();
  }
  */
}

//Het opzetten van de functies voor de go/nogo

void relais_schakelen(){
  digitalWrite(Relais_stroom, HIGH);
  delay(100);
  digitalWrite(Relais_blowers, HIGH);
}

void klein_DC_motor_forward(){
  digitalWrite(standBy, HIGH);
  delay(100);
  Serial.println("Motor gaat voorwaards");
  digitalWrite(AIN1Pin, LOW);
  digitalWrite(AIN2Pin, HIGH);
  analogWrite(PWMAPin, 255); // Set speed (0 to 255)
}

void klein_DC_motor_backward(){
  digitalWrite(standBy, HIGH);
  delay(100);
  Serial.println("Motor gaat achterwaards");
  digitalWrite(AIN1Pin, HIGH);
  digitalWrite(AIN2Pin, LOW);
  analogWrite(PWMAPin, 255); // Set speed (0 to 255)
}

void stop(){
  digitalWrite(standBy, LOW);
  Serial.println("Motor staat uit");
}

void stroommeter(){
  analogReference(INTERNAL1V1); //https://www.benselectronics.nl/acs712-30a-stroomsensor-meter.html
  analogRead();
}