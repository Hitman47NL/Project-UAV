void playTone(int tone, int duration) {
  for (long i = 0; i < duration * 100L; i += tone * 2) {
    digitalWrite(BUZZER_PIN, HIGH);
    delayMicroseconds(tone);
    digitalWrite(BUZZER_PIN, LOW);
    delayMicroseconds(tone);
  }
}
//Geluid voor succes
void playSuccessTune() {
  int melody[] = { 220, 196, 165, 131 };
  int noteDuration = 150;

  for (int note : melody) {
    playTone(note, noteDuration);
    delay(150);
  }
}
//Geluid voor vaal
void playFailTune() {
  int melody[] = { 294, 392, 600, 1600 };
  int noteDuration = 150;

  for (int note : melody) {
    playTone(note, noteDuration);
    delay(150);
  }
}