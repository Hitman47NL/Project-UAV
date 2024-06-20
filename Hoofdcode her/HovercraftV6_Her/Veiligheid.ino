void softwareReset() {
  cli();
  wdt_enable(WDTO_15MS);
  while (1) {}  // Wait for watchdog timer to trigger reset
}

void checkBattery() {
  Serial.println("Battery raw: " + String(aDC));
  Serial.println("Voltage: " + String(voltage));
  Serial.println("Battery Ampere: " + String(ampere));
  float ACSDC = stroommeter.getCurrentDC();

  Serial.println("Battery VOLT: " + String(ACSDC));

  if(ampere >= LOW_BATTERY_THRESHOLD || ACCU_SHUT_OFF == 1) {
    Serial.println("LOW battery! Ampere: " + String(ampere));
    playFailTune();
    digitalWrite(NOODSTOPRELAY, LOW);
    digitalWrite(BLOWRELAY, LOW);
    while(1);
  }
}