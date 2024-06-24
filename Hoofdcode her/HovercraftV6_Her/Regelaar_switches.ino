void Regelaars(){
    switch (currentState) {
    case Teun:
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Regelaar 1");
    
      Regelaar_Teun();
      if(Regelaar_Teun_Succes){
        currentState = Jelle;
      }
      break;

    case Jelle:
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Regelaar 2");
      Regelaar_Jelle();
      if(Regelaar_Jelle_Succes){
        currentState = Iwan;
      }
      break;

    case Bram:
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Regelaar 3");
      Regelaar_Bram();
      if(Regelaar_Bram_Succes){
        currentState = Iwan;
      }      
      break;

    case Iwan:
        lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Regelaar 4");
      Regelaar_Iwan();
            if(Regelaar_Iwan_Succes){
        currentState = Jari;
      }  
      break;

    case Jari:
            lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Regelaar 5");
      Regelaar_Jari();
                  if(Regelaar_Jari_Succes){
        currentState = Maurits;
      }  
      break;

    case Maurits:
                lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Regelaar 6");
    shutOFF();
    digitalWrite(BLOWRELAY, LOW);
      //Regelaar_Maurits();
      delay(500);
      softwareReset();
      break;

    default:
      // Handle unexpected states
      break;
  }
}