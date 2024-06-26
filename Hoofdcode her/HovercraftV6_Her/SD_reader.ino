void writeDataToFile(byte *buffer, unsigned bufferSize) {
  unsigned fileCounter = 0;
  String filename;
  do {
    filename = "data" + String(fileCounter) + ".bin";
    fileCounter++;
  } while (SD.exists(filename));  // Controleer of het bestand al bestaat
  File dataFile = SD.open(filename, FILE_WRITE);

  if (dataFile) {
    Serial.println("Schrijven naar " + filename + "...");
    // Schrijven van de buffer naar het bestand
    dataFile.write(buffer, bufferSize);

    // Sluiten van het bestand
    dataFile.close();
    Serial.println("Schrijven naar " + filename + " voltooid");
  } else {
    Serial.println("Fout bij het openen van het bestand");
  }
}

void SD_Kaart() {
  static const unsigned numElements = maxBufferSize / sizeof(SensorData);  // Bereken hoeveel elementen (structs) er weggeschreven kunnen worden in de buffer.
  static SensorData buffer[numElements];
  static boolean stopWriting = false;
  static const long samplingTimeDataLogging = 100;  // ms, bepaalt frequentie waarmee sensordata wordt weggeschreven.
  static const long timeStopWriting = 60000;        // ms, bepaalt frequentie waarmee sensordata wordt weggeschreven.
  static long t_lastDataLog = 0, t_new;             // ms, ms;
  static int currentIndex = 0;

  t_new = millis();  // Lees de tijd
  if (t_new - t_lastDataLog > samplingTimeDataLogging) {
    t_lastDataLog = t_new;

    if (!stopWriting) {
      buffer[currentIndex].time = t_new;

      // Vul de struct met je meetwaardes.
      buffer[currentIndex].value1 = currentIndex * 1.5;
      buffer[currentIndex].value2 = currentIndex * 2.5;
      buffer[currentIndex].value3 = currentIndex * 3.5;

      currentIndex++;

      // Schrijf volle buffer weg naar een bestand
      if (currentIndex == numElements) {
        writeDataToFile((byte *)buffer, numElements * sizeof(SensorData));
        currentIndex = 0;
      }

      // Stop na een bepaalde tijd met het wegschrijven van bestanden
      if (t_new > timeStopWriting) {
        Serial.println("Gestopt met meetdata wegschrijven.");
        stopWriting = true;
      }
    }
  }
}