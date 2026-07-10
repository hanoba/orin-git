#define N 5  // Anzahl der erwarteten int16_t Werte
#define RX_SERIAL Serial2 // Serieller Port für den Empfang
#define DEBUG_SERIAL Serial // Serieller Monitor für den PC (USB)

void setup() {
  RX_SERIAL.begin(115200);
  DEBUG_SERIAL.begin(9600);
  while (!DEBUG_SERIAL); // Warte auf USB-Verbindung (nur für PC-Monitor)
  DEBUG_SERIAL.println("Warte auf synchronisierten UART-Datenstrom...");
}

uint32_t numPackets = 0;
uint32_t numErrors = 0;
bool debug = true;

void loop() {
  // Wir brauchen mindestens 2 Bytes (Header), um überhaupt die Suche zu starten
  if (RX_SERIAL.available() >= 2) {
    
    // Suche nach dem ersten Start-Byte
    if (RX_SERIAL.read() == 0xAA) {
      
      // Prüfe, ob das direkt folgende Byte das zweite Start-Byte ist
      if (RX_SERIAL.read() == 0xBB) {
        
        // Header gefunden! Jetzt warten wir, bis auch die Daten + Checksum komplett im Puffer sind
        unsigned long startTime = millis();
        size_t bytesNeeded = (N * 2) + 1;
        
        // Timeout-Schleife (verhindert Blockieren, falls die Übertragung abbricht)
        while (RX_SERIAL.available() < bytesNeeded) {
          if (millis() - startTime > 50) {
            DEBUG_SERIAL.println("Fehler: Timeout während des Frames!");
            return; 
          }
        }
        
        // Daten- und Checksummen-Bytes einlesen und dabei Checksumme lokal nachrechnen
        int16_t empfangeneDaten[N];
        uint8_t calculatedChecksum = 0;
        for (int i = 0; i < N; i++) 
        {
            uint8_t highByte = RX_SERIAL.read();
            uint8_t lowByte  = RX_SERIAL.read();
            empfangeneDaten[i] = (int16_t)((highByte<<8) | lowByte);
            calculatedChecksum += highByte + lowByte;
        }
        uint8_t receivedChecksum = RX_SERIAL.read();
        
        // Validierung
        if (calculatedChecksum == receivedChecksum) 
        {
            // Ausgabe auf dem Seriellen Monitor
            if (debug)
            {
                DEBUG_SERIAL.println("--- Paket erfolgreich empfangen und synchronisiert ---");
                for (int i = 0; i < N; i++) {
                  DEBUG_SERIAL.print("Wert [");
                  DEBUG_SERIAL.print(i);
                  DEBUG_SERIAL.print("]: ");
                  DEBUG_SERIAL.println(empfangeneDaten[i]);
                }
                DEBUG_SERIAL.println();
            }
            numPackets++;
            if (numPackets % 100 == 0)
            {
                  DEBUG_SERIAL.print("numPackets: ");
                  DEBUG_SERIAL.print(numPackets);
                  DEBUG_SERIAL.print("    numErrors: ");
                  DEBUG_SERIAL.println(numErrors);
            }
          
        } 
        else 
        {
            if (debug) DEBUG_SERIAL.println("Fehler: Checksumme falsch! Paket verworfen.");
            numErrors++;
        }
      }
    }
  }
}