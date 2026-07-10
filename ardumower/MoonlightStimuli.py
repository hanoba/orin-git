#!/usr/bin/env python3
import serial
import time
import struct


class Ardumower:
    def __init__(self):
        self.cLinear = 2048.0 / 0.5
        self.cAngular = 2048.0 / 1.0
        SERIAL_PORT = '/dev/serial0'
        BAUDRATE = 115200
        print(f"Öffne seriellen Port {SERIAL_PORT} mit {BAUDRATE} Baud...")
        try:
            # Seriellen Port initialisieren
            self.ser = serial.Serial(SERIAL_PORT, BAUDRATE, timeout=1)
        except serial.SerialException as e:
            print(f"Fehler beim Öffnen des Ports: {e}")
            return

        print("Sender aktiv. Sende synchronisierte Datenpakete (Beenden mit STRG+C)...")
        
    def CalculateChecksum(self, data_bytes):
        """Berechnet die einfache 8-Bit-Checksumme durch Aufaddieren aller Bytes."""
        checksum = 0
        for byte in data_bytes:
            checksum = (checksum + byte) & 0xFF
        return checksum

    def SetSpeed(self, linear, angular):
        mode = 0
        iLinear = round(linear*self.cLinear)
        iAngular = round(angular*self.cAngular)
        daten = [iLinear, iAngular, mode]

        # 1. Header als Bytes definieren
        header = bytes([0xAA, 0xBB])
        
        # 2. Daten-Bytes erzeugen
        # 'h' steht in Pythons struct-Bibliothek für ein 16-Bit Signed Integer (short).
        # '>' erzwingt Big-Endian (High-Byte zuerst), passend zum Arduino-Code.
        # Da wir 3 Werte haben, wiederholen wir das 'h' 3-mal (z.B. '>hhhhh')
        format_string = f'>{3}h'
        
        try:
            data_bytes = struct.pack(format_string, *daten)
        except struct.error as e:
            print(f"Fehler beim Verpacken der Daten (Wert außerhalb int16-Bereich?): {e}")
            return

        # 3. Checksumme über die reinen Daten-Bytes berechnen
        checksum_byte = bytes([self.CalculateChecksum(data_bytes)])

        # 4. Den kompletten Frame zusammenfügen
        full_frame = header + data_bytes + checksum_byte

        # Frame über UART senden
        self.ser.write(full_frame)
        
        # Optionale Konsolenausgabe zur Kontrolle
        #print(f"Gesendet: {daten} | Frame-Hex: {full_frame.hex().upper()}")

    def Close(self):
        self.ser.close()
    
def main():
    ardumower = Ardumower()
    linear = 0.2   # m/s
    angular = 0.3  # rad/s
    try:
        while True:
            ardumower.SetSpeed(linear, angular)

            # 20ms Pause bis zum nächsten Paket
            time.sleep(0.02)
    except KeyboardInterrupt:
        print("\nProgramm vom Benutzer abgebrochen.")
    finally:
        ardumower.Close()

if __name__ == "__main__":
    main()
