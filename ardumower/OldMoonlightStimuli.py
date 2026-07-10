#!/usr/bin/env python3
import serial
import time
import struct

# Konfiguration
SERIAL_PORT = '/dev/serial0'  # Pfad anpassen (z.B. /dev/ttyUSB0 oder /dev/ttyAMA0 beim Raspi-Header)
BAUDRATE = 115200
N = 3                         # Anzahl der int16_t Werte

def calculate_checksum(data_bytes):
    """Berechnet die einfache 8-Bit-Checksumme durch Aufaddieren aller Bytes."""
    checksum = 0
    for byte in data_bytes:
        checksum = (checksum + byte) & 0xFF
    return checksum

def main():
    # Beispiel-Datenarray mit 3 int16-Werten
    cLinear = 2048.0 / 0.5
    cAngular = 2048.0 / 1.0
    
    linear = 0.2;   # m/s
    angular = 0.3;  # rad/s
    mode = 0
    iLinear = round(linear*cLinear)
    iAngular = round(angular*cAngular)
    daten = [iLinear, iAngular, mode]

    print(f"Öffne seriellen Port {SERIAL_PORT} mit {BAUDRATE} Baud...")
    try:
        # Seriellen Port initialisieren
        ser = serial.Serial(SERIAL_PORT, BAUDRATE, timeout=1)
    except serial.SerialException as e:
        print(f"Fehler beim Öffnen des Ports: {e}")
        return

    print("Sender aktiv. Sende synchronisierte Datenpakete (Beenden mit STRG+C)...")

    try:
        while True:
            # 1. Header als Bytes definieren
            header = bytes([0xAA, 0xBB])
            
            # 2. Daten-Bytes erzeugen
            # 'h' steht in Pythons struct-Bibliothek für ein 16-Bit Signed Integer (short).
            # '>' erzwingt Big-Endian (High-Byte zuerst), passend zum Arduino-Code.
            # Da wir N Werte haben, wiederholen wir das 'h' N-mal (z.B. '>hhhhh')
            format_string = f'>{N}h'
            
            try:
                data_bytes = struct.pack(format_string, *daten)
            except struct.error as e:
                print(f"Fehler beim Verpacken der Daten (Wert außerhalb int16-Bereich?): {e}")
                break

            # 3. Checksumme über die reinen Daten-Bytes berechnen
            checksum_byte = bytes([calculate_checksum(data_bytes)])

            # 4. Den kompletten Frame zusammenfügen
            full_frame = header + data_bytes + checksum_byte

            # Frame über UART senden
            ser.write(full_frame)
            
            # Optionale Konsolenausgabe zur Kontrolle
            #print(f"Gesendet: {daten} | Frame-Hex: {full_frame.hex().upper()}")

            # Werte für den Test leicht verändern (wie im Arduino-Beispiel)
            #daten[0] += 1
            #daten[1] -= 1
            # Grenzen einhalten, um struct.pack-Fehler bei Überlauf zu verhindern
            if daten[0] > 32767: daten[0] = 0
            if daten[1] < -32768: daten[1] = 0

            # 20ms Pause bis zum nächsten Paket
            time.sleep(0.02)

    except KeyboardInterrupt:
        print("\nProgramm vom Benutzer abgebrochen.")
    finally:
        ser.close()
        print("Serieller Port geschlossen.")

if __name__ == "__main__":
    main()
