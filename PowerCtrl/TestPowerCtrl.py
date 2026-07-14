#!/usr/bin/env python3
# Ein einfaches Python-Skript, um einen Raspberry Pi GPIO-Pin ein- und auszuschalten.
# Verwendet die moderne gpiozero Bibliothek.

from gpiozero import LED
import time

# Wähle den gewünschten GPIO-Pin (BCM-Nummerierung, hier z.B. GPIO 26)
# (Physischer Pin 37 auf dem Raspberry Pi Header)
PIN_NUMMER = 26

def main():
    # Erstelle ein LED-Objekt für den Ausgangspin.
    # Das setzt den Pin automatisch als OUTPUT und auf LOW.
    ausgang = LED(PIN_NUMMER)
    
    print(f"Starte GPIO-Toggle auf Pin {PIN_NUMMER}...")
    print("Drücke STRG+C, um das Programm zu beenden.")
    
    try:
        # Endlosschleife zum Ein- und Ausschalten
        while True:
            # input() blockiert das Skript und wartet auf die Eingabe
            wert = int(input("Wert für raspiOn eingeben: "))
            
            ausgang.value = wert
            
            # Feedback in der Konsole geben
            if ausgang.is_active:
                print(" -> Pin ist jetzt HIGH (EIN)\n")
            else:
                print(" -> Pin ist jetzt LOW (AUS)\n")            
    except KeyboardInterrupt:
        # Wird ausgeführt, wenn du STRG+C im Terminal drückst
        print("\nProgramm durch Benutzer beendet.")
    finally:
        # gpiozero räumt die Pins am Ende des Skripts normalerweise automatisch auf,
        # aber es ist eine gute Praxis, Objekte sauber zu schließen.
        ausgang.close()
        print("GPIOs wurden sicher aufgeräumt.")

if __name__ == "__main__":
    main()
