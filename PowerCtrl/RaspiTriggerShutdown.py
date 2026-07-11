#!/usr/bin/env python3
# Portiert von Orin-NX für Raspberry Pi mit gpiozero

from gpiozero import Button
import signal
import time
import subprocess

PIN_SHUTDOWN_REQ = 19   # GPIO19 = Header Pin 35
running = True
shutdown_triggered = False  # NEU: Verhindert mehrfaches Auslösen

def sigterm_handler(signum, frame):
    global running, shutdown_triggered
    if shutdown_triggered:
        # WICHTIG: Wir ignorieren SIGTERM, wenn wir bereits herunterfahren!
        # So bleibt das Skript am Leben und gpiozero behält die Kontrolle 
        # über die Pins, bis der Kernel den Pi endgültig hart abschaltet.
        print("SIGTERM ignoriert, Shutdown läuft bereits.", flush=True)
        return
        
    print("SIGTERM empfangen: Beende Service sauber.", flush=True)
    running = False

def sigint_handler(signum, frame):
    global running
    print("SIGINT empfangen: Beende Service sauber.", flush=True)
    running = False

def trigger_shutdown():
    global shutdown_triggered
    # Wenn der Shutdown schon läuft, mache nichts mehr
    if shutdown_triggered:
        return
        
    shutdown_triggered = True
    print("Shutdown vom ATtiny angefordert. System fährt herunter...", flush=True)
    
    # Verwende -P (Poweroff) statt -h, um einen echten Halt zu erzwingen
    subprocess.run(["/sbin/shutdown", "-P", "now"])

def main():
    global running
    print("Raspberry Pi Power Manager aktiv. Warte auf Signal...", flush=True)

    signal.signal(signal.SIGTERM, sigterm_handler) 
    signal.signal(signal.SIGINT,  sigint_handler)   

    shutdown_pin = Button(PIN_SHUTDOWN_REQ, pull_up=False, bounce_time=0.1)
    shutdown_pin.when_pressed = trigger_shutdown

    while running:
        time.sleep(1)

    print("Power Manager beendet.")

if __name__ == "__main__":
    main()