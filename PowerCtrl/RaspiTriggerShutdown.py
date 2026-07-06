#!/usr/bin/env python3
# Portiert von Orin-NX für Raspberry Pi mit gpiozero (Ressourcenschonend durch Interrupts)
# Use this in the crontab for root:
# @reboot /usr/bin/python3 /home/harald/orin-git/PowerCtrl/RaspiTriggerShutdown.py >/dev/null 2>&1

from gpiozero import Button
import signal
import time
import subprocess

# BCM Nummerierung: Physischer Board-Pin 35 entspricht GPIO 19
PIN_SHUTDOWN_REQ = 19

running = True

def sigterm_handler(signum, frame):
    global running
    print("SIGTERM empfangen: Beende Service sauber.", flush=True)
    running = False

def sigint_handler(signum, frame):
    global running
    print("SIGINT empfangen: Beende Service sauber.", flush=True)
    running = False

def trigger_shutdown():
    # Wird automatisch per Interrupt aufgerufen, wenn der Pin HIGH wird
    print("Shutdown vom ATtiny angefordert. System fährt herunter...", flush=True)
    # Verwende sudo, um Berechtigungsprobleme (z.B. in crontab) zu vermeiden
    subprocess.run(["sudo", "/sbin/shutdown", "-h", "now"])

def main():
    print("Raspberry Pi Power Manager aktiv. Warte auf Signal...", flush=True)

    # Signal-Handler setzen für ein sauberes Beenden des Skripts
    signal.signal(signal.SIGTERM, sigterm_handler) 
    signal.signal(signal.SIGINT,  sigint_handler)   

    global running

    # Button-Klasse ist ideal für Eingänge. 
    # pull_up=False aktiviert den internen Pull-Down. Das Signal schaltet auf HIGH.
    # bounce_time=0.1 entprellt das Signal leicht (100ms), um Fehlauslösungen zu vermeiden.
    shutdown_pin = Button(PIN_SHUTDOWN_REQ, pull_up=False, bounce_time=0.1)
    
    # Interrupt-Callback registrieren: Wenn der Pin HIGH wird, rufe die Funktion auf
    shutdown_pin.when_pressed = trigger_shutdown

    # Die Hauptschleife macht fast nichts mehr und hält das Skript nur am Leben.
    # Die tatsächliche Arbeit passiert unsichtbar im Hintergrund per Interrupt.
    while running:
        time.sleep(1)

    print("Power Manager beendet.")

if __name__ == "__main__":
    main()
