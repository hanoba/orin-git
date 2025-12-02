# This script is not working as a systemd service
# Use this in the crontab for root:
# @reboot /usr/bin/python3 /home/harald/orin-git/PowerCtrl/TriggerShutdown.py >/dev/null 2>&1

import Jetson.GPIO as GPIO
import signal
import time
import subprocess

PIN_SHUTDOWN_REQ = 35

running = True
poweroff = False

def sigterm_handler(signum, frame):
    global running
    print("SIGTERM received: stopping service")
    running = False

def sigint_handler(signum, frame):
    global running
    print("SIGINT received: stopping service")
    running = False

def main():
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(PIN_SHUTDOWN_REQ, GPIO.IN)

    print("Jetson power manager active.")

    # Handler setzen
    signal.signal(signal.SIGTERM, sigterm_handler) 
    signal.signal(signal.SIGINT, sigint_handler)   

    global running

    while running:
        if GPIO.input(PIN_SHUTDOWN_REQ) == GPIO.HIGH:
            print("Shutdown requested by ATtiny.")
            subprocess.run(["/sbin/shutdown", "-h", "now"])
            time.sleep(1)

        time.sleep(0.03)

    time.sleep(0.2)
    GPIO.cleanup()

if __name__ == "__main__":
    main()
