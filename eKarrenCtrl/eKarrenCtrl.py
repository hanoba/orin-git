#!/usr/bin/env python3
# coding: utf-8

# ==============================================================
# Projekt: Handgesteuerte Robotersteuerung mit YOLO-HandPose
# Autor: Harald Bauer 
# Datum: 2025-10-23
# Beschreibung: Steuert eKarren oder Rosmaster in Echtzeit basierend auf Handposition und -größe,
#               erkannt durch YOLO11 HandPose-Inferenz auf Jetson/TensorRT.
# ==============================================================

# ============================================================================
# Robotersteuerung mit Handverfolgung über YOLO-HandPose
# ============================================================================
# Dieses Skript nutzt eine Kamera und ein Handpose-Erkennungsnetz (YOLO11), um
# Bewegungen einer Hand zu erkennen und einen Roboter (Rosmaster) zu steuern.
# Die Bewegung erfolgt über PID-Regelung, die den Abstand (Handgröße) und die
# Position (seitliche Abweichung) der Hand ausgleicht.
#
# Ablauf:
# 1. Initialisierung des Roboters (Port, Threads, Startsignal)
# 2. Starten der Handpose-Erkennung über Kamera
# 3. Hauptschleife: Handposition und -größe werden erfasst
# 4. PID-Regelung berechnet lineare (vor/zurück) und Drehbewegung
# 5. Roboter fährt entsprechend nach
# 6. Beenden durch ESC oder STRG+C
# ============================================================================

import cv2
import time
import sys
from time import perf_counter_ns
from eKarrenLib import eKarren, DEV_ROSMASTER, DEV_EKARREN, DEV_EKARREN_PC, DEV_EKARREN_EMU
from emulator import Emulator
from AsyncHandPoseYolo11 import HandPoseApp
from pid import PID


def Usage():
    print("""    Elektrokarren Control Program

    The software is controlled via the camera. If a hand is the detected, the eKarren follows the hand.
    
    Usage: python eKarrenCtrl.py <device>
    
    <device> must be one of the following options:
    Rosmaster:  The target HW is the Rosmaster X3 PLus. 
                eKarrenCtrl directly controls Rosmaster and its motors.
    eKarren:    The target HW is the Elektrokarren. 
                eKarren commands are sent via UDP to Elektrokarren to control the motors.
    Emulator:   The target HW is an Elektrokarren emulated on Rosmaster. 
                eKarren commands are sent via UDP to Rosmaster and control the Rosmaster motors. 
    eKarrenPC:  eKarren commands are sent to the PC AZ-KENKO. 
                A test SW on the PC simply prints the received commands.
    """)
    sys.exit(0)


if __name__ == '__main__':
    useRosmaster = False
    argc = len(sys.argv)
    if argc == 2:
        if sys.argv[1] == "Rosmaster":
            device = DEV_ROSMASTER
        elif sys.argv[1] == "eKarren":
            device = DEV_EKARREN
        elif sys.argv[1] == "Emulator":
            device = DEV_EKARREN_EMU
            emulator = Emulator()
        elif sys.argv[1] == "eKarrenPC":
            device = DEV_EKARREN_PC
        else:
            Usage()
    else: Usage()
        
    # Roboter initialisieren
    bot = eKarren(device=device, debug=False)

    # Systeminformationen anzeigen
    # Abfrage der Firmware-Version und Batteriespannung des Roboters zur Statusanzeige
    print("Version:", bot.GetVersion())
    print("Vbat:", bot.GetBatteryVoltage())

    # Initialisierung von Steuerungsvariablen
    printCnt = 0
    angular = 0
    linear = 0
    wantedHandSize = 250  #HB 200 # Ziel-Handgröße: bestimmt den gewünschten Abstand zur Hand

    # PID-Regler-Parameter (Ziegler-Nichols Methode)
    Ku = 0.027 / 2      # Kritische Verstärkung (ultimate gain)
    Tu = 2.0            # Kritische Periodendauer (ultimate period) in Sekunden
    KpLinear = 0.5 * Ku
    KiLinear = 0.01 / 2
    pAngular = 1/150    # Proportionalfaktor für Drehung
    dt = 0.03           # Zeitintervall in Sekunden

    # PID-Regler für lineare Bewegung (Abstandsregelung)
    pidLinear = PID(KpLinear, KiLinear, 0, setpoint=wantedHandSize, output_limits=(-0.5, 0.5))

    # Kamera-ID festlegen und HandPoseApp starten
    CAM_ID = 0
    handPoseApp = HandPoseApp(CAM_ID)

    lastTime = perf_counter_ns()
    
    try:
        # Hauptschleife für Handverfolgung und Robotersteuerung
        while True:
            # Bildverarbeitung: Handgröße & Position bestimmen
            exitFlag, handSize, handPos = handPoseApp.ProcessFrame()
            if exitFlag:
                break  # ESC-Taste gedrückt

            fps = handPoseApp.GetFps()
            dt = 1 / fps        # Zeitintervall in Sekunden

            # Wenn keine Hand erkannt wurde
            if handSize < 0:
                break
            elif handSize > 0:
                # PID-Regler berechnet Geschwindigkeit basierend auf Handgröße
                linear = pidLinear.update(handSize, dt)

                # Seitliche Abweichung der Hand → Drehbewegung berechnen
                eHandPos = 0 - handPos
                angular = eHandPos * pAngular

                # Begrenzung der Geschwindigkeiten (Sicherheitslimit)
                angular = max(-2.5, min(angular, 2.5))
            else:
                angular = 0
                linear = 0

            # Steuerbefehl an Roboter senden (Vorwärts-/Rückwärts- und Drehbewegung)
            # v_x=[-0.7, 0.7] m/s, v_y=[-0.7, 0.7] m/s, v_z=[-3.2, 3.2] rad/sec
            bot.SetSpeed(linear, angular)
                
            # Emulation handling
            if device==DEV_EKARREN_EMU:
                emuLinear, emuOmega, udpMsg = emulator.Run()

            # Statusausgabe alle Sekunde
            now = perf_counter_ns()
            if now - lastTime > 1e9:
                lastTime = now
                Vbat = bot.GetBatteryVoltage()
                if device==DEV_EKARREN_EMU:
                    print(f"{fps:.1f} FPS lin={linear:.3f} ({emuLinear:.3f}) m/s, ang={angular:.3f} ({emuOmega:.3f}) rad/s, "
                        f"{handSize=}, {handPos=}, {Vbat=}V  {udpMsg}")
                else:
                    print(f"{fps:.1f} FPS lin={linear:.3f} m/s, ang={angular:.3f} rad/s, {handSize=}, {handPos=}, {Vbat=}V")

    except KeyboardInterrupt:
        # Beenden über STRG+C
        print("\nScript terminated")
        if device==DEV_EKARREN_EMU: emulator.Close()

    # Nach Beendigung: Licht ausschalten, Bewegung stoppen, Kamera schließen
    bot.Close()
    handPoseApp.Exit()
