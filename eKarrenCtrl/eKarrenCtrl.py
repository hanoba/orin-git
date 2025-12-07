#!/usr/bin/env python3
# coding: utf-8

# ==============================================================
# Projekt: Robotersteuerung mit yolo11n-pose
# Autor: Harald Bauer 
# Datum: 2025-12-06
# Beschreibung: Steuert eKarren oder Rosmaster in Echtzeit basierend auf Handposition und -größe,
#               erkannt durch YOLO11-Pose-Inferenz auf Jetson/TensorRT.
# ==============================================================

# ============================================================================
# Robotersteuerung mit Handverfolgung über YOLO-HandPose
# ============================================================================
# Dieses Skript nutzt eine Kamera und ein Body-Pose-Erkennungsnetz (YOLO11-Pose, um
# um Gesten Bewegungen einer Person zu erkennen und einen Roboter (Rosmaster) zu steuern.
# Die Bewegung erfolgt über PID-Regelung, die den Abstand der (gemessen über den 
# Hüftabstand) und die Position (seitliche Abweichung) der Person ausgleicht.
#
# Ablauf:
# 1. Initialisierung des Roboters (Port, Threads, Startsignal)
# 2. Starten der Handpose-Erkennung über Kamera
# 3. Hauptschleife: Position und -größe der Person werden erfasst
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
from AsyncPoseYolo11 import BodyPoseApp
from pid import PID


def Usage():
    print("""    Elektrokarren Control Program

    The software is controlled via the camera. If a person is the detected, the eKarren follows the person.
    
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
    wantedSize = 80  # Der Abstand zur Person wird durch den gemessenen Schulterabstand bestimmt

    # PID-Regler-Parameter (Ziegler-Nichols Methode)
    Ku = 0.027 / 2      # Kritische Verstärkung (ultimate gain)
    Tu = 2.0            # Kritische Periodendauer (ultimate period) in Sekunden
    KpLinear = 4/150    # 0.5 * Ku
    KiLinear = 0.01 / 2 * 0 #HB
    pAngular = 1/150    # Proportionalfaktor für Drehung
    dt = 0.03           # Zeitintervall in Sekunden

    # PID-Regler für lineare Bewegung (Abstandsregelung)
    pidLinear = PID(KpLinear, KiLinear, 0, setpoint=wantedSize, output_limits=(-0.5, 0.5))

    # Kamera-ID festlegen und HandPoseApp starten
    CAM_ID = 0
    app = BodyPoseApp(CAM_ID)

    lastTime = perf_counter_ns()
    
    try:
        # Hauptschleife für Personenverfolgung und Robotersteuerung
        followMode = False
        while True:
            # Bildverarbeitung: Handgröße & Position bestimmen
            exitFlag, personSize, personPos, pose = app.ProcessFrame()
            if exitFlag:
                break  # ESC-Taste gedrückt

            fps = app.GetFps()
            dt = 1 / fps        # Zeitintervall in Sekunden

            # Steuerung durch Pose
            if pose == "LeftArmUp": followMode = True
            elif pose == "RightArmUp": followMode = False
            
            # Wenn keine Hand erkannt wurde
            if personSize < 0:
                break
            elif personSize > 0:
                # PID-Regler berechnet Geschwindigkeit basierend auf Handgröße
                linear = pidLinear.update(personSize, dt)

                # Seitliche Abweichung der Hand → Drehbewegung berechnen
                errPersonPos = 0 - personPos
                angular = errPersonPos * pAngular

                # Begrenzung der Geschwindigkeiten (Sicherheitslimit)
                angular = max(-2.5, min(angular, 2.5))
            else:
                angular = 0
                linear = 0
            
            if followMode == False:
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
                        f"{personSize=}, {personPos=}, {pose}, {followMode=}, {Vbat=}V  {udpMsg}")
                else:
                    print(f"{fps:.1f} FPS lin={linear:.3f} m/s, ang={angular:.3f} rad/s, {personSize=}, {personPos=}, {pose}, {followMode=}, {Vbat=}V")

    except KeyboardInterrupt:
        # Beenden über STRG+C
        print("\nScript terminated")
        if device==DEV_EKARREN_EMU: emulator.Close()

    # Nach Beendigung: Licht ausschalten, Bewegung stoppen, Kamera schließen
    bot.Close()
    app.Exit()
