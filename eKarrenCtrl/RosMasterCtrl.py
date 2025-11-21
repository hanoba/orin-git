#!/usr/bin/env python3
# coding: utf-8

# ==============================================================
# Projekt: Handgesteuerte Robotersteuerung mit YOLO-HandPose
# Autor: Harald Bauer 
# Datum: 2025-10-23
# Beschreibung: Steuert einen Rosmaster-Roboter in Echtzeit basierend auf Handposition und -größe,
#                erkannt durch YOLO11 HandPose-Inferenz auf Jetson/TensorRT.
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
from time import perf_counter_ns
from Rosmaster_Lib import Rosmaster
from AsyncHandPoseYolo11 import HandPoseApp
from pid import PID

if __name__ == '__main__':
    # Roboter über USB-Port initialisieren
    bot = Rosmaster(com="/dev/ttyCH341USB0", debug=False)
    bot.create_receive_threading()  # Empfangsthread für Statuswerte starten

    # Kurze Pause für Initialisierung
    time.sleep(.1)
    # Startsignal: drei kurze Pieptöne
    for i in range(3):
        bot.set_beep(60)
        time.sleep(.2)

    # Systeminformationen anzeigen
    # Abfrage der Firmware-Version und Batteriespannung des Roboters zur Statusanzeige
    print("Version:", bot.get_version())
    print("Vbat:", bot.get_battery_voltage())

    # Optionale Lichtsteuerung (auskommentiert)
    # Effect =[0, 6], 0: stop light effect, 1: running light, 2: running horse light,
    #                 3: breathing light, 4: gradient light, 5: starlight, 6: power display 
    #                 Speed =[1, 10], the smaller the value, the faster the speed changes
    # bot.set_colorful_effect(effect=2, speed=5)

    # Initialisierung von Steuerungsvariablen
    printCnt = 0
    angular = 0
    linear = 0
    wantedHandSize = 200  # Ziel-Handgröße: bestimmt den gewünschten Abstand zur Hand

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
    CAM_ID = 0  #2
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
            bot.set_car_motion(v_x=-linear, v_y=0, v_z=-angular)

            # Statusausgabe alle Sekunde
            now = perf_counter_ns()
            if now - lastTime > 1e9:
                lastTime = now
                Vbat = bot.get_battery_voltage()
                print(f"{fps:.1f} FPS lin={linear:.3f}, ang={angular:.3f}, {handSize=}, {handPos=}, {Vbat=}V")

    except KeyboardInterrupt:
        # Beenden über STRG+C
        print("\nScript terminated")

    # Nach Beendigung: Licht ausschalten, Bewegung stoppen, Kamera schließen
    bot.set_colorful_effect(effect=0)
    bot.set_car_motion(v_x=0, v_y=0, v_z=0)
    handPoseApp.Exit()
