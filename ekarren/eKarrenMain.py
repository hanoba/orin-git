import sys
import time
import multiprocessing
import psutil
import os

from compass import Compass
#from eKarrenLidar import Lidar
from Navigator import Navigator
from UdpReceive import UdpReceive
from eKarren import eKarren, DEV_EKARREN, DEV_EKARREN_PC, DEV_EKARREN_EMU
import TaskLists

import numpy as np # Vergiss nicht numpy zu importieren!
import ydlidar
from params import LidarMaxAngle, Udp

# Constants for eKarren
rcMaxValue = 2048
tauMax = 255
radAbstand = 0.68             # meter
vLinearMax = 1.63             # m/s
vAngularMax = 0.3*vLinearMax  # m/s
omegaMax_RadPerSec = 1.44     # rad/s vAngularMax_Hz*2*pi  



class LidarProcess(multiprocessing.Process):
    def __init__(self, pipe_conn):
        super().__init__(daemon=True)
        self.pipe_conn = pipe_conn
        self.stop_event = multiprocessing.Event()

    def run(self):
        # WICHTIG: Treiber-Initialisierung MUSS im run() passieren, 
        # damit sie im neuen Prozess stattfindet!
        PORT = "/dev/ttyUSB0"
        BAUD = 512000

        self.laser = ydlidar.CYdLidar()
        self.laser.setlidaropt(ydlidar.LidarPropSerialPort, PORT)
        self.laser.setlidaropt(ydlidar.LidarPropSerialBaudrate, BAUD)
        self.laser.setlidaropt(ydlidar.LidarPropFixedResolution, False)
        self.laser.setlidaropt(ydlidar.LidarPropReversion, False)
        self.laser.setlidaropt(ydlidar.LidarPropSingleChannel, False)
        self.laser.setlidaropt(ydlidar.LidarPropLidarType, ydlidar.TYPE_TOF)
        self.laser.setlidaropt(ydlidar.LidarPropScanFrequency, 10.0)

        ret = self.laser.initialize()
        if ret:
            ret = self.laser.turnOn()
            if ret:
                print(f"YDLidar TG30 erfolgreich auf {PORT} gestartet!")
            else:
                print("ERROR Lidar Motor konnte nicht gestartet werden.")
        else:
            print(f"ERROR Lidar auf {PORT} nicht gefunden. Rechte geprüft (dialout)?")

        self.scan_data = ydlidar.LaserScan()

        counter = 0
        # Schleife bricht ab, wenn os_isOk() fehlschlägt oder das Hauptprogramm stop() ruft
        while ydlidar.os_isOk() and not self.stop_event.is_set():
            # Blockiert jetzt nur DIESEN isolierten Prozess
            if self.laser.doProcessSimple(self.scan_data):
                self.ProcessLidarData()
                if counter == 0:
                    print(f"{self.scan_data.config.min_range=}")
                    print(f"{self.scan_data.config.max_range=}")
                counter += 1

        # Aufräumen beim Beenden
        self.laser.turnOff()
        self.laser.disconnecting()
        
    def ProcessLidarData(self):
        # 1. Listen-Comprehension für Rohdaten (die einzige Python-Schleife, da p ein C++ Struct ist)
        # Wir gehen davon aus, dass p.angle in Radiant vorliegt
        # Wir setzen ein Minus vor p.angle, um die Drehrichtung umzukehren
        # und addieren pi/2 um den Einbauwinkel zu korrigieren.
        angles_rad = np.array([(np.pi/2-p.angle) % (2*np.pi) for p in self.scan_data.points])
        ranges_raw = np.array([p.range for p in self.scan_data.points])

        # 2. Winkel in Grad umrechnen und dem passenden 1°-Bucket (0-359) zuordnen
        # np.floor rundet ab (z.B. 14.8° -> 14°). Modulo 360 fängt negative Winkel/Überläufe ab.
        buckets = np.floor(np.degrees(angles_rad)).astype(int) % 360

        # 3. Ziel-Array für 360 Grad anlegen.
        # Mit np.inf (Unendlich) füllen, da wir später das Minimum suchen.
        min_ranges = np.full(360, np.inf)

        # 4. Filter: 0.0 bedeutet beim Lidar meist "Kein Echo / Fehler".
        # Diese dürfen nicht das Minimum werden!
        valid_mask = ranges_raw > 0.0
        valid_buckets = buckets[valid_mask]
        valid_ranges = ranges_raw[valid_mask]

        # 5. DIE NUMPY-MAGIC:
        # Geht alle valid_buckets durch und trägt in 'min_ranges' den kleinsten
        # Wert aus 'valid_ranges' ein, der auf diesen Index (0-359) fällt.
        #
        # Stell dir vor, du hast mehrere Pakete (Lidar-Punkte), die in denselben Eimer (bucket) geworfen werden sollen.
        # np.minimum.at sorgt dafür, dass am Ende nur das kleinste Paket im Eimer liegen bleibt.
        # min_ranges:    Das Array, das die Ergebnisse speichern soll (deine "Eimer" mit np.inf vorinitialisiert).
        # valid_buckets: Ein Array von Indizes. Es sagt: "Der Wert an Stelle i aus valid_ranges gehört in den Eimer Nummer X.
        # valid_ranges:  Die tatsächlichen Messwerte (Entfernungen), die einsortiert werden sollen.
        np.minimum.at(min_ranges, valid_buckets, valid_ranges)

        # 6. Unveränderte Werte (wo kein einziger Lidar-Punkt reinfiel) auf 0.0 setzen
        min_ranges[np.isinf(min_ranges)] = 0.0

        # Winkel auf -180° bis +179° normieren und min_ranges entsprechend umsortieren
        # Erzeugt Werte von 0 bis 180 und von -179 bis -1
        angles = np.concatenate([np.arange(0, 181), np.arange(-179, 0)])
        sort_indices = np.argsort(angles)
        sorted_ranges = min_ranges[sort_indices]

        # --- Begrenzung auf -(LidarMaxAngle-1) ... LidarMaxAngle ---
        # Wir schneiden die entsprechenden Indizes aus
        start_idx = 179 - (LidarMaxAngle - 1)
        end_idx = 180 + LidarMaxAngle
        limited_ranges = sorted_ranges[start_idx:end_idx]
        
        # wir senden die Daten über die Pipe
        self.pipe_conn.send(limited_ranges)
        
    def stop(self):
        self.stop_event.set()



# ======================================================
# Isolierter Hardware-Prozess mit unidirektionaler Pipe
# ======================================================
class CompassProcess(multiprocessing.Process):
    def __init__(self, hz, core_id, pipe_conn):
        super().__init__(daemon=True) 
        self.interval = 1.0 / hz
        self.core_id = core_id
        self.pipe_conn = pipe_conn
        self.stop_event = multiprocessing.Event()

    def run(self):
        # Den Prozess auf einen spezifischen CPU-Kern pinnen
        try:
            aktueller_prozess = psutil.Process(os.getpid())
            aktueller_prozess.cpu_affinity([self.core_id])
            print(f"[{os.getpid()}] CompassProcess läuft fest auf CPU-Kern: {aktueller_prozess.cpu_affinity()}")
        except AttributeError:
            print("Warnung: CPU Affinity wird auf diesem System nicht unterstützt.")

        # --- NEU: Wir packen den Rest in einen try-Block ---
        try:
            # Hardware erst HIER innerhalb des neuen Prozesses initialisieren!
            compass = Compass()
            #compass.GyroBiasCalibration()      # skip to save time

            # 3. Die Takt-Schleife
            naechster_aufruf = time.perf_counter() + self.interval
            
            while not self.stop_event.is_set():
                # Kompass auslesen und Theta über die Pipe an das Hauptprogramm senden
                theta = compass.ReadYaw()
                self.pipe_conn.send(theta)
                
                # Drift-Kompensation für stabile 30 Hz
                jetzt = time.perf_counter()
                schlafenszeit = naechster_aufruf - jetzt
                
                if schlafenszeit > 0:
                    self.stop_event.wait(schlafenszeit) 
                    
                naechster_aufruf += self.interval

        # --- NEU: Den Ctrl-C Absturz lautlos abfangen ---
        except KeyboardInterrupt:
            # Wird ignoriert. Das Hauptprogramm kümmert sich um den sauberen Abbruch.
            pass
            
    def stop(self):
        self.stop_event.set()


# ==========================================
# HAUPTPROGRAMM
# ==========================================
def main():
    udp_rx = UdpReceive(Udp.PORT_TELEOP)

    TaskListDict = {
        "Localization":           TaskLists.Localization_TaskList,
        "FastLocalization":       TaskLists.FastLocalization_TaskList,
        #"Mowing":                 TaskLists.Mowing_TaskList,
        "U_MowTask":              TaskLists.U_MowTask_TaskList,
        "V_MowTask":              TaskLists.V_MowTask_TaskList,
        "Fahre_zum_Schuppen":     TaskLists.Fahre_zum_Schuppen_TaskList,
        "Fahre_in_den_Wald":      TaskLists.Fahre_in_den_Wald_TaskList,
        "Fahre_in_den_Garten":    TaskLists.Fahre_in_den_Garten_TaskList,
        "Fahre_hinters_Haus":     TaskLists.Fahre_hinters_Haus_TaskList,
        "Bestimme_YawOffset":     TaskLists.Bestimme_YawOffset_TaskList,
        "Test":                   TaskLists.Test_TaskList
    }

    def Usage():
        print("Usage: ekarren [<deviceName> [<taskName>]]")
        print("<deviceName>:")
        print("    eKarren (default)")
        print("    eKarrenPC")
        print("    eKarrenEmulator")
        print("<taskName>:")
        print("    None (default)")
        index = 0
        for taskName in TaskListDict:
            print(f"    {index} | {taskName}")
            index += 1
        sys.exit(0)

    def GetLatestFromPipe(receiver):
        """
        Liest alle aktuell in der Pipe verfügbaren Daten und gibt das neueste Element zurück.
        Alle älteren Elemente werden verworfen. Gibt None zurück, wenn die Pipe leer ist.
        """
        latest_data = None
        
        # Prüfen, ob überhaupt mindestens ein Element in der Pipe ist
        while True:
            if receiver.poll(0.005):
                latest_data = receiver.recv()
                
                # Schleife läuft weiter, solange noch neue Elemente in der Pipe warten
                while receiver.poll():
                    # Überschreibt den alten Wert mit dem jeweils neueren
                    latest_data = receiver.recv()
                return latest_data
            time.sleep(0.001)


    # command line parameter handling
    taskList = None
    argc = len(sys.argv)
    if argc==1:
        deviceName = "eKarren"
        #deviceName = "eKarrenPC"
        #deviceName = "eKarrenEmulator"    
    elif argc==2:
        deviceName = sys.argv[1]
    elif argc==3:
        deviceName = sys.argv[1]
        taskListName = sys.argv[2]
        if taskListName != "None":
            if taskListName.isdigit():
                taskList = list(TaskListDict.values())[int(taskListName)]
            else:
                taskList = TaskListDict.get(taskListName)
        if taskList is None: Usage()
    else: Usage()

    if deviceName=="eKarren": deviceNum = DEV_EKARREN
    elif deviceName=="eKarrenPC": deviceNum = DEV_EKARREN_PC
    elif deviceName=="eKarrenEmulator": deviceNum = DEV_EKARREN_EMU
    else: Usage()
    print(f"Device: {deviceName}")
    
    # Hardware, die das Gehirn (Hauptprogramm) benötigt, bleibt hier!
    navigator = Navigator()
    #lidar = Lidar(navigator)
    ekarren = eKarren(device=deviceNum, debug=False)


    if taskList is not None:
        navigator.NewTaskList(taskList)
    
    # ----------------------------------------------------
    # PROZESS STARTEN (Mit Zwei-Wege-Verbindung)
    # ----------------------------------------------------
    freq_Hz = 30
    core_id = 1  # Wähle hier den Kern aus (Zählung beginnt bei 0)
    
    # Eine Ein-Weg-Pipe erstellen (unidirektional)
    # compass_rx (erster Wert) kann NUR lesen und bleibt beim Hauptprogramm.
    # compass_tx (zweiter Wert) kann NUR schreiben und geht an den Prozess.
    compass_rx, compass_tx = multiprocessing.Pipe(duplex=False)
    
    # Prozess starten
    compassProcess = CompassProcess(freq_Hz, core_id, compass_tx)    
    compassProcess.start()
    
    # 2. Pipe & Prozess für Lidar (ca. 10 Hz)
    lidar_rx, lidar_tx = multiprocessing.Pipe(duplex=False)
    lidarProcess = LidarProcess(lidar_tx)
    lidarProcess.start()

    print("System läuft. Warte auf Daten vom Hardware-Prozess...")

    try:
        # Die Hauptschleife wartet jetzt hocheffizient (mit 0% CPU Last) auf den Pipe-Eingang
        while True:
            # Empfange Theta vom separaten CPU-Kern
            theta = GetLatestFromPipe(compass_rx)
            
            # Kompass-Daten an den Navigator übergeben
            vLinear, omega = navigator.CompassCallback(theta)
            
            # --- LIDAR UPDATE (NON-BLOCKING) ---
            # Wir prüfen kurz, ob der Lidar-Prozess neue Daten gesendet hat.
            # (Das wird nur in ca. jedem dritten Durchlauf der Fall sein)
            if lidar_rx.poll():
                limited_ranges = lidar_rx.recv()
                
                # Pipe leeren, falls sich Lidar-Daten gestaut haben
                while lidar_rx.poll():
                    limited_ranges = lidar_rx.recv()
                    
                # Lidar-Daten an den Navigator übergeben
                navigator.ScanCallback(limited_ranges)

                # UDP an viz.py senden (aus dem Lidar-Code hierher verschoben)
                cm = 100.0
                ranges_cm = limited_ranges * cm
                ranges_cm = ranges_cm.astype(np.int16)
                navigator.udp.Send(Udp.LIDAR_DATA, ranges_cm.tolist())

            # --- MOTOREN UPDATEN ---
            vLinear, omega = udp_rx.ReceiveTeleop(vLinear, omega)   
            ekarren.SetSpeed(vLinear, omega)
            
            
            
    except KeyboardInterrupt:
        print("\neKarren wurde durch Benutzer abgebrochen.")
    finally:
        print("Bereinige Ressourcen...")
        # Hardware-Prozess sauber stoppen
        compassProcess.stop()
        compassProcess.join()  
        
        ekarren.Close()
        
        navigator.trace.Save()
        sys.exit(0)

if __name__ == '__main__':
    # WICHTIG: Windows und macOS benötigen zwingend diese Zeile für multiprocessing
    multiprocessing.freeze_support() 
    main()