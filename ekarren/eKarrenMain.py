import sys
import time
import multiprocessing
import psutil
import os

from compass import Compass
from eKarrenLidar import Lidar
from Navigator import Navigator
from UdpReceive import UdpReceive
from params import Udp
from eKarren import eKarren, DEV_EKARREN, DEV_EKARREN_PC, DEV_EKARREN_EMU
import TaskLists

# Constants for eKarren
rcMaxValue = 2048
tauMax = 255
radAbstand = 0.68             # meter
vLinearMax = 1.63             # m/s
vAngularMax = 0.3*vLinearMax  # m/s
omegaMax_RadPerSec = 1.44     # rad/s vAngularMax_Hz*2*pi  


# ==========================================
# Isolierter Hardware-Prozess mit Zwei-Wege-Pipe
# ==========================================
class HardwareProcess(multiprocessing.Process):
    def __init__(self, hz, core_id, device_num, pipe_conn):
        super().__init__(daemon=True) 
        self.interval = 1.0 / hz
        self.core_id = core_id
        self.device_num = device_num
        self.pipe_conn = pipe_conn
        self.stop_event = multiprocessing.Event()

    def run(self):
        # 1. Den Prozess auf einen spezifischen CPU-Kern pinnen
        try:
            aktueller_prozess = psutil.Process(os.getpid())
            aktueller_prozess.cpu_affinity([self.core_id])
            print(f"[{os.getpid()}] HardwareProcess läuft fest auf CPU-Kern: {aktueller_prozess.cpu_affinity()}")
        except AttributeError:
            print("Warnung: CPU Affinity wird auf diesem System nicht unterstützt.")

        # --- NEU: Wir packen den Rest in einen try-Block ---
        try:
            # 2. WICHTIG: Hardware erst HIER innerhalb des neuen Prozesses initialisieren!
            ekarren = eKarren(device=self.device_num, debug=False)
            compass = Compass()
            #compass.GyroBiasCalibration()      # skip to save time

            # 3. Die Takt-Schleife
            naechster_aufruf = time.perf_counter() + self.interval
            
            while not self.stop_event.is_set():
                # A) Kompass auslesen und Theta über die Pipe an das Hauptprogramm senden
                theta = compass.ReadYaw()
                self.pipe_conn.send(theta)
                
                # B) Prüfen, ob das Hauptprogramm berechnete Motorenbefehle zurückgeschickt hat.
                if self.pipe_conn.poll(0.005): 
                    vLinear, omega = self.pipe_conn.recv()
            
                    # Schleife läuft weiter, solange noch neue Elemente in der Pipe warten
                    while self.pipe_conn.poll():
                        # Überschreibt den alten Wert mit dem jeweils neueren
                        vLinear, omega = self.pipe_conn.recv()
                    ekarren.SetSpeed(vLinear, omega)
                
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
            
        finally:
            # --- NEU: Socket des eKarren sauber schließen ---
            try:
                ekarren.Close()
            except Exception:
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
    lidar = Lidar(navigator)

    if taskList is not None:
        navigator.NewTaskList(taskList)
    
    # ----------------------------------------------------
    # PROZESS STARTEN (Mit Zwei-Wege-Verbindung)
    # ----------------------------------------------------
    freq_Hz = 30
    core_id = 1  # Wähle hier den Kern aus (Zählung beginnt bei 0)
    
    # Eine Zwei-Wege-Pipe erstellen
    # main_pipe bleibt beim Hauptprogramm, hw_pipe geht an den Prozess
    main_pipe, hw_pipe = multiprocessing.Pipe(duplex=True)
    
    # Prozess starten
    print("System läuft. Warte auf Daten vom Hardware-Prozess...")
    hwProcess = HardwareProcess(freq_Hz, core_id, deviceNum, hw_pipe)    
    hwProcess.start()

    try:
        # Die Hauptschleife wartet jetzt hocheffizient (mit 0% CPU Last) auf den Pipe-Eingang
        while True:
            # 1. Empfange Theta vom separaten CPU-Kern
            #theta = 0
            #theta = main_pipe.recv()
            theta = GetLatestFromPipe(main_pipe)
            
            # 2. Berechne die neue Route (hier fließen Lidar + Kompass zusammen)
            vLinear, omega = navigator.CompassCallback(theta)
            vLinear, omega = udp_rx.ReceiveTeleop(vLinear, omega)   # check for teleop command
            #time.sleep(0.033)
            
            # 3. Sende die neuen Geschwindigkeitsbefehle zurück an den CPU-Kern
            main_pipe.send((vLinear, omega))
            
    except KeyboardInterrupt:
        print("\neKarren wurde durch Benutzer abgebrochen.")
    finally:
        print("Bereinige Ressourcen...")
        # Hardware-Prozess sauber stoppen
        hwProcess.stop()
        hwProcess.join()  
        
        # Lidar stoppen
        lidar.StopLidar()
        navigator.trace.Print()
        sys.exit(0)

if __name__ == '__main__':
    # WICHTIG: Windows und macOS benötigen zwingend diese Zeile für multiprocessing
    multiprocessing.freeze_support() 
    main()