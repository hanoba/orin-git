import sys
import socket
import time
import multiprocessing
import psutil
import os

from compass import Compass
from eKarrenLidar import Lidar
from Navigator import Navigator
from UdpReceive import UdpReceive
from params import Udp
import TaskLists

# Constants for eKarren
rcMaxValue = 2048
tauMax = 255
radAbstand = 0.68             # meter
vLinearMax = 1.63             # m/s
vAngularMax = 0.3*vLinearMax  # m/s
omegaMax_RadPerSec = 1.44     # rad/s vAngularMax_Hz*2*pi  

DEV_ROSMASTER = 0       # run natively on Rosmaster
DEV_EKARREN = 1         # send UDP commands to eKarren
DEV_EKARREN_PC = 2      # send UDP commands to PC (AZ-KENKO)
DEV_EKARREN_EMU = 3     # send UDP commands to Rosmaster

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
            # timeout=0.005 (5ms) verhindert, dass der Hardware-Takt ins Stocken gerät
            if self.pipe_conn.poll(0.005): 
                vLinear, omega = self.pipe_conn.recv()
                ekarren.SetSpeed(vLinear, omega)
            
            # Drift-Kompensation für stabile 30 Hz
            jetzt = time.perf_counter()
            schlafenszeit = naechster_aufruf - jetzt
            
            if schlafenszeit > 0:
                self.stop_event.wait(schlafenszeit) 
                
            naechster_aufruf += self.interval

    def stop(self):
        self.stop_event.set()


# ==========================================
# Schnittstelle zum eKarren (Hardware / Emulator)
# ==========================================
class eKarren:
    def __init__(self, device=DEV_EKARREN, debug=False):
        self.device = device

        if self.device==DEV_EKARREN_EMU:
            self.clientAddr = ("127.0.0.1", 4215)            # 
        elif self.device==DEV_EKARREN_PC:
            self.clientAddr = ("192.168.178.42", 4215)       # AZ-KENKO
        elif self.device==DEV_EKARREN:
            self.clientAddr = ("192.168.20.100", 4211)       # E-Karren
        else:
            print(f"Wrong device: {self.device}")
            sys.exit(1)

        self.rosmaster = None            
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        self.rcKeyStatus = 1
    
        # v = av*tau + bv
        av = 0.0176  
        bv = - 0.1348
        # tau = (v - bv)/av = at*v + bt
        self.bt = bv/av
        self.at = 1/av

    def GetVersion(self):
        if self.device==DEV_ROSMASTER: return self.rosmaster.get_version()
        return "V9"
        
    def GetBatteryVoltage(self):
        if self.device==DEV_ROSMASTER: return self.rosmaster.get_battery_voltage()
        return 24.0
        
    def CheckSum(self, send_data):
        checkSum = 0
        for i in range(len(send_data)):
            checkSum += ord(send_data[i])
        checkSum = checkSum & 255
        return checkSum
    
    def Quantize(self, x):
        y = int(round(x, 0))
        if y>= rcMaxValue: y = rcMaxValue - 1
        elif y <= -rcMaxValue: y = -rcMaxValue + 1
        return y
    
    def SetSpeed(self, vLinear, omega):
        if self.device==DEV_ROSMASTER:
            self.rosmaster.set_car_motion(v_x=-vLinear, v_y=0, v_z=-omega)
            return

        vLinearQ = self.Quantize(vLinear / vLinearMax * rcMaxValue)
        vAngular = omega*radAbstand/2
        vAngularQ = self.Quantize(vAngular / vAngularMax * rcMaxValue)
        
        send_data = f"AT+#,{vLinearQ},{vAngularQ},{self.rcKeyStatus}"
        send_data += f",0x{self.CheckSum(send_data):02X}"
        self.sock.sendto(send_data.encode('utf-8'), self.clientAddr)
 
    def Close(self):
        self.SetSpeed(0, 0)
        if not self.device==DEV_ROSMASTER: self.sock.close()


# ==========================================
# HAUPTPROGRAMM
# ==========================================
def main():
    udp_rx = UdpReceive(Udp.PORT_TELEOP)

    TaskListDict = {
        "Localization":           TaskLists.Localization_TaskList,
        "FastLocalization":       TaskLists.FastLocalization_TaskList,
        "Mowing":                 TaskLists.Mowing_TaskList,
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
        print(f"    None (default)")
        index = 0
        for taskName in TaskListDict:
            print(f"    {index} | {taskName}")
            index += 1
        sys.exit(0)

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
    freq_Hz = 10
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
            theta = main_pipe.recv()
            
            # 2. Berechne die neue Route (hier fließen Lidar + Kompass zusammen)
            vLinear, omega = navigator.CompassCallback(theta)
            vLinear, omega = udp_rx.ReceiveTeleop(vLinear, omega)   # check for teleop command
            
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