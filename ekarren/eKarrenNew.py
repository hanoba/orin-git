import sys
import socket
import time
import multiprocessing  # <--- NEU
import psutil           # <--- NEU
import os               # <--- NEU

from compass import Compass
from eKarrenLidar import Lidar
from Navigator import Navigator
import TaskLists

# Constants for eKarren
rcMaxValue = 2048
tauMax = 255
radAbstand = 0.68             # meter
vLinearMax = 1.63             # m/s
vAngularMax = 0.3*vLinearMax  # m/s
omegaMax_RadPerSec = 1.44  # rad/s vAngularMax_Hz*2*pi  

DEV_ROSMASTER = 0       # run natively on Rosmaster
DEV_EKARREN = 1         # send UDP commands to eKarren
DEV_EKARREN_PC = 2      # send UDP commands to PC (AZ-KENKO)
DEV_EKARREN_EMU = 3     # send UDP commands to Rosmaster

#deviceName = "eKarren"
deviceName = "eKarrenPC"
#deviceName = "eKarrenEmulator"

if deviceName=="eKarren": deviceNum = DEV_EKARREN
elif deviceName=="eKarrenPC": deviceNum = DEV_EKARREN_PC
elif deviceName=="eKarrenEmulator": deviceNum = DEV_EKARREN_EMU
else: 
    print(f"ERROR Illegal deviceName: {deviceName} Valid: eKarren, eKarrenPC, eKarrenEmulator")
    sys.exit()
print(f"Device: {deviceName}")


# ==========================================
# NEU: Kompass-Prozess
# ==========================================
class CompassProcess(multiprocessing.Process):
    def __init__(self, hz, core_id, device_num, compassCallback):
        super().__init__(daemon=True) 
        self.interval = 1.0 / hz
        self.core_id = core_id
        self.stop_event = multiprocessing.Event() # Event muss aus multiprocessing kommen!
        self.ekarren = eKarren(device=device_num, debug=False)
        self.compass = Compass(gyroBiasCalibration=False)
        self.CompassCallback = compassCallback

    def run(self):
        # 1. Den Prozess auf einen spezifischen CPU-Kern pinnen
        try:
            aktueller_prozess = psutil.Process(os.getpid())
            aktueller_prozess.cpu_affinity([self.core_id])
            print(f"[{os.getpid()}] TimerProcess läuft fest auf CPU-Kern: {aktueller_prozess.cpu_affinity()}")
        except AttributeError:
            print("Warnung: CPU Affinity wird auf diesem System nicht unterstützt.")

        # 2. Die unsichtbare Hintergrundschleife
        naechster_aufruf = time.perf_counter() + self.interval
        
        while not self.stop_event.is_set():
            theta = self.compass.ReadYaw()
            vLinear, omega = self.CompassCallback(theta)
            self.ekarren.SetSpeed(vLinear, omega)
            
            # Drift-Kompensation
            jetzt = time.perf_counter()
            schlafenszeit = naechster_aufruf - jetzt
            
            if schlafenszeit > 0:
                self.stop_event.wait(schlafenszeit) 
                
            naechster_aufruf += self.interval

    def stop(self):
        self.stop_event.set()


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




def main():
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
        print("Usage: ekarren <taskName>")
        print("<taskName>:")
        for taskName in TaskListDict:
            print(f"    {taskName}")
        sys.exit(0)

    # command line parameter handling
    argc = len(sys.argv)
    taskList = None
    if argc == 2:
        taskList = TaskListDict.get(sys.argv[1])
        if taskList is None: 
            Usage()
    elif argc > 2: Usage()

    # Hardware, die im HAUPTPROGRAMM gebraucht wird, kann hier bleiben
    navigator = Navigator()
    lidar = Lidar(navigator.ScanCallback)
    
    # ----------------------------------------------------
    # PROZESS STARTEN
    # ----------------------------------------------------
    freq_Hz = 30
    core_id = 1  # Wähle hier den Kern aus (Zählung beginnt bei 0)
    
    # Wir übergeben die Ziel-Funktion und die geräte-Nummer als Argument
    compassProcess = CompassProcess(freq_Hz, core_id, deviceNum, navigator.CompassCallback)    
    compassProcess.start()

    try:
        # Das Publizieren von Lidardaten und Theta erfolgt jetzt
        # auf einem komplett separaten CPU-Kern!
        while True:
            time.sleep(1.0)
    except KeyboardInterrupt:
        print("\neKarren wurde durch Benutzer abgebrochen.")
    finally:
        print("Bereinige Ressourcen...")
        compassProcess.stop()
        compassProcess.join()  # Warten, bis der Kern sauber freigegeben wurde
        
        # Falls Lidar aktiv war, hier stoppen (nur wenn definiert)
        lidar.StopLidar()
        
        sys.exit(0)

if __name__ == '__main__':
    main()