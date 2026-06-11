import sys
from compass import Compass
import socket
from eKarrenLidar import Lidar
import threading
import time
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

deviceName = "eKarren"
if deviceName=="eKarren": deviceNum = DEV_EKARREN
elif deviceName=="eKarrenPC": deviceNum = DEV_EKARREN_PC
elif deviceName=="eKarrenEmulator": deviceNum = DEV_EKARREN_EMU
else: 
    print(f"ERROR Illegal deviceName: {deviceName} Valid: eKarren, eKarrenPC, eKarrenEmulator")
    sys.exit()
print(f"Device: {deviceName}")


class Timer(threading.Thread):
    def __init__(self, hz, callback):
        super().__init__(daemon=True) # Daemon = Beendet sich automatisch mit dem Hauptskript
        self.interval = 1.0 / hz
        self.callback = callback
        self.stop_event = threading.Event()

    def run(self):
        # Das ist die unsichtbare Hintergrundschleife
        naechster_aufruf = time.perf_counter() + self.interval
        
        while not self.stop_event.is_set():
            self.callback() # Deine Funktion aufrufen
            
            # Drift-Kompensation
            jetzt = time.perf_counter()
            schlafenszeit = naechster_aufruf - jetzt
            
            if schlafenszeit > 0:
                # wait() ist besser als sleep(), da es sofort abbricht, wenn stop() aufgerufen wird
                self.stop_event.wait(schlafenszeit) 
                
            naechster_aufruf += self.interval

    def stop(self):
        self.stop_event.set()


# Die Klasse eKarren stellt im wesentlichen ein Interface zum Setzen der Geschwindigkeit bereit.
# Weiterhin erlaubt die KLasse eine Emulation des eKarrens mit dem RosMaster X3 Plus. 
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
            print("Wrong device: {self.device}")
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
    
    # vLinear = linear Geschwindigkeit in m/s
    # omega = winkelgeschwindigkeit in rad/sec
    def SetSpeed(self, vLinear, omega):
        if self.device==DEV_ROSMASTER:
            # Steuerbefehl an RosMaster senden (Vorwärts-/Rückwärts- und Drehbewegung)
            # v_x=[-0.7, 0.7] m/s, v_y=[-0.7, 0.7] m/s, v_z=[-3.2, 3.2] rad/sec
            self.rosmaster.set_car_motion(v_x=-vLinear, v_y=0, v_z=-omega)
            return

        vLinearQ = self.Quantize(vLinear / vLinearMax * rcMaxValue)
        vAngular = omega*radAbstand/2
        vAngularQ = self.Quantize(vAngular / vAngularMax * rcMaxValue)
        
        #print(f"{vLinear=}m/s  {vAngular=}m/s  {vLinearQ=}  {vAngularQ=}")
        send_data = f"AT+#,{vLinearQ},{vAngularQ},{self.rcKeyStatus}"
        send_data += f",0x{self.CheckSum(send_data):02X}"
        self.sock.sendto(send_data.encode('utf-8'), self.clientAddr)
 
    def Close(self):
        #sock.sendto(MESSAGE, (UDP_IP, UDP_PORT))
        self.SetSpeed(0, 0)
        if not self.device==DEV_ROSMASTER: self.sock.close()

vLinear = 0.0
omega = 0.0
compassCalibrationRunning = False

def CmdVelCallback(angular, linear):
    """Wird aufgerufen, wenn ROS2 einen Fahrbefehl sendet."""
    global vLinear, omega
    vLinear = linear
    omega = angular



def TimerCallback():
    # Sendet theta und Fahrbefehle (50Hz)
    if not compassCalibrationRunning:
        PublishTheta()
    ekarren.SetSpeed(vLinear, omega)


def PublishTheta(self):
    theta = compass.ReadYaw()
    #if theta is not None: 
    #    lastTheta = theta
    navigator.CompassCallback(theta)

def CalibCallback(isCompassCalibrationRunning):
    # Wird automatisch aufgerufen, wenn der CompassCalibrationNode publiziert
    global compassCalibrationRunning
    compassCalibrationRunning = isCompassCalibrationRunning
    print(f"Neuer Status empfangen: compassCalibrationRunning={compassCalibrationRunning}")


# Tasklist dictionary
TaskListDict = {
    "Localization":           TaskLists.Localization_TaskList,
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

# Roboter initialisieren
ekarren = eKarren(device=deviceNum, debug=False)
navigator = Navigator(CmdVelCallback)
lidar = Lidar(navigator.ScanCallback)
compass = Compass()
timer = Timer(30, TimerCallback)    # 30Hz-Timer ruft selbstständig TimerCallback() auf

if taskList is not None:
    navigator.NewTaskList(taskList)

try:
    # Das Publizieren von Lidardaten und Theta erfolgt in Hintergrund-Threads
    while True:
        time.sleep(1.0)
except KeyboardInterrupt:
    # Wird ausgelöst, wenn du Ctrl+C drückst
    print("eKarren wurde durch Benutzer abgebrochen.")
#except Exception as e:
#    # Fängt unerwartete Fehler ab
#    print(f"Unerwarteter Fehler: {e}")
finally:
    # Dieser Block wird IMMER ausgeführt, egal ob Fehler oder Ctrl+C
    print("Bereinige Ressourcen...")
    lidar.StopLidar()
    
    # Optional: Komplettes Beenden erzwingen (hilft bei WSL2-Hängern)
    sys.exit(0)

