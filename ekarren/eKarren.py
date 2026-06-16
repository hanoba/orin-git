import socket
import sys

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
