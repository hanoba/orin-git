import socket
import sys
import time
from Rosmaster_Lib import Rosmaster     # for eKarren emulation with RosMaster X3 PLus

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

# Die Klasse eKarren stellt im wesentlichen ein Interface zum Setzen der Geschwindigkeit bereit.
# Weiterhin erlaubt die KLasse eine Emulationdes eKarrens mit dem RosMaster X3 Plus. 
class eKarren:
    def __init__(self, device=DEV_EKARREN, debug=False):
        self.device = device
        if self.device==DEV_ROSMASTER:
            # Roboter über USB-Port initialisieren
            self.bot = Rosmaster(com="/dev/ttyCH341USB0", debug=debug)
            self.bot.create_receive_threading()  # Empfangsthread für Statuswerte starten

            # Kurze Pause für Initialisierung
            time.sleep(.1)
            # Startsignal: drei kurze Pieptöne
            for i in range(3):
                self.bot.set_beep(60)
                time.sleep(.2)
            return

        if self.device==DEV_EKARREN_EMU:
            self.clientAddr = ("127.0.0.1", 4215)            # 
        elif self.device==DEV_EKARREN_PC:
            self.clientAddr = ("192.168.178.42", 4215)       # AZ-KENKO
        elif self.device==DEV_EKARREN_PC:
            self.clientAddr = ("192.168.20.100", 4211)       # E-Karren
        else:
            print("Wrong device: {self.device}")
            sys.exit(1)

        self.bot = None            
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        self.rcKeyStatus = 1
    
        # v = av*tau + bv
        av = 0.0176  
        bv = - 0.1348
        # tau = (v - bv)/av = at*v + bt
        self.bt = bv/av
        self.at = 1/av
        tauStart =  (17 * 256) // 100;   # 17%


    def GetVersion(self):
        if self.device==DEV_ROSMASTER: return self.bot.get_version()
        return "V9"
        
    def GetBatteryVoltage(self):
        if self.device==DEV_ROSMASTER: return self.bot.get_battery_voltage()
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
            self.bot.set_car_motion(v_x=-vLinear, v_y=0, v_z=-omega)
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
        

def main():
    if len(sys.argv) != 3:
        print("Usage: eKarrenLib <vLinear_m/s> <omega_rad/s>")
        sys.exit()
        
    vLinear = float(sys.argv[1])
    omega = float(sys.argv[2])
    print(f"{vLinear=}m/s   {omega=}rad/s")

    bot = eKarren(debug=True)
    #raw:AT+#,-1,-1,0,0x53
    try:
        while True:
            bot.SetSpeed(vLinear, omega)
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("Stopped by user")
    bot.Close()

if __name__ == "__main__":
    main()