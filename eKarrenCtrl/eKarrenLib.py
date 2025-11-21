import socket
import sys
import time
from Rosmaster_Lib import Rosmaster     # for eKarren emulation with RosMaster X3 PLus

rcMaxValue = 2048
tauMax = 255

# Die Klasse eKarren stellt im wesentlichen ein Interface zum Setzen der Geschwindigkeit bereit.
# Weiterhin erlaubt die KLasse eine Emulationdes eKarrens mit dem RosMaster X3 Plus. 
class eKarren:
    def __init__(self, useRosMaster=False, debug=False):
        self.useRosMaster = useRosMaster
        if useRosMaster:
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

        if debug:
            self.clientAddr = ("192.168.178.42", 4215)       # AZ-KENKO
        else:
            self.clientAddr = ("192.168.20.100", 4211)       # E-Karren

        self.bot = None            
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        self.mode = 1
    
        # v = av*tau + bv
        av = 0.0176  
        bv = - 0.1348
        # tau = (v - bv)/av = at*v + bt
        self.bt = bv/av
        self.at = 1/av

    def GetVersion(self):
        if self.useRosMaster: return self.bot.get_version())
        return "V9"
        
    def GetBatteryVoltage(self):
        if self.useRosMaster: return self.bot.get_battery_voltage())
        return 24.0
        
    def CheckSum(self, send_data):
        checkSum = 0
        for i in range(len(send_data)):
            checkSum += ord(send_data[i])
        checkSum = checkSum & 255
        return checkSum
    
    def SetSpeed(self, vLinear, vAngular):
        if self.useRosMaster:
            # Steuerbefehl an RosMaster senden (Vorwärts-/Rückwärts- und Drehbewegung)
            # v_x=[-0.7, 0.7] m/s, v_y=[-0.7, 0.7] m/s, v_z=[-3.2, 3.2] rad/sec
            self.bot.set_car_motion(v_x=-vLinear, v_y=0, v_z=-vAngular)
            return
            
        tauLinear = self.at*vLinear + self.bt
        tauAngular = vAngular       #HB to be corrected
        x = int(round(tauLinear * rcMaxValue / 100, 0))
        y = int(round(tauAngular * rcMaxValue / 100, 0))
        send_data = f"AT+#,{x},{y},{self.mode}"
        send_data += f",0x{self.CheckSum(send_data):02X}"
        self.sock.sendto(send_data.encode('utf-8'), self.clientAddr)
 
    def Close(self):
        #sock.sendto(MESSAGE, (UDP_IP, UDP_PORT))
        self.SetSpeed(0, 0)
        if not self.useRosMaster: self.sock.close()
        

def main():
    if len(sys.argv) != 3:
        print("Usage: eKarrenLib <vLinear_m/s> <vAngular_rad/s>")
        sys.exit()
        
    vLinear = float(sys.argv[1])
    vAngular = float(sys.argv[2])
    print(f"{vLinear=}m/s   {vAngular=}rad/s")

    bot = eKarren(debug=True)
    #raw:AT+#,-1,-1,0,0x53
    try:
        while True:
            bot.SetSpeed(vLinear, vAngular)
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("Stopped by user")
    bot.Close()

if __name__ == "__main__":
    main()