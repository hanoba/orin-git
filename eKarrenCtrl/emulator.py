# Simuliert die eKarren-Firmware sowie die Motoren
# Empfängt die Sollwerte via UDP und stellt die lineare
# Geschwindigkeit und die Drehgeschwindigkeit am Rosmaster
# ein

import math
import socket
import time
from datetime import datetime
from eKarrenLib import eKarren, DEV_ROSMASTER, DEV_EKARREN, DEV_EKARREN_PC, DEV_EKARREN_EMU

UDP_PORT = 4215  #4210
bufsize = 256

# eKarren-Konstanten
rcMaxValue = 2048
tauMax = 255
tauAngularMax = 3*tauMax // 10
tauLinearMax = tauMax
radAbstand = 0.68       # meter
tauStart  =  (17 * 256) // 100;   # 17%
tauStartLeft = tauStart
tauStartRight = tauStart   # 17%

disableTotZonenKompensation = True

class Emulator:
    def __init__(self):
        host = ""
        addr = (host, UDP_PORT)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(addr)
        self.sock.setblocking(0)
        print("Start")
        self.rosmaster = eKarren(device=DEV_ROSMASTER, debug=False)
        self.vLinear = 0
        self.omega = 0

    #=================================================================================================
    # Berechnung der Geschwindigkeit aus dem Tastverhältnis tau (tau=tauMax entspricht 100%)
    #=================================================================================================
    def v(self, tau):
        av =  0.0176
        bv = -0.1348
        if disableTotZonenKompensation: 
            vMax = av*100 + bv
            return tau/tauMax*vMax
        if tau > tauStart or tau < -tauStart: 
            return tau/tauMax*100*av + bv
        return 0


    #=================================================================================================
    # FUNCTION:   TotZonenKompensation
    # ARGUMENTS:
    #    - tau:        motor duty cycle (-255 <= tau <= 255)
    #    - tauStart:   Totzone bis tauStart (0 < tauStart < 255)
    # RETURN VALUE:    kompensierter duty cycle
    #================================================================================================= 
    def TotZonenKompensation(self, tau, tauStart):
        if disableTotZonenKompensation: return tau
        absTau = abs(tau)
        tauKomp = tauStart + ((tauMax - tauStart) * absTau + (tauMax // 2)) // tauMax
        if tau < 0: return -tauKomp
        return tauKomp


    #=================================================================================================
    # FUNCTION:   MotorSimulation
    # ARGUMENTS:
    #    - tauLeft:   duty cycle left motor
    #    - tauRight:  duty cycle right motor
    #================================================================================================= 
    def MotorSimulation(self, tauLeft, tauRight):
        if (tauLeft!=0) or (tauRight!=0):
            tauLeft =  self.TotZonenKompensation(tauLeft, tauStartLeft)
            tauRight = self.TotZonenKompensation(tauRight, tauStartRight)

        vLeft = self.v(tauLeft)
        vRight = self.v(tauRight)
        
        vLinear = (vLeft + vRight)/2
        vAngular = (vLeft - vRight)/2
        omega = vAngular * 2 /  radAbstand      # rad/sec
        f = omega / math.tau                    # U/s = Hz

        return vLinear, f


    #=================================================================================================
    # FUNCTION:   eKarrenSimulation
    # ARGUMENTS:
    #    - vLinearQ:    normalized linear speed from RC (-2048 <= vLinearQ < 2048)
    #    - vAngularQ:   normalized angular speed from RC (-2048 <= vAngularQ < 2048)
    #    - rcKeyStatus: used as mode control (bit 0: slow/fast, bit 2: disableTotZonenKompensation
    #================================================================================================= 
    def eKarrenSimulation(self, vLinearQ, vAngularQ, rcKeyStatus):
        if rcKeyStatus & 4:    # motors disabled?
            tauLinearTarget = 0
            tauAngularTarget = 0
        else:
            tauLinearTarget = vLinearQ*tauLinearMax / rcMaxValue
            tauAngularTarget = vAngularQ*tauAngularMax / rcMaxValue
            if (rcKeyStatus & 1) == 0:
               tauLinearTarget = tauLinearTarget // 2
               tauAngularTarget = tauAngularTarget // 2          #(tauAngularTarget*20 + 15) / 30

        tauLinear = tauLinearTarget
        tauAngular = tauAngularTarget
          
        tauLeft = tauLinear + tauAngular
        tauRight = tauLinear - tauAngular
          
        # perform limitation (reduce tauLinear and keep tauAngular unchanged)
        if tauLeft > tauMax:
            tauLeft = tauMax
            tauRight = tauMax - 2*tauAngular
        elif tauLeft < -tauMax:
            tauLeft = -tauMax
            tauRight = -tauMax - 2*tauAngular
        elif tauRight > tauMax:
            tauRight = tauMax
            tauLeft = tauMax + 2*tauAngular
        elif tauRight < -tauMax:
            tauRight = -tauMax
            tauRight = -tauMax + 2*tauAngular
       
        return self.MotorSimulation(tauLeft, tauRight)


    # Receive message if available
    def ReceiveRaw(self):
       msg = ""
       try: 
          (data, clientAddr) = self.sock.recvfrom(bufsize)
       except socket.error:
          pass
       else:   
          msg = data.decode('utf-8', errors="ignore")
       return msg

    def Run(self):
        udpMsg = self.ReceiveRaw()
        if udpMsg != "": 
            fields = udpMsg.split(",")
            vLinearQ = int(fields[1])
            vAngularQ = int(fields[2])
            rcKeyStatus =  int(fields[3])
            self.vLinear, f = self.eKarrenSimulation(vLinearQ, vAngularQ, rcKeyStatus)
            self.omega = math.tau*f
            self.rosmaster.SetSpeed(self.vLinear, self.omega)
        return self.vLinear, self.omega, udpMsg
        
    def Close(self):
        self.rosmaster.SetSpeed(0, 0,)

if __name__ == "__main__":
    emulator = Emulator()
    while True:
        emulator.Run()
        #time.sleep(0.005)
