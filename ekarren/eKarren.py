import socket
import sys
import time
import serial
import struct

# Constants for eKarren
rcMaxValue = 2048
tauMax = 255
radAbstand = 0.68             # meter
vLinearMax = 1.63             # m/s
vAngularMax = 0.3*vLinearMax  # m/s
omegaMax_RadPerSec = 1.44     # rad/s vAngularMax_Hz*2*pi  

#DEV_ROSMASTER = 0       # run natively on Rosmaster NOT SUPPORTED
DEV_EKARREN = 1         # send UDP commands to eKarren
DEV_EKARREN_PC = 2      # send UDP commands to PC (AZ-KENKO)
DEV_EKARREN_EMU = 3     # send UDP commands to Rosmaster
DEV_ARDUMOWER = 4       # send UART commands to Ardumower/Moonlight


# ======================================
# Schnittstelle zum Ardumower/Moonlight
# ======================================
class Ardumower:
    def __init__(self):
        self.cLinear = 2048.0 / 0.5
        self.cAngular = 2048.0 / 1.0
        SERIAL_PORT = '/dev/serial0'
        BAUDRATE = 115200
        print(f"Öffne seriellen Port {SERIAL_PORT} mit {BAUDRATE} Baud...")
        try:
            # Seriellen Port initialisieren
            self.ser = serial.Serial(SERIAL_PORT, BAUDRATE, timeout=1)
        except serial.SerialException as e:
            print(f"Fehler beim Öffnen des Ports: {e}")
            return

        print("Sender aktiv. Sende synchronisierte Datenpakete (Beenden mit STRG+C)...")
        
    def CalculateChecksum(self, data_bytes):
        """Berechnet die einfache 8-Bit-Checksumme durch Aufaddieren aller Bytes."""
        checksum = 0
        for byte in data_bytes:
            checksum = (checksum + byte) & 0xFF
        return checksum

    def SetSpeed(self, linear, angular, mode):
        iLinear = round(linear*self.cLinear)
        iAngular = round(angular*self.cAngular)
        daten = [iLinear, iAngular, mode]

        # 1. Header als Bytes definieren
        header = bytes([0xAA, 0xBB])
        
        # 2. Daten-Bytes erzeugen
        # 'h' steht in Pythons struct-Bibliothek für ein 16-Bit Signed Integer (short).
        # '>' erzwingt Big-Endian (High-Byte zuerst), passend zum Arduino-Code.
        # Da wir 3 Werte haben, wiederholen wir das 'h' 3-mal (z.B. '>hhhhh')
        format_string = f'>{3}h'
        
        try:
            data_bytes = struct.pack(format_string, *daten)
        except struct.error as e:
            print(f"Fehler beim Verpacken der Daten (Wert außerhalb int16-Bereich?): {e}")
            return

        # 3. Checksumme über die reinen Daten-Bytes berechnen
        checksum_byte = bytes([self.CalculateChecksum(data_bytes)])

        # 4. Den kompletten Frame zusammenfügen
        full_frame = header + data_bytes + checksum_byte

        # Frame über UART senden
        serialEnable = True
        if serialEnable: self.ser.write(full_frame)
        
        # Optionale Konsolenausgabe zur Kontrolle
        #print(f"Gesendet: {daten} | Frame-Hex: {full_frame.hex().upper()}")

    def Close(self):
        self.ser.close()


# ================================================
# Schnittstelle zum eKarren (Hardware / Emulator)
# ================================================
class eKarren:
    def __init__(self, device=DEV_EKARREN, debug=False):
        self.device = device
        self.clientAddr = None                           
        self.rosmaster = None            
        self.rcKeyStatus = 1

        if self.device==DEV_EKARREN_EMU:
            self.clientAddr = ("127.0.0.1", 4215)            # 
        elif self.device==DEV_EKARREN_PC:
            self.clientAddr = ("192.168.178.42", 4215)       # AZ-KENKO
        elif self.device==DEV_EKARREN:
            self.clientAddr = ("192.168.20.100", 4211)       # E-Karren
        elif self.device==DEV_ARDUMOWER:
            self.ardumower = Ardumower()
        else:
            print(f"Wrong device: {self.device}")
            sys.exit(1)

        if self.device!=DEV_ARDUMOWER:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    
        ### # v = av*tau + bv
        ### av = 0.0176  
        ### bv = - 0.1348
        ### # tau = (v - bv)/av = at*v + bt
        ### self.bt = bv/av
        ### self.at = 1/av

    def GetVersion(self):
        return "V9"
        
    def GetBatteryVoltage(self):
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
    
    def SetSpeed(self, vLinear, omega, mode):
        if self.device==DEV_ARDUMOWER:
            self.ardumower.SetSpeed(vLinear, omega, mode)
            return

        vLinearQ = self.Quantize(vLinear / vLinearMax * rcMaxValue)
        vAngular = omega*radAbstand/2
        vAngularQ = self.Quantize(vAngular / vAngularMax * rcMaxValue)
        
        send_data = f"AT+#,{vLinearQ},{vAngularQ},{self.rcKeyStatus}"
        send_data += f",0x{self.CheckSum(send_data):02X}"
        self.sock.sendto(send_data.encode('utf-8'), self.clientAddr)
 
    def Close(self):
        self.SetSpeed(0, 0, 0)
        time.sleep(1)
        if self.device==DEV_ARDUMOWER: self.ardumower.Close()
        else: self.sock.close()
