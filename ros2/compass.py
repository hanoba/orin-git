# TiltCompCompass.py

# Entzerrung und Neigungskorrektur für Magnetometer MMC5883L (GY-801)
import smbus2
import time
import math
import struct
import json
import numpy as np

I2C_BUS = 7
ACC_ADDR = 0x53
MAG_ADDR = 0x30

class Compass:
    def __init__(self):
        self.bus = smbus2.SMBus(I2C_BUS)
        self.load_cal()
        # ADXL345 Setup
        self.bus.write_byte_data(ACC_ADDR, 0x2D, 0x08)
        self.bus.write_byte_data(ACC_ADDR, 0x31, 0x0B)
        self.counter = 0
        self.init_mag()
        self.yawOffset = math.pi/2

    def init_mag(self):
        # Register 0x08 (Internal Control 0)
        # Bit 3 (0x08) sendet einen SET-Puls in die interne Spule
        self.bus.write_byte_data(MAG_ADDR, 0x08, 0x08)
        time.sleep(0.05) # Kurz warten, bis der Kondensator sich entladen hat
        
        # Optional: Einmal blind lesen, um alte Daten aus dem Puffer zu spülen
        self.bus.write_byte_data(MAG_ADDR, 0x08, 0x01)
        time.sleep(0.02)
        self.bus.read_i2c_block_data(MAG_ADDR, 0x00, 6)

        # Nächstes Lesen bereits jetzt triggern
        self.bus.write_byte_data(MAG_ADDR, 0x08, 0x01)
        time.sleep(0.02)

    def load_cal(self):
        with open('compass/mag_ellipsoid.json', 'r') as f:
            cal = json.load(f)
            self.offset = np.array(cal['offset'])
            self.matrix = np.array(cal['matrix'])


    def ReadYaw(self):
        if (self.bus.read_byte_data(MAG_ADDR, 0x07) & 0x01):
            # Accel
            acc = self.bus.read_i2c_block_data(ACC_ADDR, 0x32, 6)
            ax, ay, az = struct.unpack('<hhh', bytes(acc))
            
            # Mag
            mag_raw = self.bus.read_i2c_block_data(MAG_ADDR, 0x00, 6)
            m_raw = np.array([((mag_raw[1]<<8)|mag_raw[0])-32768, 
                              ((mag_raw[3]<<8)|mag_raw[2])-32768, 
                              ((mag_raw[5]<<8)|mag_raw[4])-32768])

            # nächste Messung triggern
            self.bus.write_byte_data(MAG_ADDR, 0x08, 0x01)
            
            # Vollständige 3D-Korrektur (Hard & Soft Iron)
            m_cal = (m_raw - self.offset) @ self.matrix.T

            # --- DER FIX ---
            # Dreht die Z-Achse des Magnetometers um, damit die 
            # Neigungskorrektur in die richtige Richtung drückt!
            m_cal[2] = -m_cal[2]
            
            # Neigung berechnen
            roll = math.atan2(ay, az)
            pitch = math.atan2(-ax, math.sqrt(ay*ay + az*az))

            # Tilt Compensation (Matrix-Transformation)
            cp, sp = math.cos(pitch), math.sin(pitch)
            cr, sr = math.cos(roll), math.sin(roll)
            
            # Projektion auf die Horizontale
            xh = m_cal[0]*cp + m_cal[1]*sr*sp + m_cal[2]*cr*sp
            yh = m_cal[1]*cr - m_cal[2]*sr

            yaw = math.atan2(-yh, xh) + self.yawOffset
            return (yaw + math.pi) % math.tau - math.pi
        print("Error no MAG data")
        return None

    def run(self):
        while True:
            heading = math.degrees(self.ReadYaw())
            time.sleep(0.1)
            print(f"{self.counter:5d} Kurs:{heading:3.0f}°")
            self.counter += 1

if __name__ == "__main__": 
    Compass().run()
