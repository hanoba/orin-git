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

class ProCompass:
    def __init__(self):
        self.bus = smbus2.SMBus(I2C_BUS)
        self.load_cal()
        # ADXL345 Setup
        self.bus.write_byte_data(ACC_ADDR, 0x2D, 0x08)
        self.bus.write_byte_data(ACC_ADDR, 0x31, 0x0B)
        self.counter = 0
        self.init_mag()

    def init_mag(self):
        # Register 0x08 (Internal Control 0)
        # Bit 3 (0x08) sendet einen SET-Puls in die interne Spule
        self.bus.write_byte_data(MAG_ADDR, 0x08, 0x08)
        time.sleep(0.05) # Kurz warten, bis der Kondensator sich entladen hat
        
        # Optional: Einmal blind lesen, um alte Daten aus dem Puffer zu spülen
        self.bus.write_byte_data(MAG_ADDR, 0x08, 0x01)
        time.sleep(0.02)
        self.bus.read_i2c_block_data(MAG_ADDR, 0x00, 6)

    def load_cal(self):
        with open('mag_ellipsoid.json', 'r') as f:
            cal = json.load(f)
            self.offset = np.array(cal['offset'])
            self.matrix = np.array(cal['matrix'])

    def get_data(self):
        # Accel
        acc = self.bus.read_i2c_block_data(ACC_ADDR, 0x32, 6)
        ax, ay, az = struct.unpack('<hhh', bytes(acc))
        # Mag
        self.bus.write_byte_data(MAG_ADDR, 0x08, 0x01)
        while not (self.bus.read_byte_data(MAG_ADDR, 0x07) & 0x01): pass
        mag_raw = self.bus.read_i2c_block_data(MAG_ADDR, 0x00, 6)
        m_raw = np.array([((mag_raw[1]<<8)|mag_raw[0])-32768, 
                          ((mag_raw[3]<<8)|mag_raw[2])-32768, 
                          ((mag_raw[5]<<8)|mag_raw[4])-32768])
        
        # Vollständige 3D-Korrektur (Hard & Soft Iron)
        m_cal = (m_raw - self.offset) @ self.matrix.T

        # --- DER FIX ---
        # Dreht die Z-Achse des Magnetometers um, damit die 
        # Neigungskorrektur in die richtige Richtung drückt!
        m_cal[2] = -m_cal[2]
        return ax, ay, az, m_cal, m_raw

    def run(self):
        while True:
            ax, ay, az, m, mRaw = self.get_data()
            
            # Neigung berechnen
            roll = math.atan2(ay, az)
            pitch = math.atan2(-ax, math.sqrt(ay*ay + az*az))

            # Tilt Compensation (Matrix-Transformation)
            cp, sp = math.cos(pitch), math.sin(pitch)
            cr, sr = math.cos(roll), math.sin(roll)
            
            # Projektion auf die Horizontale
            xh = m[0]*cp + m[1]*sr*sp + m[2]*cr*sp
            yh = m[1]*cr - m[2]*sr

            heading = math.degrees(math.atan2(-yh, xh))
            if heading < 0: heading += 360
            
            mx = mRaw[0]
            my = mRaw[1]
            mz = mRaw[2]
            print(f"{self.counter:5d} Kurs:{heading:3.0f}°  | Pitch:{math.degrees(pitch):3.0f}°  |"
                  f"  ACCEL -> ax:{ax:<4}  ay:{ay:<4}  az:{az:<4}  |"
                  f"  MAG -> mx:{mx:<4}  my:{my:<4}  mz:{mz:<4}")  #, end='\r')
            self.counter += 1
            time.sleep(0.2)

if __name__ == "__main__": 
    ProCompass().run()
