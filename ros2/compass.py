# TiltCompCompass.py

# Entzerrung und Neigungskorrektur für Magnetometer MMC5883L (GY-801)
import smbus2
import time
import math
import struct
import json
import numpy as np
from params import ReadYawOffset, WriteYawOffset


I2C_BUS = 7
ACC_ADDR = 0x53
MAG_ADDR = 0x30
GYRO_ADDR = 0x69

# =====================================================================
# FILTER
# =====================================================================
class YawComplementaryFilter:
    def __init__(self, alpha=0.98):
        self.alpha = alpha
        self.current_yaw = None
        self.last_time = None

    def update(self, gyro_z_rad_s, mag_yaw_deg, current_time):
        if self.current_yaw is None or self.last_time is None:
            self.current_yaw = mag_yaw_deg
            self.last_time = current_time
            return self.current_yaw

        dt = current_time - self.last_time
        self.last_time = current_time

        predicted_yaw = self.current_yaw + (gyro_z_rad_s * dt)
        diff = mag_yaw_deg - predicted_yaw
        diff = (diff + math.pi) % math.tau - math.pi
        self.current_yaw = predicted_yaw + (1.0 - self.alpha) * diff
        self.current_yaw = (self.current_yaw + math.pi) % math.tau - math.pi
        return self.current_yaw

# =====================================================================
# GY-801 HARDWARE (Angepasst für den MMC5883MA Klon auf 0x30)
# =====================================================================
class GY801_Fast:
    def __init__(self, bus_num=7):
        self.bus = smbus2.SMBus(bus_num)
        
        print("Initialisiere Sensoren...")
        # --- Accel (ADXL345 auf ACC_ADDR) ---
        self.bus.write_byte_data(ACC_ADDR, 0x2c, 0x0a)
        self.bus.write_byte_data(ACC_ADDR, 0x2d, 0x08)
        
        # --- Gyro (L3G4200D auf 0x69) ---
        self.bus.write_byte_data(GYRO_ADDR, 0x20, 0x0f) # Aufwecken & 100Hz
        
        # CTRL_REG4 auf 500 dps setzen (Bitmuster: 00010000 = 0x10)
        self.bus.write_byte_data(GYRO_ADDR, 0x23, 0x10)
         
        # --- Mag (MMC5883MA auf 0x30) ---
        
        # Register 0x08 (Internal Control 0)
        # Bit 3 (0x08) sendet einen SET-Puls in die interne Spule
        self.bus.write_byte_data(MAG_ADDR, 0x08, 0x08)
        time.sleep(0.05) # Kurz warten, bis der Kondensator sich entladen hat
        
        # Optional: Einmal blind lesen, um alte Daten aus dem Puffer zu spülen
        self.bus.write_byte_data(MAG_ADDR, 0x08, 0x01)
        time.sleep(0.02)
        self.bus.read_i2c_block_data(MAG_ADDR, 0x00, 6)

        # Nächste Messung bereits jetzt triggern
        self.bus.write_byte_data(MAG_ADDR, 0x08, 0x01)
        time.sleep(0.1)

    def read_all(self):
        # 1. Mag MMC5883MA auslesen (Little-Endian)
        m_raw = self.bus.read_i2c_block_data(MAG_ADDR, 0x00, 6)
        
        # SOFORT nächste Messung anstoßen, während wir rechnen
        self.bus.write_byte_data(MAG_ADDR, 0x08, 0x01)
        
        # 2. Accel
        a_raw = self.bus.read_i2c_block_data(ACC_ADDR, 0x32, 6)
        # 3. Gyro
        g_raw = self.bus.read_i2c_block_data(GYRO_ADDR, 0x28 | 0x80, 6)
        
        # MMC5883MA liefert praktischerweise X, Y, Z (Little Endian)
        mx, my, mz = struct.unpack('<HHH', bytes(m_raw))    # WICHTIG: Großes 'H' für Unsigned Short (0 bis 65535)

        # Nullpunkt in die Mitte schieben (32768 = 0 Gauss)
        mx -= 32768.0
        my -= 32768.0
        mz -= 32768.0

        ax, ay, az = struct.unpack('<hhh', bytes(a_raw))
        gx, gy, gz = struct.unpack('<hhh', bytes(g_raw))
        
        return (ax, ay, az), (mx, my, mz), (gx, gy, gz)


class Compass:
    def __init__(self):
        self.imu = GY801_Fast(I2C_BUS)
        # Falls der Kompass mit alpha=0.98 (98% Gyro, 2% Mag) zu träge reagiert, dann setze alpha auf 0.95 oder 0.90.
        # Das gibt dem Magnetometer wieder etwas mehr Gewicht.
        self.filter = YawComplementaryFilter(alpha=0.90)
        self.LoadCalibrationData()
        self.counter = 0
        #self.yawOffset = np.radians(54-4)
        self.yawOffset = ReadYawOffset()
        print(f"YawOffset={np.degrees(self.yawOffset)}°")

        # Gyro-Bias kalibrieren
        calibration_samples = 100
        bias_sum = 0.0
        for _ in range(calibration_samples):
            _, _, gyro = self.imu.read_all()
            # In rad/s umrechnen!
            bias_sum += np.radians(gyro[2] * 0.0175)
            time.sleep(0.02)
            
        self.gyro_z_bias = bias_sum / calibration_samples
        print(f"Gyro-Bias-Kalibrierung fertig! Gyro-Bias: {self.gyro_z_bias:+.4f} rad/s")        

    def LoadCalibrationData(self):
        with open('compass/mag_ellipsoid.json', 'r') as f:
            cal = json.load(f)
            self.offset = np.array(cal['offset'])
            self.matrix = np.array(cal['matrix'])

    def ReadYaw(self):
        (ax, ay, az), m_raw, gyro = self.imu.read_all()
        
        # Vollständige 3D-Korrektur (Hard & Soft Iron)
        m_cal = (np.array(m_raw) - self.offset) @ self.matrix.T

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

        mag_yaw = math.atan2(-yh, xh)
        
        # Complementary Filter anwenden
        gz_rad_s = np.radians(gyro[2] * 0.0175) - self.gyro_z_bias
        current_time = time.time()
        filtered_yaw = self.filter.update(gz_rad_s, mag_yaw, current_time)        
        
        yaw = filtered_yaw + self.yawOffset
        yaw = (yaw + math.pi) % math.tau - math.pi
        return yaw

    def run(self):
        while True:
            start_t = time.time()
            
            heading = math.degrees(self.ReadYaw())
            
            if self.counter % 10 == 0:
                print(f"{self.counter:5d} Kurs:{heading:3.0f}°")
            self.counter += 1
            
            # Exakt auf 50 Hz abregeln
            elapsed = time.time() - start_t
            sleep_time = 0.02 - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

if __name__ == "__main__": 
    Compass().run()
