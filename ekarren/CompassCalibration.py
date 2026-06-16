import numpy as np
import math
import smbus2
import time
import json
from eKarrenMain import eKarren, DEV_EKARREN, DEV_EKARREN_PC

DEVICE = DEV_EKARREN_PC
I2C_BUS = 7
MAG_ADDR = 0x30


class CompassCalibration:
    def __init__(self):
        self.ekarren = eKarren(device=DEVICE, debug=False)
    
    def Close(self):
        self.ekarren.Close()
    
    def Run(self):
        circleMeasurements = 600
        zOffsetMeasurements = 100
        measCounter = 0
        time.sleep(1.0)
        self.ekarrren.SetSpeed(vLinear=0, omega=0.2)
        
        self.bus = smbus2.SMBus(I2C_BUS)

        # Sensor vor der Messung aufwecken und entmagnetisieren!
        print("Initialisiere Sensor (SET-Puls)...")
        self.init_mag()
        
        points_xy = []
        
        
        # PHASE 1: XY-Kreise
        print("Kalibrierung PHASE 1: Langsame 360° Kreise auf FLACHEM Boden fahren")
        
        for i in range(circleMeasurements):
            measCounter += 1
            points_xy.append(self.read_mag_raw())
            print(f"Kalibrierung Phase 1: Kreisfahrt - Messpunkte: {measCounter}/{circleMeasurements}")
            time.sleep(0.05)


        points_xy = np.array(points_xy)
        center_xy, T_xy = self.fit_2d_ellipse(points_xy[:, 0], points_xy[:, 1])
        
        # Radius R berechnen: Transformiere Punkte und nimm den mittleren Abstand zum Zentrum
        # Das ist genauer als eine Konstante, da es das lokale Feld misst
        transformed_points = (points_xy[:, :2] - center_xy) @ T_xy.T
        R = np.mean(np.linalg.norm(transformed_points, axis=1))
        
        # PHASE 2: Z-Offset (Statisches Messen)
        self.SetVelocities(0, 0)
        print(f"PHASE 1 beendet. Berechneter XY-Radius R: {R:.1f}")
        z_vals = []
        measCounter = 0
        
        # Warte bis E-Karren steht
        time.sleep(0.5)

        print(f"Kalibrierung Phase 2: Z-Offset-Messung ({zOffsetMeasurements} Messpunkte)")
        for _ in range(zOffsetMeasurements):
            z_vals.append(self.read_mag_raw()[2])
            time.sleep(0.02)
            measCounter += 1

        z_raw_avg = np.mean(z_vals)
        
        # Z-Offset über Inklination (65° für Deutschland)
        inclination = math.radians(65.0)
        z_earth_ideal = R * math.tan(inclination)
        
        if z_raw_avg < 0:
            z_earth_ideal = -z_earth_ideal
            
        z_offset = z_raw_avg - z_earth_ideal
        
        # Ergebnisse speichern
        center_3d = [float(center_xy[0]), float(center_xy[1]), float(z_offset)]
        matrix_3d = [
            [float(T_xy[0,0]), float(T_xy[0,1]), 0.0],
            [float(T_xy[1,0]), float(T_xy[1,1]), 0.0],
            [0.0,              0.0,              1.0]
        ]
        
        fileName = "CompassCalibrationNew.json"
        with open(fileName, 'w') as f:
            json.dump({
                'offset': center_3d, 
                'matrix': matrix_3d,
                'raw_points': points_xy.tolist()
            }, f)
            
        print("\n--- Kalibrierung fertig ---")
        print(f"Zentrum XY: {center_xy}")
        print(f"Z-Offset:   {z_offset:.1f}")
        print(f"Radius R:   {R:.1f}")
        print(f"Kalibrierung beendet (gespeichert in {fileName})")

    def init_mag(self):
        # Register 0x08 (Internal Control 0)
        # Bit 3 (0x08) sendet einen SET-Puls in die interne Spule
        self.bus.write_byte_data(MAG_ADDR, 0x08, 0x08)
        time.sleep(0.05) # Kurz warten, bis der Kondensator sich entladen hat
        
        # Optional: Einmal blind lesen, um alte Daten aus dem Puffer zu spülen
        self.bus.write_byte_data(MAG_ADDR, 0x08, 0x01)
        time.sleep(0.02)
        self.bus.read_i2c_block_data(MAG_ADDR, 0x00, 6)

    def read_mag_raw(self):
        self.bus.write_byte_data(MAG_ADDR, 0x08, 0x01)
        while not (self.bus.read_byte_data(MAG_ADDR, 0x07) & 0x01): 
            time.sleep(0.005)
        data = self.bus.read_i2c_block_data(MAG_ADDR, 0x00, 6)
        # MMC5883MA: Unsigned 16-bit, Nullpunkt bei 32768
        return np.array([
            ((data[1] << 8) | data[0]) - 32768,
            ((data[3] << 8) | data[2]) - 32768,
            ((data[5] << 8) | data[4]) - 32768
        ])

    def fit_2d_ellipse(self, x, y):
        # Ellipsen-Fit: Ax^2 + By^2 + Cxy + Dx + Ey = 1
        D_mat = np.column_stack((x**2, y**2, x*y, x, y))
        v = np.linalg.lstsq(D_mat, np.ones_like(x), rcond=None)[0]
        
        A_mat = np.array([[v[0], v[2]/2], [v[2]/2, v[1]]])
        b_vec = np.array([v[3], v[4]])
        
        # Offset (Hard-Iron XY)
        center_xy = -0.5 * np.linalg.solve(A_mat, b_vec)
        
        # Soft-Iron Matrix (T)
        evals, evecs = np.linalg.eigh(A_mat)
        # Korrigierte Gain-Berechnung (Wurzel aus evals)
        gain = np.sqrt(np.abs(evals))
        T_xy = evecs @ np.diag(gain) @ evecs.T
        
        # Skalierung: Wir berechnen den mittleren Radius R aus den Eigenwerten
        # det(A) hängt direkt mit der Fläche der Ellipse zusammen
        # Der Radius R des korrigierten Kreises ergibt sich aus 1/sqrt(det(T_xy))
        # Aber wir normalisieren T_xy hier erst am Ende.
        
        # Normalisierung der Matrix (det=1)
        T_xy /= np.sqrt(np.linalg.det(T_xy))
        
        # Radius-Bestimmung für Z: 
        # Nach der Transformation (p - center) @ T.T liegen die Punkte auf einem Kreis.
        # Wir berechnen den durchschnittlichen Radius der transformierten Punkte.
        return center_xy, T_xy

def main():
    compassCalibration = CompassCalibration()

    try:
        compassCalibration.Run()
    except KeyboardInterrupt:
        # Wird ausgelöst, wenn du Ctrl+C drückst
        print("Compass calibration durch Benutzer abgebrochen...")
    except Exception as e:
        # Fängt unerwartete Fehler ab
        print(f"Unerwarteter Fehler: {e}")
    finally:
        compassCalibration
    
if __name__ == '__main__':
    main()