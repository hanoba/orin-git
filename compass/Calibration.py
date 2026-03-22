import smbus2
import time
import json
import math
import numpy as np

I2C_BUS = 7
MAG_ADDR = 0x30

def init_mag(bus):
    # Register 0x08 (Internal Control 0)
    # Bit 3 (0x08) sendet einen SET-Puls in die interne Spule
    bus.write_byte_data(MAG_ADDR, 0x08, 0x08)
    time.sleep(0.05) # Kurz warten, bis der Kondensator sich entladen hat
    
    # Optional: Einmal blind lesen, um alte Daten aus dem Puffer zu spülen
    bus.write_byte_data(MAG_ADDR, 0x08, 0x01)
    time.sleep(0.02)
    bus.read_i2c_block_data(MAG_ADDR, 0x00, 6)

def read_mag_raw(bus):
    bus.write_byte_data(MAG_ADDR, 0x08, 0x01)
    while not (bus.read_byte_data(MAG_ADDR, 0x07) & 0x01): time.sleep(0.005)
    data = bus.read_i2c_block_data(MAG_ADDR, 0x00, 6)
    # MMC5883MA: Unsigned 16-bit, Nullpunkt bei 32768
    return np.array([
        ((data[1] << 8) | data[0]) - 32768,
        ((data[3] << 8) | data[2]) - 32768,
        ((data[5] << 8) | data[4]) - 32768
    ])

def fit_2d_ellipse(x, y):
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
    bus = smbus2.SMBus(I2C_BUS)

    # Sensor vor der Messung aufwecken und entmagnetisieren!
    print("Initialisiere Sensor (SET-Puls)...")
    init_mag(bus)
    
    points_xy = []
    
    print("--- Kombinierte Rover-Kalibrierung (2D + Z-Inklination) ---")
    
    # PHASE 1: XY-Kreise
    print("\nPHASE 1: Fahre langsame 360° Kreise auf FLACHEM Boden.")
    input("Drücke ENTER zum Starten der Messung...")
    N=600
    try:
        for i in range(N):
            points_xy.append(read_mag_raw(bus))
            print(f"Punkte gesammelt: {len(points_xy)}/{N}", end='\r')
            time.sleep(0.05)
    except KeyboardInterrupt: pass
    
    points_xy = np.array(points_xy)
    center_xy, T_xy = fit_2d_ellipse(points_xy[:, 0], points_xy[:, 1])
    
    # Radius R berechnen: Transformiere Punkte und nimm den mittleren Abstand zum Zentrum
    # Das ist genauer als eine Konstante, da es das lokale Feld misst
    transformed_points = (points_xy[:, :2] - center_xy) @ T_xy.T
    R = np.mean(np.linalg.norm(transformed_points, axis=1))
    
    # PHASE 2: Z-Offset (Statisches Messen)
    print(f"\n\nPHASE 1 beendet. Berechneter XY-Radius R: {R:.1f}")
    print("\nPHASE 2: Halte den Roboter absolut STILL und FLACH.")
    input("Drücke ENTER für Z-Messung...")
    
    z_vals = []
    for _ in range(100):
        z_vals.append(read_mag_raw(bus)[2])
        time.sleep(0.02)
    
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
    
    with open('mag_ellipsoid.json', 'w') as f:
        json.dump({
            'offset': center_3d, 
            'matrix': matrix_3d,
            'raw_points': points_xy.tolist()
        }, f)
        
    print(f"\n--- Kalibrierung fertig ---")
    print(f"Zentrum XY: {center_xy}")
    print(f"Z-Offset:   {z_offset:.1f}")
    print(f"Radius R:   {R:.1f}")
    print("Gespeichert in mag_ellipsoid.json")

if __name__ == "__main__":
    main()
