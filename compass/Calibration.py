import smbus2
import time
import struct
import json
import numpy as np

I2C_BUS = 7
MAG_ADDR = 0x30

def read_mag_raw(bus):
    bus.write_byte_data(MAG_ADDR, 0x08, 0x01)
    while not (bus.read_byte_data(MAG_ADDR, 0x07) & 0x01): time.sleep(0.005)
    data = bus.read_i2c_block_data(MAG_ADDR, 0x00, 6)
    return np.array([
        ((data[1] << 8) | data[0]) - 32768,
        ((data[3] << 8) | data[2]) - 32768,
        ((data[5] << 8) | data[4]) - 32768
    ])

def fit_ellipsoid(data):
    # Mathematisches Fitting eines Ellipsoids: Ax^2 + By^2 + Cz^2 + 2Dxy + 2Exz + 2Fyz + 2Gx + 2Hy + 2Iz = 1
    x, y, z = data[:, 0], data[:, 1], data[:, 2]
    D = np.array([x**2, y**2, z**2, 2*x*y, 2*x*z, 2*y*z, 2*x, 2*y, 2*z]).T
    v = np.linalg.lstsq(D, np.ones(len(data)), rcond=None)[0]
    
    # Rekonstruktion der Matrixform (Quadrik)
    A_mat = np.array([[v[0], v[3], v[4]], [v[3], v[1], v[5]], [v[4], v[5], v[2]]])
    b_vec = v[6:9]
    center = -np.linalg.solve(A_mat, b_vec)
    
    # Transformation in Korrekturmatrix (Soft-Iron)
    evals, evecs = np.linalg.eigh(A_mat)
    # Wir skalieren so, dass das Ergebnis eine Einheitskugel ist
    gain = np.sqrt(1.0 / np.abs(evals))
    T = evecs @ np.diag(gain) @ evecs.T
    
    return center.tolist(), T.tolist()

def main():
    bus = smbus2.SMBus(I2C_BUS)
    points = []
    print("--- Ellipsoid-Kalibrierung ---")
    input("Press RETURN to start calibration")
    print("Drehe den Roboter 360 Grad und fahre über Unebenheiten/Rampen!")
    
    try:
        for _ in range(600): # ca. 30 Sekunden bei 20Hz
            points.append(read_mag_raw(bus))
            print(f"Punkte gesammelt: {len(points)}", end='\r')
            time.sleep(0.05)
            
        center, scale_matrix = fit_ellipsoid(np.array(points))
        
        with open('mag_ellipsoid.json', 'w') as f:
            json.dump({'offset': center, 'matrix': scale_matrix}, f)
        print("\nKalibrierung gespeichert in mag_ellipsoid.json")
        
    except KeyboardInterrupt: print("\nAbbruch")

if __name__ == "__main__": 
    main()