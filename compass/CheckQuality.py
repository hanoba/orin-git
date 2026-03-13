import json
import numpy as np

def check_quality():
    try:
        with open('mag_ellipsoid.json', 'r') as f:
            cal = json.load(f)
            matrix = np.array(cal['matrix'])
            offset = np.array(cal['offset'])
            
        print("--- Analyse der Kalibrierungsmatrix ---")
        # Die Eigenwerte der Matrix sagen uns, wie stark der Sensor korrigiert werden musste
        evals = np.linalg.eigvals(matrix)
        
        print(f"Hard-Iron Offset (Zentrum): {offset}")
        print(f"Soft-Iron Skalierungsfaktoren: {evals}")
        
        # Ein Verhältnis der Skalierung > 2 deutet oft auf eine schlechte Z-Kalibrierung hin
        ratio = np.max(evals) / np.min(evals)
        if ratio > 2.0:
            print(f"\nWARNUNG: Die Z-Achse scheint sehr schwach kalibriert zu sein ({ratio=}).")
            print("Tipp: Fahre bei der Kalibrierung über steilere Rampen!")
        else:
            print("\nSTATUS: Die Ellipsoid-Anpassung sieht mathematisch solide aus.")
            
    except FileNotFoundError:
        print("Datei mag_ellipsoid.json nicht gefunden. Erst kalibrieren!")

if __name__ == "__main__":
    check_quality()
    