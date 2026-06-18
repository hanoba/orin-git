import traceback # Ganz oben im Skript importieren!
import os
# Verbietet Numpy/OpenBLAS das heimliche Starten von Unter-Threads.
# Muss GANZ OBEN stehen, vor "import numpy"!
os.environ["OPENBLAS_NUM_THREADS"] = "1"
os.environ["OMP_NUM_THREADS"] = "1"
os.environ["MKL_NUM_THREADS"] = "1"
import numpy as np

MAX_GAP = 1.5  #0.50      # 1.0
DIST_THRESH = 0.05*3   # 0.05

def find_intersection(line1, line2):
    p1, p2 = line1; p3, p4 = line2
    x1, y1 = p1; x2, y2 = p2; x3, y3 = p3; x4, y4 = p4
    denom = (y4 - y3) * (x2 - x1) - (x4 - x3) * (y2 - y1)
    if denom == 0: return None 
    ua = ((x4 - x3) * (y1 - y3) - (y4 - y3) * (x1 - x3)) / denom
    return np.array([x1 + ua * (x2 - x1), y1 + ua * (y2 - y1)])


def get_lines_with_gap_check(points):
    if len(points) < 6: return [], None
    
    # 0. SCHUTZSCHILD: Alle NaN und Inf Werte vorher ausfiltern!
    valid_mask = np.isfinite(points).all(axis=1)
    pts = points[valid_mask].astype(np.float32)
    
    if len(pts) < 6: return [], None

    best_mask = None
    best_count = 0

    for _ in range(40):
        idx = np.random.choice(len(pts), 2, replace=False)
        p1, p2 = pts[idx[0]], pts[idx[1]]
        vec = p2 - p1
        norm = np.linalg.norm(vec)
        if norm < 0.01: continue 
        normal = np.array([-vec[1], vec[0]]) / norm
        dists = np.abs(np.dot(pts - p1, normal))
        mask = dists < DIST_THRESH
        count = np.sum(mask)
        if count > best_count:
            best_count = count
            best_mask = mask

    if best_count < 6: return [], None

    inliers = pts[best_mask]
    mean = np.mean(inliers, axis=0)

    Version = 3
    if Version==3:
        # ---------------------------------------------------------
        # STABILER SVD-ERSATZ (Über Winkelberechnung)
        # ---------------------------------------------------------
        centered = inliers - mean
        
        cov = np.dot(centered.T, centered)
        a = cov[0, 0]
        b = cov[0, 1]
        d = cov[1, 1]
        
        angle = 0.5 * np.arctan2(2 * b, a - d)
        direction = np.array([np.cos(angle), np.sin(angle)])
        
        # ---------------------------------------------------------
        # DER NEUE, ABSOLUT DETERMINISTISCHE ORIENTIERUNGSSCHUTZ
        # ---------------------------------------------------------
        # 1. Wir berechnen den Normalenvektor der Wand (90 Grad gedreht zur Richtung)
        normal = np.array([-direction[1], direction[0]])
        
        # 2. 'mean' ist der Vektor vom Roboter (0,0) exakt zur Mitte der Wand.
        # Wir prüfen, ob die Normale zum Roboter zeigt oder von ihm weg.
        # Wenn das Skalarprodukt positiv ist, zeigt sie vom Roboter weg.
        if np.dot(normal, mean) > 0:
            # Wir drehen die Linie um 180 Grad! 
            # Ab jetzt ist garantiert, dass die Linie immer einheitlich relativ 
            # zum Roboter ausgerichtet ist. Kein "Flippen" mehr möglich!
            direction = -direction
        # ---------------------------------------------------------
    elif Version==0:
        # ---------------------------------------------------------
        # DER PANZER-CODE: 2x2 Eigenvektor analytisch berechnen
        # (Ohne linalg-Bibliothek -> 100% Freeze-sicher!)
        # ---------------------------------------------------------
        centered = inliers - mean
        
        # 1. 2x2 Kovarianzmatrix berechnen (A^T * A)
        cov = np.dot(centered.T, centered)
        a = cov[0, 0]
        b = cov[0, 1]
        d = cov[1, 1]
        
        # 2. Größten Eigenwert (L) über Spur und Determinante berechnen
        trace_val = a + d
        det_val = a * d - b * b
        # max(0, ...) schützt vor Mini-Rundungsfehlern im Minusbereich
        L = trace_val / 2.0 + np.sqrt(max(0.0, (trace_val / 2.0)**2 - det_val))
        
        # 3. Eigenvektor zur Hauptachse bestimmen
        if abs(b) < 1e-9:
            # Sonderfall: Punkte liegen exakt waagerecht oder senkrecht
            direction = np.array([1.0, 0.0]) if a >= d else np.array([0.0, 1.0])
        else:
            direction = np.array([b, L - a])
            direction = direction / np.linalg.norm(direction) # Normalisieren auf Länge 1

        # 4. Orientierungsschutz (Damit die Linie nicht in die falsche Lidar-Richtung zeigt)
        grobe_richtung = inliers[-1] - inliers[0]
        if np.dot(direction, grobe_richtung) < 0:
            direction = -direction
    elif Version==1:
        # 1. Zwingend in float64 umwandeln (verhindert ARM-Architektur-Bugs in der SVD)
        centered = (inliers - mean).astype(np.float64)
        
        # 2. Denormalisierte Zahlen killen! 
        # Alles was mikroskopisch klein ist, wird brutal auf exakt 0.0 gesetzt.
        # Das verhindert die Endlosschleife im Prozessor.
        centered[np.abs(centered) < 1e-7] = 0.0

        uu, dd, vv = np.linalg.svd(centered)
        direction = vv[0]
    else:
        # Finale Korrektur: Deterministische Hauptachsen-Extraktion (SVD-Äquivalenz)
        centered_inliers = inliers - mean
        cov_matrix = np.cov(centered_inliers, rowvar=False)
        eigenvalues, eigenvectors = np.linalg.eigh(cov_matrix)
        direction = eigenvectors[:, np.argmax(eigenvalues)]

        # Orientierungsschutz beibehalten
        grobe_richtung = inliers[-1] - inliers[0]
        if np.dot(direction, grobe_richtung) < 0:
            direction = -direction
    
    projections = np.dot(inliers - mean, direction)
    sort_idx = np.argsort(projections)
    sorted_inliers = inliers[sort_idx]

    diffs = np.linalg.norm(sorted_inliers[1:] - sorted_inliers[:-1], axis=1)
    gap_indices = np.where(diffs > MAX_GAP)[0]

    split_lines = []
    last_idx = 0
    
    for gap_idx in gap_indices:
        segment = sorted_inliers[last_idx : gap_idx + 1]
        if len(segment) >= 5: 
            split_lines.append((segment[0], segment[-1]))
        last_idx = gap_idx + 1
    
    final_segment = sorted_inliers[last_idx:]
    if len(final_segment) >= 5:
        split_lines.append((final_segment[0], final_segment[-1]))

    return split_lines, best_mask


def get_lines_without_gap_check(points):
    """Findet die beste Linie (ohne sie bei Lücken aufzuteilen)."""
    if len(points) < 6: return [], None
    
    pts = points.astype(np.float32)
    best_mask = None
    best_count = 0

    # 1. RANSAC (unverändert)
    for _ in range(40):
        idx = np.random.choice(len(pts), 2, replace=False)
        p1, p2 = pts[idx[0]], pts[idx[1]]
        vec = p2 - p1
        norm = np.linalg.norm(vec)
        if norm < 0.01: continue 
        normal = np.array([-vec[1], vec[0]]) / norm
        dists = np.abs(np.dot(pts - p1, normal))
        mask = dists < DIST_THRESH
        count = np.sum(mask)
        if count > best_count:
            best_count = count
            best_mask = mask

    if best_count < 6: return [], None

    # 2. Inlier verfeinern (SVD)
    inliers = points[best_mask]
    mean = np.mean(inliers, axis=0)
    uu, dd, vv = np.linalg.svd(inliers - mean)
    direction = vv[0]
    
    # 3. Endpunkte bestimmen (Ohne Gap-Check)
    # Wir projizieren alle Punkte auf die gefundene Linie
    projections = np.dot(inliers - mean, direction)
    
    # Wir sortieren die Punkte entlang der Linie
    sort_idx = np.argsort(projections)
    sorted_inliers = inliers[sort_idx]

    # Der erste und der letzte Punkt der sortierten Liste sind Start und Ende
    p_start = sorted_inliers[0]
    p_end = sorted_inliers[-1]

    # Rückgabe als Liste mit einem Tuple, um das Format beizubehalten
    # Format: [(Startpunkt, Endpunkt)], Maske
    return [(p_start, p_end)], best_mask


def LineDetection(points):
    all_detected_walls = []
    temp_points = points.copy()

    try:
        # Iterativ Linien suchen
        for i in range(10):
            if len(temp_points) < 6: 
                break
            
            lines, mask = get_lines_with_gap_check(temp_points)
            
            if not lines: 
                break
            
            all_detected_walls.extend(lines)
            
            temp_points = temp_points[~mask]

    except Exception as e:
        # HIER fangen wir den lautlosen Killer!
        print(f"[LineDetection] FATAL: {e}  {traceback.format_exc()}")
        
    return all_detected_walls


