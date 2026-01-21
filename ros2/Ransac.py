import numpy as np
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point


MAX_GAP = 5.50

def find_intersection(line1, line2):
    p1, p2 = line1; p3, p4 = line2
    x1, y1 = p1; x2, y2 = p2; x3, y3 = p3; x4, y4 = p4
    denom = (y4 - y3) * (x2 - x1) - (x4 - x3) * (y2 - y1)
    if denom == 0: return None 
    ua = ((x4 - x3) * (y1 - y3) - (y4 - y3) * (x1 - x3)) / denom
    return np.array([x1 + ua * (x2 - x1), y1 + ua * (y2 - y1)])

def get_lines_with_gap_check(points):
    """Findet die beste Linie und teilt sie bei Lücken auf."""
    if len(points) < 6: return [], None
    
    pts = points.astype(np.float32)
    best_mask = None
    best_count = 0
    dist_thresh = 0.05

    # 1. RANSAC wie gehabt
    for _ in range(40):
        idx = np.random.choice(len(pts), 2, replace=False)
        p1, p2 = pts[idx[0]], pts[idx[1]]
        vec = p2 - p1
        norm = np.linalg.norm(vec)
        if norm < 0.01: continue 
        normal = np.array([-vec[1], vec[0]]) / norm
        dists = np.abs(np.dot(pts - p1, normal))
        mask = dists < dist_thresh
        count = np.sum(mask)
        if count > best_count:
            best_count = count
            best_mask = mask

    if best_count < 6: return [], None

    # 2. Inlier verfeinern
    inliers = points[best_mask]
    mean = np.mean(inliers, axis=0)
    uu, dd, vv = np.linalg.svd(inliers - mean)
    direction = vv[0]
    
    # 3. GAP-CHECK (Tor-Erkennung)
    projections = np.dot(inliers - mean, direction)
    sort_idx = np.argsort(projections)
    sorted_inliers = inliers[sort_idx]
    sorted_proj = projections[sort_idx]

    # Abstände zwischen aufeinanderfolgenden Punkten berechnen
    diffs = np.linalg.norm(sorted_inliers[1:] - sorted_inliers[:-1], axis=1)
    # Wo ist die Lücke größer als max_gap?
    gap_indices = np.where(diffs > MAX_GAP)[0]

    split_lines = []
    last_idx = 0
    
    # Wand an den Lücken zerteilen
    for gap_idx in gap_indices:
        segment = sorted_inliers[last_idx : gap_idx + 1]
        if len(segment) >= 5: # Nur Segmente mit genug Punkten nehmen
            p_start = segment[0]
            p_end = segment[-1]
            split_lines.append((p_start, p_end))
        last_idx = gap_idx + 1
    
    # Das letzte (oder einzige) Stück hinzufügen
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
    dist_thresh = 0.05

    # 1. RANSAC (unverändert)
    for _ in range(40):
        idx = np.random.choice(len(pts), 2, replace=False)
        p1, p2 = pts[idx[0]], pts[idx[1]]
        vec = p2 - p1
        norm = np.linalg.norm(vec)
        if norm < 0.01: continue 
        normal = np.array([-vec[1], vec[0]]) / norm
        dists = np.abs(np.dot(pts - p1, normal))
        mask = dists < dist_thresh
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

    # Iterativ Linien suchen
    for _ in range(10):
        if len(temp_points) < 6: break
        lines, mask = get_lines_with_gap_check(temp_points)
        if not lines: break
        
        all_detected_walls.extend(lines)
        temp_points = temp_points[~mask]
    return all_detected_walls


def PublishMarkers(pub, all_detected_walls):
    markers = MarkerArray()
    frame_id = "lidar"
    z_height = -0.1
    
    # 1. Ein Marker für ALLE Linien
    m_lines = Marker()
    m_lines.header.frame_id = frame_id
    m_lines.header.stamp.sec = 0
    m_lines.header.stamp.nanosec = 0
    m_lines.ns = "walls_lines"
    m_lines.id = 0
    m_lines.type = Marker.LINE_LIST # LINE_LIST statt LINE_STRIP für getrennte Segmente
    m_lines.action = Marker.ADD
    m_lines.scale.x = 0.15
    m_lines.color.g = 1.0; m_lines.color.a = 0.8
    m_lines.pose.orientation.w = 1.0

    # 2. Ein Marker für ALLE Endpunkte (Sphären)
    m_spheres = Marker()
    m_spheres.header = m_lines.header
    m_spheres.ns = "walls_endpoints"
    m_spheres.id = 0
    m_spheres.type = Marker.SPHERE_LIST # <--- EXTREM EFFIZIENT
    m_spheres.action = Marker.ADD
    m_spheres.scale.x = 0.25; m_spheres.scale.y = 0.25; m_spheres.scale.z = 0.25
    m_spheres.color.r = 1.0; m_spheres.color.a = 1.0

    for start, end in all_detected_walls:
        p_start = Point(x=float(start[0]), y=float(start[1]), z=z_height)
        p_end = Point(x=float(end[0]), y=float(end[1]), z=z_height)
        
        # Punkte zur Linienliste hinzufügen
        m_lines.points.append(p_start)
        m_lines.points.append(p_end)
        
        # Punkte zur Sphärenliste hinzufügen
        m_spheres.points.append(p_start)
        m_spheres.points.append(p_end)

    markers.markers.append(m_lines)
    markers.markers.append(m_spheres)
    
    pub.publish(markers)
