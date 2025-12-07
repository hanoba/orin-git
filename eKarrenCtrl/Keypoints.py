import numpy as np

def normalize_kpts(kpts):
    """
    Garantiert, dass kpts vom Typ:
        (17, 2) oder (17, 3)
    ist und Keypoints flach sind.
    Verhindert verschachtelte Arrays wie (17,1,2) oder (1,17,3).
    """
    kpts = np.array(kpts)

    # Entferne führende Dimensionen: (1,17,2) -> (17,2)
    while kpts.ndim > 2:
        kpts = kpts[0]

    # Falls nur x,y vorhanden -> fertig
    if kpts.shape[1] == 2:
        return kpts

    # Falls (x,y,conf) -> gib (x,y) zurück
    if kpts.shape[1] >= 3:
        return kpts[:, :2]

    raise ValueError("Unbekanntes Keypoint-Format, shape=" + str(kpts.shape))

def kp(kpts, i):
    """Gibt einen flachen x,y Vektor oder None zurück."""
    if i >= len(kpts):
        return None
    xy = kpts[i]
    return np.array([float(xy[0]), float(xy[1])])  # garantiert float


def ClassifyPose(kpts_raw, conf_thresh=0.25):
    """
    kpts: (17, 2) array of xy coordinates.
    Returns: string describing pose:
        "arms_up", "t_pose", "squat", "fall", "sitting", "unknown"
    """
    
    if kpts_raw is None or len(kpts_raw) == 0:
        return "none"

    # 1) Normalisieren ? garantiert flache Struktur (17,2)
    kpts = normalize_kpts(kpts_raw)
    
    # Extract safely    
    ls = kp(kpts, 5)   # left shoulder
    rs = kp(kpts, 6)   # right shoulder
    le = kp(kpts, 7)   # Left Elbow
    re = kp(kpts, 8)   # Right Elbow
    lw = kp(kpts, 9)   # Left Wrist
    rw = kp(kpts, 10)  # Right Wrist
    lh = kp(kpts, 11)  # Left Hip
    rh = kp(kpts, 12)  # Right Hip
    lk = kp(kpts, 13)  # Left Knee
    rk = kp(kpts, 14)  # Right Knee
    la = kp(kpts, 15)  # Left Ankle
    ra = kp(kpts, 16)  # Right Ankle

    # Helper: average or None
    def avg(a, b):
        if a is None or b is None:
            return None
        return (a + b) / 2

    shoulders = avg(ls, rs)
    hips = avg(lh, rh)
    knees = avg(lk, rk)
    ankles = avg(la, ra)
    wrists = avg(lw, rw)

    # Can't classify without shoulders
    if shoulders is None:
        return "none"

    # Y-axis: smaller = higher in image
    if lw is not None and rw is not None:
        # ----------------------------------------------------
        # 1) ARMS UP: wrists significantly above shoulders
        # ----------------------------------------------------
        if lw[1] < shoulders[1] - 20 and rw[1] < shoulders[1] - 20:
            return "ArmsUp"

        # ----------------------------------------------------
        # 2) LEFT ARM UP: wrists significantly above/below shoulders
        # ----------------------------------------------------
        if lw[1] < shoulders[1] - 20 and rw[1] > shoulders[1]:
            return "LeftArmUp"

        # ----------------------------------------------------
        # 3) RIGHT ARM UP: wrists significantly above/below shoulders
        # ----------------------------------------------------
        if lw[1] > shoulders[1] and rw[1] < shoulders[1] - 20:
            return "RightArmUp"

        # ----------------------------------------------------
        # 4) T-POSE: wrists horizontally far away, same vertical height as shoulders
        # ----------------------------------------------------
        horizontal_span = abs(lw[0] - rw[0])
        vert_diff = abs(lw[1] - shoulders[1]) + abs(rw[1] - shoulders[1])
        if horizontal_span > 0.6 * np.linalg.norm(rs - ls):  # arms stretched wide
            if vert_diff < 40:  # roughly same vertical level
                return "TPose"

    # # ----------------------------------------------------
    # # 3) SQUAT: hip almost same height as knees
    # # ----------------------------------------------------
    # if hips is not None and knees is not None:
    #     if hips[1] > knees[1] - 30:  # hips not much higher than knees
    #         return "squat"
    # 
    # # ----------------------------------------------------
    # # 4) FALL: shoulders drastically close to ground level (ankles)
    # #    or body very horizontal
    # # ----------------------------------------------------
    # if shoulders is not None and ankles is not None:
    #     if shoulders[1] > ankles[1] - 40:
    #         return "fall"
    # 
    #     # Horizontal body angle
    #     if hips is not None:
    #         torso = hips - shoulders
    #         angle = abs(np.degrees(np.arctan2(torso[1], torso[0])))
    #         if angle < 20:  # near-horizontal body
    #             return "fall"
    # 
    # # ----------------------------------------------------
    # # 5) SITTING: hips below shoulders by large margin,
    # #    knees near horizontal knee angle
    # # ----------------------------------------------------
    # if hips is not None and knees is not None:
    #     if hips[1] > shoulders[1] + 120 and knees[1] > hips[1] - 40:
    #         return "sitting"

    return "none"


class KeypointFilter:
    def __init__(self, alpha_base=0.4, min_conf=0.25):
        """
        alpha_base: Grundglättung (0.0 = maximale Glättung, 1.0 = keine Glättung)
        min_conf: Keypoints unter dieser Konfidenz werden verworfen oder ignoriert
        """
        self.alpha_base = alpha_base
        self.min_conf = min_conf
        self.prev = None  # wird zu (17,2) oder (17,3)
        self.prev_vel = None  # Geschwindigkeit der Keypoints

    def smooth(self, kpts):
        """
        kpts: (17,2) oder (17,3)
        returns: robust geglättete Keypoints, gleicher Shape
        """

        kpts = np.array(kpts, dtype=float)

        # Erster Frame → keine Glättung möglich
        if self.prev is None:
            self.prev = kpts.copy()
            self.prev_vel = np.zeros_like(kpts)
            return kpts

        smoothed = self.prev.copy()

        for i in range(len(kpts)):
            kp = kpts[i]

            # Wenn Confidence existiert:
            conf = kp[2] if kp.shape[0] == 3 else 1.0

            # Wenn Conf zu gering → ignorieren und alten Wert behalten
            if conf < self.min_conf:
                continue

            # Geschwindigkeit berechnen
            vel = kp[:2] - self.prev[i][:2]

            # Adaptive alpha: je stärker die Bewegung, desto weniger wird geglättet
            vel_norm = np.linalg.norm(vel)
            alpha = min(1.0, self.alpha_base + vel_norm * 0.015)

            # Exponentielles Glätten
            smoothed[i, :2] = alpha * kp[:2] + (1 - alpha) * self.prev[i][:2]

            # Confidence separat glätten
            if kp.shape[0] == 3:
                smoothed[i, 2] = 0.6 * kp[2] + 0.4 * self.prev[i][2]

        self.prev = smoothed.copy()
        return smoothed


def FilterLowConfidenceKpts(kpts, kp_thresh=0.25):
    if len(kpts) == 0:
        return kpts

    k = kpts.copy()

    # für alle Personen, alle Keypoints
    low = k[:,2] < kp_thresh

    # setzt schlechte Kpts auf NaN ? verschwindet aus Smoothing & Classifier
    k[low, 0] = np.nan
    k[low, 1] = np.nan

    return k
