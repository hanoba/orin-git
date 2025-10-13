import numpy as np

class GestureClassifier:
    """
    Simple geometric hand gesture classifier.
    Detects:
      - 'open_hand'
      - 'fist'
      - 'thumb_up'
      - 'thumb_down'
      - 'unknown'
    """

    def __init__(self):
        # indices for fingertip keypoints in the hand pose model (for trt_pose_hand)
        self.fingertip_ids = [4, 8, 12, 16, 20]  # thumb, index, middle, ring, pinky
        self.wrist_id = 0

    def classify(self, joints):
        """
        Args:
            joints (list[list[int]]): list of (x, y) coordinates for 21 keypoints
        Returns:
            str: gesture label
        """
        joints = np.array(joints)

        # check if hand was detected
        if joints.shape[0] < 21 or np.all(joints == 0):
            return "unknown"

        wrist = joints[self.wrist_id]

        # Compute distances of each fingertip from the wrist
        dists = [np.linalg.norm(joints[i] - wrist) for i in self.fingertip_ids]

        # Normalize distances
        dists = np.array(dists)
        dists /= np.max(dists) + 1e-6

        # Measure thumb orientation (up or down)
        thumb_tip = joints[4]
        thumb_base = joints[2]
        vertical_dir = thumb_tip[1] - thumb_base[1]  # y increases downward in OpenCV

        # --- Simple rules ---
        if np.mean(dists[1:]) > 0.7:
            gesture = "open_hand"
        elif np.mean(dists[1:]) < 0.4:
            gesture = "fist"
        elif vertical_dir < -10:  # thumb pointing up
            gesture = "thumb_up"
        elif vertical_dir > 10:   # thumb pointing down
            gesture = "thumb_down"
        else:
            gesture = "unknown"

        return gesture
