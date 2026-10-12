"""
sensing.py — ArUco marker detection -> (range, bearing) measurement in the base frame.

Owner: P2.
Inputs: an RGB image (H, W, 3) uint8 from sim.camera(), the camera intrinsics K (3x3) and
the camera pose in the base frame T_bc (4x4, OpenCV camera convention) — both passed in.
Outputs: np.array([range, bearing]) of the marker in the BASE frame, or None if not visible.
Never reads ground truth.
"""
import numpy as np, cv2
from config import MARKER_SIZE

detector = cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50))


def measure(img, K, T_bc):
    """Return (range, bearing) of the marker in the BASE frame, or None if not visible."""
    corners, ids, _ = detector.detectMarkers(cv2.cvtColor(img, cv2.COLOR_RGB2GRAY))
    if ids is None:
        return None
    s = MARKER_SIZE / 2
    obj = np.array([[-s, s, 0], [s, s, 0], [s, -s, 0], [-s, -s, 0]], np.float32)
    _, rvec, tvec = cv2.solvePnP(obj, corners[0][0], K, None)
    p_cam = np.append(tvec.ravel(), 1.0)
    p_base = T_bc @ p_cam                               # camera frame -> base frame
    return np.array([np.hypot(p_base[0], p_base[1]), np.arctan2(p_base[1], p_base[0])])
    # TODO (P2): proper calibration of K instead of trusting the sim, noise characterisation -> R
