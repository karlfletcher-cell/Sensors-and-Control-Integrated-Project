"""
sensing.py: ArUco marker detection -> (range, bearing) in the base frame. Owner: P2.

Input: RGB image, camera matrix K, camera pose in the base frame T_bc.
Output: np.array([range, bearing]), or None if marker 0 is not visible.
Never reads ground truth. Calibration values come from camera_calib.json, made by Calibrate.py.
Adapted from Week 3 Part E and Week 2 Part C. Calls OpenCV directly to use sub-pixel corners.
"""
import json
import os

import cv2
import numpy as np

from config import MARKER_ID, MARKER_SIZE

# Calibration results saved by calibrate.py
CALIB_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "camera_calib.json")

# Sub-pixel corners reduce the range bias. The other two settings let the refinement run longer.
_params = cv2.aruco.DetectorParameters()
_params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
_params.cornerRefinementMaxIterations = 100
_params.cornerRefinementMinAccuracy = 0.001
detector = cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50), _params)

# Week 3 Part E: marker corners in the marker frame
# (top-left, top-right, bottom-right, bottom-left). SOLVEPNP_IPPE_SQUARE needs this order.
s = MARKER_SIZE / 2
MARKER_CORNERS = np.array([[-s, s, 0], [s, s, 0], [s, -s, 0], [-s, -s, 0]], np.float32)


def _load_calibration():
    """Read the saved calibration, or return an empty dict if there isn't one yet."""
    if os.path.exists(CALIB_FILE):
        with open(CALIB_FILE) as f:
            return json.load(f)
    return {}


# Loaded once on import. With no calibration file, the range correction is 0.
_CALIB = _load_calibration()
RANGE_BIAS_A = float(_CALIB.get("range_bias_a", 0.0))      # range bias model: b(r) = a * r^2


def load_K(default_K):
    """Calibrated K from camera_calib.json, or default_K if calibrate.py has not been run."""
    return np.array(_CALIB["K"], float) if "K" in _CALIB else default_K


def measure(img, K, T_bc):
    """Return (range, bearing) of marker MARKER_ID in the BASE frame, or None if not visible."""
    # Find markers in the greyscale image
    corners, ids, _ = detector.detectMarkers(cv2.cvtColor(img, cv2.COLOR_RGB2GRAY))
    if ids is None:
        return None

    # Keep only the dock marker. A misread ID would otherwise give a large range error.
    hit = np.where(ids.ravel() == MARKER_ID)[0]
    if len(hit) == 0:
        return None

    # Week 3 Part E: PnP with n = 4 gives the marker centre in the camera frame (tvec)
    ok, _, tvec = cv2.solvePnP(MARKER_CORNERS, corners[hit[0]][0], K, None,
                               flags=cv2.SOLVEPNP_IPPE_SQUARE)
    if not ok:
        return None

    # Week 2 Part C: frame chain, p_B = T_BC * p_C
    p_base = T_bc @ np.append(tvec.ravel(), 1.0)

    # Week 2 Part C: range and bearing, with the calibrated bias removed (Week 2 Part B.3)
    rho = np.hypot(p_base[0], p_base[1])
    alpha = np.arctan2(p_base[1], p_base[0])
    return np.array([rho - RANGE_BIAS_A * rho ** 2, alpha])