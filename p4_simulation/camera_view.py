"""
camera_view.py — the robot's camera point of view with the ArUco detection drawn on it (owner: P4).

Shown live next to the 3D viewer so the demo video proves the docking is visual, and optionally
recorded to an .mp4. Display only: the pipeline's measurement still comes from p2_sensing.measure().
"""
import numpy as np, cv2
from config import MARKER_SIZE

_detector = cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50))


def overlay(img_rgb, z, state, K, step):
    frame = cv2.cvtColor(img_rgb, cv2.COLOR_RGB2BGR)
    corners, ids, _ = _detector.detectMarkers(cv2.cvtColor(img_rgb, cv2.COLOR_RGB2GRAY))
    if ids is not None:
        cv2.aruco.drawDetectedMarkers(frame, corners, ids)
        s = MARKER_SIZE / 2
        obj = np.array([[-s, s, 0], [s, s, 0], [s, -s, 0], [-s, -s, 0]], np.float32)
        ok, rvec, tvec = cv2.solvePnP(obj, corners[0][0], K, None)
        if ok:
            cv2.drawFrameAxes(frame, K, None, rvec, tvec, MARKER_SIZE / 2)
    h = frame.shape[0]
    cv2.rectangle(frame, (0, h - 62), (frame.shape[1], h), (0, 0, 0), -1)
    cv2.putText(frame, f"step {step}   state: {state}", (8, h - 40), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
    if z is not None:
        txt, col = f"marker: range {z[0]:.2f} m   bearing {np.degrees(z[1]):+.1f} deg   -> EKF update", (80, 255, 80)
    else:
        txt, col = "marker not in view -> dead reckoning (EKF predict only)", (80, 80, 255)
    cv2.putText(frame, txt, (8, h - 14), cv2.FONT_HERSHEY_SIMPLEX, 0.6, col, 1)
    return frame


class CameraPOV:
    def __init__(self, show=True, record=None, fps=10, size=(640, 480)):
        self.show, self.writer = show, None
        if record:
            self.writer = cv2.VideoWriter(record, cv2.VideoWriter_fourcc(*"mp4v"), fps, size)

    def update(self, img_rgb, z, state, K, step):
        frame = overlay(img_rgb, z, state, K, step)
        if self.writer is not None:
            self.writer.write(frame)
        if self.show:
            cv2.imshow("camera POV (what the EKF update sees)", frame); cv2.waitKey(1)
        return frame

    def close(self):
        if self.writer is not None:
            self.writer.release()
        if self.show:
            cv2.destroyAllWindows()
