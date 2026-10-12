"""
main_loop.py — the integration skeleton (P4). Structure = Week 8 Part C run_landmark_lqg,
with the numpy plant replaced by MuJoCo and the synthetic measurement replaced by ArUco.

Fill the three TODO blocks with the team's code. Until they exist, the loop runs with
placeholders so the pipeline can be exercised end to end on day 1.
"""
import numpy as np, cv2
from p4_simulation.tb3sim import TB3Sim, wrap
from config import DT, SLIP_STD, MARKER_SIZE, WAYPOINTS, V, INIT_OFFSET, P0, Qd, Rn

sim = TB3Sim(dt=DT, slip_std=SLIP_STD)   # dt matches the Week 7/8 tutorials
MARKER = sim.marker_xy()                 # known map feature (allowed: the dock is on the map)
K_cam = sim.camera_intrinsics()
T_bc  = sim.T_base_cam()
detector = cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50))

# ---------------------------------------------------------------- P2: sensing
def measure(img):
    """Return (range, bearing) of the marker in the BASE frame, or None if not visible."""
    corners, ids, _ = detector.detectMarkers(cv2.cvtColor(img, cv2.COLOR_RGB2GRAY))
    if ids is None:
        return None
    s = MARKER_SIZE / 2
    obj = np.array([[-s, s, 0], [s, s, 0], [s, -s, 0], [-s, -s, 0]], np.float32)
    _, rvec, tvec = cv2.solvePnP(obj, corners[0][0], K_cam, None)
    p_cam = np.append(tvec.ravel(), 1.0)
    p_base = T_bc @ p_cam                               # camera frame -> base frame
    return np.array([np.hypot(p_base[0], p_base[1]), np.arctan2(p_base[1], p_base[0])])
    # TODO (P2): proper calibration of K instead of trusting the sim, noise characterisation -> R

# ---------------------------------------------------------------- P3: estimator
# TODO (P3): paste f_uni, F_uni, h_rb, H_rb, ekf_predict, ekf_update from Week 8 Part 0
# Qd, Rn: config.py

# ---------------------------------------------------------------- P1: controller
# TODO (P1): LQR gain on the error model (Week 8 B.1 / Assessment 3), waypoint reference
# WAYPOINTS, V: config.py

def controller(xh, wp):
    """Placeholder: point-and-drive. P1 replaces this with LQR on the estimate."""
    dx, dy = wp[0] - xh[0], wp[1] - xh[1]
    eth = wrap(np.arctan2(dy, dx) - xh[2])
    return V, np.clip(2.0 * eth, -1.0, 1.0)

# ---------------------------------------------------------------- the loop
sim.reset()
x_true0 = sim.true_pose()
xh = x_true0 + INIT_OFFSET                # deliberately wrong initial belief
P  = P0.copy()
log = dict(true=[], xh=[], P=[], seen=[])
wp_i = 0

docked = False
for k in range(800):
    # 1. control from the ESTIMATE
    at_last = wp_i == len(WAYPOINTS) - 1 and np.hypot(*(np.array(WAYPOINTS[wp_i]) - xh[:2])) < 0.15
    if at_last:                                  # dock: stop and face the marker, using the estimate
        eth = wrap(np.arctan2(MARKER[1] - xh[1], MARKER[0] - xh[0]) - xh[2])
        v, w = 0.0, np.clip(2.0 * eth, -1.0, 1.0)
        docked = abs(eth) < 0.05
    else:
        v, w = controller(xh, WAYPOINTS[wp_i])
    sim.step(v, w)

    # 2. odometry -> EKF predict   (placeholder: dead reckoning, no covariance)
    dl, dr = sim.odometry()
    ds, dth = (dl + dr) / 2, (dr - dl) / sim.WHEEL_SEP
    xh = np.array([xh[0] + ds * np.cos(xh[2] + dth / 2), xh[1] + ds * np.sin(xh[2] + dth / 2), wrap(xh[2] + dth)])
    # TODO (P3): xh, P = ekf_predict(xh, P, (v, w), f_uni, F_uni, Qd)

    # 3. camera -> measurement -> EKF update
    z = measure(sim.camera())
    if z is not None:
        pass
        # TODO (P3): xh, P, _, _ = ekf_update(xh, P, z, lambda s: h_rb(s, MARKER), lambda s: H_rb(s, MARKER), Rn,
        #                                     residual=lambda y, yh: np.array([y[0]-yh[0], wrap(y[1]-yh[1])]))

    # 4. waypoint switching
    if np.hypot(*(np.array(WAYPOINTS[wp_i]) - xh[:2])) < 0.15 and wp_i < len(WAYPOINTS) - 1:
        wp_i += 1

    # 5. logging — ground truth is read HERE and nowhere else
    log["true"].append(sim.true_pose()); log["xh"].append(xh.copy()); log["P"].append(P.copy()); log["seen"].append(z is not None)
    if docked:
        print(f"docked at step {k}: marker measured at range/bearing {np.round(z,3) if z is not None else 'not in view'}")
        break

T, XH = np.array(log["true"]), np.array(log["xh"])
err = np.linalg.norm(T[:, :2] - XH[:, :2], axis=1)
print(f"final true pose {np.round(T[-1],3)}  estimate {np.round(XH[-1],3)}")
print(f"position error: mean {err.mean():.3f} m, final {err[-1]:.3f} m; marker seen in {np.mean(log['seen'])*100:.0f}% of steps")
np.savez("run_log.npz", **{k: np.array(v) for k, v in log.items()})
