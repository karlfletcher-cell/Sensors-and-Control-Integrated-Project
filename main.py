"""
main.py — the integration skeleton (P4). Structure = Week 8 Part C run_landmark_lqg,
with the numpy plant replaced by MuJoCo and the synthetic measurement replaced by ArUco.

Fill the three TODO blocks with the team's code. Until they exist, the loop runs with
placeholders so the pipeline can be exercised end to end on day 1.
"""
import numpy as np
from p4_simulation.tb3sim import TB3Sim, wrap
from p1_control.controller import controller, dock_command
from p2_sensing.sensing import measure
from config import DT, SLIP_STD, WAYPOINTS, INIT_OFFSET, P0, Qd, Rn

sim = TB3Sim(dt=DT, slip_std=SLIP_STD)   # dt matches the Week 7/8 tutorials
MARKER = sim.marker_xy()                 # known map feature (allowed: the dock is on the map)
K_cam = sim.camera_intrinsics()
T_bc  = sim.T_base_cam()

# ---------------------------------------------------------------- P2: sensing
# measure(img, K, T_bc): p2_sensing/sensing.py

# ---------------------------------------------------------------- P3: estimator
# TODO (P3): paste f_uni, F_uni, h_rb, H_rb, ekf_predict, ekf_update from Week 8 Part 0
# Qd, Rn: config.py

# ---------------------------------------------------------------- P1: controller
# controller(xh, wp), dock_command(xh, marker): p1_control/controller.py
# WAYPOINTS, V: config.py

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
        v, w, docked = dock_command(xh, MARKER)
    else:
        v, w = controller(xh, WAYPOINTS[wp_i])
    sim.step(v, w)

    # 2. odometry -> EKF predict   (placeholder: dead reckoning, no covariance)
    dl, dr = sim.odometry()
    ds, dth = (dl + dr) / 2, (dr - dl) / sim.WHEEL_SEP
    xh = np.array([xh[0] + ds * np.cos(xh[2] + dth / 2), xh[1] + ds * np.sin(xh[2] + dth / 2), wrap(xh[2] + dth)])
    # TODO (P3): xh, P = ekf_predict(xh, P, (v, w), f_uni, F_uni, Qd)

    # 3. camera -> measurement -> EKF update
    z = measure(sim.camera(), K_cam, T_bc)
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
