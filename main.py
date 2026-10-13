# =============================================================================
# IMPORTS
# =============================================================================
import time
import numpy as np
import mujoco.viewer
from p4_simulation.tb3sim import TB3Sim
from p1_control.controller import avoid_controller, dock_command
from p2_sensing.sensing import measure
from p3_estimation.Estimation import (odometry_to_input, motion_model, motion_jacobian,
                                     predict_range_bearing, measurement_jacobian,
                                     range_bearing_residual, ekf_predict, ekf_update)
from config import DT, SLIP_STD, WHEEL_SEP, GOAL, GOAL_TOL, ROBOT_START, INIT_OFFSET, P0, Qd, Rn


# =============================================================================
# SETTINGS
# =============================================================================
MAX_STEPS = 800              # give up after this many control steps


# =============================================================================
# HELPERS
# =============================================================================
def distance_to(point, xh):
    """Straight-line distance [m] from the estimated position to a 2D point."""
    return np.hypot(*(np.array(point) - xh[:2]))


# =============================================================================
# SIMULATOR SETUP
# Create the sim and read the fixed values the other modules need.
# =============================================================================
sim = TB3Sim(dt=DT, slip_std=SLIP_STD)
MARKER = sim.marker_xy()          # marker (landmark) position in the world
K_cam = sim.camera_intrinsics()   # camera intrinsic matrix
T_bc  = sim.T_base_cam()          # transform from robot base to camera


# =============================================================================
# INITIAL STATE
# The estimate starts deliberately wrong so the EKF has something to correct.
# =============================================================================
sim.reset()
xh = np.array(ROBOT_START) + INIT_OFFSET   # estimated pose [x, y, theta]: known start + deliberate error
P  = P0.copy()                    # estimate covariance
docked = False

log = dict(true=[], xh=[], P=[], seen=[])
viewer = mujoco.viewer.launch_passive(sim.model, sim.data)


# =============================================================================
# MAIN LOOP
# =============================================================================
for k in range(MAX_STEPS):

    # -------------------------------------------------------------------------
    # 1. CONTROL - choose (v, w) from the ESTIMATE and the LiDAR, then step the sim
    #    The robot picks its own path to the goal, steering around obstacles.
    #    At the goal, switch to docking: stop and face the marker.
    # -------------------------------------------------------------------------
    if distance_to(GOAL, xh) < GOAL_TOL:
        v, w, docked = dock_command(xh, MARKER)
    else:
        v, w = avoid_controller(xh, GOAL, sim.lidar(), sim.lidar_angles())
    sim.step(v, w)

    # -------------------------------------------------------------------------
    # 2. PREDICT - EKF predict, using wheel odometry as the input u = [v, w]
    # -------------------------------------------------------------------------
    dl, dr = sim.odometry()           # left/right wheel travel this step [m]
    u = odometry_to_input(dl, dr, WHEEL_SEP)
    xh, P = ekf_predict(xh, P, u, motion_model, motion_jacobian, Qd)

    # -------------------------------------------------------------------------
    # 3. SENSE + UPDATE - camera -> range/bearing to marker -> EKF update
    # -------------------------------------------------------------------------
    z = measure(sim.camera(), K_cam, T_bc)   # [range, bearing], or None if marker not in view
    marker_seen = z is not None
    if marker_seen:
        xh, P, _, _ = ekf_update(xh, P, z,
                                 lambda s: predict_range_bearing(s, MARKER),
                                 lambda s: measurement_jacobian(s, MARKER),
                                 Rn, residual=range_bearing_residual)

    # -------------------------------------------------------------------------
    # 4. LOGGING - ground truth is recorded here for evaluation only
    # -------------------------------------------------------------------------
    log["true"].append(sim.true_pose())   # EVALUATION ONLY
    log["xh"].append(xh.copy())
    log["P"].append(P.copy())
    log["seen"].append(marker_seen)

    # -------------------------------------------------------------------------
    # 5. VIEWER - redraw the 3D window at real time; stop if it is closed
    # -------------------------------------------------------------------------
    viewer.sync()
    time.sleep(DT)
    if not viewer.is_running():
        break

    # -------------------------------------------------------------------------
    # 6. STOP CONDITION - finish once docked at the marker
    # -------------------------------------------------------------------------
    if docked:
        z_text = np.round(z, 3) if marker_seen else "not in view"
        print(f"docked at step {k}: marker measured at range/bearing {z_text}")
        break


# =============================================================================
# RESULTS - compare the estimate with ground truth and save the log
# =============================================================================
true_poses = np.array(log["true"])
est_poses = np.array(log["xh"])
pos_err = np.linalg.norm(true_poses[:, :2] - est_poses[:, :2], axis=1)   # [m] each step
seen_pct = np.mean(log["seen"]) * 100

print(f"final true pose {np.round(true_poses[-1], 3)}  estimate {np.round(est_poses[-1], 3)}")
print(f"position error: mean {pos_err.mean():.3f} m, final {pos_err[-1]:.3f} m; "
      f"marker seen in {seen_pct:.0f}% of steps")
np.savez("run_log.npz", **{name: np.array(values) for name, values in log.items()})


# =============================================================================
# KEEP VIEWER OPEN - wait until the user closes the window
# =============================================================================
while viewer.is_running():
    viewer.sync()
    time.sleep(0.05)
viewer.close()
