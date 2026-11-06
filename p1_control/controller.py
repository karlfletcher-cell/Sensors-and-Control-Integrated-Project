"""
controller.py — waypoint-following controller and the dock "face the marker" command.

Owner: P1.
Inputs: the pose ESTIMATE xh = (x, y, yaw) from the estimator, the current waypoint wp = (x, y)
or the known marker map position (x, y). Never ground truth.
Outputs: body velocity command (v [m/s], w [rad/s]) for sim.step(); dock_command also
returns whether the robot is facing the marker (docked).
"""
import numpy as np
from p4_simulation.tb3sim import wrap
from config import V, SAFE_DIST, HALF_WIDTH
import control as ct

# LQR controller
Ts = 0.1  # Must match the simulation control period

Ad = np.array([
    [1, V * Ts],
    [0, 1]
])

Bd = np.array([
    [0.5 * V * Ts**2],
    [Ts]
])

Q_lqr = np.diag([
    1 / 0.2**2,
    1 / np.deg2rad(15)**2
])

R_lqr = np.array([[1 / 1.0**2]])

K_lqr, _, _ = ct.dlqr(Ad, Bd, Q_lqr, R_lqr)

print("LQR gain:", K_lqr)

def controller(xh, wp, start=None):

    wp = np.array(wp, dtype=float)

    # Save the start of a new path when no start is provided
    if start is None:
        if not hasattr(controller, "last_wp") or not np.array_equal(wp, controller.last_wp):
            controller.last_wp = wp.copy()
            controller.path_start = np.array(xh[:2], dtype=float).copy()

        start = controller.path_start

    dx = wp[0] - start[0]
    dy = wp[1] - start[1]

    theta_ref = np.arctan2(dy, dx)

    # Lateral and heading errors
    e_y = -(xh[0] - start[0]) * np.sin(theta_ref) + (xh[1] - start[1]) * np.cos(theta_ref)
    e_theta = wrap(xh[2] - theta_ref)

    # Rotate on the spot for large heading errors
    if abs(e_theta) > np.deg2rad(45):
        w = np.clip(-2.0 * e_theta, -1.0, 1.0)
        return 0.0, float(w)

    # LQR state feedback
    e = np.array([e_y, e_theta])
    w = -(K_lqr @ e).item()

    w = np.clip(w, -1.0, 1.0)

    return V, float(w)


def path_is_clear(direction, scan, angles, look_ahead):
    """True if the robot could drive straight in `direction` for `look_ahead` metres
    without its body touching any point the LiDAR has hit.

    Think of the path as a strip of floor HALF_WIDTH wide on each side. For every
    LiDAR hit we work out how far it is ahead along the path and how far it is to
    the side of it. If any hit is inside the strip, the path is blocked.
    """
    for i in range(len(scan)):
        if scan[i] < 0:
            continue                      # this ray hit nothing

        angle_from_path = angles[i] - direction
        ahead = scan[i] * np.cos(angle_from_path)   # distance along the path
        side = scan[i] * np.sin(angle_from_path)    # distance to the side of the path

        if 0 < ahead < look_ahead and abs(side) < HALF_WIDTH:
            return False
    return True

def avoid_controller(xh, goal, scan, angles):

    dx = goal[0] - xh[0]
    dy = goal[1] - xh[1]

    goal_dist = np.hypot(dx, dy)
    goal_angle = wrap(np.arctan2(dy, dx) - xh[2])

    if goal_dist < 0.15:
        return 0.0, 0.0

    look_ahead = min(SAFE_DIST, goal_dist)

    # If the goal is behind the robot, turn around
    if abs(goal_angle) > np.pi / 2:
        return 0.0, float(np.clip(2.0 * goal_angle, -1.0, 1.0))

    # Check whether the previous temporary path is still safe
    ref_wp = getattr(avoid_controller, "ref_wp", None)
    ref_start = getattr(avoid_controller, "ref_start", None)

    if ref_wp is not None:
        dx_ref = ref_wp[0] - xh[0]
        dy_ref = ref_wp[1] - xh[1]

        distance_ref = np.hypot(dx_ref, dy_ref)
        direction_ref = wrap(np.arctan2(dy_ref, dx_ref) - xh[2])

        if distance_ref < 0.18 or not path_is_clear(direction_ref, scan, angles, distance_ref):
            ref_wp = None

    # Find a new path if necessary
    if ref_wp is None:

        best_angle = None
        best_error = np.inf

        for direction in wrap(angles):

            if abs(direction) > np.pi / 2:
                continue

            if not path_is_clear(direction, scan, angles, look_ahead):
                continue

            error = abs(wrap(direction - goal_angle))

            if error < best_error:
                best_error = error
                best_angle = direction

        if best_angle is None:
            avoid_controller.ref_wp = None
            return 0.0, 0.8

        # Convert the selected LiDAR direction to a world-frame waypoint
        heading = xh[2] + best_angle

        ref_start = np.array(xh[:2], dtype=float).copy()

        ref_wp = ref_start + look_ahead * np.array([
            np.cos(heading),
            np.sin(heading)
        ])

        avoid_controller.ref_wp = ref_wp
        avoid_controller.ref_start = ref_start

    # Follow the selected path using LQR
    v, w = controller(xh, ref_wp, ref_start)

    return v, w


def dock_command(xh, marker):
    """At the dock: stop and turn to face the marker, using the estimate. Returns (v, w, docked)."""
    eth = wrap(np.arctan2(marker[1] - xh[1], marker[0] - xh[0]) - xh[2])
    v, w = 0.0, np.clip(2.0 * eth, -1.0, 1.0)
    docked = abs(eth) < 0.05
    return v, w, docked
