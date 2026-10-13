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

# TODO (P1): LQR gain on the error model (Week 8 B.1 / Assessment 3), waypoint reference


def controller(xh, wp):
    """Placeholder: point-and-drive. P1 replaces this with LQR on the estimate."""
    dx, dy = wp[0] - xh[0], wp[1] - xh[1]
    eth = wrap(np.arctan2(dy, dx) - xh[2])
    return V, np.clip(2.0 * eth, -1.0, 1.0)


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
    """Drive toward the goal by itself, using the LiDAR to steer around obstacles.

    xh     : pose ESTIMATE (x, y, yaw)
    goal   : (x, y) point to drive to
    scan   : LiDAR ranges [m], one per ray; -1 means the ray hit nothing
    angles : direction of each ray in the robot frame [rad] (0 = straight ahead)
    Returns (v, w).
    """
    # Step 1: where is the goal, compared with where the robot is facing?
    dx = goal[0] - xh[0]
    dy = goal[1] - xh[1]
    goal_dist = np.hypot(dx, dy)
    goal_angle = wrap(np.arctan2(dy, dx) - xh[2])

    # Only look SAFE_DIST ahead, and never past the goal
    # (so the dock behind the goal doesn't count as an obstacle).
    look_ahead = min(SAFE_DIST, goal_dist)

    # Step 2: try every direction in front of the robot (one per LiDAR ray) and keep
    # the clear one that points closest to the goal.
    best_angle = None
    for i in range(len(angles)):
        direction = wrap(angles[i])
        if abs(direction) > np.pi / 2:
            continue                      # only consider directions in front of the robot
        if not path_is_clear(direction, scan, angles, look_ahead):
            continue                      # something is in the way
        if best_angle is None or abs(direction - goal_angle) < abs(best_angle - goal_angle):
            best_angle = direction

    # Step 3: turn toward the chosen direction.
    if best_angle is None:
        return 0.0, 0.8                   # boxed in: spin on the spot until a gap appears

    w = np.clip(2.0 * best_angle, -1.0, 1.0)
    if abs(best_angle) < np.deg2rad(30):
        v = V                             # roughly facing the right way: full speed
    else:
        v = V / 4                         # big turn needed: slow down while turning
    return v, w


def dock_command(xh, marker):
    """At the dock: stop and turn to face the marker, using the estimate. Returns (v, w, docked)."""
    eth = wrap(np.arctan2(marker[1] - xh[1], marker[0] - xh[0]) - xh[2])
    v, w = 0.0, np.clip(2.0 * eth, -1.0, 1.0)
    docked = abs(eth) < 0.05
    return v, w, docked
