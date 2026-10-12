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
from config import V

# TODO (P1): LQR gain on the error model (Week 8 B.1 / Assessment 3), waypoint reference


def controller(xh, wp):
    """Placeholder: point-and-drive. P1 replaces this with LQR on the estimate."""
    dx, dy = wp[0] - xh[0], wp[1] - xh[1]
    eth = wrap(np.arctan2(dy, dx) - xh[2])
    return V, np.clip(2.0 * eth, -1.0, 1.0)


def dock_command(xh, marker):
    """At the dock: stop and turn to face the marker, using the estimate. Returns (v, w, docked)."""
    eth = wrap(np.arctan2(marker[1] - xh[1], marker[0] - xh[0]) - xh[2])
    v, w = 0.0, np.clip(2.0 * eth, -1.0, 1.0)
    docked = abs(eth) < 0.05
    return v, w, docked
