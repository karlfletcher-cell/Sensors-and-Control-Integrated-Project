"""
config.py — shared constants, one source of truth for the whole team.

Owner: shared (changes affect everyone — tell the team).
Used by: p4_simulation/make_scene.py (scene geometry), p4_simulation/tb3sim.py (robot/sensor
dims), main.py and the P1/P2/P3 packages (timing, map, tuning).
If you change a scene value here, re-run `python -m p4_simulation.make_scene`.
"""
import numpy as np

# ---- timing / process noise ----------------------------------------------
DT       = 0.1               # control period [s]; matches the Week 7/8 tutorials
SLIP_STD = 0.02              # per-wheel multiplicative slip (process noise) in the main run

# ---- TurtleBot3 Waffle (ROBOTIS spec) + sensors ---------------------------
WHEEL_R   = 0.033            # wheel radius [m]
WHEEL_SEP = 0.287            # distance between wheel centres [m]  (Burger: 0.160)
N_RAYS    = 72               # LiDAR rays (LDS-01 is 360 @ 1 deg; 72 keeps the sim fast)
CAM_W, CAM_H = 640, 480      # camera image size [px]

# ---- map ----------------------------------------------------------------------
ROBOT_START = (-2.0, -1.5, 0.0)   # x, y, yaw — the XML keyframe the sim resets to
DOCK_XY     = (2.6, 1.5)          # charging station position (marker on its face)
MARKER_SIZE = 0.20                # ArUco side length [m]
MARKER_ID   = 0                   # ArUco id (DICT_4X4_50)

# ---- mission (P1) -------------------------------------------------------------
WAYPOINTS = [(-1.5, -2.2), (0.5, -2.2), (2.2, -1.5), (2.2, 0.0), (2.0, 1.5)]   # last one: in front of the dock
V = 0.2                           # cruise speed [m/s]

# ---- LiDAR obstacle avoidance (P1) ---------------------------------------------
GOAL       = (2.0, 1.5)           # where the robot drives to by itself: just in front of the dock
GOAL_TOL   = 0.15                 # [m] goal counts as reached when the estimate is this close
SAFE_DIST  = 0.5                  # [m] anything closer than this blocks that direction
HALF_WIDTH = 0.25                 # [m] half the width of the path the robot needs (body 0.14 + gap)

# ---- estimator (P3) -------------------------------------------------------------
INIT_OFFSET = np.array([0.3, -0.2, np.deg2rad(10)])    # deliberate error added to the initial belief
P0 = np.diag([0.3**2, 0.3**2, np.deg2rad(15)**2])      # initial covariance
Qd = np.diag([0.002**2, 0.002**2, np.deg2rad(0.3)**2])   # process noise
Rn = np.diag([0.02**2, np.deg2rad(0.5)**2])      # measurement noise (range, bearing)
