"""
check_observability.py — which parts of the pose can one range + bearing marker actually fix? (P3)
Builds the observability matrix from the EKF's own Jacobians along a real run.
Uses ground truth ONLY to score the filter, never in the EKF.
Run from the repo root:  python -m p3_estimation.check_observability
(or use the VS Code run button; the sys.path line below makes that work too)
"""
import os
import sys
import numpy as np

# let Python find the other project folders when this file is run directly
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import config
from p3_estimation.Estimation import measurement_jacobian, motion_jacobian, motion_model, wrap
from p4_simulation.episode import run_episode
from p4_simulation.tb3sim import TB3Sim
from p4_simulation.make_scene import scene_path

SEEDS = range(10)
WINDOW = 20                      # steps of readings stacked into one observability matrix (2 s)
RANK_TOL = 1e-9                  # singular values below this count as zero


def rotation_mode(pose, marker):
    """Change in the pose when it is swung around the marker by a small angle.
    Range, bearing and body-frame odometry are all unchanged by this motion."""
    return np.array([-(pose[1] - marker[1]), pose[0] - marker[0], 1.0])


def rank_and_smallest(O):
    values = np.linalg.svd(O, compute_uv=False)
    return int(np.sum(values > RANK_TOL)), values[2]


def time_varying_O(poses, commands, marker, heading_sensor=False):
    """Rows H_k Phi_k, where Phi_k is the product of F from the window's first step up to step k.
    This is the correct test: H and F change along the path."""
    rows = []
    Phi = np.eye(3)
    for k in range(len(poses)):
        H = measurement_jacobian(poses[k], marker)
        if heading_sensor:
            H = np.vstack([H, [0.0, 0.0, 1.0]])
        rows.append(H @ Phi)
        if k < len(commands):
            Phi = motion_jacobian(poses[k], commands[k]) @ Phi
    return np.vstack(rows)


def frozen_O(pose, command, marker):
    """[H; H F; H F^2] with H and F held at one step. Valid only if they don't change."""
    H = measurement_jacobian(pose, marker)
    F = motion_jacobian(pose, command)
    return np.vstack([H, H @ F, H @ F @ F])


def replay_path(start, commands):
    """Noise-free path from the commands, so F and the poses agree exactly."""
    poses = [np.array(start, float)]
    for u in commands:
        poses.append(motion_model(poses[-1], u))
    return np.array(poses)


sim = TB3Sim(xml=scene_path("default"), dt=config.DT, slip_std=0.0, enc_noise_std=0.0, seed=0)
MARKER = sim.marker_xy()

runs = [run_episode(seed=seed, viewer=False, camera_pov=False, verbose=False) for seed in SEEDS]
run = runs[0]
commands = run["cmd"]
path = replay_path(run["true"][0], commands)
moving = [k for k in range(len(commands) - WINDOW) if np.all(np.abs(commands[k:k + WINDOW, 0]) > 0.05)]

print(f"marker at {MARKER}, window {WINDOW} steps, seed 0 has {len(moving)} moving windows\n")

# ---- 1. robot stopped
pose = path[moving[0]]
stopped = np.zeros((WINDOW, 2))
print("Rank (3 = fully observable), smallest singular value in brackets")
rank, low = rank_and_smallest(time_varying_O(np.tile(pose, (WINDOW, 1)), stopped, MARKER))
print(f"stopped, range + bearing:                   rank {rank} ({low:.1e})")

# ---- 2. robot moving
results = {"frozen": [], "time": [], "estimate": [], "heading": []}
null_rows, null_F = [], []
for k in moving:
    p, c = path[k:k + WINDOW], commands[k:k + WINDOW - 1]
    results["frozen"].append(rank_and_smallest(frozen_O(path[k], commands[k], MARKER)))
    results["time"].append(rank_and_smallest(time_varying_O(p, c, MARKER)))
    results["estimate"].append(rank_and_smallest(time_varying_O(run["xh"][k:k + WINDOW], c, MARKER)))
    results["heading"].append(rank_and_smallest(time_varying_O(p, c, MARKER, heading_sensor=True)))
    n = rotation_mode(path[k], MARKER)
    null_rows.append(np.abs(measurement_jacobian(path[k], MARKER) @ n).max())
    null_F.append(np.abs(motion_jacobian(path[k], commands[k]) @ n - rotation_mode(path[k + 1], MARKER)).max())


def line(label, key):
    ranks = [r[0] for r in results[key]]
    lows = [r[1] for r in results[key]]
    print(f"{label:43s} rank {min(ranks)}-{max(ranks)} (median {np.median(lows):.1e})")


line("moving, frozen H and F (misleading test):", "frozen")
line("moving, H and F along the path:", "time")
line("moving, H and F at the EKF's estimates:", "estimate")
line("moving, if heading were also measured:", "heading")
print(f"\nswing around marker: largest |H n| {max(null_rows):.1e},  largest |F n_k - n_k+1| {max(null_F):.1e}")

# ---- 3. wheels only
no_update = run_episode(seed=0, viewer=False, camera_pov=False, verbose=False, use_ekf_update=False)
growth = np.sqrt(no_update["P"][[0, -1]][:, [0, 1, 2], [0, 1, 2]])
print(f"wheels only, no H rows:                     rank 0, sigma x {growth[0, 0] * 100:.1f} -> {growth[1, 0] * 100:.1f} cm, "
      f"y {growth[0, 1] * 100:.1f} -> {growth[1, 1] * 100:.1f} cm, heading "
      f"{np.degrees(growth[0, 2]):.1f} -> {np.degrees(growth[1, 2]):.1f} deg over {no_update['steps'] * config.DT:.0f} s")


# ---- 4. does the real filter behave as the rank says?
def swing_angle(pose):
    return np.arctan2(pose[..., 1] - MARKER[1], pose[..., 0] - MARKER[0])


def swing_std(P, n):
    """Std of the swing angle according to P. A camera update can't change this, because H n = 0
    means the update adds no information along n. Only the start belief and Qd set it."""
    return 1.0 / np.sqrt(n @ np.linalg.solve(P, n))


print("\nSwing error around the marker (estimate minus true) on each seed")
print("seed  swing start  swing first seen  swing end  heading end  range end  swing x range  final error  filter swing std start -> first seen -> end")
for seed, r in zip(SEEDS, runs):
    first = int(np.argmax(r["seen"]))
    swing = np.degrees(wrap(swing_angle(r["xh"]) - swing_angle(r["true"])))
    heading = np.degrees(wrap(r["xh"][:, 2] - r["true"][:, 2]))
    distance = np.hypot(*(r["true"][-1, :2] - MARKER))
    stds = [np.degrees(swing_std(r["P"][k], rotation_mode(r["xh"][k], MARKER))) for k in (0, first, -1)]
    print(f"{seed:4d}  {swing[0]:8.1f} deg  {swing[first]:12.1f} deg  {swing[-1]:5.1f} deg  {heading[-1]:7.1f} deg  "
          f"{distance:7.2f} m  {np.radians(abs(swing[-1])) * distance * 100:9.1f} cm  "
          f"{r['final_pos_err'] * 100:7.1f} cm  {stds[0]:.2f} -> {stds[1]:.2f} -> {stds[2]:.2f} deg")
