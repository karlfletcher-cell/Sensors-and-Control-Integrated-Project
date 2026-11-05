"""
compare_qd.py — compare process noise (Qd) values over several seeds (P3).
Runs the same docking episode as main.py --headless for each Qd and seed.
Run from the repo root:  python -m p3_estimation.compare_qd
(or use the VS Code run button; the sys.path line below makes that work too)
"""
import os
import sys
import numpy as np

# let Python find the other project folders when this file is run directly
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from p4_simulation.episode import run_episode

SEEDS = range(10)

# label: (x and y noise per step in metres, heading noise per step in degrees)
QD_OPTIONS = {
    "20 mm, 1.5 deg (old guess)": (0.020, 1.5),
    "5 mm, 0.5 deg": (0.005, 0.5),
    "2 mm, 0.3 deg": (0.002, 0.3),
    "0.4 mm, 0.17 deg (measured)": (0.0004, 0.17),
}

results = {}
for label, (position_noise, heading_noise_deg) in QD_OPTIONS.items():
    Qd = np.diag([position_noise**2, position_noise**2, np.deg2rad(heading_noise_deg)**2])
    runs = []
    for seed in SEEDS:
        run = run_episode(seed=seed, viewer=False, camera_pov=False, Qd=Qd, verbose=False)
        runs.append((run["success"], run["final_pos_err"], run["rms_pos_err"],
                     run["in_2sigma"].min(), run["final_err_over_sigma"]))
    results[label] = np.array(runs, float)

for label, runs in results.items():
    print(f"{label:28s} docked {int(runs[:, 0].sum())}/{len(SEEDS)}   "
          f"final error {runs[:, 1].mean() * 100:5.2f} cm   "
          f"rms error {runs[:, 2].mean() * 100:5.2f} cm (std {runs[:, 2].std() * 100:.2f} cm)   "
          f"inside 2-sigma {runs[:, 3].mean() * 100:3.0f}%   "
          f"final error {runs[:, 4].mean():.2f} sigma")
