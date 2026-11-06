"""
compare_rn.py — compare measurement noise (Rn) values over several seeds (P3).
Runs the same docking episode as main.py --headless for each Rn and seed.
Run from the repo root:  python -m p3_estimation.compare_rn
(or use the VS Code run button; the sys.path line below makes that work too)
"""
import os
import sys
import numpy as np

# let Python find the other project folders when this file is run directly
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from p4_simulation.episode import run_episode

SEEDS = range(10)

# label: (range noise in metres, bearing noise in degrees)
RN_OPTIONS = {
    "10 cm, 3 deg (old guess)": (0.10, 3.0),
    "6 cm, 0.5 deg (config.py)": (0.06, 0.5),
    "2 cm, 0.5 deg": (0.02, 0.5),
    "1 cm, 0.5 deg": (0.01, 0.5),
}

results = {}
for label, (range_noise, bearing_noise_deg) in RN_OPTIONS.items():
    Rn = np.diag([range_noise**2, np.deg2rad(bearing_noise_deg)**2])
    runs = []
    for seed in SEEDS:
        run = run_episode(seed=seed, viewer=False, camera_pov=False, Rn=Rn, verbose=False)
        runs.append((run["success"], run["final_pos_err"], run["rms_pos_err"]))
    results[label] = np.array(runs, float)

baseline_label = next(iter(RN_OPTIONS))
baseline_rms = results[baseline_label][:, 2]

for label, runs in results.items():
    print(f"{label:27s} success {int(runs[:, 0].sum())}/{len(SEEDS)}   "
          f"final error {runs[:, 1].mean() * 100:.2f} cm   "
          f"rms error {runs[:, 2].mean() * 100:.2f} cm (std {runs[:, 2].std() * 100:.2f} cm)   "
          f"rms lower than old guess on {int((runs[:, 2] < baseline_rms).sum())} seeds")
