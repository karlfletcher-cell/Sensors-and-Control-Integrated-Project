"""
plot_camera_noise.py — range error of every camera reading during real runs (P3).
Shows why Rn uses 6 cm: most readings are close, but some are well off.
Uses ground truth ONLY to measure the camera's error (evaluation), never in the EKF.
Run from the repo root:  python -m p3_estimation.plot_camera_noise
Output: analysis/fig_camera_noise.png
"""
import os
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import p4_simulation.episode as episode
from p4_simulation.tb3sim import TB3Sim

SEEDS = range(30)
CHOSEN_RANGE_NOISE_CM = 6

# Record the true range each time the camera takes a picture...
latest_true_range = {}
original_camera = TB3Sim.camera

def camera_that_records_truth(sim):
    latest_true_range["value"] = sim.true_range_bearing()[0]
    return original_camera(sim)

TB3Sim.camera = camera_that_records_truth

# ...and compare it with what measure() reports for that picture.
true_ranges, range_errors = [], []
original_measure = episode.measure

def measure_that_records_error(*args):
    measurement = original_measure(*args)
    if measurement is not None:
        true_range = latest_true_range["value"]
        true_ranges.append(true_range)
        range_errors.append(measurement[0] - true_range)
    return measurement

episode.measure = measure_that_records_error

for seed in SEEDS:
    episode.run_episode(seed=seed, viewer=False, camera_pov=False, verbose=False)

true_ranges = np.array(true_ranges)
range_errors_cm = np.array(range_errors) * 100

print(f"{len(range_errors_cm)} readings from {len(SEEDS)} runs")
print(f"range error std {range_errors_cm.std():.2f} cm, "
      f"over 5 cm off: {np.mean(np.abs(range_errors_cm) > 5) * 100:.1f}%, "
      f"largest {np.abs(range_errors_cm).max():.1f} cm")

os.makedirs("analysis", exist_ok=True)
figure, axes = plt.subplots(figsize=(12, 6))
axes.axhspan(-CHOSEN_RANGE_NOISE_CM, CHOSEN_RANGE_NOISE_CM, color="green", alpha=0.12,
             label=f"chosen range noise: {CHOSEN_RANGE_NOISE_CM} cm")
axes.axhline(0, color="grey", linewidth=1)
axes.scatter(true_ranges, range_errors_cm, s=6, color="black", alpha=0.35)
axes.set_xlabel("distance to marker (m)")
axes.set_ylabel("range error (cm)")
axes.set_title(f"Camera range error during real runs ({len(range_errors_cm)} readings, {len(SEEDS)} runs)")
axes.legend(loc="upper left")
figure.tight_layout()
figure.savefig("analysis/fig_camera_noise.png", dpi=150)
print("saved analysis/fig_camera_noise.png")
