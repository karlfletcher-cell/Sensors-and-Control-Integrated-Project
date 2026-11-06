import os
import gc
import csv
import numpy as np
import matplotlib.pyplot as plt

import p1_control.controller as p1
from p4_simulation.episode import run_episode

os.makedirs("analysis", exist_ok=True)

#different LQR weights
weights = {
    "Baseline": (25, 14.59, 1),
    "High Q": (50, 29.18, 1),
    "Low Q": (12.5, 7.295, 1),
    "High R": (25, 14.59, 5),
    "Low R": (25, 14.59, 0.2)
}

SEEDS = range(3)
results = []

for name, (q_y, q_theta, r) in weights.items():
    print(f"\nTesting {name}")

    Q = np.diag([q_y, q_theta])
    R = np.array([[r]])

    p1.set_lqr_weights(Q, R)

    for seed in SEEDS:
        #clear previous experiment's data
        for values in p1.tracking_log.values():
            values.clear()

        #reset the previous navigation reference
        for attr in ("ref_wp", "ref_start"):
            if hasattr(p1.avoid_controller, attr):
                delattr(p1.avoid_controller, attr)

        #run the same conditions for each weight
        run = run_episode(
            seed=seed,
            viewer=False,
            camera_pov=False,
            verbose=False
        )

        ey = np.array(p1.tracking_log["lateral_error"])
        eth = np.array(p1.tracking_log["heading_error"])
        w = np.array(p1.tracking_log["angular_velocity"])

        if len(ey) == 0:
            print(f"Seed {seed}: no LQR samples")
            continue

        results.append({
            "name": name,
            "seed": seed,
            "rms_lateral": np.sqrt(np.mean(ey**2)),
            "rms_heading_deg": np.degrees(
                np.sqrt(np.mean(eth**2))
            ),
            "rms_steering": np.sqrt(np.mean(w**2)),
            "saturation_pct": 100 * np.mean(
                np.abs(w) >= 1.0 - 1e-6
            ),
            "success": int(run["success"])
        })

        print(
            f"Seed {seed}: "
            f"lateral={results[-1]['rms_lateral']:.3f} m, "
            f"steering={results[-1]['rms_steering']:.3f} rad/s, "
            f"success={run['success']}"
        )

        del run
        gc.collect()

#save numerical results
with open("analysis/lqr_sweep.csv", "w", newline="") as file:
    writer = csv.DictWriter(file, fieldnames=results[0].keys())
    writer.writeheader()
    writer.writerows(results)

#plot mean tracking error vs steering effort
plt.figure(figsize=(9, 6))

for name in weights:
    rows = [r for r in results if r["name"] == name]

    if not rows:
        continue

    mean_error = np.mean([r["rms_lateral"] for r in rows])
    mean_effort = np.mean([r["rms_steering"] for r in rows])

    plt.scatter(mean_effort, mean_error, s=80)
    plt.annotate(
        name,
        (mean_effort, mean_error),
        xytext=(6, 6),
        textcoords="offset points"
    )

plt.xlabel("RMS Angular Velocity (rad/s)")
plt.ylabel("RMS Lateral Error (m)")
plt.title("LQR Weight Sweep")
plt.grid(True)
plt.tight_layout()
plt.savefig("analysis/lqr_weight_sweep.png", dpi=150)
plt.show()

#restore original LQR gain
p1.set_lqr_weights()
print("\nSaved results to analysis/")
