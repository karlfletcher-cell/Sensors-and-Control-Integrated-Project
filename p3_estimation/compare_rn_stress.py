"""compare_rn_stress.py — compare Rn values on 30 nominal seeds and on the stress conditions (P3).
Run from the repo root:  python -m p3_estimation.compare_rn_stress"""
import os
import sys

os.environ["OMP_NUM_THREADS"] = "1"
# let Python find the other project folders when this file is run directly
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from concurrent.futures import ProcessPoolExecutor

import numpy as np

deg = np.deg2rad
LIMIT = 7.815
CANDIDATES = {
    "2 cm, 0.5 deg (config)": (0.02, 0.5),
    "4 cm, 0.5 deg": (0.04, 0.5),
    "6 cm, 0.5 deg": (0.06, 0.5),
    "6.3 cm, 1.58 deg (x10)": (0.02 * np.sqrt(10), 0.5 * np.sqrt(10)),
}
CONDITIONS = {
    "nominal": {},
    "start_B": dict(start=(-2.5, 0.5, deg(-30))),
    "init_err_large": dict(init_offset=(0.6, -0.5, deg(25))),
    "slip_0.05": dict(slip_std=0.05),
    "slip_0.10": dict(slip_std=0.10),
    "push_mid_run": dict(push_at=150, push=(0.25, -0.25, deg(15))),
    "push_large": dict(push_at=150, push=(0.6, 0.4, deg(40))),
    "encoder_noise": dict(enc_noise_std=0.002),
}


def job(item):
    import config
    from p4_simulation.episode import run_episode
    from p4_simulation.tb3sim import wrap

    label, condition, seed = item
    r_std, b_std = CANDIDATES[label]
    Rn = np.diag([r_std**2, deg(b_std)**2])
    r = run_episode(seed=seed, viewer=False, camera_pov=False, verbose=False, Rn=Rn, **CONDITIONS[condition])
    e = r["xh"] - r["true"]
    e[:, 2] = wrap(e[:, 2])
    nees = np.einsum("ni,ni->n", e, np.linalg.solve(r["P"], e[..., None])[..., 0])
    late = nees[np.arange(r["steps"]) * config.DT >= 10.0]
    return label, condition, r["success"], r["collided"], r["final_pos_err"], r["rms_pos_err"], late.mean(), np.mean(late < LIMIT)


if __name__ == "__main__":
    items = [(l, c, s) for l in CANDIDATES for c in CONDITIONS for s in range(30 if c == "nominal" else 10)]
    with ProcessPoolExecutor(max_workers=14) as pool:
        rows = list(pool.map(job, items))
    for condition in CONDITIONS:
        print(f"--- {condition}")
        for label in CANDIDATES:
            v = np.array([r[2:] for r in rows if r[0] == label and r[1] == condition], float)
            print(f"{label:24s} success {int(v[:, 0].sum()):2d}/{len(v)}  collided {int(v[:, 1].sum()):2d}  final {v[:, 2].mean() * 100:6.2f} cm  "
                  f"rms {v[:, 3].mean() * 100:6.2f} cm  mean NEES {v[:, 4].mean():7.2f}  under limit {v[:, 5].mean() * 100:5.1f}%")
    print("--- totals over all stress conditions and nominal")
    for label in CANDIDATES:
        v = np.array([r[2:] for r in rows if r[0] == label], float)
        print(f"{label:24s} success {int(v[:, 0].sum())}/{len(v)}  collided {int(v[:, 1].sum())}")
