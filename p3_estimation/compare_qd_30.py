"""compare_qd_30.py — compare Qd values over 30 seeds, with NEES after 10 s (P3).
Run from the repo root:  python -m p3_estimation.compare_qd_30"""
import os
import sys

os.environ["OMP_NUM_THREADS"] = "1"
# let Python find the other project folders when this file is run directly
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from concurrent.futures import ProcessPoolExecutor

import numpy as np

SEEDS = range(30)
OPTIONS = {
    "5 mm, 0.5 deg": (0.005, 0.5),
    "2 mm, 0.3 deg": (0.002, 0.3),
    "0.4 mm, 0.17 deg": (0.0004, 0.17),
}
SETTLE_TIME = 10.0
NEES_LIMIT = 7.815


def one(job):
    import config
    from p4_simulation.episode import run_episode
    from p4_simulation.tb3sim import wrap

    label, seed = job
    pos, head = OPTIONS[label]
    Qd = np.diag([pos**2, pos**2, np.deg2rad(head)**2])
    run = run_episode(seed=seed, viewer=False, camera_pov=False, Qd=Qd, verbose=False)
    e = run["xh"] - run["true"]
    e[:, 2] = wrap(e[:, 2])
    nees = np.einsum("ni,ni->n", e, np.linalg.solve(run["P"], e[..., None])[..., 0])
    settled = np.arange(run["steps"]) * config.DT >= SETTLE_TIME
    late = nees[settled]
    return (label, seed, float(run["success"]), run["final_pos_err"], run["rms_pos_err"],
            run["in_2sigma"].min(), run["final_err_over_sigma"],
            late.mean(), np.mean(late < NEES_LIMIT))


if __name__ == "__main__":
    jobs = [(label, seed) for label in OPTIONS for seed in SEEDS]
    with ProcessPoolExecutor(max_workers=10) as pool:
        rows = list(pool.map(one, jobs))

    by_label = {label: np.array([r[2:] for r in rows if r[0] == label], float) for label in OPTIONS}
    for label, r in by_label.items():
        print(f"{label:18s} success {int(r[:, 0].sum())}/{len(SEEDS)}  final {r[:, 1].mean()*100:5.2f} cm  "
              f"rms {r[:, 2].mean()*100:5.2f} cm (std {r[:, 2].std()*100:.2f})  "
              f"2-sigma {r[:, 3].mean()*100:3.0f}%  final {r[:, 4].mean():.2f} sigma  "
              f"NEES>10s mean {r[:, 5].mean():5.2f}  under limit {r[:, 6].mean()*100:4.1f}%")

    a, b = by_label["5 mm, 0.5 deg"], by_label["2 mm, 0.3 deg"]
    d = (a[:, 2] - b[:, 2]) * 100
    print(f"paired rms (5mm - 2mm): mean {d.mean():.2f} cm, s.e. {d.std(ddof=1)/np.sqrt(len(d)):.2f} cm, "
          f"5mm lower on {(d < 0).sum()}/{len(d)} seeds")
    print("per-seed 2-sigma %, 5mm:", np.round(a[:, 3] * 100).astype(int).tolist())
    print("per-seed 2-sigma %, 2mm:", np.round(b[:, 3] * 100).astype(int).tolist())
