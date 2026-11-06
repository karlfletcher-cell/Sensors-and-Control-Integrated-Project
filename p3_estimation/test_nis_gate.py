"""test_nis_gate.py — would rejecting outlier camera readings help? The gate is a wrapper only, not in the filter (P3).
The notes table used Rn 2 cm, 0.5 deg in config.py.
Run from the repo root:  python -m p3_estimation.test_nis_gate"""
import os
import sys

os.environ["OMP_NUM_THREADS"] = "1"
# let Python find the other project folders when this file is run directly
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from concurrent.futures import ProcessPoolExecutor

import numpy as np

deg = np.deg2rad
GATE = 9.21            # 99% chi-squared limit, 2 degrees of freedom
LIMIT = 7.815


def job(item):
    import config
    from p4_simulation import episode
    from p4_simulation.tb3sim import wrap

    name, gated, condition, seed = item
    kw = {
        "nominal": {},
        "Rn x10": dict(Rn=config.Rn * 10),
        "init_err_large": dict(init_offset=(0.6, -0.5, deg(25))),
        "push_mid_run": dict(push_at=150, push=(0.25, -0.25, deg(15))),
        "push_large": dict(push_at=150, push=(0.6, 0.4, deg(40))),
        "R_10x_smaller": dict(Rn=config.Rn / 10),
    }[condition]
    real = episode.ekf_update
    rejected = [0]

    def gate(xh_pred, P_pred, z, h, H_jac, Rn, residual=None):
        nu = residual(z, h(xh_pred))
        H = H_jac(xh_pred)
        S = H @ P_pred @ H.T + Rn
        if nu @ np.linalg.solve(S, nu) > GATE:
            rejected[0] += 1
            return xh_pred, P_pred, nu, S
        return real(xh_pred, P_pred, z, h, H_jac, Rn, residual=residual)

    if gated:
        episode.ekf_update = gate
    try:
        r = episode.run_episode(seed=seed, viewer=False, camera_pov=False, verbose=False, **kw)
    finally:
        episode.ekf_update = real
    e = r["xh"] - r["true"]
    e[:, 2] = wrap(e[:, 2])
    nees = np.einsum("ni,ni->n", e, np.linalg.solve(r["P"], e[..., None])[..., 0])
    late = nees[np.arange(r["steps"]) * config.DT >= 10.0]
    return name, r["success"], r["collided"], r["final_pos_err"], r["rms_pos_err"], late.mean(), np.mean(late < LIMIT), rejected[0]


if __name__ == "__main__":
    items = []
    for s in range(30):
        items += [("nominal, as is", False, "nominal", s), ("nominal, gate", True, "nominal", s), ("Rn x10", False, "Rn x10", s)]
    for s in range(10):
        for c in ("init_err_large", "push_mid_run", "push_large", "R_10x_smaller"):
            items += [(f"{c}, as is", False, c, s), (f"{c}, gate", True, c, s)]
    with ProcessPoolExecutor(max_workers=14) as pool:
        rows = list(pool.map(job, items))
    print("case                      success  collided  final cm  rms cm  mean NEES  median NEES  under limit  rejected per run")
    for name in dict.fromkeys(r[0] for r in rows):
        v = np.array([r[1:] for r in rows if r[0] == name], float)
        print(f"{name:26s} {int(v[:, 0].sum()):2d}/{len(v)}  {int(v[:, 1].sum()):3d}  {v[:, 2].mean() * 100:8.2f} {v[:, 3].mean() * 100:7.2f} "
              f"{v[:, 4].mean():9.2f} {np.median(v[:, 4]):9.2f} {v[:, 5].mean() * 100:9.1f}% {v[:, 6].mean():8.1f}")
