"""explain_failures.py — what caused a failed run? Logs camera error against truth, then re-runs with the
controller given the TRUE pose (EKF unchanged). Ground truth is for diagnosis only. Edit `cases` for other runs (P3).
Run from the repo root:  python -m p3_estimation.explain_failures"""
import os
import sys

os.environ["OMP_NUM_THREADS"] = "1"
# let Python find the other project folders when this file is run directly
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from concurrent.futures import ProcessPoolExecutor

import numpy as np

deg = np.deg2rad
LIMIT = 7.815


def conditions():
    import config
    return {
        "nominal": {},
        "init_err_large": dict(init_offset=(0.6, -0.5, deg(25))),
        "slip_0.05": dict(slip_std=0.05),
        "obstacles_shifted": dict(obstacles="shifted"),
        "push_large": dict(push_at=150, push=(0.6, 0.4, deg(40))),
        "encoder_noise": dict(enc_noise_std=0.002),
        "R_10x_smaller": dict(Rn=config.Rn / 10),
        "R_10x_larger": dict(Rn=config.Rn * 10),
        "Q_10x_smaller": dict(Qd=config.Qd / 10),
    }


def run(condition, seed, truth_control=False):
    from p4_simulation import episode

    holder = {}
    camera = []
    real_sim, real_avoid, real_measure = episode.TB3Sim, episode.avoid_controller, episode.measure

    def make_sim(*a, **k):
        holder["sim"] = real_sim(*a, **k)
        return holder["sim"]

    def avoid(xh, *a, **k):
        return real_avoid(holder["sim"].true_pose() if truth_control else xh, *a, **k)

    def measure(*a, **k):
        z = real_measure(*a, **k)
        true = holder["sim"].true_range_bearing()
        camera.append((np.nan, np.nan) if z is None else (z[0] - true[0], np.degrees(z[1] - true[1])))
        return z

    # keep run_episode's own reset working on the real function
    avoid.__dict__.update(ref_wp=None, ref_start=None)
    real_avoid.ref_wp = real_avoid.ref_start = None
    episode.TB3Sim, episode.avoid_controller, episode.measure = make_sim, avoid, measure
    try:
        r = episode.run_episode(seed=seed, viewer=False, camera_pov=False, verbose=False, **conditions()[condition])
    finally:
        episode.TB3Sim, episode.avoid_controller, episode.measure = real_sim, real_avoid, real_measure
    r["camera_err"] = np.array(camera)
    return r


def case_job(job):
    from p4_simulation.tb3sim import wrap
    condition, seed = job
    r = run(condition, seed)
    t = run(condition, seed, truth_control=True)
    err = r["xh"] - r["true"]
    err[:, 2] = wrap(err[:, 2])
    pos = np.linalg.norm(err[:, :2], axis=1)
    hit = np.flatnonzero(r["collision"])
    k = int(hit[0]) if len(hit) else len(pos) - 1
    cam = r["camera_err"][:k + 1]
    bad = int(np.nanargmax(np.abs(cam[:, 0])))
    steps = np.linalg.norm(np.diff(r["xh"][:, :2], axis=0), axis=1)
    jump = int(np.argmax(steps[:k])) + 1 if k > 1 else 0
    return dict(job=job, k=k, hit=len(hit) > 0, err=pos[k], ok=r["success"], ok_t=t["success"],
                coll_t=bool(t["collision"].any()), bad=bad, bad_r=cam[bad, 0], bad_b=cam[bad, 1], bad_p=r["true"][bad],
                jump=jump, jump_m=steps[jump - 1] if jump else 0.0, before=pos[jump - 1], after=pos[jump],
                diag_t=t["diagnosis"], end=pos[-1], end_t=t["final_pos_err"])


def tuning_job(job):
    import config
    from p4_simulation.tb3sim import wrap
    condition, seed = job
    r = run(condition, seed)
    e = r["xh"] - r["true"]
    e[:, 2] = wrap(e[:, 2])
    nees = np.einsum("ni,ni->n", e, np.linalg.solve(r["P"], e[..., None])[..., 0])
    late = nees[np.arange(r["steps"]) * config.DT >= 10.0]
    return condition, late.mean(), np.mean(late < LIMIT), r["camera_err"], r["success"]


if __name__ == "__main__":
    cases = [("push_large", 2), ("push_large", 5), ("push_large", 0), ("nominal", 0)]
    tuning = [(c, s) for c in ("nominal", "R_10x_smaller", "R_10x_larger", "Q_10x_smaller") for s in range(30)]
    with ProcessPoolExecutor(max_workers=12) as pool:
        results = list(pool.map(case_job, cases))
        nees = list(pool.map(tuning_job, tuning))

    print("case                  ok hit@   err@hit | worst camera reading before hit: t, range err, bearing err, true x,y | biggest estimate jump: t, size, err before->after | controller given truth")
    for c in results:
        name, seed = c["job"]
        p = c["bad_p"]
        print(f"{name:18s}{seed:3d}  {str(c['ok'])[0]} {(c['k'] * 0.1 if c['hit'] else -1):5.1f}s {c['err'] * 100:5.1f}cm | "
              f"{c['bad'] * 0.1:5.1f}s {c['bad_r'] * 100:6.1f}cm {c['bad_b']:6.2f}deg ({p[0]:5.2f},{p[1]:5.2f}) | "
              f"{c['jump'] * 0.1:5.1f}s {c['jump_m'] * 100:5.1f}cm {c['before'] * 100:5.1f}->{c['after'] * 100:5.1f}cm | "
              f"ok={str(c['ok_t'])[0]} coll={str(c['coll_t'])[0]} end {c['end_t'] * 100:5.1f}cm {c['diag_t'][:38]}")

    print("\ncondition        mean NEES after 10 s   median seed   under limit")
    for name in dict.fromkeys(n[0] for n in nees):
        values = np.array([n[1:3] for n in nees if n[0] == name])
        print(f"{name:16s} {sum(n[4] for n in nees if n[0] == name):2d}/30 {values[:, 0].mean():10.2f}   {np.median(values[:, 0]):8.2f}   {values[:, 1].mean() * 100:5.1f}%")
    all_err = np.vstack([n[3] for n in nees if n[0] == "nominal"])
    all_err = all_err[~np.isnan(all_err[:, 0])]
    big = np.abs(all_err[:, 0]) > 0.05
    print(f"\nnominal seeds 0-29 camera readings: {len(all_err)}, range error over 5 cm: {big.sum()}, largest {np.abs(all_err[:, 0]).max() * 100:.1f} cm, "
          f"std {all_err[:, 0].std() * 100:.2f} cm, std without those {all_err[~big, 0].std() * 100:.2f} cm")
