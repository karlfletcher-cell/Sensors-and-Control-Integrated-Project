"""
run_trials.py — repeated headless trials under varied conditions (Part E evidence). Owner: P4.

    python -m p4_simulation.run_trials            # full sweep: N_SEEDS seeds per condition
    python -m p4_simulation.run_trials --quick    # 3 seeds per condition (smoke test)
    python -m p4_simulation.run_trials --only slip_0.05 push_mid_run   # a subset (replaces only those rows in the CSV)

Seeds are fixed (0..N-1) so every number in analysis/trials.csv can be regenerated exactly.
Outputs: analysis/trials.csv (one row per run) and analysis/trials_summary.md (one row per condition:
success rate, final error, time to dock, and the diagnosis of every failure).
"""
import sys, csv, os, warnings, numpy as np
warnings.filterwarnings("ignore", "Mean of empty slice")
from p4_simulation.episode import run_episode
import config

args = sys.argv[1:]
N_SEEDS = 3 if "--quick" in args else 10
deg = np.deg2rad

# name -> kwargs for run_episode. Grouped by what the brief asks us to vary.
CONDITIONS = {
    # baseline
    "nominal":            dict(),
    "no_camera_update":   dict(use_ekf_update=False),
    # start pose
    "start_B":            dict(start=(-2.5, 0.5, deg(-30))),
    "start_C":            dict(start=(0.0, -2.5, deg(90))),
    # initial estimate error
    "init_err_zero":      dict(init_offset=(0, 0, 0)),
    "init_err_large":     dict(init_offset=(0.6, -0.5, deg(25))),
    # slip (process noise)
    "slip_0.00":          dict(slip_std=0.00),
    "slip_0.05":          dict(slip_std=0.05),
    "slip_0.10":          dict(slip_std=0.10),
    # obstacles
    "obstacles_none":     dict(obstacles="none"),
    "obstacles_dense":    dict(obstacles="dense"),
    "obstacles_shifted":  dict(obstacles="shifted"),
    # disturbances
    "push_mid_run":       dict(push_at=150, push=(0.25, -0.25, deg(15))),
    "push_large":         dict(push_at=150, push=(0.6, 0.4, deg(40))),
    "marker_blackout":    dict(blackout=(150, 230)),          # 8 s with the marker hidden mid-approach
    "encoder_noise":      dict(enc_noise_std=0.002),          # 2 mm per step per wheel
    # tuning sensitivity (the viva questions)
    "R_10x_smaller":      dict(Rn=config.Rn / 10),
    "R_10x_larger":       dict(Rn=config.Rn * 10),
    "Q_10x_smaller":      dict(Qd=config.Qd / 10),
}
if "--only" in args:
    keep = args[args.index("--only") + 1:]
    CONDITIONS = {k: v for k, v in CONDITIONS.items() if k in keep}

os.makedirs("analysis", exist_ok=True)
rows = []
if "--only" in args and os.path.exists("analysis/trials.csv"):      # keep earlier results for other conditions
    rows = [r for r in csv.DictReader(open("analysis/trials.csv")) if r["condition"] not in CONDITIONS]
    for r in rows:
        for k_ in r:
            if k_ not in ("condition", "diagnosis"):
                r[k_] = float(r[k_])
for name, kw in CONDITIONS.items():
    print(f"--- {name}")
    for seed in range(N_SEEDS):
        r = run_episode(seed=seed, verbose=False, **kw)
        rows.append(dict(condition=name, seed=seed, success=int(r["success"]), docked=int(r["docked"]),
                         time_to_dock=r["time_to_dock"], final_err=r["final_pos_err"], max_err=r["max_pos_err"],
                         rms_err=r["rms_pos_err"], rms_yaw_deg=np.degrees(r["rms_yaw_err"]),
                         in2s_x=r["in_2sigma"][0], in2s_y=r["in_2sigma"][1], in2s_th=r["in_2sigma"][2],
                         seen=r["seen_frac"], collided=int(r["collided"]), end_range=r["true_range_at_end"],
                         end_bearing_deg=np.degrees(r["true_bearing_at_end"]), diagnosis=r["diagnosis"]))
        print(f"  seed {seed}: {'ok  ' if r['success'] else 'FAIL'} t={r['time_to_dock']:5.1f}s "
              f"final {r['final_pos_err']:.3f} m   {r['diagnosis'] if not r['success'] else ''}")

with open("analysis/trials.csv", "w", newline="") as f:
    w = csv.DictWriter(f, fieldnames=rows[0].keys()); w.writeheader(); w.writerows(rows)

hdr = ("| condition | success | time to dock (s) | final err (m) mean ± sd | RMS err (m) | inside ±2σ x/y/θ (%) | failures |\n"
       "|---|---|---|---|---|---|---|\n")
lines = []
for name in dict.fromkeys(r["condition"] for r in rows):
    rs = [r for r in rows if r["condition"] == name]
    ok = [r for r in rs if r["success"]]
    fe = np.array([r["final_err"] for r in rs]); rm = np.array([r["rms_err"] for r in rs])
    tt = np.array([r["time_to_dock"] for r in ok]) if ok else np.array([np.nan])
    c = np.array([[r["in2s_x"], r["in2s_y"], r["in2s_th"]] for r in rs]).mean(axis=0) * 100
    diags = {}
    for r in rs:
        if not r["success"]:
            diags[r["diagnosis"]] = diags.get(r["diagnosis"], 0) + 1
    dtxt = "; ".join(f"{n}× {d}" for d, n in diags.items()) or "—"
    lines.append(f"| {name} | {len(ok)}/{len(rs)} | {np.nanmean(tt):.1f} | {fe.mean():.3f} ± {fe.std():.3f} | "
                 f"{rm.mean():.3f} | {c[0]:.0f}/{c[1]:.0f}/{c[2]:.0f} | {dtxt} |")
with open("analysis/trials_summary.md", "w") as f:
    f.write(f"Trials: {N_SEEDS} seeds per condition (seeds 0..{N_SEEDS-1}). Time to dock averaged over successful runs.\n\n" + hdr + "\n".join(lines) + "\n")
print("\n" + hdr + "\n".join(lines))
print("\nwrote analysis/trials.csv and analysis/trials_summary.md")
