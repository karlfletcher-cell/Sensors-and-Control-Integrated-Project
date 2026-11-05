"""
plot_run.py — figures for the project page / demo video (owner: P4).
    python -m p4_simulation.plot_run run_log.npz        # one run  -> analysis/fig_map_*.png, fig_error_*.png, fig_innov_*.png
    python -m p4_simulation.plot_run --trials           # analysis/trials.csv -> analysis/fig_trials.png
fig_error_*: estimate − truth per state with the ±2σ band from P — the "estimate and its
uncertainty alongside the true state" the demo video must show.
"""
import sys, csv, os, numpy as np, matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from p4_simulation.tb3sim import wrap
from p4_simulation.make_scene import OBSTACLE_SETS, ROOM_HALF
from config import DOCK_XY, DT, GOAL


def plot_run(fn, obstacles="default"):
    os.makedirs("analysis", exist_ok=True)
    L = np.load(fn)
    T, XH, P, seen = L["true"], L["xh"], L["P"], L["seen"].astype(bool)
    t = np.arange(len(T)) * DT
    err = XH - T; err[:, 2] = wrap(err[:, 2])
    sig = np.sqrt(np.stack([P[:, i, i] for i in range(3)], axis=1))
    tag = os.path.basename(fn).replace("run_log_", "").replace("run_log", "nominal").replace(".npz", "")

    # map
    fig, ax = plt.subplots(figsize=(6, 6))
    for x, y, hx, hy in OBSTACLE_SETS[obstacles]:
        ax.add_patch(plt.Rectangle((x - hx, y - hy), 2 * hx, 2 * hy, color="0.6"))
    ax.plot(*GOAL, "x", color="0.3", ms=10, label="goal"); ax.plot(*DOCK_XY, "gs", ms=10, label="dock / marker")
    ax.plot(T[:, 0], T[:, 1], "k", lw=2, label="true"); ax.plot(XH[:, 0], XH[:, 1], "r--", lw=1.5, label="EKF estimate")
    ax.plot(XH[seen, 0], XH[seen, 1], "b.", ms=3, label="marker in view")
    ax.plot(T[0, 0], T[0, 1], "ko"); ax.plot(XH[0, 0], XH[0, 1], "ro")
    ax.set_xlim(-ROOM_HALF, ROOM_HALF); ax.set_ylim(-ROOM_HALF, ROOM_HALF); ax.set_aspect("equal")
    ax.set_xlabel("x (m)"); ax.set_ylabel("y (m)"); ax.legend(loc="upper left", fontsize=8); ax.set_title(f"path: {tag}")
    plt.tight_layout(); plt.savefig(f"analysis/fig_map_{tag}.png", dpi=120); plt.close()

    # error with ±2σ band
    fig, ax = plt.subplots(3, 1, figsize=(9, 7), sharex=True)
    for i, nm in enumerate(["x error (m)", "y error (m)", "heading error (rad)"]):
        ax[i].fill_between(t, -2 * sig[:, i], 2 * sig[:, i], color="C0", alpha=0.25, label="±2σ from P")
        ax[i].plot(t, err[:, i], "k", lw=1, label="estimate − truth")
        yl = ax[i].get_ylim()
        ax[i].fill_between(t, yl[0], yl[1], where=seen, color="g", alpha=0.08, label="marker in view")
        ax[i].set_ylim(yl); ax[i].axhline(0, color="0.5", lw=0.5); ax[i].set_ylabel(nm); ax[i].grid(alpha=0.3)
    ax[0].legend(loc="upper right", fontsize=8); ax[2].set_xlabel("time (s)")
    inside = np.mean(np.abs(err) < 2 * sig, axis=0) * 100
    ax[0].set_title(f"{tag}: truth inside ±2σ  x {inside[0]:.0f}%  y {inside[1]:.0f}%  θ {inside[2]:.0f}%  (consistent ≈ 95%)")
    plt.tight_layout(); plt.savefig(f"analysis/fig_error_{tag}.png", dpi=120); plt.close()

    # innovation consistency
    if "nu" in L and np.isfinite(L["nu"]).any():
        nu, S = L["nu"], L["S"]
        fig, ax = plt.subplots(2, 1, figsize=(9, 5), sharex=True)
        for i, nm in enumerate(["range innovation (m)", "bearing innovation (rad)"]):
            s = np.sqrt(S[:, i, i])
            ax[i].fill_between(t, -2 * s, 2 * s, color="C1", alpha=0.25, label="±2√S (predicted spread)")
            ax[i].plot(t, nu[:, i], "k.", ms=3, label="innovation ν"); ax[i].set_ylabel(nm); ax[i].grid(alpha=0.3)
        ax[0].legend(fontsize=8); ax[1].set_xlabel("time (s)"); ax[0].set_title(f"{tag}: innovation consistency")
        plt.tight_layout(); plt.savefig(f"analysis/fig_innov_{tag}.png", dpi=120); plt.close()

    print(f"{tag}: RMS err x {np.sqrt(np.mean(err[:,0]**2)):.3f}  y {np.sqrt(np.mean(err[:,1]**2)):.3f} m  "
          f"θ {np.degrees(np.sqrt(np.mean(err[:,2]**2))):.1f} deg;  final {np.linalg.norm(err[-1,:2]):.3f} m;  "
          f"seen {seen.mean()*100:.0f}%  -> analysis/fig_*_{tag}.png")


def plot_trials(fn="analysis/trials.csv"):
    rows = list(csv.DictReader(open(fn)))
    conds = list(dict.fromkeys(r["condition"] for r in rows))
    fe = [[float(r["final_err"]) for r in rows if r["condition"] == c] for c in conds]
    tt = [[float(r["time_to_dock"]) for r in rows if r["condition"] == c and float(r["success"]) == 1] or [np.nan] for c in conds]
    succ = [np.mean([float(r["success"]) for r in rows if r["condition"] == c]) * 100 for c in conds]
    fig, ax = plt.subplots(3, 1, figsize=(11, 9), sharex=True)
    ax[0].bar(range(1, len(conds) + 1), succ, color="C2"); ax[0].set_ylabel("docking success (%)"); ax[0].set_ylim(0, 105)
    ax[1].boxplot(fe, tick_labels=conds); ax[1].set_ylabel("final position error (m)"); ax[1].grid(alpha=0.3)
    ax[2].boxplot(tt, tick_labels=conds); ax[2].set_ylabel("time to dock (s)"); ax[2].grid(alpha=0.3)
    plt.setp(ax[2].get_xticklabels(), rotation=35, ha="right"); ax[0].set_title("robustness trials")
    plt.tight_layout(); plt.savefig("analysis/fig_trials.png", dpi=120); plt.close()
    print("-> analysis/fig_trials.png")


if __name__ == "__main__":
    if "--trials" in sys.argv:
        plot_trials()
    else:
        plot_run(sys.argv[1] if len(sys.argv) > 1 else "run_log.npz")
