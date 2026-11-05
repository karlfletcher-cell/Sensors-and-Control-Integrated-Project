
import csv
import json
import os
import sys

import cv2
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import mujoco
import numpy as np

from config import (DT, GOAL, GOAL_TOL, INIT_OFFSET, MARKER_SIZE, P0, Qd, ROBOT_START, Rn, SLIP_STD, WHEEL_SEP)
from p1_control.controller import avoid_controller, dock_command
from p2_sensing.sensing import load_K, measure
from p3_estimation.Estimation import (ekf_predict, ekf_update, measurement_jacobian, motion_jacobian,
                                      motion_model, odometry_to_input, predict_range_bearing,
                                      range_bearing_residual)
from p4_simulation.tb3sim import TB3Sim, wrap

OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "results")
BLUE, ORANGE, AQUA = "#2a78d6", "#eb6834", "#1baf7a"
INK, MUTED, GRID = "#0b0b0b", "#52514e", "#e4e3df"
plt.rcParams.update({"axes.spines.top": False, "axes.spines.right": False, "axes.edgecolor": MUTED,
                     "axes.grid": True, "grid.color": GRID, "grid.linewidth": 0.8, "axes.axisbelow": True,
                     "text.color": INK, "axes.labelcolor": MUTED, "xtick.color": MUTED, "ytick.color": MUTED,
                     "font.size": 10, "figure.dpi": 160, "legend.frameon": False})

DISTANCES = np.array([0.5, 0.75, 1.0, 1.5, 2.0, 2.5, 3.0, 3.5, 4.0, 4.5, 5.0])
N_SAMPLES = 50
IMAGE_NOISE = 5.0        # grey levels, the "with image noise" condition
_old = cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50))


def measure_original(img, K, T_bc):
    """The first version of measure(), kept only as the before-fix comparison."""
    corners, ids, _ = _old.detectMarkers(cv2.cvtColor(img, cv2.COLOR_RGB2GRAY))
    if ids is None:
        return None
    s = MARKER_SIZE / 2
    obj = np.array([[-s, s, 0], [s, s, 0], [s, -s, 0], [-s, -s, 0]], np.float32)
    _, _, tvec = cv2.solvePnP(obj, corners[0][0], K, None)
    p = T_bc @ np.append(tvec.ravel(), 1.0)
    return np.array([np.hypot(p[0], p[1]), np.arctan2(p[1], p[0])])


def add_noise(img, rng, sigma=IMAGE_NOISE):
    return np.clip(img + rng.normal(0, sigma, img.shape), 0, 255).astype(np.uint8)


def hide_clutter(sim):
    """Hide walls and obstacles, so detection results reflect the sensor rather than occlusion."""
    for g in range(sim.model.ngeom):
        if sim.model.geom_bodyid[g] == 0 and sim.model.geom_type[g] == mujoco.mjtGeom.mjGEOM_BOX:
            sim.model.geom_rgba[g, 3] = 0


def stats(errors):
    e = np.array(errors, float)
    return (e.mean(), e.std(ddof=1), np.abs(e).max()) if len(e) > 1 else (np.nan, np.nan, np.nan)


def noise_sweep(K_sim, K_cal, T_bc):
    """Bias and std of range and bearing at each distance. Rendered images are noise-free, so
    each sample moves the robot slightly and the spread is mainly pixel-level corner error."""
    sim = TB3Sim(dt=DT)
    hide_clutter(sim)
    mk = sim.marker_xy()
    rng = np.random.default_rng(10)
    conds = {"original": lambda i: measure_original(i, K_sim, T_bc),
             "final": lambda i: measure(i, K_cal, T_bc),
             "final_noisy": lambda i: measure(add_noise(i, rng), K_cal, T_bc)}
    rows = []
    for d in DISTANCES:
        err = {c: [] for c in conds}
        for _ in range(N_SAMPLES):
            sim.reset(pose=(mk[0] - d + rng.uniform(-.02, .02), mk[1] + rng.uniform(-.02, .02),
                            rng.uniform(-.03, .03)))
            img, tr = sim.camera(), sim.true_range_bearing()
            for c, f in conds.items():
                z = f(img)
                if z is not None:
                    err[c].append((z[0] - tr[0], wrap(z[1] - tr[1])))
        for c in conds:
            e = np.array(err[c]).reshape(-1, 2)
            rb, rs, rm = stats(e[:, 0])
            bb, bs, _ = stats(e[:, 1])
            rows.append(dict(condition=c, distance_m=float(d), n_detected=len(e), range_bias_m=rb, range_std_m=rs,
                             range_max_abs_m=rm, bearing_bias_deg=np.rad2deg(bb), bearing_std_deg=np.rad2deg(bs)))
    write_csv("noise_vs_distance.csv", rows)
    fig, ax = plt.subplots(1, 3, figsize=(11, 3.4))
    for c, col, lab in [("original", ORANGE, "Original"), ("final", BLUE, "Final"),
                        ("final_noisy", AQUA, f"Final + image noise (σ={IMAGE_NOISE:g})")]:
        r = [x for x in rows if x["condition"] == c]
        d = [x["distance_m"] for x in r]
        ax[0].plot(d, [x["range_bias_m"] * 100 for x in r], "o-", color=col, lw=2, ms=4, label=lab)
        ax[1].plot(d, [x["range_std_m"] * 100 for x in r], "o-", color=col, lw=2, ms=4, label=lab)
        ax[2].plot(d, [x["bearing_std_deg"] for x in r], "o-", color=col, lw=2, ms=4, label=lab)
    # Week 3 wrap-up: sigma_Z = Z^2 * sigma_px / (f * s). Fit sigma_px, then draw the law.
    final = [x for x in rows if x["condition"] == "final"]
    Z = np.array([x["distance_m"] for x in final]) - T_bc[0, 3]             # camera to marker
    law = Z ** 2 / (K_cal[0, 0] * MARKER_SIZE)
    measured_std = np.array([x["range_std_m"] for x in final])
    sigma_px = float(np.sum(measured_std * law) / np.sum(law ** 2))
    ax[1].plot(Z + T_bc[0, 3], law * sigma_px * 100, "--", color=MUTED, lw=1.5,
               label=f"Week 3 law, σ_px = {sigma_px:.2f}")
    ax[1].legend()
    ax[0].axhline(0, color=MUTED, lw=1)
    for a, t, y in [(ax[0], "Range bias", "cm"), (ax[1], "Range std", "cm"), (ax[2], "Bearing std", "deg")]:
        a.set_title(t, loc="left", fontweight="bold")
        a.set_xlabel("True distance to marker (m)")
        a.set_ylabel(y)
    ax[0].legend()
    save(fig, "noise_vs_distance.png")
    return rows, sigma_px


def frame_check(K_cal, T_bc):
    """Week 2 Part C.2: the same measurements with the full frame chain and with T_bc's
    translation forgotten, to show what the extrinsic is worth."""
    sim = TB3Sim(dt=DT)
    hide_clutter(sim)
    mk = sim.marker_xy()
    rng = np.random.default_rng(12)
    T_no_trans = T_bc.copy()
    T_no_trans[:2, 3] = 0
    errs = {"full chain": [], "translation forgotten": []}
    for _ in range(100):
        t, d = np.deg2rad(rng.uniform(-30, 30)), rng.uniform(.6, 3)
        x, y = mk[0] - d * np.cos(t), mk[1] + d * np.sin(t)
        sim.reset(pose=(x, y, np.arctan2(mk[1] - y, mk[0] - x) + rng.uniform(-.2, .2)))
        img, tr = sim.camera(), sim.true_range_bearing()
        for name, T in [("full chain", T_bc), ("translation forgotten", T_no_trans)]:
            z = measure(img, K_cal, T)
            if z is not None:
                errs[name].append(z[0] - tr[0])
    return {k: dict(range_bias_m=float(np.mean(v)), n=len(v)) for k, v in errs.items()}


def detection_sweep(K_cal, T_bc):
    sim = TB3Sim(dt=DT)
    hide_clutter(sim)
    mk = sim.marker_xy()
    rng = np.random.default_rng(11)
    dists = np.array([0.15, 0.2, 0.3, 0.5, 1, 2, 3, 4, 5, 6, 7, 8, 9])
    angles = np.array([0, 15, 30, 45, 60, 70, 75, 80, 85])
    rate, rows = np.zeros((len(angles), len(dists))), []
    for i, a in enumerate(angles):
        for j, d in enumerate(dists):
            seen = 0
            for _ in range(20):
                t = np.deg2rad(a)
                x, y = mk[0] - d * np.cos(t), mk[1] + d * np.sin(t)
                x, y = x + rng.uniform(-.01, .01), y + rng.uniform(-.01, .01)
                sim.reset(pose=(x, y, np.arctan2(mk[1] - y, mk[0] - x) + rng.uniform(-.01, .01)))
                seen += measure(sim.camera(), K_cal, T_bc) is not None
            rate[i, j] = seen / 20
            rows.append(dict(viewing_angle_deg=int(a), distance_m=float(d), detection_rate=rate[i, j]))
    write_csv("detection.csv", rows)
    fig, ax = plt.subplots(figsize=(7.5, 4))
    im = ax.imshow(rate * 100, cmap="Blues", vmin=0, vmax=100, aspect="auto", origin="lower")
    ax.set_xticks(range(len(dists)), [f"{d:g}" for d in dists])
    ax.set_yticks(range(len(angles)), angles)
    ax.grid(False)
    ax.set_xlabel("Distance from base to marker (m)")
    ax.set_ylabel("Viewing angle (deg)")
    ax.set_title("Detection rate (%), robot facing marker, 20 samples per cell", loc="left", fontweight="bold")
    for i in range(len(angles)):
        for j in range(len(dists)):
            ax.text(j, i, f"{rate[i, j] * 100:.0f}", ha="center", va="center", fontsize=8,
                    color="white" if rate[i, j] > .6 else INK)
    fig.colorbar(im, label="%")
    save(fig, "detection.png")
    return rows


def pipeline_run(K_sim, K_cal, T_bc, seed=0, max_steps=800):
    """Closed-loop run with the current controller and EKF, driven by the estimate only.
    Ground truth is logged for evaluation and never fed back."""
    sim = TB3Sim(dt=DT, slip_std=SLIP_STD, seed=seed)
    mk = sim.marker_xy()
    sim.reset()
    xh, P = np.array(ROBOT_START) + INIT_OFFSET, P0.copy()
    rows, docked = [], False
    for k in range(max_steps):
        if np.hypot(*(np.array(GOAL) - xh[:2])) < GOAL_TOL:
            v, w, docked = dock_command(xh, mk)
        else:
            v, w = avoid_controller(xh, GOAL, sim.lidar(), sim.lidar_angles())
        sim.step(v, w)
        dl, dr = sim.odometry()
        xh, P = ekf_predict(xh, P, odometry_to_input(dl, dr, WHEEL_SEP), motion_model, motion_jacobian, Qd)
        img, tr = sim.camera(), sim.true_range_bearing()
        z, z0 = measure(img, K_cal, T_bc), measure_original(img, K_sim, T_bc)
        if z is not None:
            xh, P, _, _ = ekf_update(xh, P, z, lambda s: predict_range_bearing(s, mk),
                                     lambda s: measurement_jacobian(s, mk), Rn, residual=range_bearing_residual)
        rows.append(dict(step=k, t_s=k * DT, true_range_m=tr[0], true_bearing_deg=np.rad2deg(tr[1]), seen=int(z is not None),
                         range_err_final_m=z[0] - tr[0] if z is not None else np.nan,
                         bearing_err_final_deg=np.rad2deg(wrap(z[1] - tr[1])) if z is not None else np.nan,
                         range_err_original_m=z0[0] - tr[0] if z0 is not None else np.nan,
                         sigma_x_m=np.sqrt(P[0, 0]), sigma_y_m=np.sqrt(P[1, 1]), sigma_yaw_deg=np.rad2deg(np.sqrt(P[2, 2]))))
        if docked:
            break
    write_csv("pipeline_run.csv", rows)
    t = np.array([r["t_s"] for r in rows])
    seen = np.array([r["seen"] for r in rows], bool)
    fig, ax = plt.subplots(2, 1, figsize=(8.5, 5), sharex=True)
    ax[0].plot(t, [r["range_err_original_m"] * 100 for r in rows], ".", color=ORANGE, ms=4, label="Original")
    ax[0].plot(t, [r["range_err_final_m"] * 100 for r in rows], ".", color=BLUE, ms=4, label="Final")
    ax[0].axhline(0, color=MUTED, lw=1)
    ax[0].set_ylabel("Range error (cm)")
    ax[0].set_title("Range error during a closed-loop run", loc="left", fontweight="bold")
    ax[0].legend(ncol=2)
    ax[1].plot(t, [r["sigma_x_m"] * 100 for r in rows], color=BLUE, lw=2, label="σx")
    ax[1].plot(t, [r["sigma_y_m"] * 100 for r in rows], color=ORANGE, lw=2, label="σy")
    ax[1].fill_between(t, 0, 1, where=seen, transform=ax[1].get_xaxis_transform(), color=GRID, alpha=.7, lw=0, label="Marker visible")
    ax[1].set_xlabel("Time (s)")
    ax[1].set_ylabel("EKF position std (cm)")
    ax[1].legend(ncol=3)
    save(fig, "pipeline_run.png")
    lost, longest = 0, 0
    for s in seen:
        lost = 0 if s else lost + 1
        longest = max(longest, lost)
    e0 = np.array([r["range_err_original_m"] for r in rows], float)
    e1 = np.array([r["range_err_final_m"] for r in rows], float)
    last = seen.nonzero()[0][-1] if seen.any() else -1
    return dict(steps=len(rows), docked=bool(docked), seen_fraction=float(seen.mean()), longest_loss_steps=int(longest),
                range_bias_original_m=float(np.nanmean(e0)), range_bias_final_m=float(np.nanmean(e1)),
                range_std_original_m=float(np.nanstd(e0)), range_std_final_m=float(np.nanstd(e1)),
                range_max_abs_original_m=float(np.nanmax(np.abs(e0))), range_max_abs_final_m=float(np.nanmax(np.abs(e1))),
                bearing_std_final_deg=float(np.nanstd([r["bearing_err_final_deg"] for r in rows])),
                true_range_span_m=[float(min(r["true_range_m"] for r in rows)), float(max(r["true_range_m"] for r in rows))],
                final_true_range_m=float(rows[-1]["true_range_m"]), last_seen_step=int(last),
                sigma_yaw_end_deg=float(rows[-1]["sigma_yaw_deg"]))


def frames_figure(T_bc, K):
    """Top view at an example pose, with T_bc and the frame chain written out."""
    x, y, yaw = 0.8, 0.9, np.deg2rad(20)
    mk = np.array([2.6, 1.5])
    R = np.array([[np.cos(yaw), -np.sin(yaw)], [np.sin(yaw), np.cos(yaw)]])
    base, cam = np.array([x, y]), np.array([x, y]) + R @ T_bc[:2, 3]
    fig, ax = plt.subplots(1, 2, figsize=(11, 4.4), gridspec_kw=dict(width_ratios=[1.5, 1]))
    a = ax[0]

    def axes(o, ux, uy, lx, ly, col, l=.28, lu=None):
        for u, lab, n in [(ux, lx, lu or l), (uy, ly, l)]:
            a.annotate("", xy=o + n * np.array(u), xytext=o, arrowprops=dict(arrowstyle="-|>", color=col, lw=1.8))
            a.text(*(o + (n + .08) * np.array(u)), lab, color=INK, ha="center", va="center", fontsize=9)

    axes(np.array([0, 0.]), [1, 0], [0, 1], "x_w", "y_w", MUTED, .4)
    a.text(-.05, -.14, "World {W}", fontweight="bold")
    axes(cam, R @ [1, 0], R @ [0, -1], "z_c", "x_c", ORANGE, lu=.62)
    axes(base, R @ [1, 0], R @ [0, 1], "x_b", "y_b", BLUE, lu=.3)
    a.text(base[0] - .62, base[1] - .12, "Base {B}", fontweight="bold")
    a.text(cam[0] + .02, cam[1] - .62, "Camera {C}\ny_c points down", fontweight="bold")
    axes(mk, [-1, 0], [0, -1], "z_m", "x_m", AQUA)
    a.text(mk[0] - .2, mk[1] + .1, "Marker {M}\ny_m points up", fontweight="bold")
    a.plot([base[0], mk[0]], [base[1], mk[1]], "--", color=MUTED, lw=1)
    a.text(*(base + .55 * (mk - base) + [0, .1]), "range, bearing", color=MUTED, rotation=np.rad2deg(np.arctan2(*(mk - base)[::-1])), ha="center")
    a.set_aspect("equal")
    a.set_xlim(-.45, 3.1)
    a.set_ylim(-.4, 2.0)
    a.grid(False)
    a.set_xlabel("x (m)")
    a.set_ylabel("y (m)")
    a.set_title("Frames from above, at an example pose", loc="left", fontweight="bold")
    b = ax[1]
    b.axis("off")
    txt = ("Chain\n"
           "M → C   solvePnP tvec gives the marker centre in {C}\n"
           "C → B   p_b = T_bc p_c\n"
           "B → z   range = √(x_b² + y_b²), bearing = atan2(y_b, x_b)\n"
           "W → z   EKF h(x) predicts z from the pose and marker map position\n\n"
           "T_bc (camera pose in base)\n"
           + "\n".join("   [" + " ".join(f"{v:6.2f}" for v in row) + " ]" for row in T_bc) +
           "\n\nWeek 2 form: p_W = T_WB T_BC p_C"
           "\n\nx_c = −y_b,  y_c = −z_b,  z_c = +x_b\n"
           "Camera origin: 0.10 m ahead of and 0.12 m above base\n"
           "Marker: ID 0, 0.20 m, at (2.6, 1.5), centre 0.12 m high")
    b.text(0, 1, txt, va="top", family="DejaVu Sans Mono", fontsize=8.2, color=INK)
    save(fig, "frames.png")


def write_csv(name, rows):
    with open(os.path.join(OUT, name), "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=rows[0].keys())
        w.writeheader()
        w.writerows(rows)


def save(fig, name):
    fig.tight_layout()
    fig.savefig(os.path.join(OUT, name), facecolor="white")
    plt.close(fig)


def main():
    os.makedirs(OUT, exist_ok=True)
    only = sys.argv[1:] or ["noise", "detect", "run", "frames", "chain"]
    sim = TB3Sim(dt=DT)
    K_sim, T_bc = sim.camera_intrinsics(), sim.T_base_cam()
    K_cal = load_K(K_sim)
    path = os.path.join(OUT, "summary.json")
    summary = json.load(open(path)) if os.path.exists(path) else {}
    if "frames" in only:
        frames_figure(T_bc, K_cal)
    if "noise" in only:
        rows, summary["sigma_px"] = noise_sweep(K_sim, K_cal, T_bc)
        for c in ["original", "final", "final_noisy"]:
            r = [x for x in rows if x["condition"] == c]
            summary[f"noise_{c}"] = [{k: x[k] for k in ("distance_m", "n_detected", "range_bias_m", "range_std_m", "range_max_abs_m", "bearing_bias_deg", "bearing_std_deg")} for x in r]
    if "chain" in only:
        summary["frame_check"] = frame_check(K_cal, T_bc)
    if "detect" in only:
        summary["detection"] = detection_sweep(K_cal, T_bc)
    if "run" in only:
        summary["run"] = pipeline_run(K_sim, K_cal, T_bc)
    if "run" in summary:
        # Week 2 Part C: R = diag(sigma_rho^2, sigma_alpha^2), from RMS error in the closed-loop run
        with open(os.path.join(OUT, "pipeline_run.csv")) as f:
            seen = [r for r in csv.DictReader(f) if r["seen"] == "1"]
        rho = np.sqrt(np.mean([float(r["range_err_final_m"]) ** 2 for r in seen]))
        alpha = np.sqrt(np.mean([float(r["bearing_err_final_deg"]) ** 2 for r in seen]))
        summary["Rn_start"] = dict(sigma_range_m=float(rho), sigma_bearing_deg=float(alpha))
    with open(path, "w") as f:
        json.dump(summary, f, indent=1, default=float)
    print(json.dumps({k: v for k, v in summary.items() if k != "detection"}, indent=0, default=float)[:6000])


if __name__ == "__main__":
    main()