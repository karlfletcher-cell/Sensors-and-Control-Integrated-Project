"""
episode.py — one closed-loop run of the full pipeline as a function (owner: P4, Part E).

run_episode(...) is main.py's loop made callable so run_trials.py can run it hundreds of times
headless with different conditions. The loop body (control -> step -> EKF predict -> camera ->
EKF update) is exactly main.py's; this file only adds the condition arguments, disturbances,
richer logging and the outcome/diagnosis at the end.

Ground truth is read ONLY for logging and for the outcome. The controller and the EKF never see it.
"""
import time
import numpy as np
from p4_simulation.tb3sim import TB3Sim, wrap
from p4_simulation.make_scene import scene_path
from p1_control.controller import avoid_controller, dock_command
from p2_sensing.sensing import load_K, measure
from p3_estimation.Estimation import (odometry_to_input, motion_model, motion_jacobian,
                                     predict_range_bearing, measurement_jacobian,
                                     range_bearing_residual, ekf_predict, ekf_update)
import config

DOCK_RANGE_OK = 0.90     # [m]   success: TRUE range to the marker below this ...
DOCK_BEARING_OK = 0.20   # [rad] ... and TRUE bearing within this (robot facing the dock)


def run_episode(seed=0, start=config.ROBOT_START, init_offset=config.INIT_OFFSET,
                slip_std=config.SLIP_STD, enc_noise_std=0.0, obstacles="default",
                Qd=config.Qd, Rn=config.Rn, use_ekf_update=True,
                push_at=None, push=(0.0, 0.0, 0.0), blackout=None,
                max_steps=800, viewer=False, camera_pov=False, record_pov=None, verbose=True):
    """
    seed           fixes slip, encoder noise and anything random -> the run is reproducible
    start          true start pose (x, y, yaw); init_offset is ADDED to it to make the first belief wrong
    slip_std       process noise (per-wheel multiplicative slip);  enc_noise_std: additive encoder noise [m/step]
    obstacles      name of an obstacle set in make_scene.OBSTACLE_SETS ("default", "dense", "shifted", "none")
    use_ekf_update False = dead reckoning only (the "no camera" baseline)
    push_at/push   at control step push_at, displace the TRUE pose by push=(dx, dy, dyaw)
    blackout       (start_step, end_step): the marker is invisible during these steps
    viewer / camera_pov / record_pov   3D window / camera-POV window / .mp4 path for the POV
    """
    # the avoidance waypoint is kept on the function, so clear it or this run follows the last run's
    avoid_controller.ref_wp = None
    avoid_controller.ref_start = None

    sim = TB3Sim(xml=scene_path(obstacles), dt=config.DT, slip_std=slip_std,
                 enc_noise_std=enc_noise_std, seed=seed)
    MARKER = sim.marker_xy()
    K_cam, T_bc = load_K(sim.camera_intrinsics()), sim.T_base_cam()
    sim.reset(pose=start)
    sim.hide_marker(False)

    xh = np.array(start, float) + np.asarray(init_offset, float)
    P = config.P0.copy()
    docked = False
    log = dict(true=[], xh=[], P=[], seen=[], nu=[], S=[], cmd=[], state=[], collision=[])

    pov = None
    if camera_pov or record_pov:
        from p4_simulation.camera_view import CameraPOV
        pov = CameraPOV(show=camera_pov, record=record_pov, fps=int(round(1 / config.DT)))
    vw = None
    if viewer:
        import mujoco.viewer
        vw = mujoco.viewer.launch_passive(sim.model, sim.data)

    for k in range(max_steps):
        # ---- disturbances (trial conditions only; nominal runs have none)
        if push_at is not None and k == push_at:
            sim.push(*push)
        if blackout is not None:
            sim.hide_marker(blackout[0] <= k < blackout[1])

        # ---- 1. control from the ESTIMATE (+ LiDAR), then step the sim
        if np.hypot(*(np.array(config.GOAL) - xh[:2])) < config.GOAL_TOL:
            v, w, docked = dock_command(xh, MARKER); state = "DOCK"
        else:
            v, w = avoid_controller(xh, config.GOAL, sim.lidar(), sim.lidar_angles()); state = "NAV"
        sim.step(v, w)

        # ---- 2. EKF predict from the encoders
        dl, dr = sim.odometry()
        u = odometry_to_input(dl, dr, config.WHEEL_SEP)
        xh, P = ekf_predict(xh, P, u, motion_model, motion_jacobian, Qd)

        # ---- 3. camera -> (range, bearing) -> EKF update
        img = sim.camera()
        z = measure(img, K_cam, T_bc)
        seen = z is not None
        nu, S = np.full(2, np.nan), np.full((2, 2), np.nan)
        if seen and use_ekf_update:
            xh, P, nu, S = ekf_update(xh, P, z, lambda s: predict_range_bearing(s, MARKER),
                                      lambda s: measurement_jacobian(s, MARKER), Rn,
                                      residual=range_bearing_residual)
        if pov is not None:
            pov.update(img, z, state, K_cam, k)

        # ---- 4. logging — ground truth read HERE only
        log["true"].append(sim.true_pose()); log["xh"].append(xh.copy()); log["P"].append(P.copy())
        log["seen"].append(seen); log["nu"].append(nu); log["S"].append(S); log["cmd"].append((v, w))
        log["state"].append(state); log["collision"].append(sim.in_collision())

        if vw is not None:
            vw.sync(); time.sleep(config.DT)
            if not vw.is_running():
                break
        if docked:
            break

    # ---- outcome (ground truth: EVALUATION only)
    rb_true = sim.true_range_bearing()
    out = {key: np.array(val) for key, val in log.items()}
    err = out["xh"] - out["true"]; err[:, 2] = wrap(err[:, 2])
    sig = np.sqrt(np.stack([out["P"][:, i, i] for i in range(3)], axis=1))
    pos_err = np.linalg.norm(err[:, :2], axis=1)
    out.update(
        steps=len(out["true"]), time_to_dock=len(out["true"]) * config.DT, docked=bool(docked),
        success=bool(docked and rb_true[0] < DOCK_RANGE_OK and abs(rb_true[1]) < DOCK_BEARING_OK),
        final_pos_err=float(pos_err[-1]), max_pos_err=float(pos_err.max()),
        rms_pos_err=float(np.sqrt(np.mean(pos_err ** 2))), rms_yaw_err=float(np.sqrt(np.mean(err[:, 2] ** 2))),
        in_2sigma=np.mean(np.abs(err) < 2 * sig, axis=0),     # consistency: fraction of steps inside ±2σ, per state
        final_err_over_sigma=float(pos_err[-1] / max(np.sqrt(out["P"][-1, 0, 0] + out["P"][-1, 1, 1]), 1e-9)),
        seen_frac=float(np.mean(out["seen"])), collided=bool(out["collision"].any()),
        seen_after_push=float(np.mean(out["seen"][push_at:])) if push_at is not None and push_at < len(out["seen"]) else np.nan,
        spin_frac=float(np.mean((out["cmd"][:, 0] == 0.0) & (out["state"] == "NAV"))),  # "boxed in" spinning
        true_range_at_end=float(rb_true[0]), true_bearing_at_end=float(rb_true[1]),
        condition=dict(seed=seed, start=tuple(start), init_offset=tuple(np.asarray(init_offset, float)),
                       slip_std=slip_std, enc_noise_std=enc_noise_std, obstacles=obstacles,
                       use_ekf_update=use_ekf_update, push_at=push_at, push=push, blackout=blackout),
    )
    out["diagnosis"] = diagnose(out)
    if verbose:
        print(f"{'DOCKED ' if docked else 'TIMEOUT'} t={out['time_to_dock']:5.1f}s  success={out['success']}  "
              f"final err {out['final_pos_err']:.3f} m  rms {out['rms_pos_err']:.3f} m  "
              f"true range/bearing at end {rb_true[0]:.2f} m / {np.degrees(rb_true[1]):.0f} deg  "
              f"marker seen {out['seen_frac']*100:.0f}%  -> {out['diagnosis']}")
    if pov is not None:
        pov.close()
    if vw is not None:
        while vw.is_running():
            vw.sync(); time.sleep(0.05)
        vw.close()
    return out


def diagnose(r):
    """Trace a failed run to the component that failed first. Heuristic, written up per-case
    on the project page; 'ok' for a success. Order matters: earlier checks are more specific."""
    if r["success"]:
        return "ok"
    if r["collided"]:
        return "controller: collision (avoidance did not keep the body clear)"
    if not r["docked"]:
        if r["spin_frac"] > 0.3:
            return "controller: boxed in, spun on the spot until timeout"
        if r["seen_frac"] == 0:
            return "sensing: marker never in view, estimate drifted with no correction"
        return "controller: timeout before reaching the goal"
    # docked according to the ESTIMATE but not actually at the dock -> the estimate was wrong
    if not r["condition"]["use_ekf_update"]:
        return "model: dead reckoning only — process noise (slip) accumulated with no camera correction"
    if r["condition"]["push_at"] is not None and r["seen_after_push"] == 0:
        return "sensing/controller: the push turned the marker out of the camera's view and nothing searches for it, so no correction ever arrived"
    if r["seen_frac"] == 0:
        return "sensing: marker never seen -> dead reckoning only (model drift)"
    if r["final_err_over_sigma"] > 3:
        return "estimator: inconsistent (true error > 3σ) — P too small for the real noise (Q/R tuning)"
    if r["in_2sigma"].min() < 0.80:
        return "estimator: covariance underestimates error (consistency < 80%)"
    return "estimator: large but consistent error — too little marker information before docking (observability)"
