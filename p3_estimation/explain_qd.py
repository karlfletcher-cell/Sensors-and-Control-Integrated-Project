"""
explain_qd.py — find out why the filter needs a larger Qd than the measured wheel slip (P3).
Runs the measured and chosen Qd with the camera error and the start error each switched off.
The "perfect camera" runs feed ground truth into the EKF. That is for this test ONLY, to remove
the camera's error, and is never done in real runs. Takes about 10 minutes.
Run from the repo root:  python -m p3_estimation.explain_qd
"""
import gc
import numpy as np
import config
import p4_simulation.episode as episode

SEEDS = range(10)

# label: (x and y noise per step in metres, heading noise per step in degrees)
QD_OPTIONS = {
    "0.4 mm, 0.17 deg (measured)": (0.0004, 0.17),
    "5 mm, 0.5 deg (config.py)": (0.005, 0.5),
}
START_ERRORS = {"start error": config.INIT_OFFSET, "no start error": np.zeros(3)}

real_measure = episode.measure
RealSim = episode.TB3Sim
current_sim = {}


class RememberedSim(RealSim):
    """TB3Sim that keeps a reference to itself, so perfect_measure can read its ground truth."""
    def __init__(self, *args, **kwargs):
        # free the last sim first: a new renderer breaks if the old one is deleted after it is made
        current_sim.clear()
        gc.collect()
        super().__init__(*args, **kwargs)
        current_sim["sim"] = self


def perfect_measure(image, camera_matrix, camera_pose_on_robot):
    """True range and bearing, but only when the real detector can see the marker."""
    if real_measure(image, camera_matrix, camera_pose_on_robot) is None:
        return None
    return current_sim["sim"].true_range_bearing()


episode.TB3Sim = RememberedSim

for camera_name, camera_function in (("real camera", real_measure), ("perfect camera", perfect_measure)):
    episode.measure = camera_function
    for start_name, start_error in START_ERRORS.items():
        for qd_name, (position_noise, heading_noise_deg) in QD_OPTIONS.items():
            Qd = np.diag([position_noise**2, position_noise**2, np.deg2rad(heading_noise_deg)**2])
            runs = []
            for seed in SEEDS:
                run = episode.run_episode(seed=seed, viewer=False, camera_pov=False, Qd=Qd,
                                          init_offset=start_error, verbose=False)
                runs.append((run["success"], run["final_pos_err"], run["rms_pos_err"]))
            runs = np.array(runs, float)
            print(f"{camera_name:14s}  {start_name:14s}  {qd_name:27s}  "
                  f"success {int(runs[:, 0].sum())}/{len(SEEDS)}   "
                  f"final error {runs[:, 1].mean() * 100:5.2f} cm   "
                  f"rms error {runs[:, 2].mean() * 100:5.2f} cm (std {runs[:, 2].std() * 100:.2f} cm)",
                  flush=True)
