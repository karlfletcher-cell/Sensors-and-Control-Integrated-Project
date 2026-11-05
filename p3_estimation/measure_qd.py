"""
measure_qd.py — measure how wrong the wheel-odometry prediction is per step, to choose Qd (P3).
Uses ground truth ONLY to characterise the motion model (calibration), never in the EKF.
Run from the repo root:  python -m p3_estimation.measure_qd
"""
import numpy as np
import config
from p4_simulation.episode import run_episode
from p4_simulation.tb3sim import TB3Sim, wrap
from p4_simulation.make_scene import scene_path
from p3_estimation.Estimation import odometry_to_input, motion_model

NUMBER_OF_STEPS = 1000
STEPS_BEFORE_RESET = 25          # put the robot back in the middle often so it never hits a wall


def measure_prediction_error(commands, slip_std, start_pose, steps_before_reset):
    """Drive the commands in an empty room. Returns one row per step: along, across, heading error."""
    sim = TB3Sim(xml=scene_path("none"), dt=config.DT, slip_std=slip_std, seed=0, render=False)
    sim.reset(pose=start_pose)
    sim.odometry()               # clear the encoder count after teleporting
    errors = []

    for step_number, (forward_speed, turn_rate) in enumerate(commands):
        reset_this_step = steps_before_reset is not None and step_number % steps_before_reset == 0
        if reset_this_step:
            sim.reset(pose=start_pose)
            sim.odometry()

        pose_before = sim.true_pose()
        sim.step(forward_speed, turn_rate)
        pose_after = sim.true_pose()

        # what the EKF's predict step would say, using only the wheel encoders
        left_travel, right_travel = sim.odometry()
        wheel_input = odometry_to_input(left_travel, right_travel, config.WHEEL_SEP)
        predicted_pose = motion_model(pose_before, wheel_input)

        # skip the first step after a reset: the robot starts from rest and lags the encoders by
        # about 2 mm even with slip off. That start-up error isn't slip and doesn't happen mid-run.
        if reset_this_step or step_number == 0:
            continue

        # error over this one step, measured along and across the robot
        # (world x and y only line up with the robot when it faces along x)
        x_error = pose_after[0] - predicted_pose[0]
        y_error = pose_after[1] - predicted_pose[1]
        heading = pose_after[2]
        errors.append((np.cos(heading) * x_error + np.sin(heading) * y_error,
                       -np.sin(heading) * x_error + np.cos(heading) * y_error,
                       wrap(pose_after[2] - predicted_pose[2])))
    return np.array(errors)


random_generator = np.random.default_rng(0)

# smoothly changing turn rate
smooth_commands = [(config.V, 0.5 * np.sin(0.1 * k)) for k in range(NUMBER_OF_STEPS)]
# turn rate jumps every step, to test whether sudden command changes add error
sudden_commands = [(config.V, random_generator.uniform(-0.5, 0.5)) for _ in range(NUMBER_OF_STEPS)]
# what the real controller commanded on a full docking run (seed 0)
real_commands = run_episode(seed=0, viewer=False, camera_pov=False, verbose=False)["cmd"]

DRIVING_STYLES = {
    "smooth turn": (smooth_commands, (0.0, 0.0, 0.0), STEPS_BEFORE_RESET),
    "sudden turn": (sudden_commands, (0.0, 0.0, 0.0), STEPS_BEFORE_RESET),
    "real commands": (real_commands, config.ROBOT_START, None),
}

print(f"slip_std = {config.SLIP_STD}, step time = {config.DT} s")
print("std per step (largest single step in brackets)")
for slip_std in (config.SLIP_STD, 0.0):
    for name, (commands, start_pose, steps_before_reset) in DRIVING_STYLES.items():
        errors = measure_prediction_error(commands, slip_std, start_pose, steps_before_reset)
        along, across, heading = errors[:, 0] * 1000, errors[:, 1] * 1000, np.degrees(errors[:, 2])
        print(f"slip {slip_std:.2f}  {name:14s} n={len(errors):3d}   "
              f"along {along.std():4.2f} mm ({np.abs(along).max():4.2f})   "
              f"across {across.std():4.2f} mm ({np.abs(across).max():4.2f})   "
              f"heading {heading.std():5.3f} deg ({np.abs(heading).max():5.3f})")
print("old Qd guess assumed:   20.00 mm (x, y)   1.500 deg (heading)")
