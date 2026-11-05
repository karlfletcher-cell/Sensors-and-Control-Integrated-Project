"""
measure_qd.py — measure how wrong the wheel-odometry prediction is per step, to choose Qd (P3).
Uses ground truth ONLY to characterise the motion model (calibration), never in the EKF.
Run from the repo root:  python -m p3_estimation.measure_qd
"""
import numpy as np
import config
from p4_simulation.tb3sim import TB3Sim, wrap
from p4_simulation.make_scene import scene_path
from p3_estimation.Estimation import odometry_to_input, motion_model

NUMBER_OF_STEPS = 1000
STEPS_BEFORE_RESET = 25          # put the robot back in the middle often so it never hits a wall

# empty room, same slip as the real runs, no camera needed
sim = TB3Sim(xml=scene_path("none"), dt=config.DT, slip_std=config.SLIP_STD,
             seed=0, render=False)

along_errors, across_errors, heading_errors = [], [], []

for step_number in range(NUMBER_OF_STEPS):
    if step_number % STEPS_BEFORE_RESET == 0:
        sim.reset(pose=(0.0, 0.0, 0.0))
        sim.odometry()           # clear the encoder count after teleporting

    # drive like the real controller does: cruise speed, smoothly changing turn rate
    forward_speed = config.V
    turn_rate = 0.5 * np.sin(0.1 * step_number)

    pose_before = sim.true_pose()
    sim.step(forward_speed, turn_rate)
    pose_after = sim.true_pose()

    # what the EKF's predict step would say, using only the wheel encoders
    left_travel, right_travel = sim.odometry()
    wheel_input = odometry_to_input(left_travel, right_travel, config.WHEEL_SEP)
    predicted_pose = motion_model(pose_before, wheel_input)

    # skip the first step after a reset: the robot starts from rest and lags the encoders by
    # about 2 mm even with slip off. That start-up error isn't slip and doesn't happen mid-run.
    if step_number % STEPS_BEFORE_RESET == 0:
        continue

    # how wrong the prediction was over this one step, measured along and across the robot
    # (world x and y only line up with the robot when it faces along x)
    x_error = pose_after[0] - predicted_pose[0]
    y_error = pose_after[1] - predicted_pose[1]
    heading = pose_after[2]
    along_errors.append(np.cos(heading) * x_error + np.sin(heading) * y_error)
    across_errors.append(-np.sin(heading) * x_error + np.cos(heading) * y_error)
    heading_errors.append(wrap(pose_after[2] - predicted_pose[2]))

along_errors = np.array(along_errors)
across_errors = np.array(across_errors)
heading_errors = np.array(heading_errors)

print(f"{len(heading_errors)} steps, slip_std = {config.SLIP_STD}, step time = {config.DT} s")
print(f"along   error per step: mean {along_errors.mean() * 1000:6.2f} mm   std {along_errors.std() * 1000:6.2f} mm")
print(f"across  error per step: mean {across_errors.mean() * 1000:6.2f} mm   std {across_errors.std() * 1000:6.2f} mm")
print(f"heading error per step: mean {np.degrees(heading_errors.mean()):6.3f} deg  "
      f"std {np.degrees(heading_errors.std()):6.3f} deg")
print(f"old Qd guess assumed:   20.00 mm (x, y)   1.500 deg (heading)")
