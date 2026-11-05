"""
measure.py — measure the camera's real range/bearing noise to choose Rn (P3).
Uses ground truth ONLY to characterise the sensor (calibration), never in the EKF.
Run from the repo root:  python -m p3_estimation.measure
(or use the VS Code run button; the sys.path line below makes that work too)
"""
import os
import sys
import numpy as np

# let Python find the other project folders when this file is run directly
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from p4_simulation.tb3sim import TB3Sim, wrap
from p2_sensing.sensing import measure

NUMBER_OF_POSES = 400

random_generator = np.random.default_rng(0)     
sim = TB3Sim()
camera_matrix = sim.camera_intrinsics()
camera_pose_on_robot = sim.T_base_cam()
marker_position = sim.marker_xy()                 # (2.6, 1.5), marker faces -x

true_ranges = []
range_errors = []
bearing_errors = []

for _ in range(NUMBER_OF_POSES):
    # pick a random pose in front of the marker, roughly facing it (within +-25 degrees)
    robot_x = random_generator.uniform(0.4, 2.3)
    robot_y = random_generator.uniform(0.8, 2.2)
    angle_to_marker = np.arctan2(marker_position[1] - robot_y, marker_position[0] - robot_x)
    robot_heading = angle_to_marker + np.deg2rad(random_generator.uniform(-25, 25))
    sim.reset(pose=(robot_x, robot_y, robot_heading))

    # what the camera says
    camera_measurement = measure(sim.camera(), camera_matrix, camera_pose_on_robot)
    if camera_measurement is None:                # marker not detected from this pose -> skip
        continue
    measured_range, measured_bearing = camera_measurement

    # what a perfect sensor would say (ground truth, used for calibration only)
    true_range, true_bearing = sim.true_range_bearing()

    range_errors.append(measured_range - true_range)
    bearing_errors.append(wrap(measured_bearing - true_bearing))
    true_ranges.append(true_range)

true_ranges = np.array(true_ranges)
range_errors = np.array(range_errors)
bearing_errors = np.array(bearing_errors)

print(len(range_errors), "detections out of", NUMBER_OF_POSES, "poses")
print("range   error: mean", round(range_errors.mean() * 100, 2), "cm    std",
      round(range_errors.std() * 100, 2), "cm")
print("bearing error: mean", round(np.degrees(bearing_errors.mean()), 2), "deg   std",
      round(np.degrees(bearing_errors.std()), 2), "deg")

# does the error get worse with distance? (decides whether a range-dependent Rn is worth it)
distance_bins = [(0.0, 0.75), (0.75, 1.25), (1.25, 1.75), (1.75, 3.0)]
for bin_start, bin_end in distance_bins:
    in_this_bin = (true_ranges >= bin_start) & (true_ranges < bin_end)
    count = in_this_bin.sum()
    if count > 5:
        print(f"  {bin_start:.2f}-{bin_end:.2f} m   n={count:3d}   "
              f"range mean {range_errors[in_this_bin].mean() * 100:5.2f} cm   "
              f"range std {range_errors[in_this_bin].std() * 100:5.2f} cm   "
              f"bearing std {np.degrees(bearing_errors[in_this_bin].std()):5.2f} deg")
print("largest range errors (cm):", np.round(np.sort(np.abs(range_errors))[-5:] * 100, 2))

largest_error_index = np.argmax(np.abs(range_errors))
print(f"largest range error: {range_errors[largest_error_index] * 100:.2f} cm "
      f"at {true_ranges[largest_error_index]:.2f} m")
without_outlier = np.delete(range_errors, largest_error_index)
print(f"range std without that one: {without_outlier.std() * 100:.2f} cm")

# same pose twice: is the error random, or fixed by the pose?
sim.reset(pose=(1.5, 1.5, 0.0))
first = measure(sim.camera(), camera_matrix, camera_pose_on_robot)
second = measure(sim.camera(), camera_matrix, camera_pose_on_robot)
print("same pose twice gives identical reading:", np.allclose(first, second))
