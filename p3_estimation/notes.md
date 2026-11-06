# Step 1: Camera noise

## Measurement

Run with `python -m p3_estimation.measure`, using P2's calibrated camera matrix and range correction from `camera_calib.json`.

The marker was detected in 352 of the 400 poses. Overall range error had a mean of -0.05 cm and a std of 1.05 cm, and bearing error had a mean of -0.02 deg and a std of 0.03 deg.

Split by distance, under 0.75 m (54 readings) the range mean was -0.10 cm with a std of 0.02 cm and a bearing std of 0.05 deg. From 0.75 to 1.25 m (101 readings) it was -0.09 cm, std 0.05 cm, bearing std 0.02 deg. From 1.25 to 1.75 m (93 readings) it was 0.10 cm, std 2.01 cm, bearing std 0.03 deg. From 1.75 to 3.00 m (104 readings) it was -0.10 cm, std 0.22 cm, bearing std 0.01 deg.

The five largest range errors were 0.51, 0.51, 0.54, 0.72 and 19.39 cm. The biggest was at 1.38 m, and without that one reading the range std is 0.13 cm. Taking the same pose twice gives an identical reading.

## Observations

* Calibration removed the range bias. Before it, the camera over-read by 0.57 cm past 1.75 m. Now the mean error is within 0.10 cm in every distance group.
* Range std without the outlier halved, from 0.27 cm to 0.13 cm.
* Range error still grows with distance: std 0.02 cm under 0.75 m, 0.22 cm past 1.75 m.
* The outlier is still there: 19.39 cm at 1.38 m, the same distance as before calibration. It inflates the 1.25-1.75 m group's std (2.01 cm) and the overall std: 1.05 cm with it, 0.13 cm without. The next largest error is 0.72 cm. Cause not checked yet, so I need to raise it with P2.
* Measured bearing error is very small (0.03 deg). A real camera would likely be noisier (blur, lens distortion).
* Repeated readings from the same pose are identical, so the error depends on the pose and isn't random noise.

## Chosen value

Rn = diag([0.06², (0.5°)²])

* Range 6 cm: the static test above missed how the camera behaves on the move. During real runs (seeds 0-29) the range error std is 2.49 cm, 9% of readings are over 5 cm off, and the largest is 24.1 cm. Without those readings the std is 1.21 cm. 6 cm is about 2.4x the std during a run.
* `python -m p3_estimation.plot_camera_noise` gives the same figures from 5312 readings over 30 runs: std 2.49 cm, 9.2% over 5 cm off, largest 24.1 cm. Chart: `analysis/fig_camera_noise.png`.
* Bearing 0.5 deg: well above measured 0.03 deg, to stop the filter becoming overconfident. During runs the bearing stays accurate even when range is far off (within about 0.13 deg on the worst readings), so I didn't inflate it.
* I first chose 2 cm, 0.5 deg from the static test. See "Rn re-check" in step 2 for why it changed.

## Before / after

These results are from before calibration and use the old Qd guess of 20 mm, 1.5 deg, so they can't be compared directly with later results. Run with `python -m p3_estimation.compare_rn` (headless, EKF on, seeds 0-9). Step 2 re-checks this with the new Qd and calibration.

All three settings got 10/10. The old guess of 10 cm, 3 deg had a final error of 4.36 cm and an rms error of 19.81 cm (std 0.34 cm). 2 cm, 0.5 deg, the value in `config.py` at the time, had 4.48 cm and 20.14 cm (std 0.97 cm), and its rms was lower than the old guess on 4 seeds. 1 cm, 0.5 deg had 4.87 cm and 21.27 cm (std 2.90 cm), also lower on 4 seeds.

* The measured Rn made no real difference compared with the old guess. Its rms error was lower on only 4 of the 10 seeds.
* Tightening range to 1 cm was worse: higher rms error and about 3x the spread across seeds.
* An earlier seed 0 run showed about 30% lower rms error, but that was with the old `sensing.py` and no longer holds.
* At this point I kept 2 cm, 0.5° because it came from the measured std including the outlier. It didn't improve the run. Step 2 changes it to 6 cm.

# Step 2: Process noise

## Measurement

Run with `python -m p3_estimation.measure_qd`. The encoders report the commanded wheel speeds, but the body moves with slipped wheel speeds, so this measures the error the predict step can't see. Slip std was 0.02 and the step time 0.1 s. All values are the std per step, with the largest single step in brackets.

With slip at 0.02, smooth turns (960 steps) gave 0.26 mm along the robot (0.96), 0.08 mm across (0.14) and 0.115 deg of heading (0.427). Sudden turns (960 steps) gave 0.26 mm (0.92), 0.07 mm (0.14) and 0.117 deg (0.344). The real controller's commands (270 steps) gave 0.31 mm (1.90), 0.06 mm (0.24) and 0.109 deg (0.341).

With slip off, smooth turns gave 0.00 mm (0.00) along, 0.07 mm (0.11) across and 0.002 deg (0.004). Sudden turns gave 0.00 mm (0.00), 0.07 mm (0.12) and 0.029 deg (0.069). Real commands gave 0.20 mm (1.90), 0.06 mm (0.23) and 0.012 deg (0.092).

For comparison, the old Qd guess assumed 20.00 mm in x and y and 1.500 deg in heading.

## Observations

* The old guess was far too large: about 77x the along-track std and 13x the heading std.
* The result matches a hand calculation from 2% slip at 0.2 m/s: along 0.1 × 0.2 × 0.02 / √2 = 0.28 mm, heading 0.1 × √2 × 0.2 × 0.02 / 0.287 = 0.113°.
* Errors are measured along and across the robot, because world x and y only line up with it when it faces along x.
* The first step after each reset is skipped. The robot starts from rest and moves about 2 mm less than the encoders report, even with slip off. That is start-up lag, not slip.
* Sudden command changes don't explain the margin in Qd. With slip on, the real controller's commands give about the same std as smooth driving (0.31 mm, 0.109°). The lag only shows up as rare single-step spikes of about 2 mm, and Qd describes the typical spread, not the largest step.

## Chosen value

Qd = diag([0.005², 0.005², (0.5°)²])

* x and y use the same value because the robot drives in every direction.
* That is about 16x the measured along-track std (0.31 mm) and 4.6x the heading std (0.109 deg) from real commands, so wheel slip alone does not justify it.
* I chose it on consistency, not accuracy. Over 30 seeds its rms error can't be separated from 2 mm, 0.3 deg, but its NEES is close to ideal and theirs isn't (see below). The margin covers the camera's repeating errors, which the EKF doesn't model.
* Before calibration I used 2 mm, 0.3 deg, because the measured value made runs worse. The results below replace that choice.

## Before / after

Run with `python -m p3_estimation.compare_qd` (headless, EKF on, seeds 0-9, Rn 6 cm, 0.5 deg, with calibration). All four settings got 10/10 and stayed 100% inside 2-sigma.

The old guess of 20 mm, 1.5 deg had a final error of 4.28 cm, an rms error of 19.63 cm (std 0.97 cm) and a final error of 0.35 sigma. 5 mm, 0.5 deg had 4.46 cm, 20.12 cm (std 0.70 cm) and 0.86 sigma. 2 mm, 0.3 deg had 4.52 cm, 20.29 cm (std 0.61 cm) and 1.07 sigma. The measured value of 0.4 mm, 0.17 deg had 4.74 cm, 20.24 cm (std 0.78 cm) and 1.24 sigma.

Accuracy and the 2-sigma check can't separate these, so I compared NEES over seeds 0-29 with `python -m p3_estimation.compare_qd_30`. NEES is for after 10 s, taken as the mean per seed. All three settings I ran got 30/30 and stayed 100% inside 2-sigma.

5 mm, 0.5 deg had a final error of 4.51 cm (0.87 sigma), rms 20.22 cm (std 0.72), mean NEES 2.01 and 96.4% under the limit. 2 mm, 0.3 deg had 4.56 cm (1.08 sigma), rms 20.28 cm (std 0.74), mean NEES 4.15 and 85.0% under the limit. 0.4 mm, 0.17 deg had 4.88 cm (1.27 sigma), rms 20.31 cm (std 0.79), mean NEES 13.03 and only 31.8% under the limit.

* Accuracy can't pick Qd. The paired rms difference between 5 mm and 2 mm is only -0.06 cm (standard error 0.05 cm).
* Consistency can. 5 mm is the only setting near the ideal mean of 3 with about 95% under the limit. The measured value has only 31.8% under the limit, even though it passes the 2-sigma check.
* I didn't run the 20 mm setting over 30 seeds. Its 0.35 sigma final error suggests it is too cautious.
* With the earlier Rn of 2 cm, the same comparison gave mean NEES 4.93 (5 mm), 17.60 (2 mm) and 150.83 (measured), so 5 mm was best under both Rn values.

## Why Qd needs a margin

Run with `python -m p3_estimation.explain_qd`. The perfect camera runs feed ground truth into the EKF, only for this test, to remove the camera's error. Every case got 10/10.

With the real camera and a start error, the measured Qd (0.4 mm, 0.17 deg) had a final error of 4.74 cm and rms 20.24 cm (std 0.78 cm), and 5 mm, 0.5 deg had 4.46 cm and 20.12 cm (std 0.70 cm). With the real camera and no start error, the measured Qd had 0.96 cm and 2.28 cm (std 0.21 cm), and 5 mm had 0.22 cm and 1.89 cm (std 0.24 cm).

With a perfect camera and a start error, the measured Qd had 4.71 cm and 20.32 cm (std 0.58 cm), and 5 mm had 4.60 cm and 20.18 cm (std 0.51 cm). With a perfect camera and no start error, the measured Qd had 0.32 cm and 0.75 cm (std 0.29 cm), and 5 mm had 0.20 cm and 0.69 cm (std 0.27 cm).

* With a perfect camera, the measured Qd does about as well as 5 mm, 0.5° (rms 20.32 vs 20.18 cm, and 0.75 vs 0.69 cm with no start error). So wheel slip alone doesn't need the margin.
* With the real camera and no start error, the measured Qd has a worse final error (0.96 vs 0.22 cm). The camera's error is what the margin is for.
* Likely reason: step 1 showed camera errors repeat for a given pose, so they don't average out the way the EKF assumes. A larger Qd stops the filter locking onto them. This last part is my reading of the results, not something I measured directly.
* The start error sets rms error: about 20 cm with it and under 3 cm without it, whichever Qd is used.
* A perfect camera with a start error still ends about 4.6 cm off. The observability section explains why.

## Rn re-check

Run with `python -m p3_estimation.compare_rn` with Qd 5 mm, 0.5 deg, with calibration. All four settings got 10/10.

The old guess of 10 cm, 3 deg had a final error of 4.58 cm and rms 20.34 cm (std 0.48 cm). 6 cm, 0.5 deg, now in `config.py`, had 4.46 cm and 20.12 cm (std 0.70 cm), with rms lower than the old guess on 7 seeds. 2 cm, 0.5 deg had 3.26 cm and 17.79 cm (std 1.85 cm), lower on all 10 seeds. 1 cm, 0.5 deg had 3.52 cm and 18.88 cm (std 4.71 cm), lower on 7 seeds.

On these 10 seeds alone, 2 cm looks best. The wider test below shows why that was misleading: seeds 0-9 happen to avoid its failures. Run with `python -m p3_estimation.compare_rn_stress` (30 nominal seeds plus 10 seeds of seven stress conditions, 100 runs per setting).

My first choice of 2 cm, 0.5 deg got 91/100 with 9 collisions, a nominal mean NEES of 4.93 with 86.4% under the limit, and 7/10 on `push_large`. 4 cm, 0.5 deg got 95/100 with 5 collisions, NEES 2.72, 92.9% under the limit and 8/10 on `push_large`. 6 cm, 0.5 deg, the value in `config.py`, got 97/100 with 2 collisions, NEES 2.01, 96.4% under the limit and 8/10 on `push_large`. 6.3 cm, 1.58 deg (old Rn x 10) got 90/100 with 8 collisions, NEES 2.04, 95.9% under the limit and only 1/10 on `push_large`.

* 6 cm, 0.5 deg has the most successes and fewest collisions, and no condition has fewer successes than with 2 cm.
* Range is what needed inflating. Keeping bearing at 0.5 deg matters: the setting that inflates both looks the same on nominal runs but falls to 1/10 on `push_large`.
* It costs some accuracy on good seeds: seeds 0-9 final error goes from 3.26 to 4.46 cm. Over 30 seeds it is better on average (4.51 vs 5.87 cm), and the rms spread across seeds drops from 7.37 to 0.72 cm.
* I chose this value using these same conditions, so it hasn't been tested on separate data.

## Rejecting outlier readings

I also tested the standard fix for outliers: skip any camera reading with NIS over 9.21 (99% limit). This is `python -m p3_estimation.test_nis_gate`, a wrapper only, and the gate is not in the filter. Rn was 2 cm, 0.5 deg for this test.

On nominal runs the gate didn't change the 29/30 successes or the 1 collision, but final error went from 5.87 to 10.32 cm, rms from 20.72 to 24.68 cm and mean NEES from 4.93 to 20.67. Median NEES went from 2.75 to 2.98 and steps under the limit from 86.4% to 86.7%. It rejected 6.8 readings per run.

On `init_err_large` both versions got 7/10 with 3 collisions. The gate took final error from 17.68 to 34.19 cm, rms from 44.58 to 55.68 cm, mean NEES from 16.30 to 57.57, median NEES from 13.93 to 14.07 and steps under the limit from 44.6% to 39.1%, rejecting 11.4 readings per run.

On `push_mid_run` success fell from 10/10 to 2/10, with no collisions either way. Final error went from 5.68 to 18.72 cm, rms from 19.51 to 20.18 cm, mean NEES from 5.64 to 16.99, median NEES from 5.38 to 17.61 and steps under the limit from 85.5% to 48.4%, rejecting 54.2 readings per run.

On `push_large` success fell from 7/10 with 3 collisions to 0/10 with none. Final error went from 5.90 to 142.08 cm, rms from 18.62 to 73.69 cm, mean NEES from 67.68 to 540.88, median NEES from 85.68 to 540.77 and steps under the limit from 72.4% to 21.5%, rejecting 11.2 readings per run.

* The gate made things worse, so I didn't use it. After a push the correct readings look like outliers, so the filter rejects the very readings it needs to recover (`push_mid_run` falls from 10/10 to 2/10).

## Limitations

* This only measures slip in normal driving (0.2 m/s, turn rate up to 0.5 rad/s). Pushes and collisions aren't covered by Qd.

# Step 3: Consistency check

Run with `python -m p3_estimation.check_consistency` (headless, EKF on, seeds 0-9, Rn 6 cm, 0.5 deg, Qd 5 mm, 0.5 deg, with calibration). NEES compares the real estimate error with `P`, so it needs ground truth, used here for scoring only. NIS compares the innovation with `S`. If the filter is consistent, about 95% of steps fall under the limit (7.815 for NEES, 5.991 for NIS).

All 10 runs succeeded. Over the whole run, NEES had a mean of 2.54 (ideal 3), a median of 1.67 and 92.6% of steps under the limit. NIS had a mean of 0.12 (ideal 2), a median of 0.03 and 100.0% under the limit. After 10 s, NEES had a mean of 1.95, a median of 1.32 and 96.1% under the limit, and NIS had a mean of 0.07, a median of 0.02 and 100.0% under the limit.

## Observations

* The filter is consistent, and slightly cautious: mean NEES is just under 3, and 96.1% of steps are under the limit after 10 s. Seeds 0-29 agree (mean 2.01, 96.4%).
* NIS is far below ideal. `Rn` is cautious for a typical reading because it has to cover the rare large ones. A single Gaussian can't describe both, so this is a deliberate trade-off.
* Short spikes remain. On seeds 0-9 the peak NEES per seed is between 7.0 and 15.5, down from 36-104 with Rn 2 cm. 202 steps are over the limit and 67% of those are before 10 s, when the marker has just come into view. I haven't tested exactly what causes them.
* Calibration helped a lot. With Qd 2 mm, 0.3 deg, Rn 2 cm and calibration turned off for one temporary run (uncalibrated K, no range correction), success was 9/10, mean NEES 30.03 and 47.7% of steps under the limit. With calibration those were 10/10, 12.50 and 76.1%.

## Why the 2-sigma check isn't enough

Every step-over-limit case above passes the per-state 2-sigma check: 202 of 202 on seeds 0-9. The measured Qd shows this most clearly: about 100% inside 2-sigma but only 31.8% under the NEES limit. The check looks at x, y and heading one at a time, so it can't see how their errors are linked. `P` has strong x-y correlation (median |correlation| 0.92), so an error in a direction `P` considers unlikely can pass on x and y separately and still give a large NEES.

## Using fewer camera updates

Camera errors repeat for a given pose, so I tested using only some of the readings to make them less correlated. This used a temporary wrapper around `ekf_update`, not committed, with Qd 2 mm, 0.3 deg and Rn 2 cm, 0.5 deg. It hasn't been re-run with the final values, and those no longer need this fix. Results are for after 10 s, all 10/10 success.

Using every reading gave mean NEES 7.14, 83.7% under the limit and a mean position error of 12.0 cm. Using every 5th step gave 5.46, 81.9% and 12.4 cm starting at step 0, and 4.67, 85.3% and 12.2 cm starting at step 2. Using every 5th detection gave 5.15, 85.5% and 10.5 cm.

Using every 10th step gave 4.71, 80.6% and 12.5 cm starting at step 0, and 3.45, 88.6% and 9.1 cm starting at step 5. Using every 10th detection gave 4.01, 85.7% and 10.1 cm.

* Mean NEES dropped every time, but the share under the limit and the position error depend on which readings happen to be used. At every 10th step, moving the starting step changes the position error from 12.5 cm to 9.1 cm.
* So this isn't a clear fix, and I kept every reading. The best result can't be trusted without many more seeds, because shifting the starting step shouldn't matter that much.

# Observability

Run with `python -m p3_estimation.check_observability` (headless, EKF on, seeds 0-9, using the EKF's own `F` and `H`). The rank tests use seed 0's path, with windows of 20 steps while the robot is moving. Rank 3 means the full pose is observable. Ground truth is used for scoring only. The numbers in brackets are the smallest singular values.

Stopped, range and bearing give rank 2 (7.9e-16). Moving, with `H` and `F` along the path, it is still rank 2 in every window (median 5.3e-16). Moving with `H` and `F` frozen, which is a misleading test, gives rank 3 (median 3.8e-03). Moving with `H` and `F` taken at the EKF's estimates also gives rank 3 (median 9.3e-04). If heading were also measured it would be rank 3 (median 1.0e+00).

For the swing around the marker, the largest |H n| is 4.4e-16 and the largest |F n_k - n_k+1| is 8.9e-16. With wheels only there are no `H` rows, so the rank is 0, and over 26 s sigma x goes from 30.0 to 90.3 cm, sigma y from 30.0 to 103.8 cm and heading sigma from 15.0 to 17.1 deg.

## Result: one marker never gives rank 3

* Moving does not make the pose fully observable with this sensor. The rank stays at 2, the same as when stopped.
* The reason is a symmetry. Swing the robot's whole path around the marker by any angle, turning the heading by the same angle. Range doesn't change, bearing is measured from the robot so it doesn't change, and odometry is measured in the robot's frame so it doesn't change either. Nothing the robot senses can tell the two paths apart.
* In the Jacobians, the swing direction is n = [-(y - my), x - mx, 1]. `H n = 0` at every pose, and `F` carries n at one step to n at the next, so every row of the observability matrix misses it. The script confirms both to about 1e-15.
* Stacking `[H; H F; H F²]` with `H` and `F` frozen at one step gives rank 3, but that is wrong here. The test assumes `H` and `F` are constant, and they change along the path.
* Linearising at the EKF's estimates also gives rank 3, with a very small singular value. Those Jacobians don't line up exactly from step to step, so the filter can act as if it has a tiny amount of swing information that doesn't really exist. I haven't measured how much this matters.
* Adding a heading measurement gives rank 3, with a singular value about 260x larger than the frozen test. The ArUco marker's orientation or a second marker could supply this. Neither is used at the moment.

## Check against real runs

I checked this on seeds 0-9. Every seed starts with a swing error of 3.5 deg. When the marker is first seen it is 3.6-3.9 deg, and at the end it is 3.2-4.1 deg. The heading error at the end is 3.3-4.2 deg, and the robot ends 0.69-0.71 m from the marker.

Final position errors for seeds 0 to 9 were 4.0, 4.6, 4.6, 4.1, 4.2, 5.1, 4.4, 4.8, 4.2 and 4.6 cm. Swing angle x range gave the same values, except seed 9 where it gave 4.5 cm.

The filter's own swing std starts at 3.14 deg on every seed, is 2.98-3.12 deg at first sight, and ends at 3.92-3.97 deg.

* Swing error here is the estimate's angle around the marker minus the true angle.
* The swing error stays near its starting value of 3.5 deg all run (3.2-4.1 deg at the end). The camera can't remove it, which is what rank 2 predicts.
* The final position error is almost entirely this swing error. Swing angle x range matches the final error to within 0.1 cm on all 10 seeds, and the swing error equals the heading error to within 0.1 deg.
* This explains why a perfect camera with a start error still ends about 4.6 cm off: 3.7 deg at 0.7 m is 4.5 cm. No camera accuracy can remove this error while the camera only gives range and bearing.
* It also explains the 0.20 cm final error with a perfect camera and no start error. The swing error comes from the start belief.
* The position error shrinks as the robot gets closer, because a swing angle gives less sideways movement at short range. Docking works despite the unobservable direction for this reason.
* With Rn 2 cm the swing error wandered further (0.5-3.8 deg at the end), which gave a smaller final error on some seeds. Rank 2 says the camera has no real information about this direction, so that may be the linearisation effect above and not a true improvement. I haven't proved that.
* The filter's own swing std stays about 3-4 deg all run, so `P` is honest about this direction. It dips slightly at first sight, which an exact linearisation wouldn't do.
* The heading error at docking is 3.3-4.2 deg, which is about the same as the swing error at the start.

## Wheels only

With no camera update there are no `H` rows, so nothing is observable and `P` only grows. Position sigma goes from 30 cm to about 1 m in 26 s. Heading sigma barely moves (15 to 17.1 deg), because Qd only adds about 0.5 deg per step in quadrature.

# Step 4: Trials

Run with `python -m p4_simulation.run_trials` (19 conditions, 10 seeds each, 190 runs), with Qd 5 mm, 0.5 deg and Rn 6 cm, 0.5 deg. Results are in `analysis/trials_summary.md`. 173 of 190 runs succeed. Ten of the 17 failures are the deliberate no-camera baseline.

## A bug in the batch runs

`avoid_controller` keeps its temporary waypoint on the function and nothing cleared it between runs, so every run in a batch could start by following the last run's waypoint. A single `main.py` run was never affected. I added a reset at the start of `run_episode`. With the fix, 10 of 190 run results flipped between pass and fail. `start_B` went from 1/10 to 9/10.

## Estimator rows

The percentages below are the share of steps inside 2-sigma for x, y and θ.

`nominal` is the baseline: 10/10, final error 4.5 ± 0.3 cm, 100/100/100%.

`no_camera_update` gets 0/10, with a final error of 47.3 ± 8.8 cm and 100/100/100%. The camera is essential. The robot docks where it thinks the goal is, but ends 32-58 deg off the marker.

`init_err_zero` gets 10/10, with 0.2 ± 0.2 cm and 100/100/100%. With no start error the final error almost disappears, so the start error causes nearly all of it. `init_err_large` also gets 10/10, but with 10.1 ± 0.6 cm and only 61/26/35%. It docks, but this start error has NEES 9.6 against `P0`, outside the 95% limit, so the filter is overconfident. The extra final error is likely more swing error, which the camera can't remove.

`R_10x_smaller` gets 10/10 with 3.7 ± 1.0 cm and 100/98/99%. It still works, but it's less consistent (see below). `R_10x_larger` gets 10/10 with 4.6 ± 0.2 cm and 100/100/100%, which is very cautious with little change in error. `Q_10x_smaller` gets 10/10 with 4.4 ± 0.3 cm and 100/100/100%. The 2-sigma check looks fine there, but NEES is worse (see below).

`marker_blackout` gets 10/10 with 4.5 ± 0.5 cm and 100/100/100%. 8 s without the marker makes no real difference, because odometry holds the estimate between readings.

`push_mid_run` gets 10/10 with 4.6 ± 0.4 cm and 99/100/100%, so the camera recovers a 35 cm, 15 deg push. `push_large` gets 8/10 with 8.5 ± 1.1 cm and 92/89/92%. See the failures below.

`slip_0.05` gets 10/10 with 4.7 ± 0.8 cm and 100/100/100%, at 2.5x the slip that `Qd` was measured at. `slip_0.10` gets the same result at 5x the slip, with 4.7 ± 1.2 cm. `encoder_noise` gets 10/10 with 4.3 ± 2.3 cm and 100/100/99%, even though `Qd` doesn't assume that noise. The margin handles all three.

## What if R were 10x smaller?

`R_10x_smaller` divides the variances by 10, so the stds are divided by 3.16 (1.9 cm, 0.16 deg). These figures use seeds 0-29, with NEES for after 10 s, and all four conditions got 30/30.

`nominal` has mean NEES 2.01 with 96.4% under the limit. `R_10x_smaller` has 3.98 and 89.2%, `R_10x_larger` has 1.59 and 100.0%, and `Q_10x_smaller` has 5.30 and 79.2%.

* Accuracy barely changes, but the filter becomes overconfident: mean NEES nearly doubles. `R_10x_larger` goes the other way and is over-cautious, with 100% under a 95% limit.
* Q 10x smaller is worse for consistency than R 10x smaller.
* With the earlier Rn of 2 cm, R 10x smaller (0.63 cm, 0.16 deg) gave only 7/10 success and mean NEES 195, because the filter trusted range readings that can be tens of centimetres off.
* The failure depends on more than the size of R. 1.9 cm, 0.16 deg docks on all 30 seeds, but 2 cm, 0.5 deg failed on some. So the balance between range and bearing matters. I haven't explained this fully.

## Failures

The diagnosis column reports every collision as "controller", so I checked the cause. `python -m p3_estimation.explain_failures` re-runs a case with the controller given the true pose while the EKF runs as normal. If the collision goes away, the estimate caused it. These runs can't show success, because the goal check still uses the estimate, so I only use the collision result.

* no_camera_update (10): the diagnosis is right. It is dead reckoning, and the position isn't observable from the wheels.
* push_large (2): both collisions are at about 28 s, with the estimate 22-24 cm off. They disappear with the true pose, so the estimate is the cause and the "controller" label is wrong. The push is 72 cm and 40 deg, and final error is about double nominal, which fits the swing part of the error not being correctable. I didn't measure that split directly. Seed 0 gets the same push and docks with 8 cm error, so these two runs are close to the limit.
* obstacles_dense (4) and start_B (1): the robot spins on the spot and the marker is never seen in those runs. These are controller or scene failures, and the estimator behaves correctly (100% inside 2-sigma). P1 owns these.

## How Rn 2 cm failed

With Rn 2 cm, 16 runs failed besides the no-camera row, 13 of them with collisions. Five were the spinning failures above. Most of the collisions followed one pattern:

* The camera gives a bad range reading on the approach, 10-66 cm off with bearing still within 0.13 deg. These are at about (-1.4 to -0.7, -0.7 to 0.1), at 6-11 s.
* The EKF trusts it and the estimate jumps, by up to about 1.5 m in one step.
* The estimate stays wrong, and the robot hits an obstacle near (-0.4, 0.6) at about 14 s, with the estimate up to 73 cm off.
* Given the true pose, 12 of the 13 collision runs I re-ran no longer collided. The diagnosis column blamed the controller for all of them.
* Not all of them fit this pattern. The `push_large` and `obstacles_shifted` collisions came at about 30 s near the dock, with the estimate only 3-7 cm off, so a bad range reading doesn't explain those directly. The Rn change removed `obstacles_shifted`'s failure, but I haven't worked out why.
* I haven't found what causes the bad range readings. Step 1 saw the same kind of outlier (19.39 cm at 1.38 m). P2 may want to look at it.

# Remaining limitations

* Rn was chosen on the same conditions it was tested on. There is no separate test set.
* A Gaussian Rn is a compromise for this camera: most readings are far better than 6 cm and a few are far worse.
* One marker leaves the swing around it unobservable, so heading at docking is about 3-4 deg off and depends on the start belief.
* The filter assumes independent camera errors, but they repeat for a given pose.
* After a large push the filter has no way to detect that `P` no longer covers the error.

## To do

* Agree with P4 where `estimator_section.md` goes on the project page.
* Confirm with P4 that the video shows the estimate with its 2-sigma band next to the true position.
* Tell the team about the new Rn, the waypoint reset, the UTF-8 fix and the Space-to-pause change in `episode.py`.
