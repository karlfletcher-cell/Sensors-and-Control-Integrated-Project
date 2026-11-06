# Step 1: Camera noise

## Measurement

Run with `python -m p3_estimation.measure`, using P2's calibrated camera matrix and range correction from `camera_calib.json`.

```
352 detections out of 400 poses
range   error: mean -0.05 cm    std 1.05 cm
bearing error: mean -0.02 deg   std 0.03 deg
  0.00-0.75 m   n= 54   range mean -0.10 cm   range std  0.02 cm   bearing std  0.05 deg
  0.75-1.25 m   n=101   range mean -0.09 cm   range std  0.05 cm   bearing std  0.02 deg
  1.25-1.75 m   n= 93   range mean  0.10 cm   range std  2.01 cm   bearing std  0.03 deg
  1.75-3.00 m   n=104   range mean -0.10 cm   range std  0.22 cm   bearing std  0.01 deg
largest range errors (cm): [ 0.51  0.51  0.54  0.72 19.39]
largest range error: 19.39 cm at 1.38 m
range std without that one: 0.13 cm
same pose twice gives identical reading: True
```

## Observations

* Calibration removed the range bias. Before it, the camera over-read by 0.57 cm past 1.75 m. Now the mean error is within 0.10 cm in every distance group.
* Range std without the outlier halved, from 0.27 cm to 0.13 cm.
* Range error still grows with distance: std 0.02 cm under 0.75 m, 0.22 cm past 1.75 m.
* The outlier is still there: 19.39 cm at 1.38 m, the same distance as before calibration. It inflates the 1.25-1.75 m group's std (2.01 cm) and the overall std: 1.05 cm with it, 0.13 cm without. The next largest error is 0.72 cm. Cause not checked yet -> raise with P2.
* Measured bearing error is very small (0.03 deg). A real camera would likely be noisier (blur, lens distortion).
* Repeated readings from the same pose are identical, so the error depends on the pose, not on random noise.

## Chosen value

Rn = diag([0.02², (0.5°)²])

* Range 2 cm: ~1.9x the measured std of 1.05 cm, which includes the outlier. Without the outlier the std is 0.13 cm, so 2 cm is about 15x larger. I kept it large because the outlier does happen.
* Bearing 0.5 deg: well above measured 0.03 deg, to stop the filter becoming overconfident.

## Before / after

This table is from before calibration and uses the old Qd guess of 20 mm, 1.5 deg, so it can't be compared directly with later results. Run with `python -m p3_estimation.compare_rn` (headless, EKF on, seeds 0-9). Step 2 re-checks this with the new Qd and calibration.

```
10 cm, 3 deg (old guess)    success 10/10   final error 4.36 cm   rms error 19.81 cm (std 0.34 cm)   rms lower than old guess on 0 seeds
2 cm, 0.5 deg (config.py)   success 10/10   final error 4.48 cm   rms error 20.14 cm (std 0.97 cm)   rms lower than old guess on 4 seeds
1 cm, 0.5 deg               success 10/10   final error 4.87 cm   rms error 21.27 cm (std 2.90 cm)   rms lower than old guess on 4 seeds
```

* The measured Rn made no real difference compared with the old guess. Its rms error was lower on only 4 of the 10 seeds.
* Tightening range to 1 cm was worse: higher rms error and about 3x the spread across seeds.
* An earlier seed 0 run showed about 30% lower rms error, but that was with the old `sensing.py` and no longer holds.
* I kept 2 cm, 0.5° because it comes from the measured std including the outlier, not because it improves the run.

# Step 2: Process noise

## Measurement

Run with `python -m p3_estimation.measure_qd`. The encoders report the commanded wheel speeds, but the body moves with slipped wheel speeds, so this measures the error the predict step can't see.

```
slip_std = 0.02, step time = 0.1 s
std per step (largest single step in brackets)
slip 0.02  smooth turn    n=960   along 0.26 mm (0.96)   across 0.08 mm (0.14)   heading 0.115 deg (0.427)
slip 0.02  sudden turn    n=960   along 0.26 mm (0.92)   across 0.07 mm (0.14)   heading 0.117 deg (0.344)
slip 0.02  real commands  n=270   along 0.31 mm (1.90)   across 0.06 mm (0.24)   heading 0.109 deg (0.341)
slip 0.00  smooth turn    n=960   along 0.00 mm (0.00)   across 0.07 mm (0.11)   heading 0.002 deg (0.004)
slip 0.00  sudden turn    n=960   along 0.00 mm (0.00)   across 0.07 mm (0.12)   heading 0.029 deg (0.069)
slip 0.00  real commands  n=270   along 0.20 mm (1.90)   across 0.06 mm (0.23)   heading 0.012 deg (0.092)
old Qd guess assumed:   20.00 mm (x, y)   1.500 deg (heading)
```

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
* I chose it on consistency, not accuracy. Over 30 seeds its rms error can't be separated from 2 mm, 0.3 deg, but its NEES is much closer to ideal (see below). The margin covers the camera's repeating errors, which the EKF doesn't model.
* Before calibration I used 2 mm, 0.3 deg, because the measured value made runs worse. The results below replace that choice.

## Before / after

Run with `python -m p3_estimation.compare_qd` (headless, EKF on, seeds 0-9, Rn 2 cm, 0.5 deg, with calibration).

```
20 mm, 1.5 deg (old guess)   success 9/10   final error 13.33 cm   rms error 26.02 cm (std 22.42 cm)   inside 2-sigma  91%   final error 0.86 sigma
5 mm, 0.5 deg                success 10/10   final error  3.26 cm   rms error 17.79 cm (std 1.85 cm)   inside 2-sigma 100%   final error 0.69 sigma
2 mm, 0.3 deg                success 10/10   final error  4.12 cm   rms error 19.77 cm (std 2.35 cm)   inside 2-sigma  92%   final error 1.16 sigma
0.4 mm, 0.17 deg (measured)  success 10/10   final error  3.74 cm   rms error 18.70 cm (std 2.51 cm)   inside 2-sigma  85%   final error 1.13 sigma
```

* The old guess is now clearly the worst: one failed run and a much larger spread across seeds.
* 5 mm, 0.5 deg has the lowest rms error and final error, and the only 100% inside 2-sigma. The gap to the other two (about 1-2 cm rms) is about the size of the spread across seeds (about 2 cm), so 10 seeds don't settle it. The 30-seed run below checks this.
* The measured value no longer does worse than 2 mm, 0.3 deg, which reverses the result from before calibration.
* The inside 2-sigma figure is the worst of x, y and heading for each run, averaged over seeds.
* Before calibration, 2 mm, 0.3 deg showed 100% here. Now it shows 92%, and nearly all of that drop is from one seed. Eight seeds are still at 100%, seed 4 is at 90% and seed 5 is at 27%.

Re-run over seeds 0-29 with a temporary script, not committed. NEES is for after 10 s, taken as the mean per seed. Same episode and Rn as above.

```
5 mm, 0.5 deg      success 29/30  final  5.87 cm  rms 20.72 cm (std  7.37)  inside 2-sigma 93%  final 0.95 sigma  mean NEES   4.93  under limit 86.4%
2 mm, 0.3 deg      success 29/30  final  5.50 cm  rms 22.67 cm (std 17.04)  inside 2-sigma 94%  final 1.38 sigma  mean NEES  16.91  under limit 82.3%
0.4 mm, 0.17 deg   success 28/30  final 10.30 cm  rms 23.95 cm (std 23.32)  inside 2-sigma 85%  final 2.54 sigma  mean NEES 150.23  under limit 33.4%
```

* Rms error doesn't separate 5 mm from 2 mm. The paired difference is -1.95 cm with a standard error of 3.46 cm, and 5 mm is lower on only 16 of 30 seeds.
* Consistency does separate them. 5 mm has about a third of the mean NEES of 2 mm, and its final error is 0.95 sigma, against 1.38.
* The measured value is clearly the worst on 30 seeds: 33.4% under the limit and mean NEES 150.
* The first 10 seeds flatter 5 mm, 0.5 deg. Its rms error goes from 17.79 cm to 20.72 cm with 30 seeds, because seeds 23 and 24 are poor (see step 3).

## Why Qd needs a margin

Run with `python -m p3_estimation.explain_qd`. The perfect camera runs feed ground truth into the EKF, only for this test, to remove the camera's error.

```
real camera     start error     0.4 mm, 0.17 deg (measured)  success 10/10   final error  3.74 cm   rms error 18.70 cm (std 2.51 cm)
real camera     start error     5 mm, 0.5 deg (config.py)    success 10/10   final error  3.26 cm   rms error 17.79 cm (std 1.85 cm)
real camera     no start error  0.4 mm, 0.17 deg (measured)  success 10/10   final error  0.68 cm   rms error  3.11 cm (std 0.95 cm)
real camera     no start error  5 mm, 0.5 deg (config.py)    success 10/10   final error  0.35 cm   rms error  2.14 cm (std 0.45 cm)
perfect camera  start error     0.4 mm, 0.17 deg (measured)  success 10/10   final error  4.65 cm   rms error 20.10 cm (std 0.61 cm)
perfect camera  start error     5 mm, 0.5 deg (config.py)    success 10/10   final error  4.54 cm   rms error 20.02 cm (std 0.55 cm)
perfect camera  no start error  0.4 mm, 0.17 deg (measured)  success 10/10   final error  0.32 cm   rms error  0.76 cm (std 0.28 cm)
perfect camera  no start error  5 mm, 0.5 deg (config.py)    success 10/10   final error  0.18 cm   rms error  0.69 cm (std 0.27 cm)
```

* With a perfect camera, the measured Qd does about as well as 5 mm, 0.5° (rms 20.10 vs 20.02 cm, and 0.76 vs 0.69 cm with no start error). So wheel slip alone doesn't need the margin.
* With the real camera, the measured Qd is worse even with no start error (rms 3.11 vs 2.14 cm, spread 0.95 vs 0.45 cm). So the start error isn't the cause either. It is the camera's error.
* Likely reason: step 1 showed camera errors repeat for a given pose, so they don't average out the way the EKF assumes. A larger Qd stops the filter locking onto them. This last part is my reading of the results, not something I measured directly.
* The start error also explains why rms error barely changes with Qd: rms is about 18-20 cm with it and about 3 cm or less without it, whichever Qd is used.

## Rn re-check with the new Qd

Run with `python -m p3_estimation.compare_rn` with Qd at 5 mm, 0.5 deg in `config.py`, with calibration.

```
10 cm, 3 deg (old guess)    success 10/10   final error 4.58 cm   rms error 20.34 cm (std 0.48 cm)   rms lower than old guess on 0 seeds
2 cm, 0.5 deg (config.py)   success 10/10   final error 3.26 cm   rms error 17.79 cm (std 1.85 cm)   rms lower than old guess on 10 seeds
1 cm, 0.5 deg               success 10/10   final error 3.52 cm   rms error 18.88 cm (std 4.71 cm)   rms lower than old guess on 7 seeds
```

* The measured Rn beats the old guess on all 10 seeds, with a lower mean rms and final error, but its spread across seeds is larger (1.85 vs 0.48 cm).
* 1 cm is worse than 2 cm on mean rms error, final error and spread (about 2.5x). Rn stays at 2 cm, 0.5 deg.

## Limitations

* This only measures slip in normal driving (0.2 m/s, turn rate up to 0.5 rad/s). Pushes and collisions aren't covered by Qd.

# Step 3: Consistency check

Run with `python -m p3_estimation.check_consistency` (headless, EKF on, seeds 0-9, Rn 2 cm, 0.5 deg, Qd 5 mm, 0.5 deg, with calibration). NEES compares the real estimate error with `P`, so it needs ground truth, used here for scoring only. NIS compares the innovation with `S`. If the filter is consistent, about 95% of steps fall under the limit (7.815 for NEES, 5.991 for NIS).

```
success 10/10
Whole run:
  NEES: mean   5.59 (ideal 3)   median  1.53   under 95% limit  82.6%
  NIS : mean   0.82 (ideal 2)   median  0.05   under 95% limit  96.0%
After 10 s:
  NEES: mean   2.41 (ideal 3)   median  0.99   under 95% limit  92.4%
  NIS : mean   0.37 (ideal 2)   median  0.04   under 95% limit  98.4%
```

## Observations

* After 10 s, NEES is close to consistent on these seeds: mean 2.41, with 92.4% of steps under the limit. The old Qd of 2 mm, 0.3 deg gave a mean of 7.14 and 83.7%.
* Over the whole run NEES is still high (mean 5.59, 82.6%), because of short spikes early on. The median is 1.53, but the peak on each seed is between 36 and 104.
* The spikes come with the first camera readings. On seeds 0-9 the marker is first seen at 5.5-6.7 s, 80% of the steps over the limit fall between 6.5 s and 10.8 s, and 73% are before 10 s. The timing suggests the error from the deliberate start offset is being measured against a `P` that has just shrunk, but I haven't tested that directly.
* Calibration helped a lot. With Qd 2 mm, 0.3 deg and calibration turned off for one temporary run (uncalibrated K, no range correction), success was 9/10, mean NEES 30.03 and 47.7% of steps under the limit. With calibration those were 10/10, 12.50 and 76.1%.
* NIS is low, so `Rn` is cautious compared with the camera's typical error. That matches step 1, where I set range noise well above the std without the outlier.

## Seeds 10-29

Seeds 0-9 look better than the wider picture. A temporary script ran seeds 0-29 with these settings, and mean NEES after 10 s rises to 4.93, with 86.4% under the limit.

* Seed 5, which was at 27% inside 2-sigma with the old Qd, is now at 100%.
* 26 of 30 seeds are at 99% or better inside 2-sigma. Seeds 17 and 18 are at 82-83%. Seed 23 is at 26% with a peak NEES of 359, though it still docks.
* Seed 24 fails. It collides, sees the marker on only 5% of steps, runs until timeout (about 80 s) and ends 58.5 cm off (3.1 sigma). The diagnosis column blames the controller. I haven't checked whether a bad estimate caused the collision.

## Why the 2-sigma check missed this

Every one of seeds 0-9 is at 100% on the per-state 2-sigma check, yet NEES still spikes. The check looks at x, y and heading one at a time, so it can't see how their errors are linked. `P` has strong x-y correlation (median |correlation| 0.97). An error in a direction `P` considers unlikely can pass the check on both x and y and still give a large NEES. On seeds 0-9, 473 of the 474 steps over the NEES limit had all three states inside 2-sigma. Over seeds 0-29 it is 1248 of 2241 (56%), because seeds 23 and 24 have real errors outside 2-sigma.

## Using fewer camera updates

Camera errors repeat for a given pose, so I tested using only some of the readings to make them less correlated. This used a temporary wrapper around `ekf_update`, not committed, with Qd 2 mm, 0.3 deg. It hasn't been re-run with 5 mm, 0.5 deg. Results are for after 10 s, all 10/10 success.

| Camera updates used | Mean NEES | Under limit | Mean position error |
|---|---|---|---|
| Every reading | 7.14 | 83.7% | 12.0 cm |
| Every 5th step, starting at step 0 | 5.46 | 81.9% | 12.4 cm |
| Every 5th step, starting at step 2 | 4.67 | 85.3% | 12.2 cm |
| Every 5th detection | 5.15 | 85.5% | 10.5 cm |
| Every 10th step, starting at step 0 | 4.71 | 80.6% | 12.5 cm |
| Every 10th step, starting at step 5 | 3.45 | 88.6% | 9.1 cm |
| Every 10th detection | 4.01 | 85.7% | 10.1 cm |

* Mean NEES dropped every time, but the share under the limit and the position error depend on which readings happen to be used. At every 10th step, moving the starting step changes the position error from 12.5 cm to 9.1 cm.
* So this isn't a clear fix, and I kept every reading. The best row can't be trusted without many more seeds, because shifting the starting step shouldn't matter that much.

## To do

* Find out why seed 24 collides and why seed 23 has such a large NEES.
* Check Rn and Qd with `run_trials.py`.
