# Step 1: Camera noise

## Measurement

Run with `python -m p3_estimation.measure`, using P2's updated `sensing.py` with no `camera_calib.json` yet.

```
352 detections out of 400 poses
range   error: mean 0.31 cm    std 1.08 cm
bearing error: mean 0.05 deg   std 0.03 deg
  0.00-0.75 m   n= 54   range mean -0.04 cm   range std  0.03 cm   bearing std  0.05 deg
  0.75-1.25 m   n=101   range mean  0.07 cm   range std  0.08 cm   bearing std  0.03 deg
  1.25-1.75 m   n= 93   range mean  0.46 cm   range std  2.03 cm   bearing std  0.03 deg
  1.75-3.00 m   n=104   range mean  0.57 cm   range std  0.24 cm   bearing std  0.01 deg
largest range errors (cm): [ 1.09  1.13  1.15  1.18 19.95]
largest range error: 19.95 cm at 1.38 m
range std without that one: 0.27 cm
same pose twice gives identical reading: True
```

## Observations

* Range error grows with distance: std 0.03 cm under 0.75 m, 0.24 cm past 1.75 m.
* One 19.95 cm outlier at 1.38 m inflates the 1.25–1.75 m group's std (2.03 cm) and the overall std: 1.08 cm with it, 0.27 cm without. The next largest error is 1.18 cm. Cause not checked yet -> raise with P2.
* Camera over-reads range at distance: mean error -0.04 cm under 0.75 m, +0.57 cm past 1.75 m.
* Measured bearing error is very small (0.03°). A real camera would likely be noisier (blur, lens distortion).
* Repeated readings from the same pose are identical, so the error depends on the pose, not on random noise.

## Chosen value

Rn = diag([0.02², (0.5°)²])

* Range 2 cm: ~1.9x the measured std of 1.08 cm, which includes the outlier. Without the outlier the std is 0.27 cm, so 2 cm is about 7x larger. I kept it large because the outlier does happen, and it also covers the far group's total error including bias (sqrt(0.57² + 0.24²) = 0.62 cm).
* Bearing 0.5°: well above measured 0.03°, to stop the filter becoming overconfident.

## Before / after

Run with `python -m p3_estimation.compare_rn` (headless, EKF on, seeds 0-9, with P2's updated `sensing.py`, using the old Qd guess of 20 mm, 1.5°). Step 2 re-checks this with the new Qd.

```
10 cm, 3 deg (old guess)    docked 10/10   final error 4.36 cm   rms error 19.81 cm (std 0.34 cm)   rms lower than old guess on 0 seeds
2 cm, 0.5 deg (config.py)   docked 10/10   final error 4.48 cm   rms error 20.14 cm (std 0.97 cm)   rms lower than old guess on 4 seeds
1 cm, 0.5 deg               docked 10/10   final error 4.87 cm   rms error 21.27 cm (std 2.90 cm)   rms lower than old guess on 4 seeds
```

* The measured Rn made no real difference compared with the old guess. Its rms error was lower on only 4 of the 10 seeds.
* Tightening range to 1 cm was worse: higher rms error and about 3x the spread across seeds.
* An earlier seed 0 run showed about 30% lower rms error, but that was with the old `sensing.py` and no longer holds.
* I kept 2 cm, 0.5° because it comes from the measured std including the outlier, not because it improves the run.

# Step 2: Process noise

## Measurement

Run with `python -m p3_estimation.measure_qd`. The encoders report the commanded wheel speeds, but the body moves with slipped wheel speeds, so this measures the error the predict step can't see.

```
960 steps, slip_std = 0.02, step time = 0.1 s
along   error per step: mean  -0.02 mm   std   0.26 mm
across  error per step: mean  -0.00 mm   std   0.08 mm
heading error per step: mean  0.001 deg  std  0.115 deg
old Qd guess assumed:   20.00 mm (x, y)   1.500 deg (heading)
```

## Observations

* The old guess was far too large: about 77x the along-track std and 13x the heading std.
* The result matches a hand calculation from 2% slip at 0.2 m/s: along 0.1 × 0.2 × 0.02 / √2 = 0.28 mm, heading 0.1 × √2 × 0.2 × 0.02 / 0.287 = 0.113°.
* Errors are measured along and across the robot, because world x and y only line up with it when it faces along x.
* The first step after each reset is skipped. The robot starts from rest and moves about 2 mm less than the encoders report, even with slip off. That is start-up lag, not slip.

## Chosen value

Qd = diag([0.002², 0.002², (0.3°)²])

* x and y use the same value because the robot drives in every direction.
* The measured std with a 1.5x margin (0.4 mm, 0.17°) made runs worse, so I raised it to the smallest tested value that didn't: 2 mm, 0.3°. That is about 7.7x the along-track std and 2.6x the heading std.
* I haven't worked out why the filter needs this much more than the measured slip.

## Before / after

Run with `python -m p3_estimation.compare_qd` (headless, EKF on, seeds 0-9, Rn 2 cm, 0.5°).

```
20 mm, 1.5 deg (old guess)   docked 10/10   final error  4.48 cm   rms error 20.14 cm (std 0.97 cm)   inside 2-sigma 100%   final error 0.40 sigma
5 mm, 0.5 deg                docked 10/10   final error  4.38 cm   rms error 19.87 cm (std 0.96 cm)   inside 2-sigma 100%   final error 0.93 sigma
2 mm, 0.3 deg                docked 10/10   final error  4.44 cm   rms error 19.90 cm (std 0.87 cm)   inside 2-sigma 100%   final error 1.17 sigma
0.4 mm, 0.17 deg (measured)  docked 10/10   final error  5.09 cm   rms error 20.99 cm (std 2.51 cm)   inside 2-sigma  93%   final error 1.84 sigma
```

* 5 mm and 2 mm are effectively tied with the old guess on rms error, but they give a more realistic covariance: the final error is about 1 sigma, not 0.4.
* The inside 2-sigma figure is the worst of x, y and heading for each run, averaged over seeds.

## Rn re-check with the new Qd

Run with `python -m p3_estimation.compare_rn` with Qd at 2 mm, 0.3° in `config.py`.

```
10 cm, 3 deg (old guess)    docked 10/10   final error 5.02 cm   rms error 20.93 cm (std 0.39 cm)   rms lower than old guess on 0 seeds
2 cm, 0.5 deg (config.py)   docked 10/10   final error 4.44 cm   rms error 19.90 cm (std 0.87 cm)   rms lower than old guess on 9 seeds
1 cm, 0.5 deg               docked 10/10   final error 5.58 cm   rms error 23.29 cm (std 5.85 cm)   rms lower than old guess on 3 seeds
```

* With the new Qd, the measured Rn beats the old guess on 9 of 10 seeds, up from 4 of 10 with the old Qd. Rn stays at 2 cm, 0.5°.

## Limitations

* This only measures slip in normal driving (0.2 m/s, turn rate up to 0.5 rad/s). Pushes and collisions aren't covered by Qd.

## To do

* When P2 adds `camera_calib.json`, re-run `measure.py`, `compare_rn.py` and `compare_qd.py` and update these numbers, because `measure()` will change.
* Check Rn and Qd with `run_trials.py`.
