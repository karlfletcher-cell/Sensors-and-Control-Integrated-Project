# Step 1: Camera noise

`python -m p3_estimation.measure`, with P2's calibration from `camera_calib.json`. Marker detected in 352/400 poses.

* Range error: mean -0.05 cm, std 1.05 cm. One 19.39 cm outlier at 1.38 m inflates this: std is 0.13 cm without it, and the next largest error is 0.72 cm. Cause unknown, raise with P2.
* Calibration removed the range bias: 0.57 cm over-read past 1.75 m before, mean within 0.10 cm in every distance group now.
* Range std grows with distance: 0.02 cm under 0.75 m, 0.22 cm past 1.75 m.
* Bearing error: mean -0.02 deg, std 0.03 deg. A real camera would likely be noisier.
* The same pose gives an identical reading, so the error depends on pose and isn't random.

## Chosen value

Rn = diag([0.06², (0.5°)²])

* Range 6 cm: the static test missed how the camera behaves on the move. Over 5312 readings (seeds 0-29) the std is 2.49 cm, 9.2% are over 5 cm off and the largest is 24.1 cm (`python -m p3_estimation.plot_camera_noise`, `analysis/fig_camera_noise.png`).
* Bearing 0.5 deg: well above the measured value to stop overconfidence. During runs it stays within about 0.13 deg even on the worst readings, so I didn't inflate it further.
* My first choice was 2 cm, 0.5 deg. See "Rn re-check".

# Step 2: Process noise

`python -m p3_estimation.measure_qd`. Encoders report commanded wheel speeds but the body moves with slipped ones, so this measures what the predict step can't see. Slip std 0.02, step 0.1 s.

| Commands | Along (mm) | Across (mm) | Heading (deg) |
|---|---|---|---|
| Smooth turns | 0.26 | 0.08 | 0.115 |
| Sudden turns | 0.26 | 0.07 | 0.117 |
| Real controller | 0.31 | 0.06 | 0.109 |

* The old guess (20 mm, 1.5 deg) was about 77x the along-track std and 13x the heading std.
* Hand calculation from 2% slip at 0.2 m/s agrees: 0.28 mm along, 0.113° heading.
* The first step after each reset is skipped: start-up lag of about 2 mm, not slip.
* Only covers normal driving (0.2 m/s, up to 0.5 rad/s). Qd doesn't cover pushes or collisions.

## Chosen value

Qd = diag([0.005², 0.005², (0.5°)²])

About 16x the measured along-track std and 4.6x the heading std, so slip alone doesn't justify it. I chose it for consistency. Accuracy can't separate the settings (paired rms difference between 5 mm and 2 mm: -0.06 cm, standard error 0.05 cm).

`python -m p3_estimation.compare_qd_30`, seeds 0-29, NEES after 10 s. All 30/30 and 100% inside 2-sigma.

| Qd | Final (cm) | RMS (cm) | Mean NEES (ideal 3) | Under limit |
|---|---|---|---|---|
| 5 mm, 0.5 deg | 4.51 | 20.22 | 2.01 | 96.4% |
| 2 mm, 0.3 deg | 4.56 | 20.28 | 4.15 | 85.0% |
| 0.4 mm, 0.17 deg (measured) | 4.88 | 20.31 | 13.03 | 31.8% |

20 mm wasn't run over 30 seeds. Its 0.35 sigma final error on seeds 0-9 suggests it is too cautious.

## Why Qd needs a margin

`python -m p3_estimation.explain_qd`, feeding ground truth to the EKF for this test only.

* With a perfect camera, the measured Qd matches 5 mm (rms 20.32 vs 20.18 cm), so slip alone doesn't need the margin.
* With the real camera and no start error, the measured Qd is worse (final 0.96 vs 0.22 cm).
* Likely reason, not measured directly: camera errors repeat for a pose and don't average out, and a larger Qd stops the filter locking onto them.
* Start error sets rms: about 20 cm with it, under 3 cm without it.

## Rn re-check

`python -m p3_estimation.compare_rn_stress`: 30 nominal seeds plus 10 seeds of seven stress conditions, 100 runs per setting. On seeds 0-9 alone 2 cm looked best (final 3.26 vs 4.46 cm), but those seeds happen to avoid its failures.

| Rn | Success | Collisions | Nominal NEES | Under limit | `push_large` |
|---|---|---|---|---|---|
| 2 cm, 0.5 deg | 91 | 9 | 4.93 | 86.4% | 7/10 |
| 4 cm, 0.5 deg | 95 | 5 | 2.72 | 92.9% | 8/10 |
| 6 cm, 0.5 deg | 97 | 2 | 2.01 | 96.4% | 8/10 |
| 6.3 cm, 1.58 deg | 90 | 8 | 2.04 | 95.9% | 1/10 |

* Only range needed inflating. Inflating bearing too looks the same on nominal runs but collapses on `push_large`.
* Over 30 seeds, 6 cm beats 2 cm on final error (4.51 vs 5.87 cm) and rms spread (0.72 vs 7.37 cm).
* Chosen on the same conditions it was tested on.

## Rejecting outliers

`python -m p3_estimation.test_nis_gate`, Rn 2 cm: skip readings with NIS over 9.21. I didn't use it. After a push the correct readings look like outliers, so the filter rejects what it needs to recover: `push_mid_run` fell from 10/10 to 2/10 and `push_large` from 7/10 to 0/10.

# Step 3: Consistency check

`python -m p3_estimation.check_consistency`, seeds 0-9, all 10/10. Limits: NEES 7.815, NIS 5.991.

| After 10 s | Mean | Median | Under limit |
|---|---|---|---|
| NEES (ideal 3) | 1.95 | 1.32 | 96.1% |
| NIS (ideal 2) | 0.07 | 0.02 | 100.0% |

* Consistent and slightly cautious. Seeds 0-29 agree (mean 2.01, 96.4%).
* NIS is far below ideal because `Rn` has to cover rare large readings. A single Gaussian can't describe both.
* Short spikes remain: peak NEES per seed is 7.0-15.5, down from 36-104 with Rn 2 cm. 67% of the 202 steps over the limit are before 10 s, when the marker has just come into view. Cause untested.
* Calibration helped (Qd 2 mm, Rn 2 cm): without it 9/10, NEES 30.03, 47.7% under limit. With it 10/10, 12.50, 76.1%.
* The 2-sigma check isn't enough: it passed all 202 over-limit steps. It looks at each state alone, and `P` has strong x-y correlation (median 0.92).
* Using fewer camera updates gave no clear gain. At every 10th step, moving the start step changed position error from 12.5 to 9.1 cm, so the best result can't be trusted. Tested with the old values only, and I kept every reading.

# Observability

`python -m p3_estimation.check_observability`, 20-step windows on seed 0's path.

| Case | Rank |
|---|---|
| Stopped | 2 |
| Moving, `H` and `F` along the path | 2 |
| `H` and `F` frozen (misleading) | 3 |
| `H` and `F` at EKF estimates | 3 (tiny singular value, 9.3e-04) |
| With heading measured | 3 (1.0) |
| Wheels only | 0 |

* One marker never gives rank 3. Swing the whole path around the marker and turn the heading by the same angle: range, bearing and odometry don't change.
* The swing direction is n = [-(y - my), x - mx, 1]. `H n = 0` and `F` carries n from step to step, both confirmed to about 1e-15.
* The frozen test is wrong here because `H` and `F` change along the path.
* Marker orientation or a second marker could give heading. Neither is used.
* Wheels only: position sigma grows from 30 cm to about 1 m in 26 s.

## Check against real runs

Seeds 0-9. Swing error starts at 3.5 deg and ends at 3.2-4.1 deg, so the camera can't remove it, as rank 2 predicts.

* Swing angle x range matches the final position error to within 0.1 cm on every seed.
* That explains the 4.6 cm final error with a perfect camera: 3.7 deg at 0.7 m is 4.5 cm.
* Position error shrinks near the marker, which is why docking still works.
* Heading error at docking is 3.3-4.2 deg.
* `P` is honest about this direction: swing std stays about 3-4 deg.

# Step 4: Trials

`python -m p4_simulation.run_trials`: 19 conditions, 10 seeds each. The committed results (P1's re-run) give 169/190, and 10 of the 21 failures are the deliberate no-camera baseline. Full results in `analysis/trials_summary.md`.

The same code gives 173/190 on my computer, and repeating my run from a clean copy matched all 190 final errors exactly, so the gap comes from the computer. Four runs change: `start_B` seeds 0 and 2, `obstacles_dense` seed 2 and `push_large` seed 1. Most likely rendering or numerical differences, not isolated. The table below uses the committed results.

Batch-run bug: `avoid_controller` kept its temporary waypoint between runs. I added a reset at the start of `run_episode`. That flipped 10 of 190 results, and `start_B` went from 1/10 to 9/10.

| Condition | Success | Final error (cm) | Note |
|---|---|---|---|
| `nominal` | 10/10 | 4.5 ± 0.3 | Baseline |
| `no_camera_update` | 0/10 | 47.4 ± 8.8 | Camera is essential |
| `init_err_zero` | 10/10 | 0.2 ± 0.2 | Start error causes nearly all final error |
| `init_err_large` | 10/10 | 10.1 ± 0.5 | Overconfident: 62/25/36% inside 2-sigma |
| `marker_blackout` | 10/10 | 4.5 ± 0.5 | Odometry holds through 8 s |
| `push_mid_run` | 10/10 | 4.5 ± 0.4 | Recovers a 35 cm, 15 deg push |
| `push_large` | 7/10 | 9.2 ± 2.2 | See failures |
| `slip_0.10` | 10/10 | 4.5 ± 1.2 | 5x the measured slip |
| `encoder_noise` | 10/10 | 4.1 ± 2.4 | Covered by the Qd margin |

R and Q sensitivity, seeds 0-29, NEES after 10 s, all 30/30:

| Condition | Mean NEES | Under limit |
|---|---|---|
| `nominal` | 2.01 | 96.4% |
| `R_10x_smaller` | 3.98 | 89.2% |
| `R_10x_larger` | 1.59 | 100.0% |
| `Q_10x_smaller` | 5.30 | 79.2% |

Accuracy barely changes, but consistency does. With Rn 2 cm, R 10x smaller gave 7/10 and mean NEES 195. 1.9 cm, 0.16 deg docks on all 30 seeds while 2 cm, 0.5 deg failed on some, so the range-bearing balance matters. I haven't fully explained this.

## Failures

The diagnosis column labels every collision "controller", so I checked with `python -m p3_estimation.explain_failures`, which gives the controller the true pose. If the collision goes away, the estimate caused it.

* `no_camera_update` (10): dead reckoning, diagnosis correct.
* `push_large` (3): 72 cm, 40 deg push. Seeds 2 and 5 collide at about 28 s with the estimate 22-24 cm off, and those collisions disappear with the true pose, so the estimate is the cause. Seed 1 fails only on P1's computer, so I couldn't re-test it.
* `obstacles_dense` (5), `start_B` (3): the robot spins and never sees the marker. Controller or scene failures, for P1.

## How Rn 2 cm failed

16 failures besides the no-camera row, 13 with collisions. Bad range readings (10-66 cm off, bearing still within 0.13 deg) made the estimate jump by up to about 1.5 m, and the robot hit an obstacle near (-0.4, 0.6) at about 14 s. 12 of 13 collisions went away with the true pose. The bad readings' cause is unknown, and P2 may want to look at it.

# Remaining limitations

* Rn has no separate test set.
* A Gaussian Rn is a compromise: most readings are far better than 6 cm and a few far worse.
* One marker leaves the swing unobservable, so docking heading is about 3-4 deg off and depends on the start belief.
* The filter assumes independent camera errors, but they repeat for a pose.
* After a large push the filter can't detect that `P` no longer covers the error.
* Results differ slightly between computers: 169/190 committed, 173/190 on mine, with the same code.

## To do

* Agree with P4 where `estimator_section.md` goes on the project page.
* Confirm with P4 that the video shows the estimate with its 2-sigma band next to the true position.
* Tell the team about the new Rn, the waypoint reset, the UTF-8 fix and the Space-to-pause change in `episode.py`.
