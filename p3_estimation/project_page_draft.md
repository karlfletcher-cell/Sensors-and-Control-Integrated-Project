<!-- DRAFT of README.md for the group to agree. Search for TODO to find what is still missing.
     The controller text is Jason's p1_control/notes.md and the estimator text is p3_estimation/estimator_section.md. -->

# TurtleBot3 visual docking (41014 Assessment 4, Track A)

A simulated TurtleBot3 Waffle docks at a charging station using only its wheel encoders, a forward camera and a LiDAR. An EKF estimates the robot's pose from odometry and an ArUco marker on the dock, and an LQR controller drives on that estimate. Ground truth is used only for logging and evaluation.

**Group:** TODO group number

| Role | Member | Responsible for |
|---|---|---|
| P1 | Jason TODO surname | LQR controller and LiDAR obstacle avoidance: `p1_control/` |
| P2 | Senith Kariyawasam | Camera calibration and marker sensing: `p2_sensing/` |
| P3 | Karl Fletcher | EKF, noise tuning, consistency and observability: `p3_estimation/` |
| P4 | Alexey TODO surname | Scene, main loop, trials and plots: `p4_simulation/`, `main.py` |

**Demo video:** TODO link

**Headline result:** 173 of 190 trial runs docked successfully (19 conditions, 10 seeds each). Ten of the 17 failures are the deliberate no-camera baseline. In the nominal condition all 10 runs dock, in about 27 s, with a final position error of 4.5 ± 0.3 cm.

## Install and run

Requires the packages in `requirements.txt` (TODO Python version).

```
pip install -r requirements.txt
python -m p4_simulation.make_scene       # build the scene (run once, or after editing make_scene.py)
python -m p4_simulation.test_drive       # check the sim works -> test_drive.png
python main.py                           # run the full pipeline
python -m p4_simulation.plot_run run_log.npz   # figures -> analysis/fig_*.png
python -m p4_simulation.run_trials       # all 190 trial runs -> analysis/trials_summary.md
```

Press **Space** with the 3D viewer focused to pause or resume. Close the viewer to stop; the log is still saved.

| Option | Effect |
|---|---|
| `--headless` | No windows |
| `--no-ekf` | Dead-reckoning baseline, camera measurements ignored |
| `--seed N` | A different random draw of slip |
| `--push` | A shove mid-run |
| `--blackout` | Marker hidden for 6 s |
| `--record` | Also writes `analysis/camera_pov.mp4` |

No screen or GPU? Prefix commands with `MUJOCO_GL=egl` (NVIDIA) or `MUJOCO_GL=osmesa` (CPU).

## Model and frames

The robot is a TurtleBot3 Waffle simulated in MuJoCo, with wheel radius 0.033 m, wheel separation 0.287 m and a control period of 0.1 s. Motion uses the unicycle model. Process noise is multiplicative per-wheel slip (std 0.02 in the main run).

| Sensor | Details | Used for |
|---|---|---|
| Wheel encoders | Wheel travel per step | EKF predict |
| Camera | 640×480 RGB, 48° vertical FOV, 0.10 m ahead of the base origin, 0.12 m up | EKF update, from the marker's range and bearing |
| LiDAR | 72 rays | Obstacle avoidance |

The marker is ArUco `DICT_4X4_50`, id 0, 0.20 m wide, at (2.6, 1.5) m and facing −x. The robot starts at (−2.0, −1.5, 0) and drives to (2.0, 1.5), just in front of the dock.

**Frame diagram (top view):**

<!-- TODO replace with a drawn figure if the group prefers one -->
```
 yW ^
    |                                                       dock
    |                                                       +---+
    |            xB (forward)              range r          |   |
    |           ^  . . . . . . . . . . . . . . . . . . . .  M   |   M = marker at (2.6, 1.5), faces -x
    |          /   bearing b is measured from xB            |   |
    |         C    camera, 0.10 m ahead of B along xB       +---+
    |        /
    | yB <- B      robot base at (x, y) with heading th
    |
    +-----------------------------------------------------------> xW

 W: world frame, fixed       B: robot base, x forward, y left, origin between the wheels
 C: camera, fixed on B       M: marker, fixed in W at a known position
```

`measure()` converts the camera's view of the marker into range and bearing from the base frame, so the camera offset is handled before the EKF.

**One control step:** read the LiDAR and command from the estimate → step the sim → EKF predict from the encoders → detect the marker → EKF update if it was seen. The loop is `run_episode` in `p4_simulation/episode.py`.

## Sensing

<!-- TODO Senith: add the calibration method and results. Results are in p2_sensing/results/ and p2_sensing/camera_calib.json. -->

TODO

## Control

<!-- Text below is p1_control/notes.md. TODO Jason: please check it and update "will be evaluated", since the sweep has been run. -->

### Modelling

The TurtleBot3 uses a differential drive system, allowing it to move forward and rotate. We used the unicycle model to represent its movement based on its position (x, y), heading (theta), forward velocity (v) and angular velocity (w).

To design the LQR controller, we linearised the model around a straight reference path using two errors. Lateral error (e_y) and heading error (e_theta). The model assumes constant forward speed, small heading errors and minimal wheel slip.

The model was discretised using a time step of 0.1 seconds. Controllability analysis showed that both tracking errors can be controlled through angular velocity when the robot is moving forward (V ≠ 0), as the controllability matrix has a rank of 2.

### Controller design

We implemented an LQR controller to follow reference paths generated by the LiDAR obstacle avoidance system. The controller uses the estimated pose from the EKF to calculate lateral and heading errors before adjusting the robot's angular velocity.

The LQR weights were selected as Q = diag(25, 14.59) and R = 1. These prioritise reducing tracking errors while limiting unnecessary steering. Higher Q values increase tracking responsiveness, while higher R values reduce control effort.

Angular velocity is limited to ±1 rad/s. When heading error exceeds 45 degrees, the robot stops moving forward and rotates towards the reference path before continuing.

The controller will be evaluated using tracking error and control effort plots, along with tests using different LQR weights to compare accuracy and steering effort.

The control effort graph shows that the robot maintained a forward velocity of approximately 0.2 m/s during most of the navigation. Angular velocity varied as the robot corrected its heading and followed the selected path. Several corrections reached the imposed limit of +/- 1 rad/s, showing that steering saturation can restrict how quickly the robot responds to large errors.

### Parameter values

| Value | Setting |
|---|---|
| Discrete model | Ad = [[1, V·Ts], [0, 1]], Bd = [0.5·V·Ts², Ts]ᵀ |
| Ts, cruise speed V | 0.1 s, 0.2 m/s |
| Q | diag(1/0.2², 1/(15°)²) = diag(25, 14.59) |
| R | 1 |
| LQR gain K | [4.08, 3.37] |
| Turn rate limit | ±1 rad/s |
| Turn-on-the-spot threshold | 45° heading error |
| Goal tolerance | 0.15 m |
| LiDAR safe distance | 0.5 m |
| Path half width | 0.25 m |
| Docked when | Facing the marker within 0.05 rad, using the estimate |

### Weight sweep

Means from `analysis/lqr_sweep.csv`.

<!-- TODO Jason: check these means and add the matching Q and R values for each row. -->
| Weights | Seeds | Success | RMS lateral (cm) | RMS heading (deg) | RMS steering (rad/s) | Saturated (%) |
|---|---|---|---|---|---|---|
| Baseline | 3 | 3/3 | 2.9 | 7.1 | 0.32 | 5.7 |
| High Q | 3 | 3/3 | 3.1 | 7.1 | 0.31 | 6.4 |
| Low Q | 3 | 3/3 | 3.0 | 7.5 | 0.29 | 3.5 |
| High R | 3 | 3/3 | 4.4 | 8.8 | 0.25 | 1.4 |
| Low R | 3 | 3/3 | 4.0 | 7.1 | 0.34 | 7.5 |

## State estimation (EKF)

The robot estimates its pose with an extended Kalman filter. It predicts from wheel odometry and corrects with the range and bearing to one ArUco marker at a known position.

### State and models

The state is the pose **x** = [x, y, θ]ᵀ in the world frame. The input is **u** = [v, ω]ᵀ, taken from the encoders.

**Motion model** (with step time Δt = 0.1 s, and θ⁺ = θ + ωΔt):

```
x⁺ = x + (v/ω)(sin θ⁺ − sin θ)
y⁺ = y + (v/ω)(cos θ − cos θ⁺)
θ⁺ = θ + ωΔt
```

When |ω| < 10⁻⁶ this becomes straight-line motion: x⁺ = x + vΔt cos θ, y⁺ = y + vΔt sin θ.

**Motion Jacobian:**

```
F = [ 1   0   (v/ω)(cos θ⁺ − cos θ) ]
    [ 0   1   (v/ω)(sin θ⁺ − sin θ) ]
    [ 0   0            1            ]
```

**Measurement model**, for a marker at (mx, my), with dx = mx − x, dy = my − y and q = dx² + dy²:

```
r = √q
b = atan2(dy, dx) − θ
```

**Measurement Jacobian:**

```
H = [ −dx/√q   −dy/√q    0 ]
    [  dy/q    −dx/q    −1 ]
```

**Bearing wrapping:** heading, bearing and the bearing innovation are wrapped to (−π, π] with atan2(sin, cos). Without this, a reading near ±180° would produce an innovation of almost 360°.

### Noise values

| Value | Setting | Reason |
|---|---|---|
| Start offset | 0.3 m, −0.2 m, 10° | Deliberate error, so the filter has to converge. |
| P₀ | diag(0.3², 0.3², (15°)²) | Covers the start offset. |
| Qd | diag(0.005², 0.005², (0.5°)²) | Measured wheel slip is 0.31 mm and 0.109° per step, so this is about 16x and 4.6x larger. The margin is there because of the camera: with a perfect camera the measured value works as well. Chosen on consistency over 30 seeds. |
| Rn | diag(0.06², (0.5°)²) | During runs the range error std is 2.49 cm, and 9% of readings are over 5 cm off (largest 24.1 cm). Bearing stays accurate, so only range was inflated. |

Rn was first set to 2 cm from a static test, which missed these outliers. Across 100 stress runs, 6 cm gave 97 successes against 91. Inflating bearing as well gave 90, and `push_large` fell to 1/10.

### Consistency

If the filter is consistent, NEES averages 3 and about 95% of steps fall below the 95% limit (7.815). Ground truth is used for scoring only, never in the filter.

| Measure (after 10 s) | Seeds 0–9 | Seeds 0–29 |
|---|---|---|
| Mean NEES (ideal 3) | 1.95 | 2.01 |
| Steps under the limit (ideal 95%) | 96.1% | 96.4% |

The filter is consistent and slightly cautious. Mean NIS is far below its ideal of 2 (0.07 after 10 s, seeds 0–9), because Rn has to cover rare large errors and most readings are much better than that.

NEES was needed as well as the ±2σ check. With the measured Qd, about 100% of steps stayed inside ±2σ on each state, but only 31.8% passed NEES. The ±2σ check looks at each state separately, and x and y are strongly correlated (median |ρ| = 0.92), so it misses errors in directions `P` considers unlikely.

### Observability

A single range and bearing marker **cannot** make the full pose observable, even when the robot is moving.

| Case | Rank of observability matrix (3 = full) |
|---|---|
| Stopped, range and bearing | 2 |
| Moving, range and bearing | 2 |
| Moving, if heading were also measured | 3 |
| Wheels only, or marker lost | 0 |

The reason is a symmetry. If the robot's whole path is swung around the marker by any angle, with the heading turned by the same angle, then range, bearing and odometry are all unchanged. The direction n = [−(y − my), x − mx, 1]ᵀ satisfies Hn = 0 at every pose, and F carries it from one step to the next, so no combination of measurements sees it.

Stacking [H; HF; HF²] at a single step gives rank 3, but that is misleading, because H and F change along the path. The rank above uses H and F evaluated along the path over 20-step windows.

Real runs match this result:

- The swing error stays near its start value of 3.5° (3.2–4.1° at the end).
- Swing angle × range matches the final position error to within 0.1 cm on all 10 seeds.
- A perfect camera with the same start error still ends about 4.6 cm off, and with no start error it ends 0.2 cm off.

Docking still works because the swing angle gives less position error at short range.

With wheels only, position sigma grows from 30 cm to about 1 m in 26 s.

### Trial results

All runs use 10 seeds per condition.

| Condition | Success | Final error (cm) | Inside ±2σ x/y/θ (%) |
|---|---|---|---|
| nominal | 10/10 | 4.5 ± 0.3 | 100/100/100 |
| no camera update | 0/10 | 47.3 ± 8.8 | 100/100/100 |
| no start error | 10/10 | 0.2 ± 0.2 | 100/100/100 |
| large start error | 10/10 | 10.1 ± 0.6 | 61/26/35 |
| R 10x smaller | 10/10 | 3.7 ± 1.0 | 100/98/99 |
| R 10x larger | 10/10 | 4.6 ± 0.2 | 100/100/100 |
| Q 10x smaller | 10/10 | 4.4 ± 0.3 | 100/100/100 |
| 8 s marker blackout | 10/10 | 4.5 ± 0.5 | 100/100/100 |
| mid-run push | 10/10 | 4.6 ± 0.4 | 99/100/100 |
| large push | 8/10 | 8.5 ± 1.1 | 92/89/92 |
| slip 0.05 | 10/10 | 4.7 ± 0.8 | 100/100/100 |
| slip 0.10 | 10/10 | 4.7 ± 1.2 | 100/100/100 |
| encoder noise | 10/10 | 4.3 ± 2.3 | 100/100/99 |

- **Camera:** without camera updates the robot never docks correctly, and it ends 32–58° off the marker.
- **Start error:** it causes nearly all of the final error. A large start error still docks, but it is outside the 95% region of P₀ (NEES 9.6), so the ±2σ coverage drops.
- **Tuning:** with R 10x smaller (variance, so std ÷ 3.16), mean NEES rises from 2.01 to 3.98 over 30 seeds. With Q 10x smaller it rises to 5.30. Accuracy barely changes, so these failures only show up in the consistency measures.
- **Large push failures:** both runs collide with the estimate 22–24 cm off. With the true pose given to the controller, neither collides, so the estimate is the cause even though the trial table labels them "controller". The final error is about double nominal, which fits the swing part of the push not being correctable.
- **Earlier tuning failure:** with Rn at 2 cm, bad range readings of 10–66 cm made the estimate jump, by up to about 1.5 m in one step, and most collisions followed. Of the 13 collision runs re-tested, 12 no longer collided with the true pose. An outlier-rejection gate was tested and rejected, because it also discards the correct readings after a push (mid-run push fell from 10/10 to 2/10).

### Limitations

- One marker leaves the swing around it unobservable, so heading at docking is about 3–4° off and depends on the start belief. The marker's orientation or a second marker would fix this.
- The filter assumes independent camera errors, but they repeat for a given pose.
- A single Gaussian Rn is a compromise, because most readings are far better than 6 cm and a few are far worse.
- Rn was chosen on the same conditions used to test it, so there is no separate test set.
- The filter can't detect when a push has left `P` too small.

## Results

A run succeeds if the robot believes it has docked and the true range to the marker is under 0.90 m with true bearing within 0.20 rad. Each condition uses seeds 0–9.

| condition | success | time to dock (s) | final err (m) mean ± sd | RMS err (m) | inside ±2σ x/y/θ (%) | failures |
|---|---|---|---|---|---|---|
| nominal | 10/10 | 27.2 | 0.045 ± 0.003 | 0.201 | 100/100/100 | — |
| no_camera_update | 0/10 | nan | 0.473 ± 0.088 | 0.268 | 100/100/100 | 10× model: dead reckoning only — process noise (slip) accumulated with no camera correction |
| start_B | 9/10 | 27.6 | 0.040 ± 0.044 | 0.211 | 100/100/100 | 1× controller: boxed in, spun on the spot until timeout |
| start_C | 10/10 | 29.1 | 0.032 ± 0.011 | 0.166 | 100/93/100 | — |
| init_err_zero | 10/10 | 27.1 | 0.002 ± 0.002 | 0.019 | 100/100/100 | — |
| init_err_large | 10/10 | 28.7 | 0.101 ± 0.006 | 0.444 | 61/26/35 | — |
| slip_0.00 | 10/10 | 27.2 | 0.043 ± 0.000 | 0.202 | 100/100/100 | — |
| slip_0.05 | 10/10 | 27.4 | 0.047 ± 0.008 | 0.206 | 100/100/100 | — |
| slip_0.10 | 10/10 | 27.5 | 0.047 ± 0.012 | 0.204 | 100/100/100 | — |
| obstacles_none | 10/10 | 25.8 | 0.047 ± 0.002 | 0.225 | 100/100/100 | — |
| obstacles_dense | 6/10 | 29.5 | 0.109 ± 0.141 | 0.224 | 100/100/100 | 4× controller: boxed in, spun on the spot until timeout |
| obstacles_shifted | 10/10 | 27.9 | 0.030 ± 0.012 | 0.236 | 94/87/100 | — |
| push_mid_run | 10/10 | 26.6 | 0.046 ± 0.004 | 0.204 | 99/100/100 | — |
| push_large | 8/10 | 23.5 | 0.085 ± 0.011 | 0.234 | 92/89/92 | 2× controller: collision (avoidance did not keep the body clear) |
| marker_blackout | 10/10 | 27.3 | 0.045 ± 0.005 | 0.201 | 100/100/100 | — |
| encoder_noise | 10/10 | 27.6 | 0.043 ± 0.023 | 0.210 | 100/100/99 | — |
| R_10x_smaller | 10/10 | 27.3 | 0.037 ± 0.010 | 0.186 | 100/98/99 | — |
| R_10x_larger | 10/10 | 27.2 | 0.046 ± 0.002 | 0.204 | 100/100/100 | — |
| Q_10x_smaller | 10/10 | 27.3 | 0.044 ± 0.003 | 0.202 | 100/100/100 | — |

Failures by cause:

- **no_camera_update (10):** the deliberate baseline. Dead reckoning cannot correct the error.
- **obstacles_dense (4), start_B (1):** the robot spins on the spot until timeout and never sees the marker. These are controller or scene failures.
- **push_large (2):** both runs collide with the estimate 22–24 cm off, and neither collides when the controller is given the true pose. The cause is the estimate, although the table labels them "controller".

<!-- TODO Alexey: add the figures. analysis/*.png is gitignored, so figures must be force-added or moved to a tracked folder, or they will not show on GitHub. -->

## Individual contributions

- **Jason:** TODO
- **Senith Kariyawasam:** TODO
- **Karl Fletcher:** TODO
- **Alexey:** TODO

## Generative AI declaration

<!-- TODO each person: name the tools used and what they were used for. -->

- **Jason:** TODO
- **Senith Kariyawasam:** TODO
- **Karl Fletcher:** TODO
- **Alexey:** TODO
