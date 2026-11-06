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
| no camera update | 0/10 | 47.4 ± 8.8 | 100/100/100 |
| no start error | 10/10 | 0.2 ± 0.2 | 100/100/100 |
| large start error | 10/10 | 10.1 ± 0.5 | 62/25/36 |
| R 10x smaller | 10/10 | 4.2 ± 0.9 | 100/98/100 |
| R 10x larger | 10/10 | 4.6 ± 0.2 | 100/100/100 |
| Q 10x smaller | 10/10 | 4.4 ± 0.4 | 100/100/100 |
| 8 s marker blackout | 10/10 | 4.5 ± 0.5 | 100/100/100 |
| mid-run push | 10/10 | 4.5 ± 0.4 | 99/100/100 |
| large push | 7/10 | 9.2 ± 2.2 | 91/89/88 |
| slip 0.05 | 10/10 | 4.8 ± 0.8 | 100/100/100 |
| slip 0.10 | 10/10 | 4.5 ± 1.2 | 100/100/99 |
| encoder noise | 10/10 | 4.1 ± 2.4 | 100/100/99 |

- **Camera:** without camera updates the robot never docks correctly, and it ends 32–58° off the marker.
- **Start error:** it causes nearly all of the final error. A large start error still docks, but it is outside the 95% region of P₀ (NEES 9.6), so the ±2σ coverage drops.
- **Tuning:** with R 10x smaller (variance, so std ÷ 3.16), mean NEES rises from 2.01 to 3.98 over 30 seeds. With Q 10x smaller it rises to 5.30. Accuracy barely changes, so these failures only show up in the consistency measures.
- **Large push failures:** two of the three runs (seeds 2 and 5) collide with the estimate 22–24 cm off. With the true pose given to the controller, neither collides, so the estimate is the cause even though the trial table labels them "controller". Seed 1 fails only on the computer that produced the committed results, so it has not been re-tested this way. The final error is about double nominal, which fits the swing part of the push not being correctable.
- **Earlier tuning failure:** with Rn at 2 cm, bad range readings of 10–66 cm made the estimate jump, by up to about 1.5 m in one step, and most collisions followed. Of the 13 collision runs re-tested, 12 no longer collided with the true pose. An outlier-rejection gate was tested and rejected, because it also discards the correct readings after a push (mid-run push fell from 10/10 to 2/10).

### Limitations

- One marker leaves the swing around it unobservable, so heading at docking is about 3–4° off and depends on the start belief. The marker's orientation or a second marker would fix this.
- The filter assumes independent camera errors, but they repeat for a given pose.
- A single Gaussian Rn is a compromise, because most readings are far better than 6 cm and a few are far worse.
- Rn was chosen on the same conditions used to test it, so there is no separate test set.
- The filter can't detect when a push has left `P` too small.
- Results can differ slightly between computers. The committed trials give 169/190. The same code gave 173/190 on a second computer, where `start_B` seeds 0 and 2, `obstacles_dense` seed 2 and `push_large` seed 1 succeed. Repeating the run on that second computer reproduced all 190 final errors exactly, so each computer is repeatable and the difference comes from the machine, most likely small rendering or numerical differences changing the camera readings. We did not isolate the exact cause.
