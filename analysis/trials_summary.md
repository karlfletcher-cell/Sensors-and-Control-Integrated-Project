Trials: 10 seeds per condition (seeds 0..9). Time to dock averaged over successful runs.

| condition | success | time to dock (s) | final err (m) mean ± sd | RMS err (m) | inside ±2σ x/y/θ (%) | failures |
|---|---|---|---|---|---|---|
| nominal | 10/10 | 27.2 | 0.045 ± 0.003 | 0.203 | 100/100/100 | — |
| no_camera_update | 0/10 | nan | 0.474 ± 0.088 | 0.268 | 100/100/100 | 10× model: dead reckoning only — process noise (slip) accumulated with no camera correction |
| start_B | 7/10 | 27.6 | 0.070 ± 0.067 | 0.208 | 100/100/100 | 3× controller: boxed in, spun on the spot until timeout |
| start_C | 10/10 | 29.3 | 0.031 ± 0.013 | 0.165 | 100/92/100 | — |
| init_err_zero | 10/10 | 27.0 | 0.002 ± 0.002 | 0.020 | 100/100/100 | — |
| init_err_large | 10/10 | 28.9 | 0.101 ± 0.005 | 0.445 | 62/25/36 | — |
| slip_0.00 | 10/10 | 27.3 | 0.042 ± 0.000 | 0.200 | 100/100/100 | — |
| slip_0.05 | 10/10 | 27.4 | 0.048 ± 0.008 | 0.207 | 100/100/100 | — |
| slip_0.10 | 10/10 | 27.4 | 0.045 ± 0.012 | 0.203 | 100/100/99 | — |
| obstacles_none | 10/10 | 25.8 | 0.047 ± 0.002 | 0.226 | 100/100/100 | — |
| obstacles_dense | 5/10 | 29.2 | 0.209 ± 0.194 | 0.283 | 100/100/100 | 5× controller: boxed in, spun on the spot until timeout |
| obstacles_shifted | 10/10 | 28.1 | 0.036 ± 0.006 | 0.205 | 98/98/100 | — |
| push_mid_run | 10/10 | 26.6 | 0.045 ± 0.004 | 0.205 | 99/100/100 | — |
| push_large | 7/10 | 23.5 | 0.092 ± 0.022 | 0.232 | 91/89/88 | 3× controller: collision (avoidance did not keep the body clear) |
| marker_blackout | 10/10 | 27.2 | 0.045 ± 0.005 | 0.202 | 100/100/100 | — |
| encoder_noise | 10/10 | 27.5 | 0.041 ± 0.024 | 0.209 | 100/100/99 | — |
| R_10x_smaller | 10/10 | 27.2 | 0.042 ± 0.009 | 0.196 | 100/98/100 | — |
| R_10x_larger | 10/10 | 27.3 | 0.046 ± 0.002 | 0.204 | 100/100/100 | — |
| Q_10x_smaller | 10/10 | 27.3 | 0.044 ± 0.004 | 0.202 | 100/100/100 | — |
