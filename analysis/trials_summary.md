Trials: 10 seeds per condition (seeds 0..9). Time to dock averaged over successful runs.

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
