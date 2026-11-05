### step 1
 # Camera noise measurement
    352 detections out of 400 poses
    range   error: mean 0.83 cm    std 1.34 cm
    bearing error: mean 0.05 deg   std 0.03 deg
     0.00-0.75 m   n= 54   range mean 0.08 cm   range std 0.08 cm   bearing std 0.04 deg
     0.75-1.25 m   n=101   range mean 0.37 cm   range std 0.33 cm   bearing std 0.03 deg
     1.25-1.75 m   n= 93   range mean 1.09 cm   range std 2.10 cm   bearing std 0.04 deg
     1.75-3.00 m   n=104   range mean 1.43 cm   range std 1.04 cm   bearing std 0.02 deg
    largest range errors (cm): 3.5, 3.54, 4.08, 4.15, 20.21
    largest range error: 20.21 cm at 1.38 m
    range std without that one: 0.85 cm
    same pose twice gives identical reading: True

 # Observations
    * Range error grows with distance: std 0.08 cm under 0.75 m, 1.04 cm past 1.75 m.
    * One 20.21 cm outlier at 1.38 m inflates the 1.25–1.75 m group's std (2.10 cm) and the overall std:
      1.34 cm with it, 0.85 cm without. The next largest error is 4.15 cm.
    * Camera over-reads range, more at distance: mean error +0.08 cm under 0.75 m, +1.43 cm past 1.75 m
      (positive in all 4 groups). Cause not checked yet -> raise with P2.
    * Measured bearing error is very small (0.03°). A real camera would likely be noisier (blur, lens distortion).
    * Repeated readings from the same pose are identical, so the error depends on the pose, not on random noise.

 ## Chosen value
    Rn = diag([0.02², (0.5°)²])
    - Range 2 cm: ~1.5x measured std (1.34 cm), covers the far group's total error including bias
    (sqrt(1.43² + 1.04²) = 1.77 cm).
    - Bearing 0.5°: well above measured 0.03°, to stop the filter becoming overconfident.

 # Before / after (python main.py --headless, seed 0)
    Old guess (10 cm, 3°): docked, success, final error 3.8 cm, rms error 19.8 cm, marker seen 71%
    New (2 cm, 0.5°):      docked, success, final error 0.4 cm, rms error 13.9 cm, marker seen 72%

    The measured Rn reduced the rms error by about 30% on this run. It's only one seed though,
    so I'll confirm it over multiple seeds with run_trials.py after tuning Qd.