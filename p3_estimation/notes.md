### step 1
 # Camera noise measurement (with P2's updated sensing.py, no camera_calib.json yet)
    352 detections out of 400 poses
    range   error: mean 0.31 cm    std 1.08 cm
    bearing error: mean 0.05 deg   std 0.03 deg
     0.00-0.75 m   n= 54   range mean -0.04 cm   range std 0.03 cm   bearing std 0.05 deg
     0.75-1.25 m   n=101   range mean 0.07 cm   range std 0.08 cm   bearing std 0.03 deg
     1.25-1.75 m   n= 93   range mean 0.46 cm   range std 2.03 cm   bearing std 0.03 deg
     1.75-3.00 m   n=104   range mean 0.57 cm   range std 0.24 cm   bearing std 0.01 deg
    largest range errors (cm): 1.09, 1.13, 1.15, 1.18, 19.95
    largest range error: 19.95 cm at 1.38 m
    range std without that one: 0.27 cm
    same pose twice gives identical reading: True

 # Observations
    * Range error grows with distance: std 0.03 cm under 0.75 m, 0.24 cm past 1.75 m.
    * One 19.95 cm outlier at 1.38 m inflates the 1.25–1.75 m group's std (2.03 cm) and the overall std:
      1.08 cm with it, 0.27 cm without. The next largest error is 1.18 cm. Cause not checked yet -> raise with P2.
    * Camera over-reads range at distance: mean error -0.04 cm under 0.75 m, +0.57 cm past 1.75 m.
    * Measured bearing error is very small (0.03°). A real camera would likely be noisier (blur, lens distortion).
    * Repeated readings from the same pose are identical, so the error depends on the pose, not on random noise.

 ## Chosen value
    Rn = diag([0.02², (0.5°)²])
    - Range 2 cm: ~1.9x measured std (1.08 cm), covers the far group's total error including bias
    (sqrt(0.57² + 0.24²) = 0.62 cm).
    - Bearing 0.5°: well above measured 0.03°, to stop the filter becoming overconfident.

 # Before / after (headless, EKF on, seeds 0-9, with P2's updated sensing.py)
    Old guess (10 cm, 3°): 10/10 docked, mean final error 4.36 cm, mean rms error 19.81 cm (std 0.34 cm)
    New (2 cm, 0.5°):      10/10 docked, mean final error 4.48 cm, mean rms error 20.14 cm (std 0.97 cm)

    The measured Rn made no real difference. Its rms error was lower on only 4 of the 10 seeds.
    An earlier seed 0 run showed about 30% lower rms error, but that was with the old sensing.py and no longer holds.
    I kept 2 cm, 0.5° because it matches the measurements, not because it improves the run.
    I'll check it again with run_trials.py after tuning Qd.