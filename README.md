# tb3sim — TurtleBot3 Track A simulation (41014 Assessment 4)

Minimal MuJoCo scene + Python wrapper for the Week 11 "working simulation" gate.

## Setup (fresh environment)
```
pip install mujoco opencv-contrib-python numpy matplotlib
python make_scene.py        # writes tb3_scene.xml + assets/aruco_0.png
python test_drive.py        # smoke test -> test_drive.png
python view_scene.py        # interactive 3D viewer (needs a display)
python main_loop.py         # end-to-end pipeline skeleton
```
Headless machine / no GPU: prefix with `MUJOCO_GL=egl` (NVIDIA) or `MUJOCO_GL=osmesa`.

## Files
| file | what |
|---|---|
| `make_scene.py` | all scene constants (robot dims, obstacles, dock, marker size, LiDAR rays). Edit, re-run. |
| `tb3_scene.xml` | generated MJCF. Don't hand-edit. |
| `tb3sim.py` | `TB3Sim` — the only interface the rest of the team uses. |
| `test_drive.py` | open-loop square + LiDAR scan + camera/ArUco check. |
| `main_loop.py` | integration loop: control → step → odometry → camera → update. TODO blocks for P1/P2/P3. |

## Interface (`tb3sim.py`)
```
sim = TB3Sim(dt=0.1, slip_std=0.02)
sim.reset(pose=(x, y, yaw))     # optional
sim.step(v, w)                  # body velocity command for one period dt
dl, dr = sim.odometry()         # wheel travel [m] since last call   -> EKF predict
scan   = sim.lidar()            # (72,) ranges, angle i*5deg in base frame, -1 = no hit
img    = sim.camera()           # 640x480 RGB from the forward camera -> ArUco
K      = sim.camera_intrinsics()   # pinhole K (sim's own fovy; P2 should still calibrate)
T_bc   = sim.T_base_cam()       # camera pose in base frame (4x4)
mk     = sim.marker_xy()        # marker world position (known map feature)
x_true = sim.true_pose()        # (x, y, yaw) GROUND TRUTH — evaluation/plots ONLY
```

## Frames
world → base (x fwd, y left, z up, origin on floor between wheels) → camera
(MuJoCo convention: looks along −z, y up; `T_base_cam()` converts to base).
ArUco `solvePnP` gives the marker in the camera frame; `main_loop.measure()` shows the
conversion to (range, bearing) in the base frame. Note the camera sits 0.10 m ahead of
the base origin — range from the camera ≠ range from the base.

## Modelling decisions (say these on the project page)
- Base is a planar joint (x, y, yaw) driven by velocity servos from the unicycle model —
  the same `f_uni` as Week 7/8. Wheel/floor contact is **not** simulated; wheels exist
  for the encoders. Process noise = per-wheel multiplicative slip (`slip_std`).
- Collisions with walls/obstacles ARE simulated (robot stops against them).
- LiDAR = 72 `rangefinder` sensors, 3.5 m cutoff (LDS-01: 360 rays, same range).
- Camera 640×480, fovy 48° (≈ Pi camera). No lens distortion.
- ArUco DICT_4X4_50 id 0, 0.20 m, on the dock face at (2.6, 1.5) facing −x.
- Robot dims: Waffle (wheel r 0.033 m, separation 0.287 m).

## Who plugs in what
- **P1**: `controller()` in `main_loop.py` — LQR on the error model, from Week 8 B.1 / Assessment 3.
- **P2**: `measure()` — calibration of K, noise characterisation → `Rn`, marker-lost handling.
- **P3**: the two `ekf_*` TODOs — paste Week 8 Part 0 functions, tune `Qd`, `Rn`, log `P`.
- **P4**: scene, this loop, batch trials, plots of `xh` ± 2σ vs `true` from `run_log.npz`.
