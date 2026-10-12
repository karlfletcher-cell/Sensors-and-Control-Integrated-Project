# tb3sim — TurtleBot3 Docking Simulation (41014 Assessment 4, Track A)

Simulated TurtleBot3 (MuJoCo) that uses wheel odometry + an ArUco marker on the dock
to estimate its pose (EKF) and drive to the dock.

## Quick start

```
pip install -r requirements.txt
python -m p4_simulation.make_scene       # build the scene (run once, or after editing make_scene.py)
python -m p4_simulation.test_drive       # check the sim works -> test_drive.png
python main.py                           # run the full pipeline
```

No screen/GPU? Prefix commands with `MUJOCO_GL=egl` (NVIDIA) or `MUJOCO_GL=osmesa` (CPU).

## Who does what

| Person | Job | Code |
|---|---|---|
| **P1** | Controller (LQR) | `controller()` in `p1_control/controller.py` |
| **P2** | Sensing — camera calibration, noise `Rn`, marker lost | `measure()` in `p2_sensing/sensing.py` |
| **P3** | EKF — tune `Qd`, `Rn`, log `P` | `p3_estimation/`, called from the `ekf_*` TODOs in `main.py` |
| **P4** | Scene, main loop, trials, plots | `p4_simulation/`, `main.py` |

## Using the simulator

```python
from p4_simulation.tb3sim import TB3Sim

sim = TB3Sim(dt=0.1, slip_std=0.02)
sim.reset(pose=(x, y, yaw))           # optional start pose
sim.step(v, w)                        # speed [m/s], turn rate [rad/s]
dl, dr = sim.odometry()               # wheel travel [m] since last call -> EKF predict
img    = sim.camera()                 # 640x480 RGB -> ArUco -> EKF update
K      = sim.camera_intrinsics()      # camera matrix
T_bc   = sim.T_base_cam()             # camera pose on the robot (4x4)
mk     = sim.marker_xy()              # marker world position
x_true = sim.true_pose()              # GROUND TRUTH — plots/evaluation only
```

**Never use `true_pose()` in the controller or EKF** — it fails the assessment.

## Things to know

- Base frame: x forward, y left, origin between the wheels. The camera is **0.10 m ahead**
  of this, so camera range ≠ robot range (`measure()` handles the conversion).
- Motion uses the unicycle model (`f_uni`, Weeks 7/8); process noise is per-wheel slip.
- Marker: ArUco `DICT_4X4_50`, id 0, 0.20 m, at (2.6, 1.5) facing −x.
- Robot: Waffle — wheel radius 0.033 m, separation 0.287 m.
- Don't hand-edit `tb3_scene.xml`; change `make_scene.py` and re-run it.
