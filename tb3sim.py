"""
tb3sim.py — thin wrapper around the MuJoCo TurtleBot3 scene.

Everyone else codes against THIS interface, not against MuJoCo directly:

    sim = TB3Sim()
    sim.reset()
    for k in range(N):
        v, w   = controller(xh)                  # from the ESTIMATE
        sim.step(v, w)                           # one control period (dt)
        dl, dr = sim.odometry()                  # wheel-encoder increments  -> EKF predict (P3)
        scan   = sim.lidar()                     # (N_RAYS,) ranges, -1 = no hit -> obstacle avoidance
        img    = sim.camera()                    # (H, W, 3) RGB uint8      -> ArUco detector (P2)
        x_true = sim.true_pose()                 # EVALUATION ONLY. Never into the controller.

Frames: world (fixed), base (robot, x forward, y left, z up, origin on the floor
between the wheels), camera (MuJoCo convention: looks along -z, y up — see
camera_intrinsics() and T_base_cam()).
"""
import os
import numpy as np
import mujoco

HERE = os.path.dirname(os.path.abspath(__file__))


def wrap(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


class TB3Sim:
    # keep these in sync with make_scene.py
    WHEEL_R = 0.033
    WHEEL_SEP = 0.287
    N_RAYS = 72
    CAM_W, CAM_H = 640, 480

    def __init__(self, xml=os.path.join(HERE, "tb3_scene.xml"), dt=0.05, render=True,
                 slip_std=0.0, seed=0):
        """slip_std: process noise. Each period the executed (v, w) are scaled by
        (1 + N(0, slip_std)) independently per wheel, emulating wheel slip. 0 = ideal."""
        self.rng = np.random.default_rng(seed)
        self.slip_std = slip_std
        self.model = mujoco.MjModel.from_xml_path(xml)
        self.data = mujoco.MjData(self.model)
        self.dt = dt                                   # control period [s]
        self.n_sub = int(round(dt / self.model.opt.timestep))
        self._enc_prev = np.zeros(2)
        self._renderer = mujoco.Renderer(self.model, self.CAM_H, self.CAM_W) if render else None
        self._vopt = mujoco.MjvOption()
        self._vopt.flags[mujoco.mjtVisFlag.mjVIS_RANGEFINDER] = 0   # hide lidar rays in camera images
        self.reset()

    # ------------------------------------------------------------ control
    def reset(self, pose=None):
        """pose = (x, y, yaw) or None for the XML start pose."""
        mujoco.mj_resetDataKeyframe(self.model, self.data, 0)      # keyframe "start" from the XML
        if pose is not None:
            self.data.qpos[0:3] = pose                                # qpos = [x, y, yaw, wheel_l, wheel_r]
        mujoco.mj_forward(self.model, self.data)
        self._enc_prev = self._encoders()

    def step(self, v, w):
        """Apply body velocity (v [m/s], w [rad/s]) for one control period."""
        # commanded wheel speeds -> what the encoders will show
        wl = (v - w * self.WHEEL_SEP / 2) / self.WHEEL_R
        wr = (v + w * self.WHEEL_SEP / 2) / self.WHEEL_R
        # executed wheel speeds (with slip) -> what the body actually does
        if self.slip_std > 0:
            wl_e = wl * (1 + self.rng.normal(0, self.slip_std))
            wr_e = wr * (1 + self.rng.normal(0, self.slip_std))
        else:
            wl_e, wr_e = wl, wr
        v_e = (wl_e + wr_e) * self.WHEEL_R / 2
        w_e = (wr_e - wl_e) * self.WHEEL_R / self.WHEEL_SEP
        for _ in range(self.n_sub):
            th = self.data.qpos[2]                       # the plant knows its own heading
            self.data.ctrl[:] = [v_e * np.cos(th), v_e * np.sin(th), w_e, wl, wr]
            mujoco.mj_step(self.model, self.data)

    # ------------------------------------------------------------ sensors
    def _encoders(self):
        return np.array([self.data.sensor("enc_l").data[0],
                         self.data.sensor("enc_r").data[0]])

    def odometry(self, noise_std=0.0):
        """Wheel travel (dl, dr) in metres since the last call.
        noise_std: optional additive Gaussian noise [m] to emulate encoder error.
        Convert to body motion with:  ds = (dl+dr)/2,  dth = (dr-dl)/WHEEL_SEP."""
        enc = self._encoders()
        d = (enc - self._enc_prev) * self.WHEEL_R
        self._enc_prev = enc
        if noise_std > 0:
            d = d + np.random.normal(0, noise_std, 2)
        return d[0], d[1]

    def odom_vw(self):
        """Same information as odometry() but as (v, w) over the last period."""
        dl, dr = self.odometry()
        return (dl + dr) / 2 / self.dt, (dr - dl) / self.WHEEL_SEP / self.dt

    def lidar(self):
        """(N_RAYS,) ranges [m], ray i at angle 2*pi*i/N_RAYS in the base frame. -1 = no hit."""
        return np.array([self.data.sensor(f"lidar{i}").data[0] for i in range(self.N_RAYS)])

    def lidar_angles(self):
        return 2 * np.pi * np.arange(self.N_RAYS) / self.N_RAYS

    def camera(self):
        """RGB image (H, W, 3) uint8 from the forward camera."""
        self._renderer.update_scene(self.data, camera="cam", scene_option=self._vopt)
        return self._renderer.render().copy()

    def camera_intrinsics(self):
        """Pinhole K for the simulated camera (OpenCV convention, no distortion)."""
        fovy = np.deg2rad(self.model.cam("cam").fovy[0])
        fy = (self.CAM_H / 2) / np.tan(fovy / 2)
        fx = fy
        return np.array([[fx, 0, self.CAM_W / 2],
                         [0, fy, self.CAM_H / 2],
                         [0, 0, 1.0]])

    def T_base_cam(self):
        """4x4 pose of the camera frame in the base frame, OpenCV convention
        (x right, y down, z forward) — i.e. the frame solvePnP's tvec is expressed in.
        Same matrix as R_RC in the Week 9 tutorial D3. Camera axes in base coords:
        x_cam = -y_base, y_cam = -z_base, z_cam = +x_base.
        (MuJoCo's own camera frame looks along -z with y up; you never need it because
        the rendered image is already what an OpenCV camera at this pose would see.)"""
        T = np.eye(4)
        T[:3, :3] = np.array([[0, 0, 1],
                              [-1, 0, 0],
                              [0, -1, 0]])
        T[:3, 3] = self.model.cam("cam").pos
        return T

    # ------------------------------------------------------------ ground truth (evaluation only)
    def true_pose(self):
        p = self.data.sensor("gt_pos").data
        q = self.data.sensor("gt_quat").data
        yaw = np.arctan2(2 * (q[0] * q[3] + q[1] * q[2]), 1 - 2 * (q[2] ** 2 + q[3] ** 2))
        return np.array([p[0], p[1], wrap(yaw)])

    def marker_xy(self):
        """World position of the dock marker (known map feature — the EKF is allowed this)."""
        return self.data.sensor("marker_pos").data[:2].copy()

    def true_range_bearing(self):
        """What a perfect range-bearing sensor would say. For debugging P2's detector only."""
        x = self.true_pose()
        mk = self.marker_xy()
        dx, dy = mk[0] - x[0], mk[1] - x[1]
        return np.array([np.hypot(dx, dy), wrap(np.arctan2(dy, dx) - x[2])])
