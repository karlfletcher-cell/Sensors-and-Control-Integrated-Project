"""
Day-1 smoke test: drive a square open-loop and compare
  - ground truth (from the simulator, evaluation only)
  - dead-reckoned odometry (what the EKF predict step will use)
  - one LiDAR scan and one camera frame with the ArUco detection
Run:  python test_drive.py        (add MUJOCO_GL=egl or =osmesa on a headless machine)
"""
import numpy as np, matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import cv2
from tb3sim import TB3Sim, wrap

sim = TB3Sim(dt=0.05)
mk = sim.marker_xy()
sim.reset(pose=(-2.5, -2.5, 0.0))

# open-loop square: 4 x (drive 1 m, turn 90 deg)
cmds = []
for _ in range(4):
    cmds += [(0.2, 0.0)] * 100          # 0.2 m/s for 5 s  = 1 m
    cmds += [(0.0, np.pi/4)] * 40       # 45 deg/s for 2 s = 90 deg

truth, odom = [sim.true_pose()], [sim.true_pose()]   # odom starts from the true pose
x = odom[0].copy()
for v, w in cmds:
    sim.step(v, w)
    dl, dr = sim.odometry()
    ds, dth = (dl + dr) / 2, (dr - dl) / sim.WHEEL_SEP
    x = np.array([x[0] + ds * np.cos(x[2] + dth / 2),
                  x[1] + ds * np.sin(x[2] + dth / 2),
                  wrap(x[2] + dth)])
    truth.append(sim.true_pose()); odom.append(x.copy())
truth, odom = np.array(truth), np.array(odom)
print("final true pose      ", np.round(truth[-1], 3))
print("final odometry pose  ", np.round(odom[-1], 3))
print("drift after square   %.3f m" % np.linalg.norm(truth[-1, :2] - odom[-1, :2]))

# LiDAR scan at the final pose
scan, ang = sim.lidar(), sim.lidar_angles()
hit = scan > 0
pts = np.column_stack([scan[hit] * np.cos(ang[hit] + truth[-1, 2]) + truth[-1, 0],
                       scan[hit] * np.sin(ang[hit] + truth[-1, 2]) + truth[-1, 1]])

# camera + ArUco from a pose that faces the dock
sim.reset(pose=(mk[0] - 1.5, mk[1], 0.0))
img = sim.camera()
det = cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50))
corners, ids, _ = det.detectMarkers(cv2.cvtColor(img, cv2.COLOR_RGB2GRAY))
vis = cv2.aruco.drawDetectedMarkers(img.copy(), corners, ids)
print("ArUco ids in view    ", None if ids is None else ids.ravel())

fig, ax = plt.subplots(1, 3, figsize=(15, 4.5))
ax[0].plot(truth[:, 0], truth[:, 1], "k", label="true"); ax[0].plot(odom[:, 0], odom[:, 1], "r--", label="odometry")
ax[0].set_aspect("equal"); ax[0].legend(); ax[0].set_title("open-loop square")
ax[1].plot(pts[:, 0], pts[:, 1], ".", ms=3); ax[1].plot(*truth[-1, :2], "r^")
ax[1].plot(*mk, "gs", label="marker"); ax[1].set_aspect("equal"); ax[1].set_xlim(-3.2, 3.2); ax[1].set_ylim(-3.2, 3.2)
ax[1].set_title("LiDAR scan (world frame)"); ax[1].legend()
ax[2].imshow(vis); ax[2].set_title("camera + ArUco"); ax[2].axis("off")
plt.tight_layout(); plt.savefig("test_drive.png", dpi=110)
print("saved test_drive.png")
