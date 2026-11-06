"""
Calibrate.py: offline camera calibration. Owner: P2. Run: python -m p2_sensing.Calibrate

Renders checkerboard views, calibrates with CentralCamera.images2C (Week 3 Part B), checks
reprojection error (Week 3 Part C) and fits the remaining range bias (Week 2 Part B.3).
The bias fit uses ground truth offline only. Writes camera_calib.json and calib_images/.
"""

import glob
import json
import os
import tempfile

import cv2
import mujoco
import numpy as np
from machinevisiontoolbox import CentralCamera, FileCollection

from p4_simulation.tb3sim import HERE as SIM_DIR, TB3Sim

P2_DIR = os.path.dirname(os.path.abspath(__file__))
IMG_DIR = os.path.join(P2_DIR, "calib_images")
CALIB_FILE = os.path.join(P2_DIR, "camera_calib.json")

gx, gy, sq = 9, 6, 0.05     # Week 3: internal corners (columns, rows), square side [m]
PX = 100                    # texture pixels per square
N_IMAGES = 24
SEED = 1


def make_calib_sim(tmp):
    """TB3Sim on a temporary copy of the scene with a movable checkerboard. Scene files stay unchanged."""
    # Draw the board with one square of white margin around it
    board = np.full(((gy + 3) * PX, (gx + 3) * PX), 255, np.uint8)
    for r in range(gy + 1):
        for c in range(gx + 1):
            if (r + c) % 2 == 0:
                board[(r + 1) * PX:(r + 2) * PX, (c + 1) * PX:(c + 2) * PX] = 0
    board_png = os.path.join(tmp, "board.png")
    cv2.imwrite(board_png, board)

    with open(os.path.join(SIM_DIR, "tb3_scene.xml")) as f:
        xml = f.read()
    xml = xml.replace('file="assets/', f'file="{SIM_DIR}/assets/')
    # Lower the floor in this copy only: the camera is 0.12 m up, so the board would cut through it
    xml = xml.replace('name="floor" type="plane"', 'name="floor" pos="0 0 -2" type="plane"', 1)
    xml = xml.replace("</asset>",
                      f'<texture name="board_tex" type="2d" file="{board_png}"/>'
                      '<material name="board_mat" texture="board_tex" reflectance="0" specular="0" '
                      'shininess="0" emission="1"/></asset>', 1)
    xml = xml.replace("</worldbody>",
                      '<body name="calib_board" mocap="true" pos="0 0 2">'
                      f'<geom type="plane" size="{(gx + 3) * sq / 2} {(gy + 3) * sq / 2} 0.001" '
                      'material="board_mat" contype="0" conaffinity="0"/></body></worldbody>', 1)
    path = os.path.join(tmp, "calib_scene.xml")
    with open(path, "w") as f:
        f.write(xml)
    return TB3Sim(xml=path, dt=0.1)


def rot(axis, a):
    """Rotation matrix for angle a about axis."""
    q, m = np.zeros(4), np.zeros(9)
    mujoco.mju_axisAngle2Quat(q, np.array(axis, float), a)
    mujoco.mju_quat2Mat(m, q)
    return m.reshape(3, 3)


def render_views(sim):
    """Save N_IMAGES checkerboard views at different positions and tilts."""
    os.makedirs(IMG_DIR, exist_ok=True)
    for old in glob.glob(os.path.join(IMG_DIR, "*.png")):
        os.remove(old)
    rng = np.random.default_rng(SEED)
    robot = (-2.0, 2.0, 0.0)
    cam_world = np.array([robot[0] + 0.10, robot[1], 0.12])
    # Board facing the camera: local x -> world -y, local y -> world +z, local z -> world -x
    face = np.array([[0, 0, -1], [-1, 0, 0], [0, 1, 0]], float)
    mocap = sim.model.body("calib_board").mocapid[0]
    saved = 0
    while saved < N_IMAGES:
        depth = rng.uniform(0.55, 1.0)
        tilt = rot([1, 0, 0], rng.uniform(-.5, .5)) @ rot([0, 1, 0], rng.uniform(-.5, .5)) \
            @ rot([0, 0, 1], rng.uniform(-.15, .15))
        sim.reset(pose=robot)
        quat = np.zeros(4)
        mujoco.mju_mat2Quat(quat, (face @ tilt).ravel())
        sim.data.mocap_pos[mocap] = cam_world + [depth, rng.uniform(-.25, .25) * depth,
                                                 rng.uniform(-.18, .18) * depth]
        sim.data.mocap_quat[mocap] = quat
        mujoco.mj_forward(sim.model, sim.data)
        img = sim.camera()
        found, _ = cv2.findChessboardCorners(cv2.cvtColor(img, cv2.COLOR_RGB2GRAY), (gx, gy))
        if found:
            saved += 1
            cv2.imwrite(os.path.join(IMG_DIR, f"view_{saved:02d}.png"), cv2.cvtColor(img, cv2.COLOR_RGB2BGR))


def measured_corners(im):
    """Week 3 Part C: sub-pixel checkerboard corners, returned as a 2 x N array."""
    g = im.mono().image
    ok, c = cv2.findChessboardCorners(g, (gx, gy))
    if not ok:
        return None
    crit = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    c = cv2.cornerSubPix(g, c, (11, 11), (-1, -1), crit)
    return c.reshape(-1, 2).T


def reprojection_errors(images, K, dist, frames):
    """Week 3 Part C: RMS reprojection error of each image, in pixels."""
    P_board = np.array([[i * sq, j * sq, 0] for j in range(gy) for i in range(gx)]).T
    cal_cam = CentralCamera(f=K[0, 0], pp=(K[0, 2], K[1, 2]),
                            imagesize=(images[0].width, images[0].height), rho=1)
    rms = []
    for fr in frames:
        und = images[fr.id].undistort(K, dist)
        cal_cam.pose = fr.pose
        e = np.linalg.norm(measured_corners(und) - cal_cam.project_point(P_board), axis=0)
        rms.append(np.sqrt((e ** 2).mean()))
    return np.array(rms)


def calibrate_range_bias(K, T_bc):
    """Week 2 Part B.3 method with the Week 3 wrap-up error law: fit bias b(r) = a * r^2."""
    import p2_sensing.sensing as sensing
    sensing.RANGE_BIAS_A = 0.0
    sim = TB3Sim(dt=0.1)
    mk = sim.marker_xy()
    rng = np.random.default_rng(SEED + 1)
    rho, err = [], []
    for d in np.arange(0.5, 5.01, 0.25):
        for _ in range(20):
            sim.reset(pose=(mk[0] - d + rng.uniform(-.02, .02), mk[1] + rng.uniform(-.02, .02),
                            rng.uniform(-.03, .03)))
            z = sensing.measure(sim.camera(), K, T_bc)
            if z is not None:
                rho.append(z[0])
                err.append(z[0] - sim.true_range_bearing()[0])       # OFFLINE CALIBRATION ONLY
    rho, err = np.array(rho), np.array(err)
    return float(np.sum(err * rho ** 2) / np.sum(rho ** 4)), len(rho)


def main():
    with tempfile.TemporaryDirectory() as tmp:
        sim = make_calib_sim(tmp)
        K_sim, T_bc = sim.camera_intrinsics(), sim.T_base_cam()
        render_views(sim)

    # Week 3 Part B
    images = FileCollection(os.path.join(IMG_DIR, "*.png"))
    K, dist, frames = CentralCamera.images2C(images, gridshape=(gx, gy), squaresize=sq)
    print(f"grid ({gx},{gy}): {len(frames)} of {len(images)} images were used")

    # Week 3 Part C
    rms = reprojection_errors(images, K, dist, frames)
    print(f"reprojection error: mean {rms.mean():.3f} px, worst image {rms.max():.3f} px")

    # Repeatability: calibrate each half of the images separately
    half_a = [images[i] for i in range(0, len(images), 2)]
    half_b = [images[i] for i in range(1, len(images), 2)]
    K_a = CentralCamera.images2C(half_a, gridshape=(gx, gy), squaresize=sq)[0]
    K_b = CentralCamera.images2C(half_b, gridshape=(gx, gy), squaresize=sq)[0]

    # Week 3 Part B2
    W = images[0].width
    for name, i, j in [("fx", 0, 0), ("fy", 1, 1), ("cx", 0, 2), ("cy", 1, 2)]:
        print(f"{name}: calibrated {K[i, j]:.2f}   simulator {K_sim[i, j]:.2f}   "
              f"half-set difference {abs(K_a[i, j] - K_b[i, j]):.2f}")
    print(f"horizontal field of view = {np.degrees(2 * np.arctan(W / (2 * K[0, 0]))):.1f} deg")
    print("distortion (k1, k2, p1, p2, k3) =", np.round(dist, 4))

    a, n_fit = calibrate_range_bias(K, T_bc)
    print(f"range bias: b(r) = {a:.5f} * r^2, from {n_fit} samples")

    with open(CALIB_FILE, "w") as f:
        json.dump(dict(K=K.tolist(), dist=np.ravel(dist).tolist(), K_sim=K_sim.tolist(),
                       K_half_a=K_a.tolist(), K_half_b=K_b.tolist(), n_images=len(frames),
                       reprojection_rms_px=rms.tolist(), range_bias_a=a, range_fit_samples=n_fit), f, indent=2)


if __name__ == "__main__":
    main()