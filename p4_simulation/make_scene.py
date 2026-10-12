"""
make_scene.py — generates p4_simulation/assets/aruco_0.png and p4_simulation/tb3_scene.xml.

Owner: P4.
Inputs: shared values from config.py + scene-only constants below.
Outputs: the MJCF scene and the ArUco texture, written next to this file.
Run once from the repo root:  python -m p4_simulation.make_scene
Edit the constants (here or in config.py), re-run, and the scene regenerates.
"""
import os, numpy as np, cv2
from config import (WHEEL_R, WHEEL_SEP, N_RAYS, CAM_W, CAM_H,   # shared values live in config.py
                    DOCK_XY, MARKER_SIZE, MARKER_ID, ROBOT_START)

HERE = os.path.dirname(os.path.abspath(__file__))
ASSETS = os.path.join(HERE, "assets")
os.makedirs(ASSETS, exist_ok=True)

# ---- TurtleBot3 Waffle geometry (ROBOTIS spec) ----------------------------
# WHEEL_R, WHEEL_SEP: config.py
BODY_HX, BODY_HY, BODY_HZ = 0.14, 0.14, 0.035   # chassis half-sizes
BODY_Z     = 0.075      # chassis centre height
LIDAR_Z    = 0.17       # LDS-01 scan plane height
CAM_POS    = (0.10, 0.0, 0.12)   # camera in robot frame (x fwd, y left, z up)
CAM_FOVY   = 48         # vertical FOV [deg]; Pi cam is ~62 deg horizontal

# ---- LiDAR --------------------------------------------------------------
# N_RAYS: config.py
LIDAR_MAX  = 3.5        # LDS-01 max range [m]

# ---- World --------------------------------------------------------------
ROOM_HALF  = 3.0        # room is 6 m x 6 m
WALL_H     = 0.5
OBSTACLES  = [          # (x, y, half_x, half_y)
    ( 0.0,  0.8, 0.25, 0.25),
    ( 1.2, -0.9, 0.40, 0.20),
    (-1.0, -1.2, 0.20, 0.40),
]
# DOCK_XY: config.py
DOCK_YAW   = np.pi           # marker normal points this way (-x: faces into room)
# MARKER_SIZE, MARKER_ID, ROBOT_START: config.py
MARKER_Z   = 0.12            # marker centre height (roughly camera height)

# ---- ArUco texture --------------------------------------------------------
aruco = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
img = cv2.aruco.generateImageMarker(aruco, MARKER_ID, 400)
img = cv2.copyMakeBorder(img, 50, 50, 50, 50, cv2.BORDER_CONSTANT, value=255)  # white quiet zone
cv2.imwrite(os.path.join(ASSETS, f"aruco_{MARKER_ID}.png"), img)
PLANE_HALF = MARKER_SIZE / 2 * (500 / 400)   # texture includes the border

# ---- rangefinder sites ----------------------------------------------------
rays = []
for i in range(N_RAYS):
    a = 2 * np.pi * i / N_RAYS
    # rangefinder measures along the site's +z axis
    rays.append(f'      <site name="ray{i}" pos="0 0 0" zaxis="{np.cos(a):.6f} {np.sin(a):.6f} 0" size="0.005"/>')
ray_sites = "\n".join(rays)
ray_sensors = "\n".join(f'    <rangefinder name="lidar{i}" site="ray{i}" cutoff="{LIDAR_MAX}"/>' for i in range(N_RAYS))

obst_xml = "\n".join(
    f'    <geom name="obs{i}" type="box" pos="{x} {y} 0.25" size="{hx} {hy} 0.25" rgba="0.6 0.3 0.2 1"/>'
    for i, (x, y, hx, hy) in enumerate(OBSTACLES))

# dock: a box behind a visual-only plane that carries the marker texture
dx, dy = DOCK_XY
c, s = np.cos(DOCK_YAW), np.sin(DOCK_YAW)
R = ROOM_HALF
xml = f"""<mujoco model="tb3_waffle_room">
  <option timestep="0.002" integrator="implicitfast" cone="elliptic" impratio="10"/>
  <compiler angle="radian"/>
  <visual><global offwidth="{CAM_W}" offheight="{CAM_H}"/></visual>

  <asset>
    <texture name="floor_tex" type="2d" builtin="checker" rgb1="0.8 0.8 0.8" rgb2="0.6 0.6 0.6" width="300" height="300"/>
    <material name="floor_mat" texture="floor_tex" texrepeat="12 12" reflectance="0"/>
    <texture name="aruco_tex" type="2d" file="assets/aruco_{MARKER_ID}.png"/>
    <material name="aruco_mat" texture="aruco_tex" reflectance="0" specular="0" shininess="0" emission="1"/>
  </asset>

  <default>
    <geom friction="1 0.005 0.0001"/>
    
  </default>

  <worldbody>
    <light pos="0 0 4" dir="0 0 -1" diffuse="0.8 0.8 0.8" specular="0 0 0" castshadow="false"/>
    <light pos="2 2 3" dir="-1 -1 -1" diffuse="0.5 0.5 0.5" specular="0 0 0" castshadow="false"/>
    <geom name="floor" type="plane" size="{R} {R} 0.1" material="floor_mat"/>

    <!-- walls -->
    <geom type="box" pos="0 {R} {WALL_H/2}"  size="{R} 0.05 {WALL_H/2}" rgba="0.85 0.85 0.9 1"/>
    <geom type="box" pos="0 {-R} {WALL_H/2}" size="{R} 0.05 {WALL_H/2}" rgba="0.85 0.85 0.9 1"/>
    <geom type="box" pos="{R} 0 {WALL_H/2}"  size="0.05 {R} {WALL_H/2}" rgba="0.85 0.85 0.9 1"/>
    <geom type="box" pos="{-R} 0 {WALL_H/2}" size="0.05 {R} {WALL_H/2}" rgba="0.85 0.85 0.9 1"/>

{obst_xml}

    <!-- charging dock with ArUco marker on its front face -->
    <body name="dock" pos="{dx} {dy} 0" euler="0 0 {DOCK_YAW}">
      <geom name="dock_box" type="box" pos="-0.05 0 0.15" size="0.05 0.2 0.15" rgba="0.2 0.2 0.25 1"/>
      <!-- plane normal is +z; rotate so it faces -x of the dock body (= into the room) -->
      <geom name="marker" type="plane" pos="0.001 0 {MARKER_Z}" zaxis="1 0 0"
            size="{PLANE_HALF:.4f} {PLANE_HALF:.4f} 0.001" material="aruco_mat" contype="0" conaffinity="0"/>
      <site name="marker_site" pos="0 0 {MARKER_Z}" size="0.01" rgba="0 1 0 0"/>
    </body>

    <!-- TurtleBot3 Waffle (differential drive) -->
    <body name="robot" pos="0 0 0">
      <!-- planar base: x, y, yaw, driven kinematically (unicycle model) by velocity servos.
           MuJoCo supplies the world, collisions, LiDAR ray-casting and the camera; the
           wheel/floor contact is NOT simulated (wheels are visual + encoder joints only).
           Wheel slip / process noise is injected in tb3sim.step(). -->
      <joint name="base_x"   type="slide" axis="1 0 0"/>
      <joint name="base_y"   type="slide" axis="0 1 0"/>
      <joint name="base_yaw" type="hinge" axis="0 0 1"/>
      <site name="base_site" pos="0 0 0" size="0.01" rgba="1 0 0 0"/>
      <geom name="chassis" type="box" pos="0 0 {BODY_Z}" size="{BODY_HX} {BODY_HY} {BODY_HZ}" rgba="0.1 0.1 0.1 1" mass="1.5" friction="0.2"/>
      <geom name="nose" type="box" pos="{BODY_HX} 0 {BODY_Z}" size="0.02 0.05 0.02" rgba="1 0.5 0 1" mass="0.01"/>
      <!-- caster balls, visual only (the planar joint carries the weight) -->
      <geom name="caster_f" type="sphere" pos="{BODY_HX-0.03} 0 0.015" size="0.015" contype="0" conaffinity="0" rgba="0.5 0.5 0.5 1" mass="0.05"/>
      <geom name="caster_b" type="sphere" pos="{-(BODY_HX-0.03)} 0 0.015" size="0.015" contype="0" conaffinity="0" rgba="0.5 0.5 0.5 1" mass="0.05"/>

      <body name="wheel_l" pos="0 {WHEEL_SEP/2} {WHEEL_R}">
        <joint name="wheel_l" type="hinge" axis="0 1 0"/>
        <geom type="cylinder" size="{WHEEL_R} 0.018" euler="1.5707963 0 0" rgba="0.2 0.2 0.2 1" mass="0.1" contype="0" conaffinity="0"/>
      </body>
      <body name="wheel_r" pos="0 {-WHEEL_SEP/2} {WHEEL_R}">
        <joint name="wheel_r" type="hinge" axis="0 1 0"/>
        <geom type="cylinder" size="{WHEEL_R} 0.018" euler="1.5707963 0 0" rgba="0.2 0.2 0.2 1" mass="0.1" contype="0" conaffinity="0"/>
      </body>

      <!-- LDS-01 LiDAR: N rays in the horizontal plane -->
      <body name="lidar" pos="0 0 {LIDAR_Z}">
        <geom type="cylinder" size="0.035 0.02" rgba="0.3 0.3 0.3 1" mass="0.1" contype="0" conaffinity="0"/>
{ray_sites}
      </body>

      <!-- forward-looking camera: MuJoCo cameras look along -z with y up -->
      <camera name="cam" pos="{CAM_POS[0]} {CAM_POS[1]} {CAM_POS[2]}" xyaxes="0 -1 0 0 0 1" fovy="{CAM_FOVY}"/>
    </body>
  </worldbody>

  <keyframe>
    <key name="start" qpos="{ROBOT_START[0]} {ROBOT_START[1]} {ROBOT_START[2]} 0 0"/>
  </keyframe>

  <actuator>
    <!-- base velocity servos: ctrl = (vx, vy, yaw rate) in the WORLD frame, set by tb3sim.step() -->
    <velocity name="act_x"   joint="base_x"   kv="200"/>
    <velocity name="act_y"   joint="base_y"   kv="200"/>
    <velocity name="act_yaw" joint="base_yaw" kv="20"/>
    <!-- wheel servos so the encoders turn like a real robot's -->
    <velocity name="act_l" joint="wheel_l" kv="2.0" ctrlrange="-30 30"/>
    <velocity name="act_r" joint="wheel_r" kv="2.0" ctrlrange="-30 30"/>
  </actuator>

  <sensor>
    <!-- wheel encoders (what the real robot has) -->
    <jointpos name="enc_l" joint="wheel_l"/>
    <jointpos name="enc_r" joint="wheel_r"/>
    <!-- ground truth: EVALUATION ONLY, never feed to the controller -->
    <framepos  name="gt_pos" objtype="site" objname="base_site"/>
    <framequat name="gt_quat" objtype="site" objname="base_site"/>
    <framepos  name="marker_pos" objtype="site" objname="marker_site"/>
{ray_sensors}
  </sensor>
</mujoco>
"""
with open(os.path.join(HERE, "tb3_scene.xml"), "w") as f:
    f.write(xml)
print("wrote tb3_scene.xml with", N_RAYS, "lidar rays; marker", MARKER_SIZE, "m at", DOCK_XY)
