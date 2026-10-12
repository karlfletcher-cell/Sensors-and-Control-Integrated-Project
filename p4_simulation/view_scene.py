"""view_scene.py — interactive 3D viewer (needs a display). Owner: P4.
Drive with the arrow keys is NOT built in —
this just lets you look at the scene and drag the robot around. Ctrl+scroll to zoom.
Inputs: none. Outputs: a MuJoCo viewer window.
Run from the repo root:  python -m p4_simulation.view_scene"""
import mujoco, mujoco.viewer, time
from p4_simulation.tb3sim import TB3Sim
sim = TB3Sim(render=False)
with mujoco.viewer.launch_passive(sim.model, sim.data) as v:
    while v.is_running():
        sim.step(0.15, 0.3)      # slow left circle so you can see it move
        v.sync(); time.sleep(sim.dt)
