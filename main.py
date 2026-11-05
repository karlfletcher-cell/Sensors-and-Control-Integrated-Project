"""
main.py — run the full pipeline once and save the log.

    python main.py                   # 3D viewer + camera-POV window (nominal run, seed 0)
    python main.py --headless        # no windows (servers / CI / recording figures)
    python main.py --no-ekf          # dead-reckoning baseline: camera measurements ignored
    python main.py --seed 3          # a different random draw of slip
    python main.py --push            # a shove mid-run         (disturbed trial for the video)
    python main.py --blackout        # marker hidden for 6 s   (disturbed trial for the video)
    python main.py --record          # also write analysis/camera_pov.mp4
Then:  python -m p4_simulation.plot_run run_log.npz   -> analysis/fig_*.png
The loop itself lives in p4_simulation/episode.py (run_episode), so run_trials.py can call it too.
"""
import sys, os, numpy as np
from p4_simulation.episode import run_episode

args = sys.argv[1:]
seed = int(args[args.index("--seed") + 1]) if "--seed" in args else 0
headless = "--headless" in args
os.makedirs("analysis", exist_ok=True)

kw = dict(seed=seed, viewer=not headless, camera_pov=not headless,
          record_pov="analysis/camera_pov.mp4" if "--record" in args else None,
          use_ekf_update="--no-ekf" not in args)
if "--push" in args:
    kw.update(push_at=150, push=(0.25, -0.25, np.deg2rad(15)))
if "--blackout" in args:
    kw.update(blackout=(250, 310))

log = run_episode(**kw)
tag = "deadreck" if "--no-ekf" in args else "push" if "--push" in args else "blackout" if "--blackout" in args else "nominal"
fn = f"run_log_{tag}.npz" if tag != "nominal" else "run_log.npz"
np.savez(fn, **{k: v for k, v in log.items() if isinstance(v, np.ndarray)})
print("saved", fn)