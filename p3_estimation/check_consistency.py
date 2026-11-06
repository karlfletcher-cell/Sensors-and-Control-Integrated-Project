"""
check_consistency.py — check the EKF's covariances against its real errors with NEES and NIS (P3).
Runs the same docking episode as main.py --headless over several seeds.
Uses ground truth ONLY to score the filter (NEES), never in the EKF.
Run from the repo root:  python -m p3_estimation.check_consistency
(or use the VS Code run button; the sys.path line below makes that work too)
"""
import os
import sys
import numpy as np

# let Python find the other project folders when this file is run directly
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import config
from p4_simulation.episode import run_episode
from p4_simulation.tb3sim import wrap

SEEDS = range(10)
SETTLE_TIME = 10.0               # [s] skip this long to see the filter after the start error has gone

# 95% chi-squared limits. NEES has 3 degrees of freedom (x, y, heading), NIS has 2 (range, bearing).
NEES_LIMIT = 7.815
NIS_LIMIT = 5.991


def nees_per_step(run):
    """NEES at every step: e' P^-1 e, where e is the true estimate error (ideal mean 3)."""
    errors = run["xh"] - run["true"]
    errors[:, 2] = wrap(errors[:, 2])
    return np.einsum("ni,ni->n", errors, np.linalg.solve(run["P"], errors[..., None])[..., 0])


def nis_per_step(run):
    """NIS at every step: nu' S^-1 nu, where nu is the innovation (ideal mean 2).
    Steps with no camera reading have no innovation, so they come back as NaN."""
    seen = run["seen"]
    nis = np.full(len(seen), np.nan)
    nu, S = run["nu"][seen], run["S"][seen]
    nis[seen] = np.einsum("ni,ni->n", nu, np.linalg.solve(S, nu[..., None])[..., 0])
    return nis


def print_line(name, values, ideal, limit):
    values = values[~np.isnan(values)]
    print(f"  {name}: mean {values.mean():6.2f} (ideal {ideal})   median {np.median(values):5.2f}   "
          f"under 95% limit {np.mean(values < limit) * 100:5.1f}%")


runs = [run_episode(seed=seed, viewer=False, camera_pov=False, verbose=False) for seed in SEEDS]
nees = np.concatenate([nees_per_step(run) for run in runs])
nis = np.concatenate([nis_per_step(run) for run in runs])
time_in_run = np.concatenate([np.arange(run["steps"]) * config.DT for run in runs])
settled = time_in_run >= SETTLE_TIME

print(f"success {sum(run['success'] for run in runs)}/{len(SEEDS)}")
print("Whole run:")
print_line("NEES", nees, 3, NEES_LIMIT)
print_line("NIS ", nis, 2, NIS_LIMIT)
print(f"After {SETTLE_TIME:.0f} s:")
print_line("NEES", nees[settled], 3, NEES_LIMIT)
print_line("NIS ", nis[settled], 2, NIS_LIMIT)
