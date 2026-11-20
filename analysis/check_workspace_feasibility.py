"""Sampled re-check of data/workspace_feasibility.csv.

Re-runs the feasibility test of the paper on a smaller sample and prints each
cell next to the committed value with a 95% sampling interval. It uses its own
random streams, so agreement is statistical, not digit for digit.

    python analysis/check_workspace_feasibility.py                  # 50 targets x 10 seeds
    python analysis/check_workspace_feasibility.py --per-seed 1200  # full size (hours)
"""
from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

import numpy as np
import pandas as pd

ROOT = Path(__file__).resolve().parents[1]
for _p in (ROOT, ROOT / "src"):
    if str(_p) not in sys.path:
        sys.path.insert(0, str(_p))

import twinbench.constants as C
import experiments.run_main_grid as R
from twinbench.dynamics.robot_loader import get_limits_from_spec, load_robot_spec, robot_spec_path_from_id

N_SEEDS = 10
IK_ATTEMPTS = 5


def measure(robot: str, scene: str, per_seed: int):
    spec = load_robot_spec(robot_spec_path_from_id(robot, repo_root=ROOT))
    lo, hi = get_limits_from_spec(spec)
    qmin, qmax = np.deg2rad(np.asarray(lo, float)), np.deg2rad(np.asarray(hi, float))
    q_home = 0.5 * (qmin + qmax)
    urdf = None
    if robot in C.ROBOTS_URDF:
        from twinbench.dynamics.urdf_plant import get_urdf_robot
        urdf = get_urdf_robot(ROOT / C.URDF_PATHS[robot])
    dh = None if urdf is not None else R.build_dh_from_spec(spec)
    scene_spec = R.CATALOG.build_scaled_scene(robot, scene)
    sampler = R.sample_open_reach if scene == "open_reach" else R.sample_shelf_reach
    total = feasible = 0
    for seed in range(N_SEEDS):
        rng = np.random.default_rng(1000 + seed)
        for _ in range(per_seed):
            pt = sampler(scene_spec, rng)
            if pt.get("rejected_reason"):
                continue
            p_des = np.array([pt["x_m"], pt["y_m"], pt["z_m"]])
            total += 1
            for attempt in range(IK_ATTEMPTS):
                q0 = q_home if attempt == 0 else rng.uniform(qmin, qmax)
                if urdf is not None:
                    _, ok, _ = R.ik_pos_urdf(p_des, q0, urdf, qmin, qmax)
                else:
                    _, ok, _ = R.ik_pos(p_des, q0, dh, qmin, qmax)
                if ok:
                    feasible += 1
                    break
    return total, feasible


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--per-seed", type=int, default=50, help="targets per seed per cell (10 seeds)")
    ap.add_argument("--cells", default="all", help="comma-separated robot:scene pairs, or 'all'")
    a = ap.parse_args()
    ref = pd.read_csv(ROOT / "data" / "workspace_feasibility.csv").set_index(["robot", "scene"]).pct_feasible
    cells = list(ref.index) if a.cells == "all" else [tuple(c.split(":")) for c in a.cells.split(",")]
    outside = 0
    for robot, scene in cells:
        t0 = time.time()
        total, feasible = measure(robot, scene, a.per_seed)
        p = feasible / total
        half = 196.0 * np.sqrt(max(p * (1 - p), 0.0) / total)
        committed = float(ref[(robot, scene)])
        # at 0% or 100% the normal interval collapses; allow one sample's worth of slack
        ok = abs(100 * p - committed) <= max(half, 100.0 / total)
        outside += not ok
        print(f"{robot:8s} {scene:11s} n={total:5d}  measured {100 * p:5.1f}% +/- {half:3.1f}   "
              f"committed {committed:5.1f}%   {'ok' if ok else 'OUTSIDE'}   [{time.time() - t0:.0f} s]", flush=True)
    print(f"{len(cells) - outside}/{len(cells)} cells within the 95% sampling interval of the committed value")
    return 0 if outside == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
