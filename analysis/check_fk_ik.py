"""FK/IK round-trip error of the kinematic models used in the benchmark.

For each robot, joint configurations are drawn uniformly within the joint limits,
the end-effector position is computed by forward kinematics, and the benchmark's
position IK (from the mid-range home configuration, then up to four random
restarts) is solved back to that position. Prints the per-robot mean and maximum
position error of the solved targets.

    python analysis/check_fk_ik.py [--n 60]
"""

import argparse
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
for _p in (ROOT, ROOT / "src"):
    if str(_p) not in sys.path:
        sys.path.insert(0, str(_p))

import twinbench.constants as C
import experiments.run_main_grid as R
from twinbench.dynamics.robot_loader import get_limits_from_spec, load_robot_spec, robot_spec_path_from_id

IK_ATTEMPTS = 5


def round_trip(robot: str, n: int, seed: int = 0):
    spec = load_robot_spec(robot_spec_path_from_id(robot, repo_root=ROOT))
    lo, hi = get_limits_from_spec(spec)
    qmin, qmax = np.deg2rad(np.asarray(lo, float)), np.deg2rad(np.asarray(hi, float))
    q_home = 0.5 * (qmin + qmax)
    urdf = None
    if robot in C.ROBOTS_URDF:
        from twinbench.dynamics.urdf_plant import get_urdf_robot
        urdf = get_urdf_robot(ROOT / C.URDF_PATHS[robot])
    dh = None if urdf is not None else R.build_dh_from_spec(spec)

    rng = np.random.default_rng(seed)
    errors = []
    for _ in range(n):
        p_des = R.ee_pos(rng.uniform(qmin, qmax), dh=dh, urdf_robot=urdf)
        for attempt in range(IK_ATTEMPTS):
            q0 = q_home if attempt == 0 else rng.uniform(qmin, qmax)
            if urdf is not None:
                q, ok, _ = R.ik_pos_urdf(p_des, q0, urdf, qmin, qmax)
            else:
                q, ok, _ = R.ik_pos(p_des, q0, dh, qmin, qmax)
            if ok:
                p = R.ee_pos(q, dh=dh, urdf_robot=urdf)
                errors.append(float(np.linalg.norm(p - p_des)))
                break
    return np.asarray(errors)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--n", type=int, default=60, help="configurations per robot")
    a = ap.parse_args()
    means, maxes = [], []
    print(f"{'robot':8s} {'solved':>8s} {'mean [mm]':>10s} {'max [mm]':>10s}")
    for robot in C.ROBOTS_PROXY + C.ROBOTS_URDF:
        e = round_trip(robot, a.n) * 1000.0
        means.append(e.mean()); maxes.append(e.max())
        print(f"{robot:8s} {len(e):>4d}/{a.n:<3d} {e.mean():10.3f} {e.max():10.3f}")
    print(f"\nper-robot mean {min(means):.2f}-{max(means):.2f} mm, worst case {max(maxes):.2f} mm")


if __name__ == "__main__":
    main()
