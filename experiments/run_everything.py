"""Regenerate the simulation results in data/.

    python experiments/run_everything.py                 # main grid, 43,200 runs
    python experiments/run_everything.py --workers 4
    python experiments/run_everything.py --steps supplementary
    python experiments/run_everything.py --n-seeds 1 --outdir /tmp/smoke

Main steps (runs, output):

    urdf          5,760   all_results_main.csv
    datasheet     5,760   all_results_datasheet.csv, all_results_d3_kr6_decoupled_20seed.csv
    coupled       9,600   d3_coupled_all_20seed.csv
    raised_clamp  1,920   all_results_nocap_ur5e_eps0p09_{coupled,decoupled}.csv
    cap           1,440   all_results_cap0p9_ctcsmc_check.csv
    frozen        2,880   all_results_frozen_gains_rigid.csv
    dt            2,880   all_results_dt0p5.csv, all_results_dt0p25.csv
    mismatch     12,960   d1_mismatch_all.csv
    total        43,200

Supplementary steps (run only when named):

    symmetric     2,880   all_results_symmetric_uncertainty_rigid.csv  (rigid plant scaled in I and G)
    factors       1,920   diag_proxy_factor_restore.csv                (one proxy factor set to nominal)
    eps0            960   diag_ur5e_coupled_eps0.csv                   (coupled plant, eps_c = 0)
    nominal_gains 5,760   diag_nominal_inertia_gains.csv               (gains from nominal inertia)

Seeds depend only on (robot, scene, trajectory, params, seed), so results do not
depend on --workers. data/workspace_feasibility.csv is not produced here.
"""
from __future__ import annotations

import argparse
import functools
import itertools
import multiprocessing as mp
import sys
import time
from pathlib import Path

import pandas as pd

ROOT = Path(__file__).resolve().parents[1]
for _p in (ROOT, ROOT / "src"):
    if str(_p) not in sys.path:
        sys.path.insert(0, str(_p))

import twinbench.constants as C
import experiments.run_main_grid as R

URDF = list(C.ROBOTS_URDF)
DATASHEET = list(C.ROBOTS_PROXY)
TRAJ = list(C.TRAJECTORIES)
CTRL = list(C.CONTROLLERS)
PARAMS = list(C.PARAMS)
EPS_LIST = [0.03, 0.06, 0.09, 0.15, 0.25]
RAISED_CLAMP_MULT = 1.0e4      # torque-cap ceiling of the raised-clamp diagnostic, x tau_max
RAISED_CLAMP_EPS = 0.09
MISMATCH_PCTS = [0.10, 0.20, 0.30]
MISMATCH_REALIZATIONS = [0, 1, 2, 3, 4, 5]
MATCHED_SEEDS = [0, 1, 2, 3, 4]   # step-size and mismatch arms
FACTOR_RESTORE = ["I", "B", "G", "BG"]   # plant factors put back to nominal, one set per arm

COUPLED_COLS = ["robot", "eps_c", "trajectory", "controller", "params", "seed",
                "RMSE_cart_norm", "Max_cart_err_norm", "E_star", "Tau_rms_norm", "Tau_peak_ratio",
                "Sat_frac_any", "Vel_violation_frac", "Acc_violation_frac",
                "Jerk_violation_frac", "ik_ok", "cart_err_ok"]
MISMATCH_COLS = ["robot", "pct", "realization", "trajectory", "controller", "params", "seed",
                 "RMSE_cart_norm", "Tau_peak_ratio", "Sat_frac_any", "Vel_violation_frac",
                 "Acc_violation_frac"]


# job runners
def _main(job):
    rob, scene, fid, traj, ctrl, prm, sd, kw = job
    return R.run_single(rob, scene, traj, ctrl, prm, sd, fidelity=fid, **kw)


def _dt(job):
    dt, rob, fid, traj, ctrl, prm, sd = job
    old = R.DT
    R.DT = dt
    C.SIM_DT = dt
    try:
        return R.run_single(rob, "open_reach", traj, ctrl, prm, sd, fidelity=fid)
    finally:
        R.DT = old
        C.SIM_DT = old


def _frozen(job):
    import twinbench.dynamics.urdf_plant as U
    import experiments.run_frozen_gain_ablation as F
    rob, traj, ctrl, prm, sd = job
    old = U.control_torque_pinocchio
    U.control_torque_pinocchio = F.control_torque_pinocchio_frozen
    try:
        res = R.run_single(rob, "open_reach", traj, ctrl, prm, sd, fidelity="rigid_body")
    finally:
        U.control_torque_pinocchio = old
    return {c: res.get(c) for c in F.COLS}


def _mismatch(job):
    rob, pct, realz, traj, ctrl, prm, sd = job
    R.PARAM_MISMATCH_FRAC = pct
    R.PARAM_MISMATCH_SEED = realz
    try:
        res = R.run_single(rob, "open_reach", traj, ctrl, prm, sd, fidelity="rigid_body")
    finally:
        R.PARAM_MISMATCH_FRAC = None
        R.PARAM_MISMATCH_SEED = 42
    out = dict(robot=rob, pct=pct, realization=realz, trajectory=traj, controller=ctrl,
               params=prm, seed=sd)
    out.update({c: res[c] for c in MISMATCH_COLS if c not in out})
    return out


def _coupled(job):
    rob, eps, traj, ctrl, prm, sd, cap_mult = job
    if rob == "kr6":
        import experiments.run_coupled_sweep as M
    else:
        import experiments.run_coupled_sweep_ur5e as M
    old = M.control_torque
    if cap_mult is not None:
        M.control_torque = functools.partial(R.control_torque, tau_cap_frac=cap_mult,
                                             tau_cap_frac_smc=cap_mult)
    try:
        res = M.run_coupled_single(rob, "open_reach", traj, ctrl, prm, sd, coupling_strength=eps)
    finally:
        M.control_torque = old
    out = {"robot": rob, "eps_c": eps}
    out.update({c: res.get(c) for c in COUPLED_COLS if c not in out})
    return out


def _decoupled_raised(job):
    """Decoupled proxy run with the torque cap raised (gains unchanged)."""
    traj, ctrl, prm, sd, cap_mult = job
    orig = R.simulate_proxy

    def raised(**kw):
        kw["tau_cap_frac"] = cap_mult
        kw["tau_cap_frac_smc"] = cap_mult
        return orig(**kw)

    R.simulate_proxy = raised
    try:
        res = R.run_single("ur5e", "open_reach", traj, ctrl, prm, sd, fidelity="proxy")
    finally:
        R.simulate_proxy = orig
    out = {"robot": "ur5e"}
    out.update({c: res.get(c) for c in COUPLED_COLS if c not in out and c != "eps_c"})
    return out


def _factor(job):
    """Proxy run with the listed plant factors restored to nominal (e.g. 'BG')."""
    rob, traj, ctrl, prm, sd, restore = job
    orig = R.simulate_proxy

    def restored(**kw):
        for f in restore:
            kw[f + "_eff"] = kw[f + "_nom"]
        return orig(**kw)

    R.simulate_proxy = restored
    try:
        res = R.run_single(rob, "open_reach", traj, ctrl, prm, sd, fidelity="proxy")
    finally:
        R.simulate_proxy = orig
    out = dict(robot=rob, restored=restore, trajectory=traj, controller=ctrl, params=prm, seed=sd)
    out.update({c: res[c] for c in ("RMSE_cart_norm", "Tau_peak_ratio", "Sat_frac_any",
                                    "Vel_violation_frac", "Acc_violation_frac")})
    return out


# steps
def _pool_map(fn, jobs, workers, label):
    t0 = time.time()
    print(f"> {label}: {len(jobs):,} runs on {workers} worker(s)", flush=True)
    if workers <= 1:
        rows = [fn(j) for j in jobs]
    else:
        # fork where available (Linux, macOS); spawn on Windows. Each job sets its own
        # module state, so both give the same results.
        method = "fork" if "fork" in mp.get_all_start_methods() else "spawn"
        with mp.get_context(method).Pool(workers) as pool:
            rows = pool.map(fn, jobs, chunksize=max(1, len(jobs) // (workers * 16)))
    print(f"  done in {time.time() - t0:.0f} s", flush=True)
    return rows


def _save(rows, path):
    df = pd.DataFrame(rows)
    df.to_csv(path, index=False)
    print(f"  wrote {len(df):,} rows -> {path}", flush=True)
    return df


def step_urdf(a):
    jobs = [(r, "open_reach", fid, t, c, p, s, {})
            for fid in ("proxy", "rigid_body")
            for r, t, c, p, s in itertools.product(URDF, TRAJ, CTRL, PARAMS, a.seeds)]
    _save(_pool_map(_main, jobs, a.workers, "URDF grid (proxy + rigid-body)"),
          a.outdir / "all_results_main.csv")


def step_datasheet(a):
    jobs = [(r, scene, "proxy", t, c, p, s, {})
            for scene in ("open_reach", "shelf_reach")
            for r, t, c, p, s in itertools.product(DATASHEET, TRAJ, CTRL, PARAMS, a.seeds)]
    df = _save(_pool_map(_main, jobs, a.workers, "datasheet grid (proxy, both scenes)"),
               a.outdir / "all_results_datasheet.csv")
    kr6 = df[(df.robot == "kr6") & (df.scene == "open_reach")]
    kr6.to_csv(a.outdir / "all_results_d3_kr6_decoupled_20seed.csv", index=False)
    print(f"  wrote {len(kr6):,} rows -> all_results_d3_kr6_decoupled_20seed.csv (subset)")


def step_coupled(a):
    jobs = [(r, eps, t, c, p, s, None)
            for r in ("kr6", "ur5e") for eps in EPS_LIST
            for t, c, p, s in itertools.product(TRAJ, CTRL, PARAMS, a.seeds)]
    df = _save(_pool_map(_coupled, jobs, a.workers, "coupled-proxy sweep"),
               a.outdir / "d3_coupled_all_20seed.csv")
    return df


def step_raised_clamp(a):
    grid = list(itertools.product(TRAJ, CTRL, PARAMS, a.seeds))
    jobs = [("ur5e", RAISED_CLAMP_EPS, t, c, p, s, RAISED_CLAMP_MULT) for t, c, p, s in grid]
    _save(_pool_map(_coupled, jobs, a.workers, "raised-clamp diagnostic, coupled"),
          a.outdir / "all_results_nocap_ur5e_eps0p09_coupled.csv")
    jobs = [(t, c, p, s, RAISED_CLAMP_MULT) for t, c, p, s in grid]
    _save(_pool_map(_decoupled_raised, jobs, a.workers, "raised-clamp diagnostic, decoupled"),
          a.outdir / "all_results_nocap_ur5e_eps0p09_decoupled.csv")


def step_cap(a):
    jobs = [(r, "open_reach", fid, t, "ctc_smc", p, s, {"tau_cap_frac_smc": 0.9})
            for fid in ("proxy", "rigid_body")
            for r, t, p, s in itertools.product(URDF, TRAJ, PARAMS, a.seeds)]
    _save(_pool_map(_main, jobs, a.workers, "torque-cap ablation (CTC-SMC at 0.9 tau_max)"),
          a.outdir / "all_results_cap0p9_ctcsmc_check.csv")


def step_frozen(a):
    jobs = list(itertools.product(URDF, TRAJ, CTRL, PARAMS, a.seeds))
    _save(_pool_map(_frozen, jobs, a.workers, "frozen-gain ablation (rigid-body tier)"),
          a.outdir / "all_results_frozen_gains_rigid.csv")


def step_dt(a):
    seeds = [s for s in MATCHED_SEEDS if s in a.seeds] or a.seeds
    for dt, tag in ((0.0005, "dt0p5"), (0.00025, "dt0p25")):
        jobs = [(dt, r, fid, t, c, p, s)
                for fid in ("proxy", "rigid_body")
                for r, t, c, p, s in itertools.product(URDF, TRAJ, CTRL, PARAMS, seeds)]
        _save(_pool_map(_dt, jobs, a.workers, f"step-size ablation, dt = {dt * 1e3:g} ms"),
              a.outdir / f"all_results_{tag}.csv")


def step_mismatch(a):
    seeds = [s for s in MATCHED_SEEDS if s in a.seeds] or a.seeds
    jobs = list(itertools.product(URDF, MISMATCH_PCTS, MISMATCH_REALIZATIONS, TRAJ, CTRL, PARAMS, seeds))
    rows = _pool_map(_mismatch, jobs, a.workers, "parametric-mismatch sweep (rigid-body tier)")
    _save(rows, a.outdir / "d1_mismatch_all.csv")


def step_symmetric(a):
    jobs = [(r, "open_reach", "rigid_body", t, c, p, s, {"symmetric_rigid_plant": True})
            for r, t, c, p, s in itertools.product(URDF, TRAJ, CTRL, PARAMS, a.seeds)]
    _save(_pool_map(_main, jobs, a.workers, "symmetric rigid-plant uncertainty (supplementary)"),
          a.outdir / "all_results_symmetric_uncertainty_rigid.csv")


def step_factors(a):
    seeds = [s for s in MATCHED_SEEDS if s in a.seeds] or a.seeds
    jobs = [(r, t, c, p, s, restore)
            for restore in FACTOR_RESTORE for p in ("light", "heavy")
            for r, t, c, s in itertools.product(URDF, TRAJ, CTRL, seeds)]
    _save(_pool_map(_factor, jobs, a.workers, "proxy factor-restore diagnostic (supplementary)"),
          a.outdir / "diag_proxy_factor_restore.csv")


def step_nominal_gains(a):
    jobs = [(r, "open_reach", fid, t, c, p, s, {"gain_inertia_source": "I_nom"})
            for fid in ("proxy", "rigid_body")
            for r, t, c, p, s in itertools.product(URDF, TRAJ, CTRL, PARAMS, a.seeds)]
    _save(_pool_map(_main, jobs, a.workers, "URDF grid, gains from nominal inertia (supplementary)"),
          a.outdir / "diag_nominal_inertia_gains.csv")


def step_eps0(a):
    jobs = [("ur5e", 0.0, t, c, p, s, None) for t, c, p, s in itertools.product(TRAJ, CTRL, PARAMS, a.seeds)]
    _save(_pool_map(_coupled, jobs, a.workers, "UR5e coupled plant, coupling off (supplementary)"),
          a.outdir / "diag_ur5e_coupled_eps0.csv")


MAIN_STEPS = ["urdf", "datasheet", "coupled", "raised_clamp", "cap", "frozen", "dt", "mismatch"]

STEPS = {
    "urdf": step_urdf, "datasheet": step_datasheet, "coupled": step_coupled,
    "raised_clamp": step_raised_clamp, "cap": step_cap, "frozen": step_frozen,
    "dt": step_dt, "mismatch": step_mismatch, "symmetric": step_symmetric,
    "factors": step_factors, "eps0": step_eps0, "nominal_gains": step_nominal_gains,
}


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--steps", default="all",
                    help="'all' (the 43,200 runs), 'supplementary', or a comma-separated list of: "
                         + ",".join(STEPS))
    ap.add_argument("--workers", type=int, default=1)
    ap.add_argument("--n-seeds", type=int, default=None, help="smoke test: first N seeds only")
    ap.add_argument("--outdir", type=Path, default=ROOT / "data")
    a = ap.parse_args()
    a.seeds = list(C.SEEDS) if a.n_seeds is None else list(C.SEEDS)[:a.n_seeds]
    a.outdir.mkdir(parents=True, exist_ok=True)
    if a.steps == "all":
        names = MAIN_STEPS
    elif a.steps == "supplementary":
        names = [s for s in STEPS if s not in MAIN_STEPS]
    else:
        names = [s.strip() for s in a.steps.split(",")]
    t0 = time.time()
    for n in names:
        STEPS[n](a)
    print(f"ALL DONE: {', '.join(names)} in {(time.time() - t0) / 60:.1f} min")


if __name__ == "__main__":
    main()
