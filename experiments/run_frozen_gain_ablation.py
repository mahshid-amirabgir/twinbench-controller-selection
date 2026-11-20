"""Gain-scaling ablation: frozen proxy gains on rigid-body tier."""

from __future__ import annotations

import sys
import time
from pathlib import Path

import numpy as np
import pandas as pd

ROOT = Path(__file__).resolve().parents[1]
for _p in (ROOT, ROOT / "src"):
    if str(_p) not in sys.path:
        sys.path.insert(0, str(_p))

import twinbench.dynamics.urdf_plant as U
import experiments.run_main_grid as R

OUT = ROOT / "data" / "all_results_frozen_gains_rigid.csv"
COLS = ["robot", "scene", "trajectory", "controller", "params", "seed", "fidelity",
        "RMSE_cart_norm", "Max_cart_err_norm", "E_star", "Tau_rms_norm", "Tau_peak_ratio",
        "Sat_frac_any", "Vel_violation_frac", "Acc_violation_frac", "ik_ok"]


def control_torque_pinocchio_frozen(
    q, dq, qd_k, dqd_k, ddqd_k, *,
    urdf_robot, tau_max, ddq_cap, dq_cap, B_nom,
    ctrl,
    WN_FRAC, ZETA, T_BASE, DT,
    TAU_CAP_FRAC, TAU_CAP_FRAC_SMC,
    SMC_PHI_FRAC, SMC_KS_FRAC,
    I_nom, G_nom,
    include_B_feedforward: bool = True,
):
    """Same as control_torque_pinocchio, except that CTC-SMC's K_s and PBC's K_D
    are scaled by the proxy-tier inertia I_nom = diag(M(0)) instead of the live
    diag(M(q)) (lines marked 'frozen')."""
    EPS = 1e-9
    wn = WN_FRAC * np.sqrt(np.maximum(ddq_cap, 1e-6))
    Kp, Kd = wn**2, 2.0 * ZETA * wn
    e, de = qd_k - q, dqd_k - dq

    M = urdf_robot.mass_matrix(q)
    nle = urdf_robot.nonlinear_effects(q, dq)
    g_q = urdf_robot.gravity(q)

    I_nom_arr = np.asarray(I_nom, float)  # proxy-tier nominal inertia diag(M(0))

    if ctrl == "pdg":
        tau = M @ (Kp * e + Kd * de) + g_q
        cap = TAU_CAP_FRAC * tau_max
    elif ctrl == "ctc_pd":
        v = ddqd_k + Kp * e + Kd * de
        tau = M @ v + nle
        if include_B_feedforward:
            tau = tau + B_nom * dq
        cap = TAU_CAP_FRAC * tau_max
    elif ctrl == "ctc_smc":
        wn_smc = 2.0 * np.pi / max(T_BASE, 1e-6)
        Lambda = 2.0 * ZETA * wn_smc
        s = de + Lambda * e
        v_eq = ddqd_k + Lambda * de
        tau_eq = M @ v_eq + nle
        if include_B_feedforward:
            tau_eq = tau_eq + B_nom * dq
        M_diag = I_nom_arr  # frozen: proxy-tier inertia instead of diag(M(q))
        Ks = SMC_KS_FRAC * M_diag * np.maximum(Kp, 1.0)
        phi = SMC_PHI_FRAC * dq_cap
        tau = tau_eq + Ks * np.tanh(s / (phi + EPS))
        cap = TAU_CAP_FRAC_SMC * tau_max
    elif ctrl == "pbc":
        wn_pbc = 2.0 * np.pi / max(T_BASE, 1e-6)
        Lambda = 2.0 * ZETA * wn_pbc
        qd_r = dqd_k + Lambda * e
        qdd_r = ddqd_k + Lambda * de
        s = de + Lambda * e
        C = urdf_robot.coriolis_matrix(q, dq)
        K_D = I_nom_arr * Kd  # frozen: proxy-tier inertia instead of diag(M(q))
        tau = M @ qdd_r + C @ qd_r + g_q + K_D * s
        if include_B_feedforward:
            tau = tau + B_nom * qd_r
        cap = TAU_CAP_FRAC * tau_max
    else:
        raise ValueError(f"Unknown controller: {ctrl}")
    return np.asarray(tau, float), np.asarray(cap, float)


def main():
    U.control_torque_pinocchio = control_torque_pinocchio_frozen

    rows = []
    t0 = time.time()
    n = 0
    for rob in ["ur5e", "ur10e", "tx60"]:
        for traj in ["cubic", "minjerk", "fsmj", "fsmj_novia"]:
            for ctrl in ["pdg", "ctc_pd", "ctc_smc", "pbc"]:
                for prm in ["light", "nominal", "heavy"]:
                    for seed in range(20):
                        res = R.run_single(rob, "open_reach", traj, ctrl, prm, seed,
                                           fidelity="rigid_body")
                        rows.append([res.get(c) for c in COLS])
                        n += 1
        pd.DataFrame(rows, columns=COLS).to_csv(OUT, index=False)
        print(f"{rob} done | n={n} | {time.time()-t0:.0f}s", flush=True)

    df = pd.DataFrame(rows, columns=COLS)
    df.to_csv(OUT, index=False)
    print(f"DONE {n} runs {time.time()-t0:.0f}s -> {OUT}")
    print("Sat_frac_any max:", df.Sat_frac_any.max(),
          " Acc_violation max:", df.Acc_violation_frac.max())


if __name__ == "__main__":
    main()
