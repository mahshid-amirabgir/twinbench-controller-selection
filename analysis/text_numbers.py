"""Numbers quoted in the paper text, recomputed from data/*.csv.

    python analysis/text_numbers.py [--data DIR]
"""
from __future__ import annotations

import argparse
import itertools
import sys
from pathlib import Path

import numpy as np
import pandas as pd
from scipy.stats import kendalltau, mannwhitneyu

ROOT = Path(__file__).resolve().parents[1]
for _p in (ROOT, ROOT / "src"):
    if str(_p) not in sys.path:
        sys.path.insert(0, str(_p))

from twinbench.metrics.statistics import compute_per_context_cross_fidelity_tau

LABEL = {"pbc": "PBC", "ctc_smc": "CTC-SMC", "ctc_pd": "CTC-PD", "pdg": "PD+g"}
MODEL_BASED = ["ctc_pd", "ctc_smc", "pdg"]
PARAMS = ["nominal", "heavy", "light"]


def order(means: pd.Series) -> str:
    return " < ".join(LABEL[c] for c in means.sort_values().index)


def cohen_d(a, b):
    a, b = np.asarray(a, float), np.asarray(b, float)
    sp = np.sqrt(((len(a) - 1) * a.var(ddof=1) + (len(b) - 1) * b.var(ddof=1)) / (len(a) + len(b) - 2))
    return (a.mean() - b.mean()) / sp


def holm(pvals):
    p = np.asarray(pvals, float)
    idx = np.argsort(p)
    adj = np.empty_like(p)
    running = 0.0
    for rank, i in enumerate(idx):
        running = max(running, (len(p) - rank) * p[i])
        adj[i] = min(1.0, running)
    return adj


def h(title):
    print("\n" + "=" * 78 + f"\n{title}\n" + "=" * 78)


def section_datasheet(D):
    h("Section 4.1 -- datasheet robots, proxy tier")
    df = pd.read_csv(D / "all_results_datasheet.csv")
    means = df.groupby("controller").RMSE_cart_norm.mean()
    print("overall ordering:", order(means))
    cells = df.groupby(["robot", "scene", "controller"]).RMSE_cart_norm.mean().unstack()
    orders = cells.apply(order, axis=1)
    print(f"orderings over the {len(orders)} (robot, scene) cells:")
    for o, n in orders.value_counts().items():
        print(f"   {n} x  {o}")
    pairs = list(itertools.combinations(["pbc", "ctc_smc", "ctc_pd", "pdg"], 2))
    pv, ds = [], []
    for a, b in pairs:
        x, y = df[df.controller == a].RMSE_cart_norm, df[df.controller == b].RMSE_cart_norm
        pv.append(mannwhitneyu(x, y, alternative="two-sided").pvalue)
        ds.append(cohen_d(x, y))
    adj = holm(pv)
    print("pairwise Mann-Whitney U (two-sided), Holm-corrected p, and Cohen's d:")
    for (a, b), p, d in zip(pairs, adj, ds):
        print(f"   {LABEL[a]:8s} vs {LABEL[b]:8s}  p_holm = {p:.2e}   d = {d:+.2f}")
    print(f"   largest corrected p = {adj.max():.2e}")
    mb = means[MODEL_BASED]
    ctrl_spread = (mb.max() - mb.min()) / mb.min()
    traj = df[df.controller.isin(MODEL_BASED)].groupby("trajectory").RMSE_cart_norm.mean()
    traj_spread = (traj.max() - traj.min()) / traj.min()
    print(f"model-based controllers: controller-induced spread {100 * ctrl_spread:.0f}% "
          f"({mb.min():.4f} -> {mb.max():.4f}); trajectory-induced spread {100 * traj_spread:.1f}% "
          f"(ratio {ctrl_spread / traj_spread:.0f}x)")
    traj_all = df.groupby("trajectory").RMSE_cart_norm.mean()
    print("mean RMSE by trajectory (all four controllers):",
          {k: round(v, 4) for k, v in traj_all.sort_values().items()},
          f" range {100 * (traj_all.max() - traj_all.min()) / traj_all.min():.1f}%")
    print("by uncertainty level:")
    for p in PARAMS:
        print(f"   {p:8s} {order(df[df.params == p].groupby('controller').RMSE_cart_norm.mean())}")
    vv = df.groupby(["params", "controller"]).Vel_violation_frac.mean().unstack()
    print(f"max saturated fraction: {df.Sat_frac_any.max():.3f}")
    print(f"mean velocity-limit violation by (level, controller): "
          f"{vv.min().min():.3f} to {vv.max().max():.3f}")
    print(vv[["pbc", "ctc_smc", "ctc_pd", "pdg"]].rename(columns=LABEL).round(3).to_string())


def section_fidelity(D):
    h("Section 4.2 -- URDF robots, proxy vs rigid-body")
    df = pd.read_csv(D / "all_results_main.csv")
    for fid in ("proxy", "rigid_body"):
        sub = df[df.fidelity == fid]
        print(f"{fid:10s} overall: {order(sub.groupby('controller').RMSE_cart_norm.mean())}")
        for p in PARAMS:
            m = sub[sub.params == p].groupby("controller").RMSE_cart_norm.mean()
            print(f"   {p:8s} {order(m)}    " + "  ".join(f"{LABEL[c]} {m[c]:.4f}" for c in m.sort_values().index))
    ctx = df.groupby(["fidelity", "robot", "params", "trajectory", "controller"]).RMSE_cart_norm.mean().unstack()
    for fid in ("proxy", "rigid_body"):
        orders = ctx.loc[fid].apply(order, axis=1)
        pbc_first = int((ctx.loc[fid].idxmin(axis=1) == "pbc").sum())
        print(f"{fid}: PBC rank 1 in {pbc_first}/{len(orders)} contexts; "
              f"{orders.nunique()} distinct four-controller orderings")
    sub = compute_per_context_cross_fidelity_tau(df, metric="RMSE_cart_norm", exclude_controllers=["pbc"])
    print("model-based subset tau_b by robot:", sub.groupby("robot").tau_b.mean().round(3).to_dict())
    print("model-based subset tau_b by uncertainty:", sub.groupby("params").tau_b.mean().round(3).to_dict())
    print("model-based subset tau_b by robot x uncertainty:")
    print(sub.groupby(["robot", "params"]).tau_b.mean().unstack().round(3).to_string())


def section_coupled(D):
    h("Section 4.3 -- coupled-proxy sweep and raised-clamp diagnostic")
    c = pd.read_csv(D / "d3_coupled_all_20seed.csv")
    kd = pd.read_csv(D / "all_results_d3_kr6_decoupled_20seed.csv")
    main = pd.read_csv(D / "all_results_main.csv")
    ud = main[(main.robot == "ur5e") & (main.fidelity == "proxy")]
    for rob, dec in (("kr6", kd), ("ur5e", ud)):
        print(f"{rob}:")
        for label, plist in (("all levels", PARAMS), ("nominal+heavy", ["nominal", "heavy"]), ("light", ["light"])):
            d0 = dec[dec.params.isin(plist)].groupby("controller").RMSE_cart_norm.mean()
            taus, chg = [], []
            for eps, g in c[(c.robot == rob) & c.params.isin(plist)].groupby("eps_c"):
                m = g.groupby("controller").RMSE_cart_norm.mean()
                taus.append(kendalltau(d0.rank()[m.index], m.rank()).statistic)
                chg.append(float(((m - d0[m.index]).abs() / d0[m.index]).max()))
            print(f"   {label:14s} decoupled {order(d0)};  tau_b vs decoupled per eps_c = "
                  f"{[round(float(t), 3) for t in taus]};  max RMSE change {100 * max(chg):.1f}%")
        sat = c[c.robot == rob].groupby(["params", "controller"]).Sat_frac_any.mean().unstack()
        print("   mean saturated fraction (all eps_c):")
        print("   " + sat.round(3).to_string().replace("\n", "\n   "))
    for f in ("all_results_nocap_ur5e_eps0p09_coupled.csv", "all_results_nocap_ur5e_eps0p09_decoupled.csv"):
        r = pd.read_csv(D / f)
        s = r.groupby(["params", "controller"])[["Sat_frac_any", "Vel_violation_frac"]].mean()
        light = s.loc["light"]
        print(f"{f}: light-level mean saturated fraction "
              + ", ".join(f"{LABEL[k]} {v:.3f}" for k, v in light.Sat_frac_any.items())
              + f"; max over nominal+heavy {float(s.drop('light').Sat_frac_any.max()):.3f}")


def _rigid_means(df):
    return df[df.fidelity == "rigid_body"].groupby("controller").RMSE_cart_norm.mean()


def section_ablations(D):
    h("Section 4.4 -- torque cap, frozen gains, step size")
    main = pd.read_csv(D / "all_results_main.csv")
    key = ["robot", "fidelity", "trajectory", "params", "seed"]
    cap = pd.read_csv(D / "all_results_cap0p9_ctcsmc_check.csv")
    m = main[main.controller == "ctc_smc"].merge(cap, on=key, suffixes=("_07", "_09"))
    d = (m.RMSE_cart_norm_09 - m.RMSE_cart_norm_07).abs()
    print(f"cap 0.9 vs 0.7 (CTC-SMC, {len(m)} matched runs): max |dRMSE| = {d.max():.3g}; "
          f"runs that differ: {(d > 0).sum()}")
    by = m.assign(d=d).groupby(["fidelity", "params"]).d.max().unstack()
    print("   max |dRMSE| by tier and uncertainty:\n   " + by.round(4).to_string().replace("\n", "\n   "))
    if "Tau_peak_ratio" in main.columns:
        pk = main.groupby(["params", "controller"]).Tau_peak_ratio.max().unstack()
        print("peak |tau|/tau_max, main URDF grid (both tiers, 20 seeds):\n   "
              + pk.round(3).to_string().replace("\n", "\n   "))

    fr = pd.read_csv(D / "all_results_frozen_gains_rigid.csv")
    k2 = ["robot", "trajectory", "controller", "params", "seed"]
    m = main[main.fidelity == "rigid_body"].merge(fr, on=k2, suffixes=("_live", "_frozen"))
    m["d"] = (m.RMSE_cart_norm_frozen - m.RMSE_cart_norm_live).abs()
    print("frozen vs tier-scaled gains, rigid-body tier, max |dRMSE| by controller:",
          {LABEL[c]: float(f"{v:.3g}") for c, v in m.groupby("controller").d.max().items()})
    lm, fm = _rigid_means(main), fr.groupby("controller").RMSE_cart_norm.mean()
    print("   rigid-body mean RMSE live -> frozen:",
          {LABEL[c]: f"{lm[c]:.4f} -> {fm[c]:.4f}" for c in lm.index})
    for label, plist in (("nominal+heavy", ["nominal", "heavy"]), ("light", ["light"])):
        a = main[(main.fidelity == "rigid_body") & main.params.isin(plist)].groupby("controller").RMSE_cart_norm.mean()
        b = fr[fr.params.isin(plist)].groupby("controller").RMSE_cart_norm.mean()
        print(f"   {label:14s} live {order(a)} | frozen {order(b)}")
    cells = fr.groupby(["robot", "params", "controller"]).RMSE_cart_norm.mean().unstack()
    best_mb = cells[MODEL_BASED].idxmin(axis=1).value_counts().to_dict()
    print("   frozen arm: best model-based controller per (robot, uncertainty) cell:",
          {LABEL[k]: v for k, v in best_mb.items()})

    print("step size, rigid-body mean RMSE on the matched five-seed subset:")
    arms = {"1.0": main[main.seed.isin([0, 1, 2, 3, 4])],
            "0.5": pd.read_csv(D / "all_results_dt0p5.csv"), "0.25": pd.read_csv(D / "all_results_dt0p25.csv")}
    for label, plist in (("all levels", PARAMS), ("nominal+heavy", ["nominal", "heavy"])):
        for c in ("ctc_smc", "pbc"):
            vals = [float(_rigid_means(a[a.params.isin(plist)])[c]) for a in arms.values()]
            print(f"   {label:14s} {LABEL[c]:8s} " + " -> ".join(f"{v:.4f}" for v in vals) + "   (1.0 -> 0.5 -> 0.25 ms)")
    for tag, a in arms.items():
        for fid in ("proxy", "rigid_body"):
            print(f"   dt={tag:4s} {fid:10s} {order(a[a.fidelity == fid].groupby('controller').RMSE_cart_norm.mean())}")


def section_mismatch(D):
    h("Section 4.5 -- parametric (link-mass) mismatch, rigid-body tier")
    df = pd.read_csv(D / "d1_mismatch_all.csv")
    for label, plist in (("all levels", PARAMS), ("nominal+heavy", ["nominal", "heavy"])):
        print(f"{label}:")
        for pct, g in df[df.params.isin(plist)].groupby("pct"):
            per = g.groupby(["realization", "controller"]).RMSE_cart_norm.mean().unstack()
            first = int((per.idxmin(axis=1) == "pbc").sum())
            mb_first = per[MODEL_BASED].idxmin(axis=1).value_counts().to_dict()
            print(f"   +/-{100 * pct:.0f}%: {order(per.mean())}; PBC rank 1 in {first}/{len(per)} realizations; "
                  f"best model-based: { {LABEL[k]: v for k, v in mb_first.items()} }")
    sat = df.groupby(["params", "controller"]).Sat_frac_any.mean().unstack()
    print("mean saturated fraction:\n   " + sat.round(3).to_string().replace("\n", "\n   "))


def section_symmetric(D):
    f = D / "all_results_symmetric_uncertainty_rigid.csv"
    if not f.exists():
        return
    h("Supplementary -- rigid-body plant with M and g scaled like the proxy tier's (I, G)")
    main = pd.read_csv(D / "all_results_main.csv")
    sym = pd.read_csv(f)
    both = pd.concat([main[main.fidelity == "proxy"], sym], ignore_index=True)
    for label, d in (("main grid (rigid plant: B scaled only)", main), ("symmetric (rigid plant: M, B, g scaled)", both)):
        sub = compute_per_context_cross_fidelity_tau(d, metric="RMSE_cart_norm", exclude_controllers=["pbc"])
        print(f"{label}: subset tau_b = {sub.tau_b.mean():.3f}; by uncertainty "
              f"{sub.groupby('params').tau_b.mean().round(3).to_dict()}")
    for p in PARAMS:
        m = sym[sym.params == p].groupby("controller").RMSE_cart_norm.mean()
        s = sym[sym.params == p].groupby("controller").Sat_frac_any.mean()
        print(f"   symmetric rigid, {p:8s} {order(m)}   saturated: "
              + ", ".join(f"{LABEL[c]} {s[c]:.3f}" for c in m.sort_values().index))


def section_factors(D):
    f = D / "diag_proxy_factor_restore.csv"
    if not f.exists():
        return
    h("Supplementary -- proxy tier, URDF robots: one plant factor restored to nominal")
    main = pd.read_csv(D / "all_results_main.csv")
    main5 = main[main.seed.isin([0, 1, 2, 3, 4])]
    fr = pd.read_csv(f)
    base = main5[main5.fidelity == "proxy"].assign(restored="none")
    allarms = pd.concat([base[base.params.isin(["light", "heavy"])], fr], ignore_index=True)
    for prm in ("heavy", "light"):
        print(f"{prm}: mean RMSE | mean velocity-limit violation, by restored factor (seeds 0-4)")
        g = allarms[allarms.params == prm].groupby(["restored", "controller"])
        m, v = g.RMSE_cart_norm.mean().unstack(), g.Vel_violation_frac.mean().unstack()
        for arm in ("none", "I", "B", "G", "BG"):
            print(f"   {arm:5s} " + "  ".join(f"{LABEL[c]} {m.loc[arm, c]:.4f}|{v.loc[arm, c]:.3f}"
                                              for c in ("pbc", "ctc_smc", "ctc_pd", "pdg"))
                  + f"    {order(m.loc[arm])}")
    rigid = main5[main5.fidelity == "rigid_body"]
    print("heavy level, model-based subset tau_b against the rigid-body tier (12 contexts, seeds 0-4):")
    for arm in ("none", "I", "B", "G", "BG"):
        px = allarms[(allarms.restored == arm) & (allarms.params == "heavy")].assign(fidelity="proxy")
        both = pd.concat([px, rigid[rigid.params == "heavy"]], ignore_index=True)
        sub = compute_per_context_cross_fidelity_tau(both, metric="RMSE_cart_norm", exclude_controllers=["pbc"])
        top_p = sub[[f"proxy_{c}" for c in MODEL_BASED]].idxmin(axis=1).str.replace("proxy_", "")
        top_r = sub[[f"rb_{c}" for c in MODEL_BASED]].idxmin(axis=1).str.replace("rb_", "")
        print(f"   proxy plant with {arm:5s} restored: tau_b = {sub.tau_b.mean():+.3f}; "
              f"top-controller disagreement {int((top_p != top_r).sum())}/{len(sub)}")
    try:
        import pinocchio as pin
        import twinbench.constants as C
        from twinbench.dynamics.robot_loader import (
            get_dynamics_cfg_for_params, load_robot_spec, robot_spec_path_from_id)
        from twinbench.dynamics.urdf_plant import get_urdf_robot
        print("proxy gravity amplitude G_nom against the largest |g_i(q)| of the URDF model "
              "(5,000 random configurations within joint limits):")
        rng = np.random.default_rng(0)
        for rob in C.ROBOTS_URDF:
            dyn = get_dynamics_cfg_for_params(load_robot_spec(robot_spec_path_from_id(rob, repo_root=ROOT)), "nominal")
            ur = get_urdf_robot(ROOT / C.URDF_PATHS[rob])
            lo = np.maximum(ur.model.lowerPositionLimit, -np.pi)
            hi = np.minimum(ur.model.upperPositionLimit, np.pi)
            gmax = np.zeros(ur.model.nv)
            for _ in range(5000):
                q = rng.uniform(lo, hi)
                gmax = np.maximum(gmax, np.abs(pin.computeGeneralizedGravity(ur.model, ur.data, q)))
            G = np.asarray(dyn["G_nom"], float)
            print(f"   {rob:6s} G_nom = {np.round(G, 2)}   max|g(q)| = {np.round(gmax, 2)}   "
                  f"ratio = {np.round(G / np.maximum(gmax, 1e-9), 1)}")
    except Exception as e:   # Pinocchio missing or model attribute names differ
        print(f"   (gravity-amplitude comparison skipped: {type(e).__name__}: {e})")


def section_eps0(D):
    f = D / "diag_ur5e_coupled_eps0.csv"
    if not f.exists():
        return
    h("Supplementary -- UR5e on the coupled-proxy plant with coupling switched off (eps_c = 0)")
    d = pd.read_csv(f)
    g = d.groupby(["params", "controller"])
    print("mean saturated fraction:\n   " + g.Sat_frac_any.mean().unstack().round(3).to_string().replace("\n", "\n   "))
    print("mean RMSE:\n   " + g.RMSE_cart_norm.mean().unstack().round(4).to_string().replace("\n", "\n   "))


def section_nominal_gains(D):
    f = D / "diag_nominal_inertia_gains.csv"
    if not f.exists():
        return
    h("Supplementary -- URDF grid with controller gains set from the nominal inertia")
    import analysis.generate_tables as G
    old = G.DATA
    try:
        for lab, d in (("main grid (gains from the level's inertia)", D),
                       ("gains from nominal inertia", None)):
            if d is None:
                tmp = D / "_nominal_gains_view"
                tmp.mkdir(exist_ok=True)
                (tmp / "all_results_main.csv").write_bytes(f.read_bytes())
                G.DATA = tmp
            else:
                G.DATA = d
            by = G.TABLES["tab_fidelity_by_uncertainty"]()[0]
            st = G.TABLES["tab_fidelity_stable"]()[0]
            print(f"{lab}:")
            for p in PARAMS:
                print(f"   {p:8s} subset tau_b {by[p]['subset_tau_b']:.3f}, disagreement "
                      f"{by[p]['disagreement'][0]}/{by[p]['disagreement'][1]}")
            print(f"   nominal+heavy: subset tau_b {st['subset_tau_b']:.3f} "
                  f"[{st['subset_ci'][0]:.3f}, {st['subset_ci'][1]:.3f}], four-controller "
                  f"{st['all4_tau_b']:.3f}, disagreement {st['disagreement'][0]}/{st['disagreement'][1]}")
    finally:
        G.DATA = old
        tmp = D / "_nominal_gains_view"
        if tmp.exists():
            (tmp / "all_results_main.csv").unlink()
            tmp.rmdir()


def section_gains():
    """Gain figures quoted in the text. Needs Pinocchio and regenerates the 240 URDF
    reference trajectories (a few seconds); run with --gains."""
    h("Gain figures (Sections 4.4 and 5.3): effective stiffness and K_s scaling")
    import itertools as it
    import pinocchio as pin
    import twinbench.constants as C
    import experiments.run_main_grid as R
    from twinbench.dynamics.robot_loader import (
        get_dynamics_cfg_for_params, load_robot_spec, robot_spec_path_from_id)
    from twinbench.dynamics.urdf_plant import get_urdf_robot
    np.set_printoptions(precision=2, suppress=True, linewidth=160)
    lam = 2.0 * R.ZETA * 2.0 * np.pi / R.T_BASE
    print("effective position-loop stiffness per unit inertia, relative to CTC-PD and PD+g (K_p):")
    print("   PBC: K_d * Lambda / K_p;  CTC-SMC inside its boundary layer: K_s * Lambda / (I * Phi * K_p)")
    pbc_all, smc_all, pbc_mean = [], [], []
    for rob in list(C.ROBOTS_PROXY) + list(C.ROBOTS_URDF):
        d = get_dynamics_cfg_for_params(load_robot_spec(robot_spec_path_from_id(rob, repo_root=ROOT)), "nominal")
        I, tm = np.asarray(d["I_proxy"], float), np.asarray(d["tau_max"], float)
        kp, kd = R.compute_gains(np.clip(R.TAU_CAP_FRAC * tm / I, 10.0, 1000.0))
        phi = R.SMC_PHI_FRAC * R.QD_CAP_FRAC * np.asarray(d["qd_max_rad"], float)
        r_pbc = kd * lam / kp
        r_smc = R.SMC_KS_FRAC * np.maximum(kp, 1.0) * lam / (phi * kp)
        pbc_all += list(r_pbc); smc_all += list(r_smc); pbc_mean.append(r_pbc.mean())
        print(f"   {rob:8s} PBC {r_pbc} (mean {r_pbc.mean():.2f})   CTC-SMC {r_smc}")
    print(f"   PBC / CTC-PD: {min(pbc_all):.1f}-{max(pbc_all):.1f} over joints; per-robot means "
          f"{min(pbc_mean):.1f}-{max(pbc_mean):.1f}; CTC-SMC / CTC-PD: {min(smc_all):.2f}-{max(smc_all):.2f}")

    print("rigid-tier K_s relative to proxy-tier K_s = diag(M(q)) / diag(M(0)) along the reference trajectories:")
    store = {}
    orig = R.gen_trajectory_for_run

    def capture(*a, **k):
        out = orig(*a, **k)
        store["Q"] = out[1]
        return out

    R.gen_trajectory_for_run = capture
    try:
        for rob in C.ROBOTS_URDF:
            ur = get_urdf_robot(ROOT / C.URDF_PATHS[rob])
            m0 = np.diag(pin.crba(ur.model, ur.data, np.zeros(6))).copy()
            per = []
            for traj, sd in it.product(C.TRAJECTORIES, C.SEEDS):
                R.run_single(rob, "open_reach", traj, "pdg", "nominal", sd, fidelity="proxy")
                per.append(np.array([np.diag(pin.crba(ur.model, ur.data, q)) / m0 for q in store["Q"][::2]]))
            mean = np.array([x.mean(axis=0) for x in per]).mean(axis=0)
            print(f"   {rob:6s} mean over trajectories {mean}   max {np.concatenate(per).max(axis=0)}")
    finally:
        R.gen_trajectory_for_run = orig


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--data", type=Path, default=ROOT / "data")
    ap.add_argument("--gains", action="store_true",
                    help="also recompute the gain figures (needs Pinocchio, takes a minute)")
    a = ap.parse_args()
    pd.set_option("display.width", 200)
    for fn in (section_datasheet, section_fidelity, section_coupled, section_ablations,
               section_mismatch, section_symmetric, section_factors, section_eps0,
               section_nominal_gains):
        fn(a.data)
    if a.gains:
        section_gains()


if __name__ == "__main__":
    main()
