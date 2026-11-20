"""Check the tables and headline numbers against analysis/expected_values.json.

    python analysis/verify_paper_numbers.py                   # check
    python analysis/verify_paper_numbers.py --write-expected  # update the reference values

Run counts and robot datasheet values are fixed in this file.
"""
from __future__ import annotations

import json
import sys
from pathlib import Path

import numpy as np
import pandas as pd

ROOT = Path(__file__).resolve().parents[1]
for _p in (ROOT, ROOT / "src"):
    if str(_p) not in sys.path:
        sys.path.insert(0, str(_p))

from analysis.generate_tables import TABLES, generate_all, _load
from twinbench.metrics.statistics import kendall_stability_matrix, kendall_offdiag_summary

DATA = ROOT / "data"
EXPECTED_JSON = Path(__file__).resolve().parent / "expected_values.json"
TOL = 1e-7   # deterministic pipeline under a pinned bootstrap seed

VALUES: dict[str, object] = {}     # label -> float | str, compared against the JSON
FAILURES: list[str] = []
FIXED_CHECKS = 0


def record(label: str, value):
    """Flatten nested table output (dicts, tuples, Series) into VALUES."""
    if isinstance(value, pd.Series):
        value = value.to_dict()
    if isinstance(value, dict):
        for k, v in value.items():
            k = "/".join(str(x) for x in k) if isinstance(k, tuple) else str(k)
            record(f"{label}[{k}]", v)
    elif isinstance(value, (tuple, list)):
        for i, v in enumerate(value):
            record(f"{label}.{i}", v)
    elif isinstance(value, str):
        VALUES[label] = value
    else:
        VALUES[label] = float(value)


def fixed(label: str, got, expected, tol: float = 1e-9):
    """Structural check against a value fixed in this file."""
    global FIXED_CHECKS
    FIXED_CHECKS += 1
    if abs(float(got) - float(expected)) > tol:
        FAILURES.append(f"{label}: got {got!r}, expected {expected!r}")


# 1. Table fragments
ROBOT_FACTS = {
    "kr6_payload_kg": 6, "kr6_reach_m": 1.82, "kr6_tau_max": 20.0,
    "rs006l_payload_kg": 6, "rs006l_reach_m": 1.65, "rs006l_tau_max": 25.0,
    "mpx1950_payload_kg": 7, "mpx1950_reach_m": 1.45, "mpx1950_tau_max": 18.0,
    "ur5e_payload_kg": 5, "ur5e_reach_m": 0.85, "ur5e_tau_max": 150.0,
    "ur10e_payload_kg": 10, "ur10e_reach_m": 1.30, "ur10e_tau_max": 330.0,
    "tx60_payload_kg": 4.5, "tx60_reach_m": 0.67, "tx60_tau_max": 126.0,
}


def verify_tables():
    print("=" * 78)
    print("PART 1 -- table fragments")
    print("=" * 78)
    results = generate_all(write=True)
    for name in TABLES:
        record(name, results[name]["computed"])
    c = results["tab_robots"]["computed"]
    for k, v in ROBOT_FACTS.items():
        fixed(f"tab_robots[{k}]", c[k], v)
    print(f"  {len(TABLES)} table fragments regenerated -> tables/")


# 2. Within-tier tau_b (self-consistency of rankings inside one tier)
def within_tier_tau_b(df: pd.DataFrame, context_cols: tuple[str, ...]):
    """Per-context single-metric (RMSE_cart_norm) controller ranking, pairwise
    tau_b across all context pairs within one tier."""
    means = (df.groupby(list(context_cols) + ["controller"], observed=True)["RMSE_cart_norm"]
             .mean().reset_index())
    means["rank"] = means.groupby(list(context_cols), observed=True)["RMSE_cart_norm"].rank(method="average")
    _, mat = kendall_stability_matrix(means, context_cols, ("controller",), rank_col="rank")
    return kendall_offdiag_summary(mat)


def verify_within_tier():
    print("\n" + "=" * 78)
    print("PART 2 -- within-tier tau_b (self-consistency)")
    print("=" * 78)
    ds = _load("all_results_datasheet.csv")
    urdf = _load("all_results_main.csv")
    mu, sd, n = within_tier_tau_b(ds, ("robot", "scene", "params", "trajectory"))
    print(f"  datasheet/proxy: {mu:.3f} +/- {sd:.3f} (n={n})")
    record("within_tier[datasheet/proxy]", {"mean": mu, "std": sd})
    for fid in ("proxy", "rigid_body"):
        mu, sd, n = within_tier_tau_b(urdf[urdf.fidelity == fid], ("robot", "params", "trajectory"))
        print(f"  urdf/{fid}: {mu:.3f} +/- {sd:.3f} (n={n})")
        record(f"within_tier[urdf/{fid}]", {"mean": mu, "std": sd})


# 3. Run counts / grand total (fixed)
RUN_COUNT_GROUPS = {
    "main": (["all_results_main.csv", "all_results_datasheet.csv"], 11520),
    "coupled (20-seed)": (["d3_coupled_all_20seed.csv"], 9600),
    "raised-clamp diagnostic": (["all_results_nocap_ur5e_eps0p09_coupled.csv",
                                 "all_results_nocap_ur5e_eps0p09_decoupled.csv"], 1920),
    "cap": (["all_results_cap0p9_ctcsmc_check.csv"], 1440),
    "frozen-gain (gain-scaling) ablation": (["all_results_frozen_gains_rigid.csv"], 2880),
    "step-size": (["all_results_dt0p5.csv", "all_results_dt0p25.csv"], 2880),
    "mismatch": (["d1_mismatch_all.csv"], 12960),
}
EXPECTED_GRAND_TOTAL = 43200


def verify_run_counts():
    print("\n" + "=" * 78)
    print("PART 3 -- run counts / grand total")
    print("=" * 78)
    grand = 0
    for name, (files, expected) in RUN_COUNT_GROUPS.items():
        n = sum(len(pd.read_csv(DATA / f)) for f in files)
        grand += n
        print(f"  {name:38s} {n:>7,}")
        fixed(f"run_count[{name}]", n, expected)
    print(f"  {'GRAND TOTAL':38s} {grand:>7,}")
    fixed("run_count[grand_total]", grand, EXPECTED_GRAND_TOTAL)
    for f in ("all_results_main.csv", "all_results_datasheet.csv"):
        fixed(f"ik_ok[{f}]", float(pd.read_csv(DATA / f)["ik_ok"].astype(bool).mean()), 1.0)


# 4. Saturation and limit violations -- every result file, by uncertainty level and controller
STABILITY_FILES = {
    "main grid, proxy": ("all_results_main.csv", {"fidelity": "proxy"}),
    "main grid, rigid-body": ("all_results_main.csv", {"fidelity": "rigid_body"}),
    "datasheet grid": ("all_results_datasheet.csv", {}),
    "cap ablation, proxy": ("all_results_cap0p9_ctcsmc_check.csv", {"fidelity": "proxy"}),
    "cap ablation, rigid-body": ("all_results_cap0p9_ctcsmc_check.csv", {"fidelity": "rigid_body"}),
    "frozen-gain ablation": ("all_results_frozen_gains_rigid.csv", {}),
    "step-size dt=0.5, proxy": ("all_results_dt0p5.csv", {"fidelity": "proxy"}),
    "step-size dt=0.5, rigid-body": ("all_results_dt0p5.csv", {"fidelity": "rigid_body"}),
    "step-size dt=0.25, proxy": ("all_results_dt0p25.csv", {"fidelity": "proxy"}),
    "step-size dt=0.25, rigid-body": ("all_results_dt0p25.csv", {"fidelity": "rigid_body"}),
    "mismatch sweep": ("d1_mismatch_all.csv", {}),
    "KR6 coupled": ("d3_coupled_all_20seed.csv", {"robot": "kr6"}),
    "UR5e coupled": ("d3_coupled_all_20seed.csv", {"robot": "ur5e"}),
    "raised-clamp, coupled": ("all_results_nocap_ur5e_eps0p09_coupled.csv", {}),
    "raised-clamp, decoupled": ("all_results_nocap_ur5e_eps0p09_decoupled.csv", {}),
}


def verify_stability():
    print("\n" + "=" * 78)
    print("PART 4 -- torque saturation / velocity / acceleration violations")
    print("          (mean fraction of timesteps; cells with any non-zero saturation are listed)")
    print("=" * 78)
    for label, (f, flt) in STABILITY_FILES.items():
        df = pd.read_csv(DATA / f)
        for k, v in flt.items():
            df = df[df[k] == v]
        cols = [c for c in ("Sat_frac_any", "Vel_violation_frac", "Acc_violation_frac") if c in df.columns]
        m = df.groupby(["params", "controller"])[cols].mean()
        record(f"stability[{label}]", {f"{p}/{c}/{col}": m.loc[(p, c), col]
                                       for (p, c) in m.index for col in cols})
        sat = m["Sat_frac_any"]
        bad = sat[sat > 0]
        print(f"  {label:30s} max mean Sat_frac_any = {float(sat.max()):.4f}"
              + ("" if bad.empty else "   <- " + ", ".join(f"{p}/{c} {v:.3f}" for (p, c), v in bad.items())))


# 5. Peak applied torque, full 20-seed URDF grid
def verify_peak_torque():
    print("\n" + "=" * 78)
    print("PART 5 -- peak |tau| / tau_max, URDF grid (all 20 seeds)")
    print("=" * 78)
    df = _load("all_results_main.csv")
    peak = df.groupby(["fidelity", "params", "controller"])["Tau_peak_ratio"].max()
    print(peak.unstack("controller").round(4).to_string())
    record("peak_torque", {f"{f}/{p}/{c}": v for (f, p, c), v in peak.items()})


# 6. Workspace feasibility (unchanged, controller-independent)
def verify_workspace_feasibility():
    print("\n" + "=" * 78)
    print("PART 6 -- workspace feasibility range (full protocol, 12 cells)")
    print("=" * 78)
    df = pd.read_csv(DATA / "workspace_feasibility.csv")
    fixed("workspace_feasibility[n_cells]", len(df), 12)
    min_row = df.loc[df.pct_feasible.idxmin()]
    print(f"  min: {min_row.robot}/{min_row.scene} = {min_row.pct_feasible:.3f}%")
    print(f"  max: {float(df.pct_feasible.max()):.3f}%")
    fixed("workspace_feasibility[min_is_mpx1950_open_reach]",
          float((min_row.robot, min_row.scene) == ("mpx1950", "open_reach")), 1.0)
    fixed("workspace_feasibility[max_pct]", float(df.pct_feasible.max()), 100.0, tol=0.05)
    fixed("workspace_feasibility[uniform_seeds]", float(df["seeds"].nunique()), 1.0)
    fixed("workspace_feasibility[uniform_ik_restarts]", float(df["ik_restarts"].nunique()), 1.0)


def compare_with_expected():
    expected = json.loads(EXPECTED_JSON.read_text(encoding="utf-8"))
    for k in sorted(set(expected) | set(VALUES)):
        if k not in VALUES:
            FAILURES.append(f"{k}: in expected_values.json but no longer computed")
        elif k not in expected:
            FAILURES.append(f"{k}: computed but missing from expected_values.json")
        else:
            got, exp = VALUES[k], expected[k]
            if isinstance(exp, str) or isinstance(got, str):
                if got != exp:
                    FAILURES.append(f"{k}: got {got!r}, expected {exp!r}")
            elif not (np.isnan(got) and np.isnan(exp)) and abs(got - exp) > TOL * max(1.0, abs(exp)):
                FAILURES.append(f"{k}: got {got!r}, expected {exp!r} (|diff|={abs(got - exp):.3g})")


def main():
    write = "--write-expected" in sys.argv
    verify_tables()
    verify_within_tier()
    verify_run_counts()
    verify_stability()
    verify_peak_torque()
    verify_workspace_feasibility()

    if write:
        EXPECTED_JSON.write_text(json.dumps(VALUES, indent=1, sort_keys=True) + "\n", encoding="utf-8")
        print(f"\nWrote {len(VALUES)} reference values -> {EXPECTED_JSON}")
    else:
        compare_with_expected()

    n = FIXED_CHECKS + (0 if write else len(VALUES))
    print("\n" + "=" * 78)
    print(f"SUMMARY: {n} checks, {len(FAILURES)} failures")
    print("=" * 78)
    if FAILURES:
        print("\nMISMATCHES:")
        for f in FAILURES:
            print(f"  - {f}")
        return 1
    print("\nAll checks passed.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
