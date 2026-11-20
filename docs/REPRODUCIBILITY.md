# Reproducibility

How to check, and how to regenerate, every number, table and figure in the
manuscript from this repository.

---

## 1. Environment

```bash
conda env create -f environment.yml
conda activate twinbench
```

`environment.yml`:

| package | version |
|---|---|
| python | 3.11 |
| pinocchio | 3.9.0 |
| numpy | 2.4.3 |
| pandas | 3.0.1 |
| scipy | 1.17.1 |
| matplotlib | 3.10.8 |
| pyyaml | 6.0.3 |
| pytest | 9.0.3 |

Pinocchio is pinned exactly because the rigid-body tier's numbers depend on
its ABA/CRBA implementation.

`pip install -r requirements.txt` is a tested alternative on Linux with
Python 3.11 (it resolves to pin 3.9.0 with numpy 2.3.5; the pip wheel of
Pinocchio 3.9.0 does not accept numpy 2.4). The committed results were
produced in that pip environment. Read the comments in `requirements.txt`
before changing any pin. Without Pinocchio, only proxy-tier runs on the
datasheet robots work.

---

## 2. Seeding and what "reproducible" means here

Two SHA-256-derived seeds drive every stochastic element of a run
(`experiments/run_main_grid.py`, `stable_int_seed`):

| seed | inputs | used for |
|---|---|---|
| target seed | (`robot`, `scene`, `seed`) | sampling the goal configuration |
| noise seed | (`robot`, `scene`, `trajectory`, `params`, `seed`) | per-step measurement noise on q and q̇ |

Neither depends on the controller, so all controllers on a given
(robot, scene, trajectory, params, seed) tuple see identical targets and
identical noise. Neither depends on execution order, so results are the same
for any `--workers` setting.

A re-run in the same environment reproduces the committed CSVs exactly. Across
environments (different numpy/BLAS builds) individual metrics typically agree to
about 1e-10 relative on convergent runs. Runs that diverge (Section 5) are
chaotic and can differ more across environments; their summary statistics
(saturated fraction, rank) are stable.

Bootstrap confidence intervals in `analysis/generate_tables.py` use one pinned
seed (`BOOTSTRAP_SEED`), so regenerating the tables from the same CSVs gives
byte-identical `.tex` fragments.

---

## 3. Check everything from the committed data (no simulation)

```bash
python analysis/verify_paper_numbers.py     # under a minute
python analysis/text_numbers.py --gains > tables/text_numbers.txt
python analysis/figures/make_all.py
pytest tests -q
```

`verify_paper_numbers.py` regenerates every table fragment under `tables/`,
recomputes the within-tier τ_b, the saturation / velocity / acceleration
violation fractions of every result file and the peak-torque ratios, checks the
run counts and the 43,200 total, and compares every value with
`analysis/expected_values.json`. It exits non-zero on any difference.

| output | source |
|---|---|
| `tables/tab_robots.tex` | robot specs |
| `tables/tab_proxy.tex` | `all_results_datasheet.csv` |
| `tables/tab_fidelity.tex`, `tab_fidelity_stable.tex`, `tab_fidelity_by_uncertainty.tex` | `all_results_main.csv` |
| `tables/tab_stability.tex` | `all_results_main.csv`, `all_results_datasheet.csv` |
| `tables/tab_coupled.tex` | `d3_coupled_all_20seed.csv`, `all_results_d3_kr6_decoupled_20seed.csv` |
| `tables/tab_cap.tex` | main grid + `all_results_cap0p9_ctcsmc_check.csv` |
| `tables/tab_frozen.tex` | main grid + `all_results_frozen_gains_rigid.csv` |
| `tables/tab_dt.tex` | main grid (seeds 0–4) + `all_results_dt0p5.csv`, `all_results_dt0p25.csv` |
| `tables/tab_mismatch.tex`, `tab_mismatch_stable.tex`, `tab_mismatch_light.tex` | `d1_mismatch_all.csv` |
| `tables/tab_energy.tex`, `tab_energy_stable.tex` | `all_results_main.csv` |
| `tables/cross_fidelity_tau_per_context.csv` | per-context cross-fidelity τ_b of the 36 main-grid contexts |
| numbers quoted in the text | `python analysis/text_numbers.py` |

Where each table of the paper comes from (the fragments are not typeset
verbatim; the paper's tables are laid out by hand from them):

| paper table | generated from |
|---|---|
| robot specifications (`tab:robots`) | `tab_robots.tex` |
| proxy-tier statistics (`tab:proxy`) | `tab_proxy.tex` |
| divergent light-level cells (`tab:stability`) | light-level rows of `tab_stability.tex` |
| factor restoration (`tab:factors`) | `text_numbers.py`, factor-restoration section |
| cross-fidelity agreement (`tab:fidelity`) | `tab_fidelity_stable.tex` |
| rankings by uncertainty level (`tab:bylevel`) | `text_numbers.py`, by-level section; τ_b from `tab_fidelity_by_uncertainty.tex` |
| coupled proxy (`tab:coupled`) | `tab_coupled.tex` |
| robustness checks (`tab:robust`) | `tab_cap_stable.tex`, `tab_frozen_stable.tex`, `tab_dt_stable.tex`, summarised in `text_numbers.py` |
| link-mass mismatch (`tab:mismatch`) | `tab_mismatch_stable.tex` (light level: `tab_mismatch_light.tex`) |
| energy (`tab:energy`) | `tab_energy_stable.tex` |

The `*_stable` tables are the same computations restricted to the nominal and
heavy uncertainty levels (Section 5).

Figures: `python analysis/figures/make_all.py` rebuilds every figure under
`figures/` from `data/`; each `make_*.py` can also be run on its own.
`data/workspace_feasibility.csv` (supplementary Fig. S1) depends only on robot
kinematics and scene geometry. Its original generator
is not part of this repository; `python analysis/check_workspace_feasibility.py`
re-measures every cell on a smaller sample with the protocol described in the
paper and compares it with the committed value (all 12 cells agree within the
95% sampling interval at 500 targets per cell). At the default of 50 targets per
seed it takes roughly 10–20 minutes.

The forward/inverse-kinematics round-trip error quoted in the paper comes from
`python analysis/check_fk_ik.py` (60 random configurations per robot, about 10 s).

---

## 4. Re-run the simulations

```bash
python experiments/run_everything.py --workers 4
```

writes the 43,200 runs under `data/`. `--workers` uses separate processes and works on
Linux, macOS and Windows; the results do not depend on it. Steps can be selected with `--steps`;
`--steps supplementary` runs the four supplementary diagnostics.
Runtimes below were measured on 2 CPU cores (`--workers 2`).

| step | runs | writes | runtime |
|---|---|---|---|
| `urdf` | 5,760 | `all_results_main.csv` | 6 min |
| `datasheet` | 5,760 | `all_results_datasheet.csv`, `all_results_d3_kr6_decoupled_20seed.csv` | 19 min |
| `coupled` | 9,600 | `d3_coupled_all_20seed.csv` | 42 min |
| `raised_clamp` | 1,920 | `all_results_nocap_ur5e_eps0p09_coupled.csv`, `..._decoupled.csv` | 3 min |
| `cap` | 1,440 | `all_results_cap0p9_ctcsmc_check.csv` | 2 min |
| `frozen` | 2,880 | `all_results_frozen_gains_rigid.csv` | 3 min |
| `dt` | 2,880 | `all_results_dt0p5.csv`, `all_results_dt0p25.csv` | 7 min |
| `mismatch` | 12,960 | `d1_mismatch_all.csv` | 14 min |
| **total** | **43,200** | | |
| `symmetric` (supplementary) | 2,880 | `all_results_symmetric_uncertainty_rigid.csv` | 3 min |
| `factors` (supplementary) | 1,920 | `diag_proxy_factor_restore.csv` | 2 min |
| `eps0` (supplementary) | 960 | `diag_ur5e_coupled_eps0.csv` | 1 min |
| `nominal_gains` (supplementary) | 5,760 | `diag_nominal_inertia_gains.csv` | 6 min |

To compare a re-run with the committed files, write it elsewhere and diff:

```bash
python experiments/run_everything.py --workers 4 --outdir /tmp/rerun
python - <<'EOF'
import pandas as pd, pathlib
for f in sorted(pathlib.Path("/tmp/rerun").glob("*.csv")):
    a, b = pd.read_csv(f), pd.read_csv(pathlib.Path("data") / f.name)
    pd.testing.assert_frame_equal(a, b, rtol=1e-9)
    print("ok", f.name)
EOF
```

A one-seed smoke test of the whole pipeline takes a few minutes:
`python experiments/run_everything.py --n-seeds 1 --outdir /tmp/smoke`.
`scripts/reproduce.sh` (Linux/macOS) and `scripts/reproduce.ps1` (Windows) wrap these
commands: `check`, `tables`, `figures`, `tests`, `simulate`, `all`.

The older single-purpose drivers remain usable for individual cells:
`experiments/run_main_grid.py` (flags `--robots`, `--scenes`, `--fidelities`,
`--trajectories`, `--controllers`, `--params`, `--seeds`, `--dt`,
`--smc-tau-cap-frac`, `--parametric-mismatch`, `--mismatch-seed`,
`--output-tag`), `experiments/run_coupled_sweep.py` (KR6),
`experiments/run_coupled_sweep_ur5e.py`, `experiments/run_frozen_gain_ablation.py`
and `experiments/drivers/mismatch_driver.py`.

After a deliberate change to the simulation code, refresh the reference set
with `python analysis/verify_paper_numbers.py --write-expected` and update the
manuscript from the regenerated tables.

---

## 5. Divergent cells

At the light uncertainty level the plant's viscous damping is 0.7× nominal
while CTC-PD and CTC-SMC feed forward the nominal value. On the URDF robots'
wrist joints this over-compensation exceeds the derivative action and the
joints diverge, at every fidelity tier (`docs/CONTROLLERS.md`,
`tables/tab_stability.tex`). On the proxy plants the gravity amplitude is also
below nominal at this level, which over-compensates gravity too. How the
divergence shows up depends on the plant's clamps:

| plant | velocity clamp | symptom in the light cells (CTC-PD, CTC-SMC; URDF robots) |
|---|---|---|
| decoupled proxy | 3 × velocity limit | velocity-limit violation on ~98–99% of timesteps, no torque saturation |
| rigid body | none | torque saturation on ~80% of timesteps |
| coupled proxy | none | torque saturation on ~98.5% of timesteps |

RMSE and energy averaged over such runs are not tracking measurements. On the
rigid-body and coupled plants PBC and PD+g do not saturate. On the decoupled proxy,
gravity over-compensation at the light level also raises the velocity-limit violation
of PD+g (0.67 of timesteps) and PBC (0.23); see `tables/tab_stability.tex`. The
datasheet robots are not affected.
