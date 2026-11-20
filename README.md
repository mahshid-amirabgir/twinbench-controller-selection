# TwinBench

Multi-fidelity benchmarking of trajectory controllers for 6-DoF industrial manipulators.

Code and results accompanying the paper
*TwinBench: Multi-Fidelity Benchmarking for Controller Selection in 6-DOF Industrial Manipulators*.

| Resource | Link |
|----------|------|
| **GitHub** | https://github.com/mahshid-amirabgir/twinbench-controller-selection |
| **Archive (Zenodo)** | https://doi.org/10.5281/zenodo.24407565 |

## What this repository contains

| Path | Role |
|------|------|
| `src/twinbench/` | Dynamics (proxy + Pinocchio), trajectories, metrics |
| `experiments/` | Simulation drivers; `run_everything.py` regenerates every result file |
| `analysis/` | Table generation, in-text numbers, numerical checks, figure scripts |
| `data/` | Committed experimental results (source of all reported numbers); columns in `data/README.md` |
| `figures/` | Published figures |
| `tables/` | LaTeX table fragments and per-context Kendall τ values |
| `robot_specs/`, `urdf_models/`, `scenes/` | Robot and task definitions (third-party URDFs: see `urdf_models/README.md`) |
| `docs/` | Controller laws and reproducibility notes |
| `tests/` | Unit tests |

## Setup

```bash
conda env create -f environment.yml
conda activate twinbench
```

pip alternative (Linux/macOS, Python 3.11): `pip install -r requirements.txt`.
The pins in that file matter; see the comments there. Proxy-only runs on the
datasheet robots (`kr6`, `rs006l`, `mpx1950`) do not need Pinocchio.

## Check the reported numbers (no simulation, under a minute)

```bash
python analysis/verify_paper_numbers.py   # recompute every table and scalar, compare with the reference set
python analysis/text_numbers.py --gains   # numbers quoted in the running text (saved copy: tables/text_numbers.txt)
pytest tests -q                           # unit tests
python analysis/figures/make_all.py       # optional: regenerate figures from data/
```

`verify_paper_numbers.py` exits 0 only if every check passes.

## Re-run the simulations

```bash
# the 43,200 runs of the paper (rewrites the files under data/)
python experiments/run_everything.py --workers 4

# supplementary diagnostics (11,520 runs)
python experiments/run_everything.py --steps supplementary --workers 4

# selected steps, or a one-seed smoke test, into a scratch directory
python experiments/run_everything.py --steps urdf,cap --outdir /tmp/twinbench_steps
python experiments/run_everything.py --n-seeds 1 --outdir /tmp/twinbench_smoke

# a single cell through the grid driver (writes data/all_results_spotcheck.csv)
python experiments/run_main_grid.py --robots kr6 --scenes open_reach --fidelities proxy --trajectories cubic --params nominal --seeds 0 --output-tag spotcheck
```

See `docs/REPRODUCIBILITY.md` for the step list, runtimes and how to compare a
re-run with the committed files.

## Controllers

PD+g (`pdg`), CTC-PD (`ctc_pd`), CTC-SMC (`ctc_smc`), PBC (`pbc`).
Laws and conventions: `docs/CONTROLLERS.md`.

## Citation

If you use TwinBench, please cite the paper and this software archive. See
`CITATION.cff` for machine-readable citation metadata.

## License

MIT (see `LICENSE`), except the robot descriptions in `urdf_models/`, which keep
their upstream BSD and Apache-2.0 licenses (see `urdf_models/README.md`).
