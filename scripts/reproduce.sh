#!/usr/bin/env bash
# TwinBench reproduction helpers. Run from the repository root.
#   ./scripts/reproduce.sh tables|figures|tests|check     (minutes, no simulation)
#   ./scripts/reproduce.sh simulate [workers]             (re-runs all 43,200 simulations)
#   ./scripts/reproduce.sh all [workers]                  (simulate, then check and figures)
set -euo pipefail
PY="${PYTHON:-python}"
STEP="${1:-check}"
WORKERS="${2:-1}"
export PYTHONUTF8=1

case "$STEP" in
  tables)
    "$PY" analysis/generate_tables.py --write
    ;;
  check)
    "$PY" analysis/verify_paper_numbers.py
    ;;
  figures)
    "$PY" analysis/figures/make_all.py
    ;;
  tests)
    "$PY" -m pytest tests -q
    ;;
  simulate)
    "$PY" experiments/run_everything.py --workers "$WORKERS"
    ;;
  all)
    "$PY" -m pytest tests -q
    "$PY" experiments/run_everything.py --workers "$WORKERS"
    "$PY" analysis/verify_paper_numbers.py
    "$PY" analysis/figures/make_all.py
    ;;
  *)
    echo "Usage: $0 {tables|check|figures|tests|simulate|all} [workers]" >&2
    exit 2
    ;;
esac
