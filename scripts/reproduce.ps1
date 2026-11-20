# TwinBench reproduction helpers (Windows PowerShell). Run from the repository root.
#   .\scripts\reproduce.ps1 tables|figures|tests|check
#   .\scripts\reproduce.ps1 simulate      (re-runs all 43,200 simulations, single process)
# The parallel --workers option of run_everything.py relies on fork and is
# Linux/macOS only; on Windows the simulations run in one process.
param([string]$Step = "check")
$ErrorActionPreference = "Stop"
$env:PYTHONUTF8 = "1"
$py = if ($env:PYTHON) { $env:PYTHON } else { "python" }

switch ($Step) {
  "tables"   { & $py analysis/generate_tables.py --write }
  "check"    { & $py analysis/verify_paper_numbers.py }
  "figures"  { & $py analysis/figures/make_all.py }
  "tests"    { & $py -m pytest tests -q }
  "simulate" { & $py experiments/run_everything.py }
  "all"      {
    & $py -m pytest tests -q
    & $py experiments/run_everything.py
    & $py analysis/verify_paper_numbers.py
    & $py analysis/figures/make_all.py
  }
  default    { Write-Error "Usage: reproduce.ps1 {tables|check|figures|tests|simulate|all}"; exit 2 }
}
