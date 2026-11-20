# Result files

Every number in the paper is computed from these CSVs by the scripts in
`analysis/`. `experiments/run_everything.py` regenerates them; see
`docs/REPRODUCIBILITY.md` for which step writes which file.

| file | content | runs |
|------|---------|------|
| `all_results_main.csv` | URDF robots, proxy and rigid-body tiers, open reach | 5,760 |
| `all_results_datasheet.csv` | datasheet robots, proxy tier, both scenes | 5,760 |
| `all_results_d3_kr6_decoupled_20seed.csv` | KR6 open-reach subset of the datasheet grid (coupled-proxy baseline) | 960 |
| `d3_coupled_all_20seed.csv` | coupled proxy, KR6 and UR5e, five coupling strengths | 9,600 |
| `all_results_nocap_ur5e_eps0p09_{coupled,decoupled}.csv` | UR5e with the torque clamp raised to 10^4 τ_max | 1,920 |
| `all_results_cap0p9_ctcsmc_check.csv` | CTC-SMC with the cap at 0.9 τ_max | 1,440 |
| `all_results_frozen_gains_rigid.csv` | rigid tier with CTC-SMC/PBC gains frozen at proxy-tier values | 2,880 |
| `all_results_dt0p5.csv`, `all_results_dt0p25.csv` | step size 0.5 ms and 0.25 ms (seeds 0–4) | 2,880 |
| `d1_mismatch_all.csv` | rigid tier with perturbed link masses (±10/20/30 %, six realizations) | 12,960 |
| `all_results_symmetric_uncertainty_rigid.csv` | supplementary: rigid plant scaled in I and G as well as B | 2,880 |
| `diag_proxy_factor_restore.csv` | supplementary: proxy with one or more plant factors restored to nominal | 1,920 |
| `diag_ur5e_coupled_eps0.csv` | supplementary: UR5e coupled plant with ε_c = 0 | 960 |
| `diag_nominal_inertia_gains.csv` | supplementary: gains set from the nominal inertia | 5,760 |
| `workspace_feasibility.csv` | IK feasibility of each (robot, scene) target region | – |

## Columns

One row per simulation run. Files from the diagnostic steps carry a subset of
these columns.

| column | meaning |
|--------|---------|
| `robot`, `scene`, `trajectory`, `controller`, `params`, `seed`, `fidelity` | run identifiers; `params` is the uncertainty level (`light`, `nominal`, `heavy`), `fidelity` is `proxy` or `rigid_body` |
| `eps_c` | coupling strength of the coupled proxy |
| `pct`, `realization` | link-mass perturbation magnitude (fraction) and realization index |
| `restored` | plant factors restored to nominal (`I`, `B`, `G` or `BG`) |
| `robot_group` | `proxy` (datasheet robot) or `urdf` |
| `version` | software label, constant `twinbench` |
| `R_max_m` | maximum reach [m], used for normalisation |
| `payload_used_kg` | payload in the simulation (0.3 × rated payload) |
| `sim_dt` | integration step [s] |
| `target_x_m`, `target_y_m`, `target_z_m` | sampled end-effector target [m] |
| `parametric_mismatch_frac` | link-mass perturbation of the plant (0 in the main grid) |
| `ik_ok` | target reached by IK; runs with `False` are excluded from all statistics |
| `rejected_reason` | why a target was rejected (empty if `ik_ok`) |
| `T_eff` | duration of the reference trajectory [s] |
| `traj_s_star`, `traj_T_eff` | FSMJ time-scaling factor s* and scaled duration [s]; empty for cubic and min-jerk |
| `RMSE_cart_m`, `Max_cart_err_m` | RMS and maximum Cartesian tracking error [m] |
| `RMSE_cart_norm`, `Max_cart_err_norm` | the same divided by `R_max_m`; `RMSE_cart_norm` is the ranking metric |
| `Energy` | ∫ Σ\|τ·q̇\| dt [J] |
| `E_star` | `Energy` / (`payload_used_kg` · g · `R_max_m`) |
| `Tau_rms`, `Tau_rms_norm` | RMS applied torque [N·m], and divided by the largest τ_max |
| `Tau_peak_ratio` | peak \|applied torque\| / τ_max, over joints and time |
| `Jerk_rms`, `Jerk_rms_norm` | RMS joint jerk, and divided by the jerk limit |
| `Sat_frac_any` | fraction of timesteps with any joint's commanded torque above its cap |
| `Vel_violation_frac`, `Acc_violation_frac`, `Jerk_violation_frac` | fraction of timesteps with any joint above its velocity, acceleration or jerk limit |
| `cart_err_ok` | Cartesian error could be evaluated |

## `workspace_feasibility.csv`

| column | meaning |
|--------|---------|
| `robot`, `scene` | cell |
| `n_points`, `n_feasible`, `pct_feasible` | sampled targets, targets reached by IK, and their percentage |
| `seeds`, `ik_restarts` | number of sampling seeds, and IK attempts per target (home start plus random restarts) |
| `source` | how the cell was produced |
