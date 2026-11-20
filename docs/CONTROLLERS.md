# Where the control laws live

TwinBench evaluates four controllers. Each is implemented twice, once for the proxy
(decoupled second-order) plant and once for the rigid-body (Pinocchio ABA) plant, because
the two plants expose different model information to the controller. Each is a single
`if/elif` function called once per integration step.

| controller | id | proxy tier | rigid-body tier |
|---|---|---|---|
| PD + gravity | `pdg` | `experiments/run_main_grid.py` → `control_torque()` | `src/twinbench/dynamics/urdf_plant.py` → `control_torque_pinocchio()` |
| Computed torque + PD | `ctc_pd` | same | same |
| Computed torque + sliding mode | `ctc_smc` | same | same |
| Passivity-based | `pbc` | same | same |

Each tier's laws are kept in one function so that a single call per integration step
computes the torque; the laws are documented below and each branch is commented in place.

## The laws (identical structure at both tiers)

With `e = q_d − q`, `s = ė + Λe`, `q̇_r = q̇_d + Λe`, `q̈_r = q̈_d + Λė`:

| controller | proxy tier | rigid-body tier |
|---|---|---|
| PD+g | `I_nom(K_p e + K_d ė) + G_nom sin q` | `M(q)(K_p e + K_d ė) + g(q)` |
| CTC-PD | `I_nom(q̈_d + K_p e + K_d ė) + B_nom q̇ + G_nom sin q` | `M(q)(q̈_d + K_p e + K_d ė) + C(q,q̇)q̇ + g(q) + B_nom q̇` |
| CTC-SMC | `I_nom(q̈_d + Λė) + B_nom q̇ + G_nom sin q + K_s tanh(s/Φ)` | `M(q)(q̈_d + Λė) + C(q,q̇)q̇ + g(q) + B_nom q̇ + K_s tanh(s/Φ)` |
| PBC | `I_nom q̈_r + B_nom q̇_r + G_nom sin q + K_D s` | `M(q) q̈_r + C(q,q̇) q̇_r + g(q) + B_nom q̇_r + K_D s` |

The `B_nom` damping feedforward is the **nominal** damping at both tiers, never the
uncertainty-scaled plant value (`include_B_feedforward=True`, the default).

**Effect at the light uncertainty level.** At the *light* uncertainty level the plant damping is
`0.7 · B_nom`, so the `B_nom q̇` feedforward of CTC-PD and CTC-SMC over-compensates by
`0.3 · B_nom q̇`, a positive velocity feedback. On the URDF robots' wrist joints, whose
inertia (and therefore derivative gain `I · K_d`) is tiny compared with `B_nom`, this
destabilizes the joint at **both** tiers: see `tables/tab_stability.tex`. PBC is not
affected in the same way because its damping term multiplies the reference velocity.
On the proxy tier the light level also lowers the plant's gravity amplitude to
`0.9 · G_nom`, so the `G_nom sin q` feedforward over-compensates gravity as well; the
`factors` diagnostic (`experiments/run_everything.py --steps factors`) shows that either
over-compensation alone sustains the divergence there and that removing both restores
stable tracking.

**Proxy gravity amplitude.** `G_nom` is set by a fixed rule, `G_nom,i = 0.25 τ_max,i`
(`G_NOM_FRAC` in `src/twinbench/constants.py`), for every robot; it is not derived from the
URDF. Against the largest gravity torque the URDF model produces at each joint, it is
0.6–0.7× on joint 2 and 1.0–2.4× on joint 3, but 5–30× on joint 4 and 50–79× on joint 5,
and non-zero on joints 1 and 6 where the URDF gravity torque is zero or negligible
(`analysis/text_numbers.py`). A relative error in `G` is therefore a much larger
disturbance on the proxy's distal joints than the same relative error in `g(q)`; this is
what makes the two tiers rank the model-based controllers differently at the heavy level.

## Sign conventions

Error convention throughout is `e = q_d − q`, so the sliding variable
`s = ė + Λe` equals `−s_std` (the textbook `s` under `e = q − q_d`).

- **CTC-SMC switching term is `+Ks·tanh(s/φ)`.** Under this repo's error convention the
  plus sign is the stabilizing one; implemented in the
  `ctc_smc` branch of both functions.
- **PBC uses `+Λ` reference velocity** (`q̇_r = q̇_d + Λe`), matching CTC-SMC's `s`.
- **PBC damping gain** is `K_D = I_nom · K_d` on the proxy tier and
  `K_D = diag(M(q)) · K_d` (live inertia) on the rigid-body tier.
- **PBC's rigid Coriolis term** is the explicit `pin.computeCoriolisMatrix(q, q̇) @ q̇_r` —
  *not* `rnea(q, q̇_r, 0)`, which is the wrong form for a passivity-based law.
- **Torque caps.** `pdg`, `ctc_pd` and `pbc` clamp at `TAU_CAP_FRAC = 0.90 · τ_max`;
  only `ctc_smc` reads `TAU_CAP_FRAC_SMC = 0.70 · τ_max`. The caps never bind at the
  nominal and heavy uncertainty levels; at the light level CTC-PD and CTC-SMC saturate
  on the URDF robots (see *Effect at the light uncertainty level* above). Per-run peak applied torque is recorded as
  `Tau_peak_ratio`; `analysis/text_numbers.py` reports the cap ablation.
- **Clamps differ between plants.** The decoupled proxy plant clamps joint velocity at
  `3 · q̇_cap` and acceleration at 500 rad/s²; the rigid-body plant clamps acceleration
  at 500 rad/s² and has no velocity clamp; the coupled-proxy plant has neither. A joint
  that runs away therefore shows up as velocity-limit violation on the decoupled proxy
  and as torque saturation on the other two plants.

## Gain normalization

No controller is hand-tuned per robot. All gains follow one protocol, implemented in
`run_main_grid.run_single` / `compute_gains` with constants from `src/twinbench/constants.py`:

```
ddq_cap = clip(TAU_CAP_FRAC · τ_max / I, 10, 1000)
ω_n     = WN_FRAC · sqrt(ddq_cap)
K_p     = ω_n²          K_d = 2 · ZETA · ω_n
Λ       = 2 · ZETA · (2π / T_BASE)
K_s     = SMC_KS_FRAC · I · max(K_p, 1)      Φ = SMC_PHI_FRAC · q̇_cap
```

`I` here is the inertia of the run's uncertainty level (`I_eff = s_I · I_nom`, the
default `gain_inertia_source="I_eff"`), so the light and heavy levels also change
`K_p`, `K_d`, `K_s` and `K_D`, at both tiers; the rigid-body plant itself scales only
its damping. `gain_inertia_source="I_nom"` sets the gains from the nominal inertia
instead; the `nominal_gains` diagnostic (`experiments/run_everything.py`) shows that this
leaves every cross-fidelity statistic on the nominal and heavy levels unchanged.

This shared protocol does not give the four controllers equal *effective* bandwidth.
Per unit inertia, the position-loop stiffness of CTC-PD and PD+g is `K_p`, that of PBC is
`K_d·Λ`, and the boundary-layer equivalent of CTC-SMC is `K_s·Λ/(I·Φ)`. PBC's is 1.3–8.6×
CTC-PD's depending on the joint (per-robot mean over joints 2.3–7.2×; 7.2× on KR6), and
CTC-SMC's is 0.05–0.56× (`python analysis/text_numbers.py --gains`). That
asymmetry is a stated limitation, not a tuning artifact — it follows from applying one
normalization protocol to laws with different structures.
