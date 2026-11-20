"""Benchmark constants: physical parameters, uncertainty sets, and controller defaults."""

from __future__ import annotations

G0:           float = 9.80665
PAYLOAD_FRAC: float = 0.30
G_NOM_FRAC:   float = 0.25
T_NOM:        float = 2.0

PARAM_SCALES = {
    "light":   {"I": 0.8, "B": 0.7, "G": 0.9},
    "nominal": {"I": 1.0, "B": 1.0, "G": 1.0},
    "heavy":   {"I": 1.2, "B": 1.3, "G": 1.1},
}

PARAM_SCALES_FACTORIAL = {
    "nominal":  {"I": 1.0, "B": 1.0, "G": 1.0},
    "i_light":  {"I": 0.8, "B": 1.0, "G": 1.0},
    "i_heavy":  {"I": 1.2, "B": 1.0, "G": 1.0},
    "b_light":  {"I": 1.0, "B": 0.7, "G": 1.0},
    "b_heavy":  {"I": 1.0, "B": 1.3, "G": 1.0},
    "g_light":  {"I": 1.0, "B": 1.0, "G": 0.9},
    "g_heavy":  {"I": 1.0, "B": 1.0, "G": 1.1},
}

DEFAULT_NOISE = {"sigma_q": 0.002, "sigma_qdot": 0.01}

DEFAULT_DISTURB = {
    "joint_index0": 1,
    "tau_frac_of_max": 0.05,
    "onset": 1.0,
    "duration": 0.2,
}

QD_CAP_FRAC:       float = 0.40
TAU_CAP_FRAC:      float = 0.90
TAU_CAP_FRAC_SMC:  float = 0.70

SIM_DT:         float = 0.001
DDQ_MA_WIN:     int   = 9

# Datasheet robots (proxy tier only)
ROBOTS_PROXY = ("kr6", "rs006l", "mpx1950")

# URDF robots (proxy and rigid-body tiers; public, community-maintained URDFs)
ROBOTS_URDF = ("ur5e", "ur10e", "tx60")

# All robots
ROBOTS = ROBOTS_PROXY + ROBOTS_URDF

SCENES = ("open_reach", "shelf_reach")

CONTROLLERS = ("pdg", "ctc_pd", "ctc_smc", "pbc")

TRAJECTORIES = ("cubic", "minjerk", "fsmj", "fsmj_novia")

PARAMS = ("light", "nominal", "heavy")

SEEDS = tuple(range(20))

WN_FRAC:  float = 0.25
ZETA:     float = 0.9

SMC_PHI_FRAC:  float = 0.60
SMC_KS_FRAC:   float = 0.05


FSMJ_BSEARCH_EPSILON:   float = 1e-4
FSMJ_BSEARCH_N_SAMPLES: int   = 500
FSMJ_BSEARCH_S_MIN:     float = 1.0


URDF_PATHS = {
    "ur5e":  "urdf_models/ur5e.urdf",
    "ur10e": "urdf_models/ur10e.urdf",
    "tx60":  "urdf_models/staubli_tx60.urdf",
}
