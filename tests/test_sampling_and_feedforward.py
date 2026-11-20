"""Tests for FSMJ sampling, the rigid-body damping feedforward, and the peak-torque metric."""
from __future__ import annotations

import inspect

import numpy as np
import pytest

from tests.conftest import SKIP_PINOCCHIO
import experiments.run_main_grid as R
from twinbench.dynamics.robot_loader import (
    get_dynamics_cfg_for_params, load_robot_spec, robot_spec_path_from_id,
)


def _dyn(robot):
    spec = load_robot_spec(robot_spec_path_from_id(robot, repo_root=R.ROOT))
    return get_dynamics_cfg_for_params(spec, "nominal")


def _slow_move():
    # A move long enough that the feasibility search stretches it (s_star > 1).
    return np.zeros(6), np.array([2.5, -2.0, 2.0, 2.0, 2.0, 2.0])


def test_fsmj_keeps_the_nominal_step_when_stretched():
    q0, qf = _slow_move()
    t, *_ , T_eff, info = R.gen_trajectory_for_run("fsmj", q0, qf, R.T_BASE, R.DT, _dyn("kr6"))
    assert info["s_star"] > 1.05, "test move should be time-scaled"
    step = float(t[1] - t[0])
    assert abs(step - R.DT) / R.DT < 2e-3
    assert len(t) == int(round(T_eff / R.DT))


def test_cubic_and_minjerk_sampling():
    q0, qf = _slow_move()
    for traj in ("cubic", "minjerk"):
        t, *_ = R.gen_trajectory_for_run(traj, q0, qf, R.T_BASE, R.DT, _dyn("kr6"))
        assert len(t) == int(round(R.T_BASE / R.DT))


def test_rigid_tier_damping_feedforward_is_on_by_default():
    assert inspect.signature(R.run_single).parameters["include_B_feedforward"].default is True
    assert inspect.signature(R.run_all).parameters["include_B_feedforward"].default is True


@pytest.mark.skipif(SKIP_PINOCCHIO, reason="Pinocchio not installed")
def test_rigid_controllers_default_to_table2_laws():
    import twinbench.dynamics.urdf_plant as U
    import experiments.run_frozen_gain_ablation as F
    for fn in (U.control_torque_pinocchio, U.simulate_urdf_full, F.control_torque_pinocchio_frozen):
        assert inspect.signature(fn).parameters["include_B_feedforward"].default is True


@pytest.mark.skipif(SKIP_PINOCCHIO, reason="Pinocchio not installed")
def test_frozen_gain_controller_runs_and_differs_only_in_gain_scaling():
    """The frozen-gain law must accept the same call the simulator makes, and
    must equal the live law for controllers that carry neither K_s nor K_D."""
    import twinbench.dynamics.urdf_plant as U
    import experiments.run_frozen_gain_ablation as F
    live = R.run_single("ur5e", "open_reach", "cubic", "ctc_pd", "nominal", 0, fidelity="rigid_body")
    old = U.control_torque_pinocchio
    U.control_torque_pinocchio = F.control_torque_pinocchio_frozen
    try:
        frozen = R.run_single("ur5e", "open_reach", "cubic", "ctc_pd", "nominal", 0, fidelity="rigid_body")
        frozen_pbc = R.run_single("ur5e", "open_reach", "cubic", "pbc", "nominal", 0, fidelity="rigid_body")
    finally:
        U.control_torque_pinocchio = old
    assert frozen["RMSE_cart_norm"] == live["RMSE_cart_norm"]
    assert np.isfinite(frozen_pbc["RMSE_cart_norm"])


def test_peak_torque_ratio_is_recorded_and_bounded_by_the_cap():
    out = R.run_single("kr6", "open_reach", "cubic", "ctc_smc", "nominal", 0, fidelity="proxy")
    assert 0.0 < out["Tau_peak_ratio"] <= R.TAU_CAP_FRAC_SMC + 1e-12
