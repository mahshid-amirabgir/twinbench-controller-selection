"""Rigid-tier and proxy-tier control laws agree on a plant with
M = diag(I_nom), C = 0 and g = G_nom*sin(q).
"""
from __future__ import annotations

import numpy as np
import pytest

from tests.conftest import SKIP_PINOCCHIO

if SKIP_PINOCCHIO:
    pytest.skip("Pinocchio not installed", allow_module_level=True)

import experiments.run_main_grid as R
from twinbench.dynamics.urdf_plant import control_torque_pinocchio

DOF = 6
RNG = np.random.default_rng(20260923)


class _DiagonalMockPlant:
    """A fake 'rigid' plant whose M(q)/C(q,qd)/g(q) exactly reproduce the
    proxy model's diagonal I_nom, zero Coriolis, and G_nom*sin(q)."""

    def __init__(self, I_nom, G_nom):
        self.I_nom = np.asarray(I_nom, float)
        self.G_nom = np.asarray(G_nom, float)

    def mass_matrix(self, q):
        return np.diag(self.I_nom)

    def coriolis_matrix(self, q, dq):
        return np.zeros((DOF, DOF))

    def nonlinear_effects(self, q, dq):
        # RNEA at zero commanded accel = C(q,dq)@dq + g(q); with C=0 this is g(q)
        return self.G_nom * np.sin(q)

    def gravity(self, q):
        return self.G_nom * np.sin(q)


@pytest.mark.parametrize("ctrl", ["pdg", "ctc_pd", "ctc_smc", "pbc"])
def test_tier_laws_match_when_dynamics_match(ctrl):
    I_nom = RNG.uniform(0.1, 5.0, DOF)
    B_nom = RNG.uniform(0.01, 0.2, DOF)
    G_nom = RNG.uniform(1.0, 10.0, DOF)
    tau_max = RNG.uniform(10, 300, DOF)
    ddq_cap = RNG.uniform(50, 500, DOF)
    dq_cap = RNG.uniform(1, 5, DOF)

    q = RNG.uniform(-1, 1, DOF)
    dq = RNG.uniform(-1, 1, DOF)
    qd_k = RNG.uniform(-1, 1, DOF)
    dqd_k = RNG.uniform(-1, 1, DOF)
    ddqd_k = RNG.uniform(-1, 1, DOF)

    tau_proxy, cap_proxy = R.control_torque(
        q, dq, qd_k, dqd_k, ddqd_k,
        I_nom=I_nom, B_nom=B_nom, G_nom=G_nom, tau_max=tau_max,
        ddq_cap=ddq_cap, dq_cap=dq_cap, ctrl=ctrl,
        tau_cap_frac=R.TAU_CAP_FRAC, tau_cap_frac_smc=R.TAU_CAP_FRAC_SMC,
    )

    mock_plant = _DiagonalMockPlant(I_nom, G_nom)
    tau_rigid, cap_rigid = control_torque_pinocchio(
        q, dq, qd_k, dqd_k, ddqd_k,
        urdf_robot=mock_plant, tau_max=tau_max, ddq_cap=ddq_cap, dq_cap=dq_cap,
        B_nom=B_nom, ctrl=ctrl,
        WN_FRAC=R.WN_FRAC, ZETA=R.ZETA, T_BASE=R.T_BASE, DT=R.DT,
        TAU_CAP_FRAC=R.TAU_CAP_FRAC, TAU_CAP_FRAC_SMC=R.TAU_CAP_FRAC_SMC,
        SMC_PHI_FRAC=R.SMC_PHI_FRAC, SMC_KS_FRAC=R.SMC_KS_FRAC,
        I_nom=I_nom, G_nom=G_nom,
        include_B_feedforward=True,
    )

    np.testing.assert_allclose(tau_rigid, tau_proxy, rtol=1e-10, atol=1e-12)
    np.testing.assert_allclose(cap_rigid, cap_proxy, rtol=1e-10, atol=1e-12)


@pytest.mark.parametrize("ctrl", ["ctc_pd", "ctc_smc", "pbc"])
def test_without_flag_laws_differ_by_the_B_term(ctrl):
    """With include_B_feedforward=False the two laws differ by exactly the
    B term."""
    I_nom = RNG.uniform(0.1, 5.0, DOF)
    B_nom = RNG.uniform(0.01, 0.2, DOF)
    G_nom = RNG.uniform(1.0, 10.0, DOF)
    tau_max = RNG.uniform(10, 300, DOF)
    ddq_cap = RNG.uniform(50, 500, DOF)
    dq_cap = RNG.uniform(1, 5, DOF)

    q = RNG.uniform(-1, 1, DOF)
    dq = RNG.uniform(-1, 1, DOF)
    qd_k = RNG.uniform(-1, 1, DOF)
    dqd_k = RNG.uniform(-1, 1, DOF)
    ddqd_k = RNG.uniform(-1, 1, DOF)

    tau_proxy, _ = R.control_torque(
        q, dq, qd_k, dqd_k, ddqd_k,
        I_nom=I_nom, B_nom=B_nom, G_nom=G_nom, tau_max=tau_max,
        ddq_cap=ddq_cap, dq_cap=dq_cap, ctrl=ctrl,
        tau_cap_frac=R.TAU_CAP_FRAC, tau_cap_frac_smc=R.TAU_CAP_FRAC_SMC,
    )
    mock_plant = _DiagonalMockPlant(I_nom, G_nom)
    tau_rigid_noB, _ = control_torque_pinocchio(
        q, dq, qd_k, dqd_k, ddqd_k,
        urdf_robot=mock_plant, tau_max=tau_max, ddq_cap=ddq_cap, dq_cap=dq_cap,
        B_nom=B_nom, ctrl=ctrl,
        WN_FRAC=R.WN_FRAC, ZETA=R.ZETA, T_BASE=R.T_BASE, DT=R.DT,
        TAU_CAP_FRAC=R.TAU_CAP_FRAC, TAU_CAP_FRAC_SMC=R.TAU_CAP_FRAC_SMC,
        SMC_PHI_FRAC=R.SMC_PHI_FRAC, SMC_KS_FRAC=R.SMC_KS_FRAC,
        I_nom=I_nom, G_nom=G_nom,
        include_B_feedforward=False,
    )
    expected_B_term = B_nom * dq if ctrl != "pbc" else B_nom * (dqd_k + (2*R.ZETA*(2*np.pi/R.T_BASE)) * (qd_k - q))
    np.testing.assert_allclose(tau_proxy - tau_rigid_noB, expected_B_term, rtol=1e-8, atol=1e-10)
