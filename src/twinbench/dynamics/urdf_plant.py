"""URDF-based rigid-body dynamics via Pinocchio (Articulated-Body Algorithm)."""

from __future__ import annotations
from typing import Any, Dict, Tuple
from pathlib import Path
import numpy as np

try:
    import pinocchio as pin
except ImportError:
    raise ImportError("Pinocchio required: conda install -c conda-forge pinocchio")

DOF = 6


class URDFRobot:
    def __init__(self, urdf_path: str | Path):
        self.urdf_path = str(urdf_path)
        self.model = pin.buildModelFromUrdf(self.urdf_path)
        self.data = self.model.createData()
        if self.model.nv != DOF:
            raise ValueError(f"Expected 6-DOF, got nv={self.model.nv}")
        q0 = np.zeros(self.model.nq)
        M0 = pin.crba(self.model, self.data, q0).copy()
        self.I_proxy = np.maximum(np.diag(M0), 1e-4)
        self.g_q0 = np.array(pin.computeGeneralizedGravity(self.model, self.data, q0)).copy()

    def mass_matrix(self, q):
        return pin.crba(self.model, self.data, q).copy()

    def nonlinear_effects(self, q, dq):
        return np.array(pin.rnea(self.model, self.data, q, dq, np.zeros(DOF))).ravel()

    def coriolis_matrix(self, q, dq):
        # Explicit Coriolis matrix C(q, dq) for the Slotine-Li passivity term
        # C(q, dq) @ qd_r used by PBC. Consistent with rnea() used elsewhere:
        # C(q,v) @ v + gravity(q) == rnea(q, v, 0) (verified to ~1e-14).
        return np.array(pin.computeCoriolisMatrix(self.model, self.data, q, dq))

    def gravity(self, q):
        return np.array(pin.computeGeneralizedGravity(self.model, self.data, q)).ravel()

    def forward_dynamics_full(self, q, dq, tau, B_eff):
        tau_net = tau - B_eff * dq
        return np.asarray(pin.aba(self.model, self.data, q, dq, tau_net), float).ravel()

    def fk(self, q):
        pin.forwardKinematics(self.model, self.data, q)
        pin.updateFramePlacements(self.model, self.data)
        return self.data.oMf[self.model.nframes - 1].translation.copy()

    def fk_all_positions(self, Q):
        return np.array([self.fk(Q[i]) for i in range(len(Q))])


class PerturbedURDFRobot:
    """URDFRobot with each link's mass and rotational inertia scaled by a random
    factor from U[1-frac, 1+frac]. COM positions are unchanged.
    """

    def __init__(self, urdf_path: str | Path, mass_frac: float = 0.10, seed: int = 42):
        self.urdf_path = str(urdf_path)
        self.mass_frac = mass_frac
        self.seed = seed

        # Load a fresh model (independent of any cached nominal model)
        self.model = pin.buildModelFromUrdf(self.urdf_path)
        self.data = self.model.createData()

        if self.model.nv != DOF:
            raise ValueError(f"Expected 6-DOF, got nv={self.model.nv}")

        # Perturb link masses deterministically
        rng = np.random.default_rng(seed)
        self.mass_scales = {}

        for i in range(1, self.model.njoints):  # skip universe joint (index 0)
            scale = float(rng.uniform(1.0 - mass_frac, 1.0 + mass_frac))
            self.mass_scales[i] = scale

            original = self.model.inertias[i]
            new_mass = original.mass * scale
            new_lever = original.lever.copy()  # CoM unchanged (same geometry)
            new_inertia = original.inertia * scale  # I ∝ mass for fixed geometry

            self.model.inertias[i] = pin.Inertia(new_mass, new_lever, new_inertia)

        # Recreate data after modifying model
        self.data = self.model.createData()

        # Recompute proxy quantities on perturbed model
        q0 = np.zeros(self.model.nq)
        M0 = pin.crba(self.model, self.data, q0).copy()
        self.I_proxy = np.maximum(np.diag(M0), 1e-4)

    def mass_matrix(self, q):
        return pin.crba(self.model, self.data, q).copy()

    def nonlinear_effects(self, q, dq):
        return np.array(pin.rnea(self.model, self.data, q, dq, np.zeros(DOF))).ravel()

    def coriolis_matrix(self, q, dq):
        # Explicit Coriolis matrix C(q, dq) for the Slotine-Li passivity term
        # C(q, dq) @ qd_r used by PBC. Consistent with rnea() used elsewhere:
        # C(q,v) @ v + gravity(q) == rnea(q, v, 0) (verified to ~1e-14).
        return np.array(pin.computeCoriolisMatrix(self.model, self.data, q, dq))

    def gravity(self, q):
        return np.array(pin.computeGeneralizedGravity(self.model, self.data, q)).ravel()

    def forward_dynamics_full(self, q, dq, tau, B_eff):
        tau_net = tau - B_eff * dq
        return np.asarray(pin.aba(self.model, self.data, q, dq, tau_net), float).ravel()

    def fk(self, q):
        pin.forwardKinematics(self.model, self.data, q)
        pin.updateFramePlacements(self.model, self.data)
        return self.data.oMf[self.model.nframes - 1].translation.copy()

    def fk_all_positions(self, Q):
        return np.array([self.fk(Q[i]) for i in range(len(Q))])


def get_perturbed_urdf_robot(urdf_path, mass_frac=0.10, seed=42) -> PerturbedURDFRobot:
    """Create a perturbed URDF robot (not cached: each call creates fresh perturbation)."""
    return PerturbedURDFRobot(str(urdf_path), mass_frac=mass_frac, seed=seed)


class ScaledURDFRobot:
    """URDFRobot with M(q) scaled by I_factor and g(q) by G_factor.

    All link masses and inertias are scaled by I_factor; gravity is set to
    g0 * G_factor / I_factor so that g(q) scales by G_factor.
    """

    def __init__(self, urdf_path: str | Path, I_factor: float = 1.0, G_factor: float = 1.0):
        self.urdf_path = str(urdf_path)
        self.I_factor = float(I_factor)
        self.G_factor = float(G_factor)

        self.model = pin.buildModelFromUrdf(self.urdf_path)
        self.data = self.model.createData()
        if self.model.nv != DOF:
            raise ValueError(f"Expected 6-DOF, got nv={self.model.nv}")

        for i in range(1, self.model.njoints):
            original = self.model.inertias[i]
            new_mass = original.mass * self.I_factor
            new_lever = original.lever.copy()  # CoM unchanged (same geometry)
            new_inertia = original.inertia * self.I_factor  # I ∝ mass for fixed geometry
            self.model.inertias[i] = pin.Inertia(new_mass, new_lever, new_inertia)

        # Net gravity torque g(q) is already scaled by I_factor from the mass
        # scaling above; rescale the gravitational acceleration itself by
        # (G_factor / I_factor) so the net g(q) scales by G_factor.
        g = self.model.gravity
        gain = self.G_factor / self.I_factor
        self.model.gravity = pin.Motion(np.array(g.linear) * gain, np.array(g.angular) * gain)

        # Recreate data after modifying model
        self.data = self.model.createData()

        q0 = np.zeros(self.model.nq)
        M0 = pin.crba(self.model, self.data, q0).copy()
        self.I_proxy = np.maximum(np.diag(M0), 1e-4)

    def mass_matrix(self, q):
        return pin.crba(self.model, self.data, q).copy()

    def nonlinear_effects(self, q, dq):
        return np.array(pin.rnea(self.model, self.data, q, dq, np.zeros(DOF))).ravel()

    def coriolis_matrix(self, q, dq):
        return np.array(pin.computeCoriolisMatrix(self.model, self.data, q, dq))

    def gravity(self, q):
        return np.array(pin.computeGeneralizedGravity(self.model, self.data, q)).ravel()

    def forward_dynamics_full(self, q, dq, tau, B_eff):
        tau_net = tau - B_eff * dq
        return np.asarray(pin.aba(self.model, self.data, q, dq, tau_net), float).ravel()

    def fk(self, q):
        pin.forwardKinematics(self.model, self.data, q)
        pin.updateFramePlacements(self.model, self.data)
        return self.data.oMf[self.model.nframes - 1].translation.copy()

    def fk_all_positions(self, Q):
        return np.array([self.fk(Q[i]) for i in range(len(Q))])


_scaled_urdf_cache: Dict[Tuple[str, float, float], ScaledURDFRobot] = {}
def get_scaled_urdf_robot(urdf_path, I_factor: float = 1.0, G_factor: float = 1.0) -> ScaledURDFRobot:
    """Create or reuse an I/G-scaled URDF robot.

    Cached by (urdf_path, I_factor, G_factor), since the scaled model depends only
    on the robot and the uncertainty level.
    """
    key = (str(urdf_path), float(I_factor), float(G_factor))
    if key not in _scaled_urdf_cache:
        _scaled_urdf_cache[key] = ScaledURDFRobot(key[0], I_factor=key[1], G_factor=key[2])
    return _scaled_urdf_cache[key]


def control_torque_pinocchio(
    q, dq, qd_k, dqd_k, ddqd_k, *,
    urdf_robot, tau_max, ddq_cap, dq_cap, B_nom,
    ctrl,
    WN_FRAC, ZETA, T_BASE, DT,
    TAU_CAP_FRAC, TAU_CAP_FRAC_SMC,
    SMC_PHI_FRAC, SMC_KS_FRAC,
    I_nom, G_nom,
    include_B_feedforward: bool = True,
):
    """Controller with Pinocchio-based feedforward for Tier B-full.

    include_B_feedforward: if True (default), ctc_pd and ctc_smc include the
        B_nom*qd term and pbc the B_nom*qd_r term (nominal B_nom), as on the
        proxy tier. False omits it. No effect on pdg.
    """
    EPS = 1e-9
    wn = WN_FRAC * np.sqrt(np.maximum(ddq_cap, 1e-6))
    Kp, Kd = wn**2, 2.0 * ZETA * wn
    e, de = qd_k - q, dqd_k - dq

    M = urdf_robot.mass_matrix(q)
    nle = urdf_robot.nonlinear_effects(q, dq)
    g_q = urdf_robot.gravity(q)

    if ctrl == "pdg":
        tau = M @ (Kp * e + Kd * de) + g_q
        cap = TAU_CAP_FRAC * tau_max
    elif ctrl == "ctc_pd":
        v = ddqd_k + Kp * e + Kd * de
        tau = M @ v + nle
        if include_B_feedforward:
            tau = tau + B_nom * dq
        cap = TAU_CAP_FRAC * tau_max
    elif ctrl == "ctc_smc":
        wn_smc = 2.0 * np.pi / max(T_BASE, 1e-6)
        Lambda = 2.0 * ZETA * wn_smc
        s = de + Lambda * e
        v_eq = ddqd_k + Lambda * de
        tau_eq = M @ v_eq + nle
        if include_B_feedforward:
            tau_eq = tau_eq + B_nom * dq
        # Normalized Ks: scales with robot's natural torque level
        M_diag = np.diag(M)
        Ks = SMC_KS_FRAC * M_diag * np.maximum(Kp, 1.0)
        phi = SMC_PHI_FRAC * dq_cap
        tau = tau_eq + Ks * np.tanh(s / (phi + EPS))
        cap = TAU_CAP_FRAC_SMC * tau_max
    elif ctrl == "pbc":
        # PBC (Slotine-Li passivity-based), rigid-body tier, stateless.
        # Lambda is the same sliding-surface slope as in CTC-SMC.
        wn_pbc = 2.0 * np.pi / max(T_BASE, 1e-6)
        Lambda = 2.0 * ZETA * wn_pbc
        # Reference velocity qd_r = qd_d + Lambda*e, qdd_r = qdd_d + Lambda*de,
        # consistent with s = de + Lambda*e.
        qd_r = dqd_k + Lambda * e
        qdd_r = ddqd_k + Lambda * de
        s = de + Lambda * e
        # Passivity Coriolis term C(q, dq) @ qd_r: Coriolis matrix at the actual
        # velocity times the reference velocity. rnea(q, qd_r, 0) would evaluate
        # C at qd_r in both slots, which is not the passivity form.
        C = urdf_robot.coriolis_matrix(q, dq)
        # K_D from the gain-normalization protocol: diag(M(q)) * K_d. The
        # dissipation gain is diagonal; the feedforward term uses the full M(q).
        K_D = np.diag(M) * Kd
        tau = M @ qdd_r + C @ qd_r + g_q + K_D * s
        if include_B_feedforward:
            tau = tau + B_nom * qd_r
        # Same torque cap as pdg and ctc_pd.
        cap = TAU_CAP_FRAC * tau_max
    else:
        raise ValueError(f"Unknown controller: {ctrl}")
    return np.asarray(tau, float), np.asarray(cap, float)


def simulate_urdf_full(
    *, urdf_robot,
    urdf_robot_plant=None,
    qd, dqd, ddqd, t,
    q_init, qmin, qmax,
    I_nom, B_nom, G_nom, tau_max,
    I_eff, B_eff, G_eff,
    dq_cap, ddq_cap, ctrl,
    rng_noise, sigma_q, sigma_qdot,
    disturb_joint, disturb_tau, disturb_onset, disturb_end,
    control_fn,
    WN_FRAC=0.25, ZETA=0.9, T_BASE=2.0, DT=0.001,
    TAU_CAP_FRAC=0.90, TAU_CAP_FRAC_SMC=0.70,
    SMC_PHI_FRAC=0.60, SMC_KS_FRAC=0.05,
    include_B_feedforward: bool = True,
):
    """Closed-loop on Pinocchio plant with Pinocchio-based controllers."""
    N = len(t)
    dt = float(t[1] - t[0]) if N > 1 else 0.001
    q = np.zeros((N, DOF)); dq_sim = np.zeros((N, DOF))
    tau_hist = np.zeros((N, DOF)); ddq_hist = np.zeros((N, DOF))
    sat_hist = np.zeros((N, DOF), dtype=bool)
    q[0] = q_init
    dq_sim[0] = dqd[0] if len(dqd) > 0 else np.zeros(DOF)

    # If a separate plant robot is provided, use it for dynamics
    # Controller feedforward still uses urdf_robot (nominal)
    plant = urdf_robot_plant if urdf_robot_plant is not None else urdf_robot

    for k in range(1, N):
        q_meas = q[k-1] + rng_noise.normal(0, sigma_q, DOF)
        dq_meas = dq_sim[k-1] + rng_noise.normal(0, sigma_qdot, DOF)

        tau_cmd, tau_cap = control_torque_pinocchio(
            q_meas, dq_meas, qd[k], dqd[k], ddqd[k],
            urdf_robot=urdf_robot, tau_max=tau_max,
            ddq_cap=ddq_cap, dq_cap=dq_cap, B_nom=B_nom,
            ctrl=ctrl,
            WN_FRAC=WN_FRAC, ZETA=ZETA, T_BASE=T_BASE, DT=DT,
            TAU_CAP_FRAC=TAU_CAP_FRAC, TAU_CAP_FRAC_SMC=TAU_CAP_FRAC_SMC,
            SMC_PHI_FRAC=SMC_PHI_FRAC, SMC_KS_FRAC=SMC_KS_FRAC,
            I_nom=I_nom, G_nom=G_nom,
            include_B_feedforward=include_B_feedforward,
        )

        if disturb_onset <= t[k] <= disturb_end:
            tau_cmd = tau_cmd.copy(); tau_cmd[disturb_joint] += disturb_tau

        sat_hist[k] = np.abs(tau_cmd) > (tau_cap + 1e-12)
        tau = np.clip(tau_cmd, -tau_cap, tau_cap); tau_hist[k] = tau

        ddq_k = plant.forward_dynamics_full(q[k-1], dq_sim[k-1], tau, B_eff)
        ddq_k = np.clip(ddq_k, -500.0, 500.0); ddq_hist[k] = ddq_k

        dq_sim[k] = dq_sim[k-1] + ddq_k * dt
        q[k] = np.clip(q[k-1] + dq_sim[k] * dt, qmin, qmax)

    return q, dq_sim, tau_hist, ddq_hist, sat_hist


_urdf_cache: Dict[str, URDFRobot] = {}
def get_urdf_robot(urdf_path) -> URDFRobot:
    key = str(urdf_path)
    if key not in _urdf_cache:
        _urdf_cache[key] = URDFRobot(key)
    return _urdf_cache[key]