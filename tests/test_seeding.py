"""Seeding invariants (docs/REPRODUCIBILITY.md, Section 2).

  target seed = stable_int_seed("target", robot, scene,           seed)
  noise  seed = stable_int_seed("noise",  robot, scene, traj, prm, seed)

Both seeds are deterministic and independent of the controller, so all
controllers on a (robot, scene, trajectory, params, seed) tuple see the same
target and the same noise.
"""
from __future__ import annotations

import tests.conftest  # noqa: F401 : bootstraps sys.path

from experiments.run_main_grid import seed_noise, seed_target, stable_int_seed


# stable_int_seed: pure deterministic function over its inputs

def test_stable_int_seed_is_deterministic() -> None:
    a = stable_int_seed("target", "kr6", "open_reach", 0, base=11111)
    b = stable_int_seed("target", "kr6", "open_reach", 0, base=11111)
    assert a == b


def test_stable_int_seed_seed_axis_is_significant() -> None:
    a = stable_int_seed("target", "kr6", "open_reach", 0, base=11111)
    b = stable_int_seed("target", "kr6", "open_reach", 1, base=11111)
    assert a != b


def test_stable_int_seed_robot_axis_is_significant() -> None:
    a = stable_int_seed("target", "kr6", "open_reach", 0, base=11111)
    b = stable_int_seed("target", "rs006l", "open_reach", 0, base=11111)
    assert a != b


def test_stable_int_seed_base_axis_is_significant() -> None:
    # Different bases (11111 vs 22222) must give different seeds even
    # for otherwise-identical inputs: this is what keeps target and
    # noise streams disjoint.
    a = stable_int_seed("target", "kr6", "open_reach", 0, base=11111)
    b = stable_int_seed("target", "kr6", "open_reach", 0, base=22222)
    assert a != b


# seed_target: bit-identical across calls; no controller dependence

def test_seed_target_is_deterministic() -> None:
    a = seed_target("kr6", "open_reach", 5)
    b = seed_target("kr6", "open_reach", 5)
    assert a == b


def test_seed_target_signature_excludes_controller() -> None:
    # The seed must not depend on the controller.
    import inspect

    sig = inspect.signature(seed_target)
    params = list(sig.parameters.keys())
    assert params == ["rid", "sid", "seed"], (
        f"seed_target signature changed: {params}; the controller must not be a "
        "seeding input (docs/REPRODUCIBILITY.md, Section 2)."
    )


# seed_noise: bit-identical across calls; no controller dependence

def test_seed_noise_is_deterministic() -> None:
    a = seed_noise("ur5e", "open_reach", "cubic", "nominal", 3)
    b = seed_noise("ur5e", "open_reach", "cubic", "nominal", 3)
    assert a == b


def test_seed_noise_signature_excludes_controller() -> None:
    import inspect

    sig = inspect.signature(seed_noise)
    params = list(sig.parameters.keys())
    assert params == ["rid", "sid", "traj", "prm", "seed"], (
        f"seed_noise signature changed: {params}; the controller must not be a "
        "seeding input (docs/REPRODUCIBILITY.md, Section 2)."
    )


# Disjointness between target and noise streams

def test_target_and_noise_seeds_are_disjoint() -> None:
    # Different prefixes ("target" vs "noise") + different bases (11111
    # vs 22222) ensure target sampling and per-step measurement noise
    # come from independent RNG streams for the same (robot, scene, seed).
    s_t = seed_target("kr6", "open_reach", 0)
    s_n = seed_noise("kr6", "open_reach", "cubic", "nominal", 0)
    assert s_t != s_n
