"""Offline tests of the emulated NMPC engine: the half-precision step equals the October study's
bit-level emulation, the double cost equals mpc_cost, the PSO plans are feasible."""

from __future__ import annotations

import numpy as np
import pytest

import nmpc_engine as ne
from nmpc_engine import am, hw

LIM = am.AckermannLimits(v_min=0.0, v_max=2.0, kappa_max=1.352, accel_max=3.0, decel_max=4.0, kappa_rate_max=18.18)
W = am.CostWeights(q_pos=100.0, q_psi=10.0, q_v=1.0, r_kappa=0.1, s_dv=1.0, s_dkappa=1.0, qf_pos=1000.0, qf_psi=100.0)


def test_fx_grid_and_saturation():
    assert ne.fx(1.0 + 0.4 * ne.LSB) == 1.0 and ne.fx(1.0 + 0.5 * ne.LSB) == 1.0 + ne.LSB
    assert ne.fx(-(1.0 + 0.5 * ne.LSB)) == -(1.0 + ne.LSB)  # ties away from zero
    assert ne.fx(1e9) == ne.TOP_MAX and ne.fx(-1e9) == -ne.TOP_MAX
    assert all(ne.fx(v) == hw.fx(v) for v in np.random.default_rng(0).uniform(-100, 100, 1000))


def test_half_step_equals_the_bit_level_emulation():
    rng = np.random.default_rng(1)
    n = 3000
    state = ne.fx(np.column_stack([rng.uniform(-2, 2, n), rng.uniform(-2, 2, n), rng.uniform(-1.5, 1.5, n)]))
    v, k = rng.uniform(0, 2, n), rng.uniform(-1.352, 1.352, n)
    vec = ne.step_half(state, v, k, 0.05)
    ref = np.array([hw.step_chord_hw(state[i], np.array([v[i], k[i]]), 0.05) for i in range(n)])
    assert np.array_equal(vec, ref)


def test_rollouts_agree_across_arithmetic():
    v = np.full((1, 25), 1.0)
    k = np.full((1, 25), 0.8)
    d = ne.rollout(np.zeros(3), v, k, 0.05, "double")[0]
    ref = am.rollout(np.zeros(3), np.column_stack([v[0], k[0]]), 0.05, method="chord", wrap=False)[1:]
    assert np.allclose(d, ref, atol=1e-12)
    assert np.max(np.abs(ne.rollout(np.zeros(3), v, k, 0.05, "float")[0] - d)) < 1e-5
    assert np.max(np.abs(ne.rollout(np.zeros(3), v, k, 0.05, "half")[0] - d)) < 5e-3  # half model error


def problem(rng=None) -> ne.Problem:
    ts, nh = 0.05, 25
    s = np.arange(1, nh + 1) * 1.0 * ts
    r = 1.5  # a left circle from the origin heading +x
    xref = np.column_stack([r * np.sin(s / r), r * (1 - np.cos(s / r)), s / r])
    uref = np.column_stack([np.full(nh, 1.0), np.full(nh, 1 / r)])
    return ne.Problem(np.zeros(3), xref, uref, np.array([1.0, 0.0]), LIM, ts, nh)


def test_double_cost_equals_mpc_cost():
    pb = problem()
    plan = np.column_stack([np.full(25, 0.9), np.linspace(0.0, 0.7, 25)])
    states = am.rollout(np.zeros(3), plan, 0.05, method="chord", wrap=False)
    ref = am.mpc_cost(states, plan, pb.u_prev, pb.xref, W, pb.uref)["total"]
    assert ne.double_cost(pb, plan, W) == pytest.approx(ref, rel=1e-9)


@pytest.mark.parametrize("arith", ne.ARITHS)
def test_pso_plan_is_feasible_and_tracks(arith):
    pb = problem()
    pb.warm_start = am.project_controls(am.feedforward_from_reference(pb.xref, pb.x0, pb.ts), pb.u_prev, LIM, pb.ts)
    sol = ne.solve(pb, ne.PsoConfig(10, 40), W, np.random.default_rng(3), arith)
    check = am.check_controls(sol.plan, pb.u_prev, LIM, pb.ts, tol=1e-6)
    assert check["ok"], check
    assert np.all(np.diff(sol.history) <= 1e-9)  # the global best never gets worse
    # the plan steers left onto the circle and keeps the speed near 1 m/s
    assert sol.plan[5:, 1].mean() == pytest.approx(1 / 1.5, abs=0.15) and abs(sol.plan[:, 0].mean() - 1.0) < 0.1


def test_same_rng_seed_same_solution():
    pb = problem()
    a = ne.solve(pb, ne.PsoConfig(10, 40), W, np.random.default_rng(7))
    b = ne.solve(pb, ne.PsoConfig(10, 40), W, np.random.default_rng(7))
    c = ne.solve(pb, ne.PsoConfig(10, 40), W, np.random.default_rng(8))
    assert np.array_equal(a.plan, b.plan) and not np.array_equal(a.plan, c.plan)


def test_half_cost_saturates():
    pb = problem()
    pb.xref = pb.xref + 50.0  # absurd references: the ap_fixed accumulator saturates
    z = am.normalize_controls(np.column_stack([np.ones(25), np.zeros(25)]), LIM, 100.0)[None]
    assert ne.cost(pb, z, W, "half")[0] == ne.TOP_MAX


@pytest.mark.parametrize("arith", ne.ARITHS)
def test_vectorized_rollout_equals_sequential(arith):
    rng = np.random.default_rng(11)
    v = rng.uniform(0, 2, (10, 25))
    k = rng.uniform(-1.352, 1.352, (10, 25))
    x0 = np.array([0.013, -0.02, 0.3])
    a = ne.rollout(x0, v, k, 0.05, arith)
    b = ne.rollout_sequential(x0, v, k, 0.05, arith)
    if arith == "half":
        assert np.array_equal(a, b)  # bit-identical, including heading wraps
    else:
        assert np.allclose(a, b, atol=1e-5 if arith == "float" else 1e-12)
