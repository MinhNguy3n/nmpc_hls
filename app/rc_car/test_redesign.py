"""Offline tests of the PSO redesign switches R1-R4 (2026-11-22 brief).

With all switches off the engine and the closed loop are BIT-IDENTICAL to the frozen phase 4
controller (fixture recorded from the unmodified code: testdata/phase4_frozen_golden.npz)."""

from __future__ import annotations

from pathlib import Path

import numpy as np
import pytest

import nmpc_controller as nc
import nmpc_engine as ne
import nmpc_offline as no
import paths as P
import test_nmpc_engine as te
from nmpc_engine import am

GOLDEN = np.load(Path(__file__).resolve().parent / "testdata" / "phase4_frozen_golden.npz")
CFG = nc.load_config()


def warm_problem():
    pb = te.problem()
    pb.warm_start = am.project_controls(am.feedforward_from_reference(pb.xref, pb.x0, pb.ts), pb.u_prev, te.LIM, pb.ts)
    return pb


@pytest.mark.parametrize("arith", ne.ARITHS)
def test_engine_bit_identical_with_all_switches_off(arith):
    sol = ne.solve(warm_problem(), ne.PsoConfig(10, 40), te.W, np.random.default_rng(3), arith)
    assert np.array_equal(sol.plan, GOLDEN[f"engine_{arith}_plan"])
    assert np.array_equal(sol.history, GOLDEN[f"engine_{arith}_hist"])


@pytest.mark.parametrize("name", ["lane_change", "circle"])
@pytest.mark.parametrize("arith", ["half", "float"])
def test_closed_loop_bit_identical_with_all_switches_off(name, arith):
    ctrl = nc.NmpcController(CFG, "10x40", "realistic_guess", compensation=True, arith=arith, redesign=nc.Redesign())
    r = no.run(ctrl, P.build(name, kappa_max=CFG.limits.kappa_max), 1.0, 2, t_max=2.0)
    assert np.array_equal(np.column_stack([r.x, r.y, r.yaw]), GOLDEN[f"loop_{name}_{arith}_xy"])
    assert np.array_equal(np.array([c[:3] for c in r.commands]), GOLDEN[f"loop_{name}_{arith}_cmd"])


def test_r2_interpolation_and_rate_windows():
    knots = ne.make_knots(5, 25)
    assert knots == (0, 6, 12, 18, 24)
    W = ne.interp_matrix(knots, 25)
    assert np.allclose(W.sum(axis=1), 1.0) and np.allclose(W[list(knots)], np.eye(5))
    assert np.allclose(W @ np.arange(5.0), np.interp(np.arange(25), knots, np.arange(5.0)))
    pb = warm_problem()
    pb.knots = knots
    for arith in ("half", "float"):
        sol = ne.solve(pb, ne.PsoConfig(10, 40), te.W, np.random.default_rng(4), arith)
        assert sol.plan.shape == (25, 2)
        assert am.check_controls(sol.plan, pb.u_prev, te.LIM, pb.ts, tol=1e-6)["ok"]  # rate limits hold per step
        z = am.normalize_controls(sol.plan, te.LIM, 100.0)
        assert np.allclose(z, W @ z[list(knots)], atol=1e-9)  # the plan is piecewise linear between knots


def test_r1_seeds_and_perturbed_copies():
    pb = warm_problem()
    other = np.column_stack([np.full(25, 0.8), np.full(25, 0.3)])
    pb.extra_starts = [other]
    pb.n_perturb = 2
    # one iteration: the seeded particles are evaluated at least once; the global best can only be
    # as good as the best seed or better
    sol = ne.solve(pb, ne.PsoConfig(10, 1), te.W, np.random.default_rng(5), "double")
    seeds = [ne.double_cost(pb, pb.warm_start, te.W), ne.double_cost(pb, other, te.W)]
    assert sol.fitness <= min(seeds) + 1e-9
    # R1 off draws no extra random numbers: same result as without the fields
    a = ne.solve(warm_problem(), ne.PsoConfig(10, 40), te.W, np.random.default_rng(3), "float")
    b_pb = warm_problem()
    b_pb.extra_starts, b_pb.n_perturb = None, 0
    b = ne.solve(b_pb, ne.PsoConfig(10, 40), te.W, np.random.default_rng(3), "float")
    assert np.array_equal(a.plan, b.plan)


def test_r3_path_frame_cost_matches_a_hand_rotation():
    pb = warm_problem()
    pb.path_frame = ne.PathFrame(q_lat=500.0, q_lon=50.0, qf_lat=5000.0, qf_lon=500.0)
    pb.ref_cs = np.column_stack([np.cos(pb.xref[:, 2]), np.sin(pb.xref[:, 2])])
    plan = np.column_stack([np.full(25, 0.9), np.linspace(0.0, 0.6, 25)])
    states = am.rollout(np.zeros(3), plan, pb.ts, method="chord", wrap=False)[1:]
    d = states[:, :2] - pb.xref[:, :2]
    c, s = pb.ref_cs[:, 0], pb.ref_cs[:, 1]
    lat, lon = -s * d[:, 0] + c * d[:, 1], c * d[:, 0] + s * d[:, 1]
    q_lat = np.r_[np.full(24, 500.0), 5000.0]
    q_lon = np.r_[np.full(24, 50.0), 500.0]
    w0 = am.CostWeights(q_pos=0.0, q_psi=te.W.q_psi, q_v=te.W.q_v, r_kappa=te.W.r_kappa, s_dv=te.W.s_dv,
                        s_dkappa=te.W.s_dkappa, qf_pos=0.0, qf_psi=te.W.qf_psi)
    rest = am.mpc_cost(np.vstack([np.zeros(3), states]), plan, pb.u_prev, pb.xref, w0, pb.uref)["total"]
    expected = rest + float(np.sum(q_lat * lat ** 2 + q_lon * lon ** 2))
    assert ne.double_cost(pb, plan, te.W) == pytest.approx(expected, rel=1e-9)
    # equal lateral and along-track weights = the original isotropic cost
    pb.path_frame = ne.PathFrame(te.W.q_pos, te.W.q_pos, te.W.qf_pos, te.W.qf_pos)
    iso = warm_problem()
    assert ne.double_cost(pb, plan, te.W) == pytest.approx(ne.double_cost(iso, plan, te.W), rel=1e-12)


def test_r4_and_labels_and_controller_wiring():
    rd = nc.Redesign(r1=True, r2=5, r3=(3160.0, 316.0), r4=10.0)
    assert rd.label == "R1+R2+R3+R4" and nc.Redesign().label == "frozen"
    ctrl = nc.NmpcController(CFG, "10x40", "realistic_guess", redesign=rd)
    assert ctrl.weights.s_dkappa == 10.0 and ctrl.path_frame.q_lon == 316.0
    assert ctrl.path_frame.qf_lat == pytest.approx(10 * 3160.0)
    r = no.run(ctrl, P.build("lane_change", kappa_max=CFG.limits.kappa_max), 1.0, 3, t_max=1.5)
    assert r.stats.fallbacks == 0 and len(r.commands) > 20
    plan = ctrl.prepare(10.0, ctrl.measured and None)  # no pose -> None, counted
    assert plan is None and ctrl.stats.no_pose == 1


def test_frozen_redesign_loads_from_yaml():
    rd = nc.load_redesign()
    assert rd == nc.Redesign(r1=True, r2=None, r3=(1000.0, 31.6), r4=10.0) and rd.label == "R1+R3+R4"
    # the phase 4 weights are untouched
    assert CFG.weights.q_pos == 3160.0 and CFG.weights.s_dkappa == 1.0


def test_smooth_variant_loads_and_is_labelled():
    rd = nc.load_redesign(section="redesign_smooth")
    assert rd == nc.Redesign(r1=True, r2=5, r3=(3160.0, 316.0), r4=10.0) and rd.label == "R1+R2+R3+R4"
