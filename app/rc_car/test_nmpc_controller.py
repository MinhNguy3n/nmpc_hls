"""Offline tests of the NMPC arm's host side: limits from the YAML files, the timing model, the
prediction, the path-following reference, seed reproducibility, and that a slow Python solve
changes neither the applied times nor the trajectory (phase 4 conditions 1, 3, 4, 5)."""

from __future__ import annotations

import math

import numpy as np
import pytest
import yaml

import nmpc_controller as nc
import nmpc_offline as no
import paths as P
import vehicle_params as vp
from nmpc_engine import am
from pose_input import Pose

CFG = nc.load_config()


def test_limits_come_from_the_vehicle_and_chain_files(tmp_path):
    g = vp.load_geometry()
    assert CFG.delta_max == g.delta_max == 0.356
    assert CFG.limits.kappa_max == pytest.approx(math.tan(0.356) / 0.275)
    assert CFG.limits.kappa_rate_max == pytest.approx(5.0 / 0.275)  # realistic_guess delta_rate_max
    assert (CFG.limits.accel_max, CFG.limits.decel_max, CFG.limits.v_max) == (3.0, 4.0, 2.0)
    # a different delta_max in reely_new1.yaml changes the NMPC bound (not the October code's examples)
    raw = yaml.safe_load(open(vp.DEFAULT_PARAMS))
    raw["delta_max"]["value"] = 0.2
    other = tmp_path / "car.yaml"
    other.write_text(yaml.safe_dump(raw))
    cfg2 = nc.load_config(params_file=other)
    assert cfg2.limits.kappa_max == pytest.approx(math.tan(0.2) / 0.275) and cfg2.delta_max == 0.2


def test_solve_times_and_weights():
    assert CFG.sizes["3x24"]["solve_time"] == 0.00925 and CFG.sizes["10x40"]["solve_time"] == 0.02635
    assert CFG.sizes["10x60"]["status"] == "estimate"
    assert CFG.weights.qf_pos == 10 * CFG.weights.q_pos and CFG.weights.qf_psi == 10 * CFG.weights.q_psi


def test_tick_anchored_start_and_apply_times():
    """The solve starts at the tick and the command is applied solve_time later (board, not Python)."""
    c = nc.NmpcController(CFG, "10x40", "realistic_guess")
    c.start_episode(P.build("line"), 1.0, 1)
    plan = c.prepare(10.000, Pose(9.990, -4.0, 0.0, 0.0))
    assert (plan.t_start, plan.t_apply) == (10.000, pytest.approx(10.000 + 0.02635)) and not plan.deadline_miss
    assert plan.t_eff == pytest.approx(plan.t_apply + 0.0242 + 0.011)
    # an old pose does not move the apply time; it is counted as stale (> one period), not as a miss
    plan = c.prepare(10.050, Pose(9.940, -4.0, 0.0, 0.0))
    assert plan.t_apply == pytest.approx(10.050 + 0.02635) and not plan.deadline_miss
    assert c.stats.stale_pose == 1 and c.stats.deadline_misses == 0


def test_single_solver_never_overlaps():
    """Solves never overlap, also when a (hypothetical) solve is longer than the period."""
    for size, solve_time in (("10x60", None), ("10x40", 0.07)):
        c = nc.NmpcController(CFG, size, "realistic_guess")
        if solve_time is not None:
            c.solve_time = solve_time  # slower than the 50 ms period
        c.start_episode(P.build("line"), 1.0, 1)
        rng = np.random.default_rng(0)
        for k in range(200):
            t = 10.0 + 0.05 * k
            age = float(rng.choice([0.005, 0.011, 0.02, 0.09]))  # spread and spikes
            plan = c.prepare(t, Pose(t - age, -4.0, 0.0, 0.0))
            c.finish(plan, c.solve(plan))
        starts, ends = np.array(c.solves).T
        assert np.all(starts[1:] >= ends[:-1] - 1e-12)  # the next solve starts after the previous one ended
        assert np.allclose(ends - starts, c.solve_time)
        if solve_time is None:
            assert np.allclose(starts, 10.0 + 0.05 * np.arange(200)) and c.stats.deadline_misses == 0
        else:
            assert c.stats.deadline_misses > 0  # only when the solve exceeds the period


def test_prediction_through_the_command_buffer():
    pose = Pose(1.0, 0.0, 0.0, 0.0)
    assert np.allclose(nc.predict(pose, 1.5, []), [0, 0, 0])  # nothing sent: at rest
    buf = [(0.9, 1.0, 0.0), (1.2, 1.0, 0.5)]  # straight from 0.9 s, a left arc from 1.2 s
    x = nc.predict(pose, 1.5, buf)
    ref = am.step_exact_arc(am.step_exact_arc(np.zeros(3), (1.0, 0.0), 0.2), (1.0, 0.5), 0.3)
    assert np.allclose(x, ref, atol=1e-12)


def test_reference_window_follows_the_path_at_the_commanded_speed():
    c = nc.NmpcController(CFG, "3x24", "ideal", compensation=False)
    path = P.build("line")
    c.start_episode(path, 1.2, 1)
    plan = c.prepare(0.05, Pose(0.04, path.start[0] + 1.0, 0.02, 0.0))  # 2 cm left of the line
    xr = plan.problem.xref
    assert np.allclose(xr[:, 0], 1.2 * 0.05 * np.arange(1, 26), atol=0.011)  # spacing = commanded speed
    assert np.allclose(xr[:, 1], -0.02, atol=1e-9) and np.allclose(xr[:, 2], 0.0)
    assert np.allclose(plan.problem.uref, [[1.2, 0.0]] * 25)


def offline(seed, slow=0.0, compensation=True, t_max=1.5):
    ctrl = nc.NmpcController(CFG, "3x24", "realistic_guess", compensation=compensation)
    return no.run(ctrl, P.build("lane_change"), 1.0, seed, t_max=t_max, slow_solve_s=slow)


def test_repeated_seed_reproduces_the_run_exactly():
    a, b, c = offline(4), offline(4), offline(5)
    assert np.array_equal(a.x, b.x) and np.array_equal(a.y, b.y) and a.commands == b.commands
    assert not np.array_equal(a.y, c.y)


def test_slow_python_changes_neither_the_applied_times_nor_the_trajectory():
    fast, slow = offline(6), offline(6, slow=0.06)  # 60 ms of wall time per solve, > the 50 ms period
    assert [c[0] for c in fast.commands] == [c[0] for c in slow.commands]
    assert np.array_equal(fast.x, slow.x) and np.array_equal(fast.yaw, slow.yaw)
    assert fast.overruns == slow.overruns == 0


def test_scheduler_publishes_at_t_apply_and_counts_overruns():
    s = nc.Scheduler()
    cmd = nc.Command(1.0, 1.0, 0.0, 0.0, 0.0, 0.0, False, False, 0.0, False)
    s.submit(0.98, cmd)
    assert s.due(0.999) == [] and s.due(1.0) == [cmd] and s.overruns == 0
    late = nc.Command(2.0, 1.0, 0.0, 0.0, 0.0, 0.0, False, False, 0.0, False)
    s.submit(2.004, late)  # the result arrived 4 ms after its t_apply
    assert s.due(2.004) == [late] and s.overruns == 1 and s.overrun_max == pytest.approx(0.004)


def test_closed_loop_tracks_and_feasible_commands():
    r = offline(7, t_max=4.0)
    deltas = np.array([c[2] for c in r.commands])
    assert np.max(np.abs(deltas)) <= CFG.delta_max + 1e-9
    assert r.stats.fallbacks == 0
    import tracking_metrics as tm
    m = tm.track(P.build("lane_change"), r.t, r.x, r.y, r.yaw, end_margin=0.5)
    assert m["cross_track_max_m"] < 0.02
