"""Offline tests of sil.offline_loop: determinism, equivalence with MpcCore / pso.solve, the
increment statistics and the yaw-effort bias it is used to screen (no ROS)."""

from __future__ import annotations

from dataclasses import replace
from pathlib import Path
import sys

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from sil import metrics, offline_loop as ol, pso  # noqa: E402
from sil.reference import reference_window  # noqa: E402
import unicycle_model as um  # noqa: E402

SHORT = ol.LoopConfig(path="line", size=0.5, speed=0.2, pso="4x10", seed=4)  # plant_delay 0


def commands(res: ol.LoopResult) -> list[tuple]:
    return [(r["kind"], r["v"], r["w"]) for r in res.ticks]


def test_loop_is_deterministic_for_a_configuration_and_seed():
    a, b = ol.run_loop(SHORT), ol.run_loop(SHORT)
    assert a.ended == "run finished" and commands(a) == commands(b)
    assert [r["x"] for r in a.truth] == [r["x"] for r in b.truth]
    c = ol.run_loop(replace(SHORT, seed=5))
    assert commands(c) != commands(a)


def test_loop_drives_the_core_through_arming_and_its_first_plan_is_the_solver_output():
    """Equivalence: the first owned solve of MpcCore inside the loop equals pso.solve on the
    independently built problem (zero applied command, no warm start, reference at tau = 0)
    with a fresh generator of the same seed."""
    res = ol.run_loop(SHORT)
    kinds = [r["kind"] for r in res.ticks]
    assert kinds[:2] == ["probe", "probe"] and kinds.count("plan") > 10
    core_cfg = SHORT.core_config()
    origin = np.array(res.origin)
    xref = reference_window(core_cfg.path, origin, 0.0, core_cfg.horizon, core_cfg.ts, origin)
    problem = pso.Problem(np.zeros(3), xref, np.zeros(2), core_cfg.limits, core_cfg.ts, core_cfg.control_horizon,
                          None, pso.yaw_rate_gain(SHORT.gain_mode))
    expected = pso.solve(problem, core_cfg.pso, core_cfg.cost, np.random.default_rng(SHORT.seed)).plan[0]
    first_plan = next(r for r in res.ticks if r["kind"] == "plan")
    assert (first_plan["v"], first_plan["w"]) == pytest.approx(tuple(expected), abs=1e-12)
    assert first_plan["t"] == pytest.approx(res.tau0)


def test_config_sets_the_weight_and_the_run_id_records_it():
    cfg = replace(SHORT, r_omega=5e-6, gain_mode="sim-lowk", plant_yaw_gain=0.92)
    core = cfg.core_config()
    assert core.cost.r == (0.0, 5e-6) and core.gain_mode == "sim-lowk"
    assert core.cost.q == pso.CostConfig().q and core.cost.qf == pso.CostConfig().qf
    assert "romega5e-06" in cfg.run_id and "plant1x0.92" in cfg.run_id


def test_increment_statistics_deadband_reversals_and_slew_bounds():
    ts = 0.05
    values = np.array([0.0, 0.2, 0.1, 0.1005, 0.0, 0.2])  # +0.2, -0.1, +0.0005 (in deadband), -0.1005, +0.2
    windows = [(p - 0.2, p + 0.2) for p in values[:-1]]
    st = metrics.increments(values, 0.002, windows, ts)
    assert st["reversals_per_s"] == pytest.approx(2 / (5 * ts))  # + - (skip) - + -> 2 reversals
    assert st["max"] == pytest.approx(0.2) and st["at_upper_slew"] == pytest.approx(2 / 5)
    assert st["at_lower_slew"] == 0.0
    assert st["rms"] == pytest.approx(np.sqrt(np.mean(np.diff(values) ** 2)))


def test_published_violations_flag_a_slew_jump():
    limits = SHORT.core_config().limits
    ok = np.array([[0.0, 0.0], [0.1, 0.2], [0.2, 0.3]])
    assert ol.published_violations(ok, limits, 0.05)["ticks_violating"] == 0
    bad = np.array([[0.0, 0.0], [0.1, 0.5]])  # +0.5 rad/s in one tick > 0.2
    rep = ol.published_violations(bad, limits, 0.05)
    assert rep["ticks_violating"] == 1 and rep["max"]["angular_slew"] == pytest.approx(0.3 / 0.05)


def test_zero_centred_yaw_effort_pushes_an_ideal_unicycle_outside_the_circle():
    """The screened effect (plan, 2026-11-19): r_omega (omega - 0)^2 biases the plan toward straight
    motion on a curved path; without it the ideal plant tracks the circle. Cheap PSO size."""
    base = ol.LoopConfig(path="circle", size=1.0, speed=0.2, pso="3x5", seed=1)
    with_effort = ol.summarize(ol.run_loop(replace(base, r_omega=5e-5)))
    without = ol.summarize(ol.run_loop(replace(base, r_omega=0.0)))
    assert with_effort["published_violations"] == 0 and without["published_violations"] == 0
    assert with_effort["radial_offset_mean"] > 0.1 and with_effort["cross_track_rmse"] > 0.1
    assert abs(without["radial_offset_mean"]) < 0.03 and without["cross_track_rmse"] < 0.03


def test_csv_row_flattens_the_smoothness_metrics():
    s = ol.summarize(ol.run_loop(SHORT))
    row = ol.csv_row(s)
    assert row["r_omega"] == SHORT.r_omega and row["domega_rms"] == s["smoothness"]["domega"]["rms"]
    assert set(row) == set(ol.CSV_FIELDS)
    assert s["plant"] == ol.PLANT_LABEL and um is not None


def prediction_errors(res: ol.LoopResult) -> np.ndarray:
    """|core estimate of the pose at t + Ts - plant pose at t + Ts| for every owned solve."""
    truth = {round(r["stamp"], 6): r for r in res.truth}
    out = []
    for r in res.ticks:
        if r["estimate"] is None:
            continue
        actual = truth.get(round(r["t"] + 0.05, 6))
        if actual is not None:
            out.append(np.hypot(r["estimate"][0] - actual["x"], r["estimate"][1] - actual["y"]))
    return np.array(out)


def test_plant_delay_of_one_tick_matches_the_cores_actuation_prediction():
    """MpcCore predicts t -> t + Ts with the applied command only: exact when the plant executes
    a new command one tick late, one command behind when it executes it at once."""
    late = prediction_errors(ol.run_loop(replace(SHORT, path="circle", size=1.0, plant_delay=0.05)))
    prompt = prediction_errors(ol.run_loop(replace(SHORT, path="circle", size=1.0, plant_delay=0.0)))
    assert len(late) > 50 and late.max() < 1e-9
    assert prompt.max() > 1e-4
    assert "d50ms" in replace(SHORT, plant_delay=0.05).run_id
