"""Offline tests of the SIL core (no ROS): PSO structure, reference paths, the controller's
ownership gating, state gate, fallbacks, deadline accounting, the ground-truth pump and scoring."""

from __future__ import annotations

import io
import json
import math
from pathlib import Path
import sys
import time

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from sil import pso, reference  # noqa: E402
from sil.controller import CoreConfig, MpcCore, OdomSample, rtf_for_deadline  # noqa: E402
from sil.gz_truth import pump  # noqa: E402
from sil.logs import EPISODE_FIELDS, SIGNAL_FIELDS, TICK_FIELDS, TRANSITION_FIELDS, CsvLog, tick_row, write_meta  # noqa: E402
from sil import metrics  # noqa: E402
import command_ownership as co  # noqa: E402
import unicycle_model as um  # noqa: E402

TS = 0.05
LIMITS = um.limits_from_parameter_set("husky_a300", v_min=0.0, v_max=0.3, omega_max=0.75)  # PLACEHOLDER_ envelope


def straight_problem(nu: int = 25, warm=None, gain="plain") -> pso.Problem:
    xref = np.column_stack([0.3 * TS * np.arange(1, 26), np.zeros(25), np.zeros(25)])
    return pso.Problem(np.zeros(3), xref, np.array([0.3, 0.0]), LIMITS, TS, nu, warm, pso.yaw_rate_gain(gain))


# ---------------------------------------------------------------------------------------- PSO
def test_pso_config_parse_and_inertia_schedule():
    cfg = pso.PsoConfig.parse("7x30")
    assert (cfg.particles, cfg.iterations) == (7, 30)
    assert cfg.w0 + cfg.slope * cfg.iterations == pytest.approx(cfg.wf)
    with pytest.raises(ValueError):
        pso.PsoConfig(1, 10)


def test_pso_solution_is_feasible_and_tracks_a_straight_line():
    for text in ("4x50", "7x30", "9x24"):
        sol = pso.solve(straight_problem(), pso.PsoConfig.parse(text), pso.CostConfig(), np.random.default_rng(1))
        assert um.check_controls(sol.plan, [0.3, 0.0], LIMITS, TS)["ok"]
        assert np.all(np.diff(sol.best_fitness_history) <= 1e-12)  # global best never worsens
        assert abs(sol.plan[0, 1]) < 0.2 and sol.plan[0, 0] > 0.2


def test_pso_is_deterministic_for_a_seed_and_keeps_the_warm_start_when_best():
    problem = straight_problem()
    a = pso.solve(problem, pso.PsoConfig(4, 10), pso.CostConfig(), np.random.default_rng(5))
    b = pso.solve(problem, pso.PsoConfig(4, 10), pso.CostConfig(), np.random.default_rng(5))
    assert np.array_equal(a.plan, b.plan)
    perfect = np.tile([0.3, 0.0], (25, 1))  # exactly the reference: cost 0 at particle 0
    sol = pso.solve(straight_problem(warm=perfect), pso.PsoConfig(4, 1), pso.CostConfig(), np.random.default_rng(0))
    assert np.allclose(sol.plan, perfect) and sol.fitness == pytest.approx(0.0, abs=1e-12)


def test_pso_stable_zero_particle_and_du_projection():
    problem = straight_problem()
    du = um.normalized_slew_limits(LIMITS, TS)
    z_prev = um.normalize_controls([0.3, 0.0], LIMITS)[0]
    x = np.random.default_rng(0).uniform(-300, 300, (3, 25, 2))
    projected = pso._du_project(x, z_prev, du)
    steps = np.diff(np.concatenate([np.tile(z_prev, (3, 1, 1)), projected], axis=1), axis=1)
    assert np.all(np.abs(steps) <= du + 1e-9) and np.all(np.abs(projected) <= 100)
    # Stable zero: with a single iteration and no warm start, particle 1 decelerates at the du limit.
    seen = {}
    original = pso.cost

    def spy(prob, z, weights, uss):
        seen.setdefault("x", z.copy())
        return original(prob, z, weights, uss)

    pso.cost = spy
    try:
        pso.solve(problem, pso.PsoConfig(3, 1), pso.CostConfig(), np.random.default_rng(0))
    finally:
        pso.cost = original
    stable = um.denormalize_controls(seen["x"][1], LIMITS)
    dv = min(LIMITS.accel_max, LIMITS.decel_max) * TS
    assert stable[0, 0] == pytest.approx(0.3 - dv) and np.allclose(stable[-1], [0.0, 0.0])


def test_predictor_gain_option():
    z = um.normalize_controls(np.tile([0.0, 0.5], (25, 1)), LIMITS)[None]
    plain = pso.predict(straight_problem(gain="plain"), z)[0, -1, 2]
    sim = pso.predict(straight_problem(gain="sim-derived"), z)[0, -1, 2]
    assert sim == pytest.approx(pso.SIM_DERIVED_GAIN_IN_PLACE * plain)
    z_arc = um.normalize_controls(np.tile([0.3, 0.3], (25, 1)), LIMITS)[None]
    arc_plain = pso.predict(straight_problem(gain="plain"), z_arc)[0, -1, 2]
    arc_sim = pso.predict(straight_problem(gain="sim-derived"), z_arc)[0, -1, 2]
    assert arc_sim == pytest.approx(pso.SIM_DERIVED_GAIN_ARC * arc_plain)
    with pytest.raises(ValueError):
        pso.yaw_rate_gain("pooled")


def test_rk4_predictor_matches_the_reference_model():
    controls = np.column_stack([np.linspace(0.1, 0.3, 25), np.linspace(-0.5, 0.5, 25)])
    z = um.normalize_controls(controls, LIMITS)[None]
    states = pso.predict(straight_problem(), z)[0]
    expected = um.rollout(np.zeros(3), controls, TS, method="rk4", wrap=False)[1:]
    assert np.allclose(states, expected, atol=1e-12)


# ---------------------------------------------------------------------------------- reference
def test_reference_paths_are_continuous_and_closed():
    for kind in ("line", "circle", "figure8"):
        spec = reference.PathSpec(kind, 0.3, 1.0)
        poses = spec.poses(np.arange(0, spec.duration, 0.01))
        steps = np.hypot(*np.diff(poses[:, :2], axis=0).T)
        assert np.all(steps <= 0.3 * 0.01 + 1e-9)
        assert np.all(np.abs(np.diff(poses[:, 2])) < 0.01)  # heading continuous (unwrapped)
        assert np.allclose(spec.pose(spec.duration + 5.0), spec.pose(spec.duration))  # holds the end
    end = reference.PathSpec("figure8", 0.3, 1.0).pose(1e9)
    assert np.allclose(end[:2], 0.0, atol=1e-9)
    assert reference.cross_track(np.array([[0.0, 0.5]]), np.array([[0, 0], [1, 0.0]]))[0] == pytest.approx(0.5)


def test_reference_window_is_in_the_local_frame():
    spec = reference.PathSpec("line", 0.3, 1.0)
    origin = np.array([2.0, 1.0, math.pi / 2])
    window = reference.reference_window(spec, origin, 0.0, 25, TS, origin)
    assert np.allclose(window[:, 1], 0.0, atol=1e-12) and np.allclose(window[:, 0], 0.3 * TS * np.arange(1, 26))


# --------------------------------------------------------------------------------- controller
class FakeRobot:
    """Echoes published commands through a fake mux and supplies filtered odometry and cmd_vel_out."""

    def __init__(self, core: MpcCore) -> None:
        self.core, self.t, self.pose = core, 10.0, np.zeros(3)
        self.u = np.zeros(2)
        self.imu = True

    def feed(self) -> None:
        self.core.on_odom(OdomSample(self.t - 0.01, self.t, *self.pose, *self.u))
        if self.imu:
            self.core.on_imu(self.t - 0.005)
        self.core.on_applied(co.AppliedSample(self.t - 0.005, self.t, *self.u))

    def tick(self, echo: bool = True, solve_time: float = 0.0):
        self.feed()
        stamp = int(round(self.t * 1e9))
        clock = {"t": self.t}

        def sim_clock():
            return clock["t"]

        def slow_solver(*a, **k):
            clock["t"] += solve_time
            return pso.solve(*a, **k)

        if solve_time:
            self.core.solver = slow_solver
        record = self.core.begin_tick(self.t, stamp, time.perf_counter)
        if record.publish is not None:
            if echo:
                self.core.on_mux(stamp, record.publish[0], record.publish[1], self.t + 0.003)
            self.u = np.array(record.publish)
        self.core.finish_tick(record, sim_clock, time.perf_counter)
        self.pose = um.step_chord(self.pose, self.u, TS, wrap=False)
        self.t += TS
        return record


def make_core(**kw) -> MpcCore:
    cfg = CoreConfig(limits=LIMITS, pso=pso.PsoConfig(4, 10), path=reference.PathSpec("line", 0.3, 1.0), **kw)
    return MpcCore(cfg, seed=3)


def test_controller_publishes_only_when_owned_and_tracks():
    core = make_core()
    robot = FakeRobot(core)
    assert robot.tick().publish is None  # disarmed: nothing published
    robot.feed()
    assert core.request_arm(robot.t)[0]
    kinds = [robot.tick().publish_kind for _ in range(40)]
    assert kinds[:2] == ["probe", "probe"] and "plan" in kinds
    assert core.sm.state == co.State.OWNED
    assert robot.pose[0] > 0.3  # it drives along the line


def test_controller_foreign_takeover_stops_publishing_and_resets():
    core = make_core()
    robot = FakeRobot(core)
    robot.feed()
    core.request_arm(robot.t)
    for _ in range(10):
        robot.tick()
    core.on_mux(co.stamp_ns(999, 0), 0.0, 0.5, robot.t)  # joystick takes over
    records = [robot.tick(echo=False) for _ in range(10)]
    assert all(r.publish is None for r in records)
    assert core.last_plan is None and core.sm.state in (co.State.FOREIGN, co.State.DISARMED)


def test_controller_stale_feedback_disarms_without_cached_plan():
    core = make_core()
    robot = FakeRobot(core)
    robot.feed()
    core.request_arm(robot.t)
    for _ in range(8):
        robot.tick()
    assert core.sm.state == co.State.OWNED
    core.odom = OdomSample(robot.t - 1.0, robot.t - 1.0, 0, 0, 0, 0, 0)  # feedback stops
    stamp = int(round(robot.t * 1e9))
    record = core.begin_tick(robot.t, stamp, time.perf_counter)
    core.on_mux(stamp, *record.publish, robot.t)
    core.finish_tick(record, lambda: robot.t, time.perf_counter)
    assert core.sm.state == co.State.DISARMED and "stale" in record.note
    robot.t += TS
    assert core.begin_tick(robot.t, int(round(robot.t * 1e9)), time.perf_counter).publish is None



def test_controller_disarms_when_the_imu_stops():
    core = make_core()
    robot = FakeRobot(core)
    robot.feed()
    core.request_arm(robot.t)
    for _ in range(8):
        robot.tick()
    assert core.sm.state == co.State.OWNED
    robot.imu = False  # the bridge stops; filtered odometry keeps arriving (wheel only)
    records = [robot.tick() for _ in range(4)]
    assert core.sm.state == co.State.DISARMED and any("IMU" in r.note for r in records)
    assert records[-1].publish is None
    robot.feed()  # fresh odometry and applied command, still no IMU
    ok, why = core.request_arm(robot.t)  # re-arming is refused while the IMU is missing
    assert not ok and "IMU" in why
    core = make_core(max_imu_age=None)  # gate disabled
    robot = FakeRobot(core)
    robot.imu = False
    robot.feed()
    core.request_arm(robot.t)
    assert all("IMU" not in robot.tick().note for _ in range(10)) and core.sm.state == co.State.OWNED


def test_controller_deadline_miss_uses_fallback_then_stop():
    core = make_core()
    robot = FakeRobot(core)
    robot.feed()
    core.request_arm(robot.t)
    for _ in range(8):
        robot.tick()
    kinds = []
    for _ in range(6):
        record = robot.tick(solve_time=0.045)  # 45 ms of simulation time per solve: misses C <= 40 ms
        assert record.deadline_miss and record.c_sim["total"] == pytest.approx(0.045)
        kinds.append(record.fallback.split(":")[1].strip().split(" ")[0])
    assert kinds[:3] == ["next", "next", "next"] and kinds[3] == "stop"


def test_rtf_for_deadline():
    assert rtf_for_deadline(0.2) == pytest.approx(0.16)
    assert rtf_for_deadline(0.01) == 1.0


# ------------------------------------------------------------------------- truth and scoring
GZ_TEXT = """header {
  stamp {
    sec: 5
  }
}
pose {
  name: "a300_00000/robot"
  position {
    x: 1
  }
  orientation {
    w: 1
  }
}
pose {
  name: "box"
}
"""


def test_gz_truth_pump_selects_the_robot():
    rows = []
    n = pump(io.StringIO(GZ_TEXT), "a300_00000/robot", lambda: 5.01, lambda *r: rows.append(r))
    assert n == 1 and rows[0] == (5.0, 5.01, 1.0, 0.0, 0.0, 0.0, 0.0)  # ..., yaw, z, tilt
    tipped = GZ_TEXT.replace("    w: 1", "    x: 0.5315\n    y: 0.1449\n    z: 0.0095\n    w: 0.8345")  # 2026-11-19
    rows.clear()
    pump(io.StringIO(tipped), "a300_00000/robot", lambda: 5.01, lambda *r: rows.append(r))
    assert rows[0][6] == pytest.approx(1.17, abs=0.01)  # 67 deg from upright


def test_metrics_validity_flags_a_tipped_robot():
    upright = [{"stamp": t, "tilt": 0.01} for t in (1.0, 2.0)]
    assert metrics.validity(upright)["valid"]
    result = metrics.validity(upright + [{"stamp": 3.0, "tilt": 1.2}, {"stamp": 4.0, "tilt": 1.2}])
    assert result["valid"] is False and result["tipped_at"] == 3.0
    assert metrics.validity([{"stamp": 1.0, "tilt": None}])["valid"] is None  # older logs: unknown


def test_metrics_report_separates_result_families(tmp_path):
    spec = reference.PathSpec("line", 0.3, 1.0)
    run = tmp_path / "run"
    run.mkdir()
    write_meta(run / "meta.json", {"run_id": "t", "path": {"kind": "line", "speed": 0.3, "size": 1.0},
                                   "limits": {k: getattr(LIMITS, k) for k in LIMITS.__dataclass_fields__},
                                   "ts": TS, "deadline": 0.04, "pso": "4x10", "gain_mode": "plain", "seed": 1})
    core = make_core()
    robot = FakeRobot(core)
    robot.feed()
    core.request_arm(robot.t)
    ticks = CsvLog(run / "ticks.csv", TICK_FIELDS)
    truth = CsvLog(run / "truth.csv", SIGNAL_FIELDS)
    feedback = CsvLog(run / "feedback.csv", SIGNAL_FIELDS)
    for _ in range(int(spec.duration / TS) + 20):
        record = robot.tick()
        ticks.write(tick_row(record))
        truth.write({"stamp": robot.t, "received": robot.t, "x": robot.pose[0] + 5, "y": robot.pose[1] - 2, "yaw": robot.pose[2]})
        feedback.write({"stamp": robot.t, "received": robot.t + 0.002, "x": robot.pose[0], "y": robot.pose[1], "yaw": robot.pose[2]})
    for log in (ticks, truth, feedback):
        log.close()
    episodes = CsvLog(run / "episodes.csv", EPISODE_FIELDS)
    episodes.write({"episode": 1, "tau0": core.tau0, "origin_x": core.origin[0], "origin_y": core.origin[1], "origin_yaw": core.origin[2]})
    episodes.close()
    CsvLog(run / "transitions.csv", TRANSITION_FIELDS).close()
    rep = metrics.report(run)
    track = rep["tracking_vs_ground_truth"][0]
    # the robot starts from rest while the time-indexed reference starts at 0.3 m/s: a lag, small cross-track
    assert track["pos_rmse"] < 0.2 and track["cross_track_rmse"] < track["pos_rmse"]
    assert rep["constraint_violations"]["published_ticks_violating"] == 0
    assert rep["deadline"]["solved"] > 0 and rep["deadline"]["misses_sim"] == 0
    assert rep["diagnostic_feedback_vs_truth"]["drift_pos_max"] < 1e-9
    assert metrics.main([str(run), "--out", str(run / "report.json")]) == 0
    assert json.loads((run / "report.json").read_text())["run"] == "t"
    # no transitions and no tilt in this synthetic run: rejected, with every reason listed
    assert rep["acceptance"] == {"accepted": False, "reasons": ["never owned", "validity unknown"],
                                 "note": rep["acceptance"]["note"]}
    assert rep["deadline"]["c_wall_over_period"] == 0 and rep["executor"]["late_ticks"] == 0
    from sil import batch_summary
    assert batch_summary.main([str(run / "report.json"), str(tmp_path / "summary.csv")]) == 0
    row = next(iter(__import__("csv").DictReader(open(tmp_path / "summary.csv"))))
    assert row["verdict"] == "REJECT" and row["reasons"] == "never owned; validity unknown"


def test_acceptance_rejects_each_hard_failure():
    base = {"ownership_transitions": [{"old": "arming", "new": "owned", "reason": "ownership confirmed"},
                                      {"old": "owned", "new": "disarmed", "reason": "run finished"}],
            "validity": {"valid": True}, "constraint_violations": {"published_ticks_violating": 0},
            "deadline": {"solved": 400, "misses_sim": 4}, "fallbacks": {"published_kinds": {"stop": 0}},
            "tracking_vs_ground_truth": [{"cross_track_rmse": 0.05, "cross_track_max": 0.2}]}
    assert metrics.acceptance(base)["accepted"]
    bad = json.loads(json.dumps(base))
    bad["ownership_transitions"][1]["reason"] = "IMU stale or missing"
    bad["constraint_violations"]["published_ticks_violating"] = 1
    bad["deadline"]["misses_sim"] = 5
    bad["fallbacks"]["published_kinds"]["stop"] = 2
    bad["tracking_vs_ground_truth"][0]["cross_track_max"] = 0.3
    bad["validity"]["valid"] = False
    assert metrics.acceptance(bad)["reasons"] == [
        "ownership lost: IMU stale or missing", "robot tipped", "published constraint violation",
        "C_sim misses > 1 %", "stop fallback", "tracking above the provisional thresholds"]
