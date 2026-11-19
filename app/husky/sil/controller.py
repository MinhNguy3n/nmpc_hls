"""ROS-free core of the SIL MPC node: ownership, applied command, state estimate, solve, checks,
fallbacks and the deadline accounting C. The rclpy node (sil_node.py) only moves messages in and
out; everything here is driven by explicit times and is tested offline.

Timing (one-period scheme, plan section 3.1): at each tick t the core first decides what to
publish AT t (the command computed at the previous tick, or the arming probe), then freezes its
inputs and computes the command for t + Ts. C = t_ready - t_tick is measured both in simulation
time (the deadline that counts in SIL) and in wall-clock time (reported separately; Python timing
is not a hardware claim).

Signals are kept distinct:
- feedback: platform/odom/filtered (EKF with the IMU bridge) -> the only state input;
- diagnostic: wheel odometry (logged by the node, never used here);
- scoring: Gazebo ground truth (logged by the node, never passed to this module).

Every threshold below is PROVISIONAL until V13 is measured under active publishing (plan V13-active).
"""

from __future__ import annotations

from dataclasses import dataclass, field
import math
from typing import Callable

import numpy as np

from . import _HUSKY  # noqa: F401
from . import pso as pso_mod
from .reference import PathSpec, reference_window
import command_ownership as co
import unicycle_model as um


@dataclass(frozen=True)
class OdomSample:
    """platform/odom/filtered: pose and twist with their stamp and receive time."""

    stamp: float
    received: float
    x: float
    y: float
    yaw: float
    v: float
    omega: float


@dataclass(frozen=True)
class CoreConfig:
    ts: float = 0.05
    horizon: int = 25
    control_horizon: int = 25
    limits: um.UnicycleLimits = field(default_factory=lambda: um.limits_from_parameter_set(
        "husky_a300", v_min=0.0, v_max=0.3, omega_max=0.75))  # PLACEHOLDER_ SIL envelope (slow mode)
    pso: pso_mod.PsoConfig = field(default_factory=lambda: pso_mod.PsoConfig(4, 50))
    cost: pso_mod.CostConfig = field(default_factory=pso_mod.CostConfig)
    gain_mode: str = "plain"
    path: PathSpec = field(default_factory=PathSpec)
    deadline: float = 0.040  # C budget (s), provisional
    max_odom_age: float = 0.06  # provisional: 3 EKF periods at 50 Hz
    # provisional: 5 IMU periods at 100 Hz; None disables. Without the IMU, odom/filtered keeps
    # publishing but silently follows wheel odometry (V14, 2026-11-19), so a fresh filtered
    # message alone does not prove the feedback is the one the run is scored on.
    max_imu_age: float | None = 0.05
    fallback_steps: int = 3  # PLACEHOLDER_ policy
    ownership: co.OwnershipConfig = field(default_factory=co.OwnershipConfig)


@dataclass
class TickRecord:
    """Everything the node logs for one tick (feedback, decisions and timing; no ground truth)."""

    t: float
    state: str
    publish: tuple[float, float] | None  # command published AT this tick
    publish_kind: str  # plan | fallback | stop | probe | none
    u_prev: tuple[float, float] | None  # the applied command (cmd_vel_out)
    estimate: tuple[float, float, float] | None
    odom_age: float | None
    solved: bool = False
    accepted: bool = False
    fitness: float | None = None
    violations: dict[str, float] = field(default_factory=dict)
    fallback: str = ""
    c_sim: dict[str, float] = field(default_factory=dict)
    c_wall: dict[str, float] = field(default_factory=dict)
    deadline_miss: bool = False
    note: str = ""
    u_curr: tuple[float, float] | None = None  # the command in force when the new plan starts


class MpcCore:
    def __init__(self, config: CoreConfig, seed: int = 0,
                 solver: Callable[..., pso_mod.Solution] | None = None) -> None:
        self.config = config
        self.sm = co.OwnershipStateMachine(config.ownership)
        self.applied = co.AppliedCommandTracker(config.ownership.max_applied_age)
        self.rng = np.random.default_rng(seed)
        self.solver = solver or pso_mod.solve
        self.gain = pso_mod.yaw_rate_gain(config.gain_mode)
        self.odom: OdomSample | None = None
        self.imu_stamp: float | None = None
        self.pending: tuple[np.ndarray, str] | None = None  # command for the next tick
        self.last_plan: np.ndarray | None = None
        self.fallback_used = 0
        self.origin: np.ndarray | None = None  # path origin (odom frame), set at the first owned tick
        self.tau0: float | None = None
        self._wall0 = 0.0
        self._published: tuple[float, float] | None = None

    # ------------------------------------------------------------------ inputs
    def on_odom(self, sample: OdomSample) -> None:
        if self.odom is None or sample.stamp > self.odom.stamp:
            self.odom = sample

    def on_imu(self, stamp: float) -> None:
        if self.imu_stamp is None or stamp > self.imu_stamp:
            self.imu_stamp = stamp

    def on_applied(self, sample: co.AppliedSample) -> None:
        self.applied.on_sample(sample)

    def on_mux(self, stamp_ns: int, v: float, omega: float, received: float) -> None:
        self.sm.on_mux_output(stamp_ns, v, omega, received)

    def on_lock(self, name: str, engaged: bool, received: float) -> None:
        self.sm.on_lock(name, engaged, received)

    def in_envelope(self, u: np.ndarray) -> bool:
        return bool(um.check_controls(np.asarray(u)[None, :], u, self.config.limits, self.config.ts)["ok"])

    def imu_problem(self, t: float) -> str:
        """Why the IMU does not back the filtered state at t ('' when it does or the gate is off)."""
        if self.config.max_imu_age is None:
            return ""
        if self.imu_stamp is None or t - self.imu_stamp > self.config.max_imu_age:
            return "IMU stale or missing: the filtered heading would no longer follow the gyro"
        return ""

    def request_arm(self, now: float) -> tuple[bool, str]:
        """Refused while the feedback or the IMU is stale or missing (a new subscriber can take
        seconds to be matched in DDS discovery), then the ownership preconditions."""
        odom_age = None if self.odom is None else now - self.odom.stamp
        if odom_age is None or not (0.0 <= odom_age <= self.config.max_odom_age):
            return False, "filtered odometry stale or missing"
        problem = self.imu_problem(now)
        if problem:
            return False, problem
        return self.sm.request_arm(now, self.applied, self.in_envelope)

    def disarm(self, now: float, reason: str = "operator disarm") -> None:
        self.sm.disarm(now, reason)

    # ------------------------------------------------------------------ tick
    def tick(self, t: float, stamp_ns: int, sim_clock: Callable[[], float],
             wall_clock: Callable[[], float]) -> TickRecord:
        """begin_tick + finish_tick (offline use; the node publishes in between)."""
        return self.finish_tick(self.begin_tick(t, stamp_ns, wall_clock), sim_clock, wall_clock)

    def begin_tick(self, t: float, stamp_ns: int, wall_clock: Callable[[], float]) -> TickRecord:
        """Ownership update and the command to publish AT t (record.publish). stamp_ns is the stamp
        the node puts on that TwistStamped (strictly increasing). Publish it before finish_tick."""
        self._wall0 = wall_clock()
        state = self.sm.tick(t)
        if self.sm.take_warm_start_reset():
            self.last_plan, self.pending, self.fallback_used = None, None, 0
        if state in (co.State.DISARMED, co.State.FOREIGN):
            self.origin = self.tau0 = None  # a new arming starts a new path episode
        u_prev = self.applied.u_prev(t)
        odom_age = None if self.odom is None else t - self.odom.stamp
        record = TickRecord(t, state.value, None, "none", None if u_prev is None else tuple(u_prev), None, odom_age)

        # ---- 1. what to publish at t
        if state == co.State.ARMING and u_prev is not None:
            record.publish, record.publish_kind = (float(u_prev[0]), float(u_prev[1])), "probe"
        elif state == co.State.OWNED and self.pending is not None:
            command, kind = self.pending
            record.publish, record.publish_kind = (float(command[0]), float(command[1])), kind
        if record.publish is not None:
            self.sm.on_published(stamp_ns, record.publish[0], record.publish[1], t)
        self._published = record.publish
        self.pending = None
        return record

    def finish_tick(self, record: TickRecord, sim_clock: Callable[[], float],
                    wall_clock: Callable[[], float]) -> TickRecord:
        """Compute the command for t + Ts (owned only). sim_clock / wall_clock measure C."""
        cfg = self.config
        t, wall0 = record.t, self._wall0
        if record.state != co.State.OWNED.value:
            return record
        u_prev = None if record.u_prev is None else np.array(record.u_prev)
        odom_age = record.odom_age

        # ---- 2. safety-data gate: stale feedback or applied command -> disarm, never a cached plan
        if u_prev is None or self.odom is None or odom_age is None or not (0.0 <= odom_age <= cfg.max_odom_age):
            reason = "applied command stale" if u_prev is None else "filtered odometry stale or missing"
            self.sm.disarm(t, reason)
            record.state, record.note = self.sm.state.value, reason
            return record
        reason = self.imu_problem(t)
        if reason:
            self.sm.disarm(t, reason)
            record.state, record.note = self.sm.state.value, reason
            return record
        if not self.in_envelope(u_prev):
            self.sm.disarm(t, "applied command outside the MPC envelope")
            record.state, record.note = self.sm.state.value, "applied command outside the MPC envelope"
            return record

        # ---- u_curr: the plan computed now starts at t + Ts, after the command published at t. That
        # command is used as the slew base only if it is reachable in one step from the APPLIED
        # command (cmd_vel_out); otherwise (limiter active, lag, mismatch) the applied command is.
        u_curr = u_prev
        if self._published is not None and um.check_controls([self._published], u_prev, cfg.limits, cfg.ts)["ok"]:
            u_curr = np.array(self._published)
        elif self._published is not None:
            record.note = "published command not reachable from the applied one: solving from cmd_vel_out"
        record.u_curr = (float(u_curr[0]), float(u_curr[1]))

        # ---- 3. state estimate at the actuation time t + Ts (measured twist, then applied command)
        prep0_sim, prep0_wall = sim_clock(), wall_clock()
        o = self.odom
        _, estimate = um.estimate_state_at_actuation(
            [o.x, o.y, o.yaw], [o.v, o.omega], o.stamp, t, t + cfg.ts, [[t, u_prev[0], u_prev[1]]],
            cfg.max_odom_age, wrap=False)
        record.estimate = tuple(float(e) for e in estimate)
        if self.origin is None:  # first owned tick: the path starts here
            self.origin, self.tau0 = estimate.copy(), t + cfg.ts
        tau = t + cfg.ts - self.tau0
        xref = reference_window(cfg.path, self.origin, tau, cfg.horizon, cfg.ts, estimate)
        warm = None
        if self.last_plan is not None:
            warm = um.project_controls(um.shift_warm_start(self.last_plan), u_curr, cfg.limits, cfg.ts)
        problem = pso_mod.Problem(np.zeros(3), xref, u_curr, cfg.limits, cfg.ts, cfg.control_horizon, warm, self.gain)
        prep1_sim, prep1_wall = sim_clock(), wall_clock()

        # ---- 4. solve
        solution = self.solver(problem, cfg.pso, cfg.cost, self.rng)
        solve1_sim, solve1_wall = sim_clock(), wall_clock()
        record.solved, record.fitness = True, solution.fitness

        # ---- 5. host-side checks (each violation separately) and the deadline
        report = um.check_controls(solution.plan, u_curr, cfg.limits, cfg.ts)
        states_ok = bool(np.all(np.isfinite(um.rollout(np.zeros(3), solution.plan, cfg.ts, cfg.horizon, wrap=False))))
        record.violations = {k: float(v) for k, v in report.items() if isinstance(v, float)}
        ready_sim, ready_wall = sim_clock(), wall_clock()
        record.c_sim = {"prep": prep1_sim - prep0_sim, "solve": solve1_sim - prep1_sim,
                        "check": ready_sim - solve1_sim, "total": ready_sim - t}
        record.c_wall = {"prep": prep1_wall - prep0_wall, "solve": solve1_wall - prep1_wall,
                         "check": ready_wall - solve1_wall, "total": ready_wall - wall0}
        record.deadline_miss = record.c_sim["total"] > cfg.deadline
        if report["ok"] and states_ok and not record.deadline_miss:
            record.accepted = True
            self.last_plan, self.fallback_used = solution.plan, 0
            self.pending = (solution.plan[0], "plan")
            return record

        # ---- 6. fallback (only here: owned, fresh feedback and applied command)
        record.fallback = "deadline" if record.deadline_miss else "check"
        if self.last_plan is not None and self.fallback_used < cfg.fallback_steps:
            shifted = um.shift_warm_start(self.last_plan)
            if um.check_controls(shifted, u_curr, cfg.limits, cfg.ts)["ok"]:
                self.last_plan, self.fallback_used = shifted, self.fallback_used + 1
                self.pending = (shifted[0], "fallback")
                record.fallback += ": next element of the last accepted plan"
                return record
        stop = um.stop_candidate(u_curr, cfg.control_horizon, cfg.limits, cfg.ts)
        self.last_plan, self.pending = None, (stop[0], "stop")
        record.fallback += ": stop candidate (best effort, not a safety stop)"
        return record


def deadline_summary(records: list[TickRecord], budget: float) -> dict[str, float]:
    """C statistics over solved ticks, simulation and wall clock kept apart."""
    solved = [r for r in records if r.solved]
    if not solved:
        return {"solved": 0}
    sim = np.array([r.c_sim["total"] for r in solved])
    wall = np.array([r.c_wall["total"] for r in solved])
    return {"solved": len(solved), "misses": int(np.sum(sim > budget)),
            "c_sim_median": float(np.median(sim)), "c_sim_p95": float(np.percentile(sim, 95)),
            "c_sim_max": float(sim.max()), "c_wall_median": float(np.median(wall)),
            "c_wall_p95": float(np.percentile(wall, 95)), "c_wall_max": float(wall.max()),
            "solve_wall_median": float(np.median([r.c_wall["solve"] for r in solved]))}


def rtf_for_deadline(wall_cycle: float, budget: float = 0.040, margin: float = 0.8) -> float:
    """Largest Gazebo real-time factor at which a cycle taking wall_cycle seconds of wall time still
    meets the simulation-time budget with the given margin: rtf * wall_cycle <= margin * budget."""
    if wall_cycle <= 0:
        return 1.0
    return min(1.0, margin * budget / wall_cycle)
