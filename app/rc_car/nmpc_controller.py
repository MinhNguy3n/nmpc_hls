"""NMPC arm of the RC car study: the host side around the emulated engine (plain Python, tested).

SIMULATION ONLY. Per control tick (20 Hz, simulation time):
1. TIMING MODEL (tick-anchored, as the real KV260 loop; STOP 4a review section 2.1): at each tick
   t_k (every Ts = 50 ms) the newest mocap pose is taken (capture stamp c, age a_k = t_k - c) and
   the solve STARTS at t_k, or when the single solver becomes free if a previous solve is still
   running (one KV260 solver: solves never overlap). The command is applied at
   t_apply = t_start + solve_time(board), the board-measured solver + PS median of the
   configuration (10x60: extrapolated from the board's 10x40 measurement; no 10x60 board run).
   A DEADLINE MISS is t_apply > t_k + Ts (never with the board times; the counter stays). A pose
   older than one period at the tick is counted as a STALE POSE AT TICK (the freshness gate and
   the prediction handle it; it is not a deadline miss). Python's own solve time is not part of
   this model; the node only checks that Python finished before t_apply (overruns).
2. DELAY COMPENSATION (on by design; "off" skips step 2 with the same timing): the state is
   predicted from the pose at c to the time the new command takes effect at the wheels,
   t_eff = t_apply + chain_term(profile) + gazebo_term, a span of a_k + solve_time + chain +
   Gazebo that varies with a_k but is known exactly every step, through the buffer of commands
   already sent (each in effect from its own t_eff), with the exact-arc kinematics in double.
3. PATH-FOLLOWING REFERENCE (not a time schedule): the predicted pose is projected onto the path
   (forward-only local search) and the next Nh reference points are taken from there at the
   commanded speed, with v_ref = the commanded speed and kappa_ref = the path curvature. Both
   arms track the same geometric path and are scored on the same cross-track error.
4. Warm start (host, double precision, docs/ackermann/ackermann_mpc_pso_model.md section 9.2):
   the cheaper of the shifted previous plan and the projected feed-forward of the reference.
5. Solve with the emulated engine (nmpc_engine.solve; the PSO RNG is seeded from the episode
   seed, so a repeated seed reproduces the run). The plan is checked in double precision
   (check_controls); a failing plan is replaced by the warm start (a counted fallback).
6. Output: speed v and bicycle steering delta = atan(L kappa) of the plan's first input.

Limits come from the same files as the plant: delta_max, wheelbase and v_max_sim from
params/reely_new1.yaml; delta_rate_max, accel_max, decel_max from params/command_chain.yaml
(vehicle_limits_profile). Nothing comes from the imported October code's examples.
"""

from __future__ import annotations

from dataclasses import dataclass, field
import math
from pathlib import Path
import sys
from typing import Optional

import numpy as np
import yaml

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import nmpc_engine as ne  # noqa: E402
import paths as P  # noqa: E402
import vehicle_params as vp  # noqa: E402
from nmpc_engine import am  # noqa: E402
from pose_input import Pose  # noqa: E402

DEFAULT_NMPC = HERE / "params" / "nmpc.yaml"
DEFAULT_CHAIN = HERE / "params" / "command_chain.yaml"
PATH_TRIM = 0.3  # m, as checks/track_run.py: the episode ends near length - PATH_TRIM


@dataclass(frozen=True)
class NmpcConfig:
    limits: am.AckermannLimits
    wheelbase: float
    delta_max: float
    nh: int
    nu: int
    ts: float
    sizes: dict
    pso: dict
    weights: am.CostWeights
    weights_status: str
    gazebo_term: float
    chain_term: dict
    end_tolerance: float


def weights_from(d: dict, q_pos: float | None = None, q_psi: float | None = None) -> am.CostWeights:
    qp = float(d["q_pos"] if q_pos is None else q_pos)
    qh = float(d["q_psi"] if q_psi is None else q_psi)
    ratio = float(d["terminal_ratio"])
    return am.CostWeights(q_pos=qp, q_psi=qh, q_v=float(d["q_v"]), r_kappa=float(d["r_kappa"]),
                          s_dv=float(d["s_dv"]), s_dkappa=float(d["s_dkappa"]), qf_pos=ratio * qp, qf_psi=ratio * qh)


def load_config(nmpc_file: str | Path = DEFAULT_NMPC, params_file: str | Path = vp.DEFAULT_PARAMS,
                chain_file: str | Path = DEFAULT_CHAIN) -> NmpcConfig:
    n = yaml.safe_load(Path(nmpc_file).read_text())
    raw = vp.load_raw(params_file)
    geometry = vp.load_geometry(params_file)
    chain = yaml.safe_load(Path(chain_file).read_text())
    veh = chain["actuator_profiles"][chain["vehicle_limits_profile"]]
    for key in ("accel_max", "decel_max", "delta_rate_max"):
        if veh[key] is None:
            raise ValueError(f"vehicle_limits_profile must give {key}")
    L = geometry.wheelbase
    limits = am.AckermannLimits(
        v_min=0.0, v_max=float(raw["v_max_sim"]["value"]),
        kappa_max=am.kappa_limit_from_steering(geometry.delta_max, L),
        accel_max=float(veh["accel_max"]), decel_max=float(veh["decel_max"]),
        kappa_rate_max=am.kappa_rate_limit_from_steering_rate(float(veh["delta_rate_max"]), L))
    h, t = n["horizon"], n["timing"]
    return NmpcConfig(limits, L, geometry.delta_max, int(h["Nh"]), int(h["Nu"]), float(h["Ts"]), n["sizes"], n["pso"],
                      weights_from(n["weights"]), str(n["weights"]["status"]),
                      float(t["gazebo_term"]), {k: float(v) for k, v in t["chain_term"].items()},
                      float(t["end_tolerance"]))


# ----------------------------------------------------------------------------------------------
# Path projection and reference window
# ----------------------------------------------------------------------------------------------

class PathFollower:
    def __init__(self, path: P.Path2D) -> None:
        self.path = path
        self.ds = float(path.s[1] - path.s[0])
        self.yaw_u = np.unwrap(path.yaw)
        self.idx = None

    def project(self, x: float, y: float, ahead_m: float = 1.0, back_m: float = 0.2) -> float:
        """Arc length of the nearest path point, searched forward-only around the last match."""
        p = self.path
        if self.idx is None:
            lo, hi = 0, min(len(p.s), int(1.0 / self.ds) + 1)
        else:
            lo, hi = max(0, self.idx - int(back_m / self.ds)), min(len(p.s), self.idx + int(ahead_m / self.ds) + 1)
        i = lo + int(np.argmin((p.x[lo:hi] - x) ** 2 + (p.y[lo:hi] - y) ** 2))
        self.idx = i if self.idx is None else max(i, self.idx - int(back_m / self.ds))
        return float(p.s[i])

    def sample(self, s: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        """(x, y, unwrapped yaw, kappa) at arc lengths s: closed paths wrap, open paths continue
        straight beyond their end."""
        p = self.path
        L = p.length
        lap = np.zeros_like(s)
        if p.closed:  # continue round the loop; the heading keeps growing by one loop's turn per lap
            lap = np.floor_divide(s, L)
            s = s - lap * L
        beyond = np.maximum(s - L, 0.0)
        sc = np.minimum(s, L)
        x, y = np.interp(sc, p.s, p.x), np.interp(sc, p.s, p.y)
        yaw = np.interp(sc, p.s, self.yaw_u) + lap * (self.yaw_u[-1] - self.yaw_u[0])
        kappa = np.interp(sc, p.s, p.kappa)
        if not p.closed:
            x = x + beyond * np.cos(yaw)
            y = y + beyond * np.sin(yaw)
            kappa = np.where(beyond > 0, 0.0, kappa)
        return x, y, yaw, kappa


def to_local(x0: np.ndarray, x: np.ndarray, y: np.ndarray, yaw_u: np.ndarray) -> np.ndarray:
    """World references -> the frame of x0 (heading continuous from the first sample, near 0)."""
    c, s = math.cos(x0[2]), math.sin(x0[2])
    dx, dy = x - x0[0], y - x0[1]
    psi = am.wrap_angle(yaw_u[0] - x0[2]) + (yaw_u - yaw_u[0])
    return np.column_stack([c * dx + s * dy, -s * dx + c * dy, psi])


def predict(pose: Pose, t_end: float, buffer: list[tuple[float, float, float]]) -> np.ndarray:
    """Exact-arc prediction from the pose at its capture time to t_end through the commands in
    effect: buffer = [(t_eff, v, kappa)] sorted by t_eff; before the first one, (0, 0)."""
    state = np.array([pose.x, pose.y, pose.yaw])
    t = pose.stamp
    times = [b[0] for b in buffer]
    while t < t_end - 1e-12:
        i = int(np.searchsorted(times, t, side="right")) - 1
        u = (0.0, 0.0) if i < 0 else buffer[i][1:]
        t_next = t_end if i + 1 >= len(buffer) else min(t_end, buffer[i + 1][0])
        state = am.step_exact_arc(state, u, t_next - t)
        t = t_next
    return state


# ----------------------------------------------------------------------------------------------
# The controller
# ----------------------------------------------------------------------------------------------

@dataclass
class TickPlan:
    t_tick: float
    t_start: float
    t_apply: float
    t_eff: float
    deadline_miss: bool
    pose: Pose
    x_pred: np.ndarray
    s0: float
    problem: ne.Problem
    warm_cost: float


@dataclass
class Command:
    t_apply: float
    v: float
    kappa: float
    delta: float
    fitness: float
    double_cost: float
    fallback: bool
    deadline_miss: bool
    s_measured: float
    done: bool


@dataclass
class Stats:
    ticks: int = 0
    deadline_misses: int = 0
    stale_pose: int = 0      # pose older than one period at the tick (not a miss)
    no_pose: int = 0
    fallbacks: int = 0
    costs: list = field(default_factory=list)


@dataclass(frozen=True)
class Redesign:
    """The PSO redesign switches (2026-11-22 brief); all off = the frozen phase 4 controller.

    r1:     warm start: particle 0 is ALWAYS the previous best shifted by one step (when there is
            one), the feed-forward is kept as a further seed, plus `r1_perturb` perturbed copies of
            particle 0 (default: max(1, S // 5) when S >= 4)
    r2:     move blocking with this many knots per input (None = off)
    r3:     path-frame position cost with these (q_lat, q_lon) stage weights (terminal x ratio)
    r4:     the delta-kappa weight s_dkappa (None = the frozen value)
    """

    r1: bool = False
    r1_perturb: Optional[int] = None
    r2: Optional[int] = None
    r3: Optional[tuple] = None
    r4: Optional[float] = None
    max_v: Optional[float] = None   # PSO velocity clamp (normalized units); None = params/nmpc.yaml (30)

    @property
    def label(self) -> str:
        parts = [n for n, on in (("R1", self.r1), ("R2", self.r2), ("R3", self.r3), ("R4", self.r4 is not None)) if on]
        if self.max_v is not None:
            parts.append(f"max_v{self.max_v:g}")
        return "+".join(parts) or "frozen"


def load_redesign(nmpc_file: str | Path = DEFAULT_NMPC, section: str = "redesign") -> Redesign:
    """A redesign variant from params/nmpc.yaml: `redesign` (the frozen, rule-selected one) or
    `redesign_smooth` (not selected by the rule; run for steering smoothness)."""
    d = yaml.safe_load(Path(nmpc_file).read_text())[section]
    return Redesign(r1=bool(d["r1"]), r2=None if d["r2"] is None else int(d["r2"]),
                    r3=None if d["r3"] is None else (float(d["r3"][0]), float(d["r3"][1])),
                    r4=None if d["r4"] is None else float(d["r4"]))


class NmpcController:
    def __init__(self, cfg: NmpcConfig, size: str, profile: str, compensation: bool = True, arith: str = "half",
                 weights: Optional[am.CostWeights] = None, redesign: Optional[Redesign] = None) -> None:
        if size not in cfg.sizes:
            raise KeyError(f"size {size!r} not in {sorted(cfg.sizes)}")
        if arith not in ne.ARITHS:
            raise ValueError(arith)
        self.cfg, self.size, self.profile = cfg, size, profile
        self.compensation, self.arith = compensation, arith
        self.weights = weights or cfg.weights
        self.redesign = redesign or Redesign()
        if self.redesign.r4 is not None:
            import dataclasses
            self.weights = dataclasses.replace(self.weights, s_dkappa=float(self.redesign.r4))
        self.path_frame = None
        if self.redesign.r3 is not None:
            ratio = self.weights.qf_pos / self.weights.q_pos
            q_lat, q_lon = (float(v) for v in self.redesign.r3)
            self.path_frame = ne.PathFrame(q_lat, q_lon, ratio * q_lat, ratio * q_lon)
        sz = cfg.sizes[size]
        pso_kw = {k: float(v) for k, v in cfg.pso.items()}
        if self.redesign.max_v is not None:
            pso_kw["max_v"] = float(self.redesign.max_v)
        self.pso = ne.PsoConfig(int(sz["particles"]), int(sz["iterations"]), **pso_kw)
        self.solve_time = float(sz["solve_time"])
        self.tau_eff = cfg.chain_term[profile] + cfg.gazebo_term
        self.path: Optional[P.Path2D] = None

    def start_episode(self, path: P.Path2D, speed: float, seed: int) -> None:
        self.path, self.speed, self.seed = path, float(speed), int(seed)
        self.follower = PathFollower(path)
        self.measured = PathFollower(path)
        self.rng = np.random.default_rng([int(seed), 7_919])  # the PSO stream, from the episode seed
        self.buffer: list[tuple[float, float, float]] = []   # (t_eff, v, kappa)
        self.applied: list[tuple[float, float, float]] = []  # (t_apply, v, kappa)
        self.prev_plan: Optional[np.ndarray] = None
        self.solver_free_at = -math.inf  # the single solver: end of the last solve
        self.solves: list[tuple[float, float]] = []  # (start, end) of every solve (tests, reports)
        self.stats = Stats()
        self.end_s = path.length - PATH_TRIM - self.cfg.end_tolerance

    @property
    def u_prev(self) -> np.ndarray:
        return np.array(self.applied[-1][1:]) if self.applied else np.zeros(2)

    def timing(self, t_tick: float) -> tuple[float, float, bool]:
        """(t_start, t_apply, deadline_miss): the solve starts at the tick or when the single
        solver is free, and lasts solve_time; a miss is an apply time beyond the next tick."""
        t_start = max(t_tick, self.solver_free_at)
        t_apply = t_start + self.solve_time
        return t_start, t_apply, t_apply > t_tick + self.cfg.ts + 1e-9

    def prepare(self, t_tick: float, pose: Optional[Pose]) -> Optional[TickPlan]:
        self.stats.ticks += 1
        if pose is None:
            self.stats.no_pose += 1
            return None
        t_start, t_apply, miss = self.timing(t_tick)
        self.solver_free_at = t_apply
        self.solves.append((t_start, t_apply))
        if t_tick - pose.stamp > self.cfg.ts + 1e-9:
            self.stats.stale_pose += 1
        t_eff = t_apply + self.tau_eff
        x_pred = predict(pose, t_eff, self.buffer) if self.compensation else np.array([pose.x, pose.y, pose.yaw])
        s0 = self.follower.project(x_pred[0], x_pred[1])
        cfg = self.cfg
        s = s0 + self.speed * cfg.ts * np.arange(1, cfg.nh + 1)
        xr, yr, yawr, _ = self.follower.sample(s)
        _, _, _, kr = self.follower.sample(s0 + self.speed * cfg.ts * np.arange(cfg.nh))
        xref = to_local(x_pred, xr, yr, yawr)
        uref = np.column_stack([np.full(cfg.nh, self.speed), np.clip(kr, -cfg.limits.kappa_max, cfg.limits.kappa_max)])
        pb = ne.Problem(np.zeros(3), xref, uref, self.u_prev, cfg.limits, cfg.ts, cfg.nu)
        rd = self.redesign
        if rd.r2:
            pb.knots = ne.make_knots(int(rd.r2), cfg.nh)
        if self.path_frame is not None:
            pb.path_frame = self.path_frame
            pb.ref_cs = np.column_stack([np.cos(xref[:, 2]), np.sin(xref[:, 2])])
        cands = [am.project_controls(am.feedforward_from_reference(xref[: cfg.nu], np.zeros(3), cfg.ts),
                                     self.u_prev, cfg.limits, cfg.ts)]
        if self.prev_plan is not None:
            cands.append(am.project_controls(am.shift_warm_start(self.prev_plan), self.u_prev, cfg.limits, cfg.ts))
        costs = [ne.double_cost(pb, c, self.weights) for c in cands]
        if rd.r1 and self.prev_plan is not None:  # R1: the shifted previous best is always particle 0
            best = 1
            pb.extra_starts = [cands[0]]
        else:
            best = int(np.argmin(costs))
        if rd.r1:
            n = rd.r1_perturb if rd.r1_perturb is not None else (max(1, self.pso.particles // 5)
                                                                   if self.pso.particles >= 4 else 0)
            pb.n_perturb = int(n)
        pb.warm_start = cands[best]
        return TickPlan(t_tick, t_start, t_apply, t_eff, miss, pose, x_pred, s0, pb, costs[best])

    def solve(self, plan: TickPlan) -> ne.Solution:
        """The heavy part (may run elsewhere; it only uses the problem and this controller's RNG)."""
        return ne.solve(plan.problem, self.pso, self.weights, self.rng, self.arith)

    def finish(self, plan: TickPlan, sol: ne.Solution) -> Command:
        cfg, pb = self.cfg, plan.problem
        result = sol.plan
        fallback = not am.check_controls(result, pb.u_prev, cfg.limits, cfg.ts, tol=1e-6)["ok"]
        if fallback:
            result = pb.warm_start
            self.stats.fallbacks += 1
        v, kappa = float(result[0, 0]), float(result[0, 1])
        self.prev_plan = result
        self.applied.append((plan.t_apply, v, kappa))
        self.buffer.append((plan.t_eff, v, kappa))
        self.stats.deadline_misses += int(plan.deadline_miss)
        dc = ne.double_cost(pb, result, self.weights)
        self.stats.costs.append(dc)
        s_meas = self.measured.project(plan.pose.x, plan.pose.y)
        return Command(plan.t_apply, v, kappa, math.atan(cfg.wheelbase * kappa), sol.fitness, dc, fallback,
                       plan.deadline_miss, s_meas, s_meas >= self.end_s)

    def step(self, t_tick: float, pose: Optional[Pose]) -> Optional[Command]:
        plan = self.prepare(t_tick, pose)
        return None if plan is None else self.finish(plan, self.solve(plan))


class Scheduler:
    """Publishes each command at its t_apply (simulation time). A result that arrives after its
    t_apply (Python slower than the timing model allows at this real-time factor) is published at
    once and counted as an OVERRUN: such a run does not follow the timing model."""

    def __init__(self) -> None:
        self.pending: list[tuple[float, Command]] = []
        self.overruns = 0
        self.overrun_max = 0.0

    def submit(self, now: float, cmd: Command) -> None:
        if now > cmd.t_apply + 1e-9:
            self.overruns += 1
            self.overrun_max = max(self.overrun_max, now - cmd.t_apply)
        self.pending.append((cmd.t_apply, cmd))
        self.pending.sort(key=lambda p: p[0])

    def due(self, now: float) -> list[Command]:
        out = [c for t, c in self.pending if t <= now + 1e-9]
        self.pending = [(t, c) for t, c in self.pending if t > now + 1e-9]
        return out
