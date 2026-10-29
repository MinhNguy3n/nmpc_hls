"""Reference-model PSO with the structure of the hardware solver (hw/hls/source/src/hls_pso.cpp).

Mirrored from the HLS code (read at 3145854), in double precision and vectorized over particles:
- decision variables normalized to [-100, 100] per dimension (the existing ABI);
- initializeParticlesWithDuConstrains: every particle starts as a random walk from u_curr with
  per-stage steps uniform in [-du_max, du_max], clamped to the box; y = x; v = 0.1 * max_v;
- particle 0 = the warm start (last_best), copied UNPROJECTED (the host must send a feasible one);
- particle 1 = "stable zero": uss reached from u_curr through the du window stage by stage;
- per iteration k: evaluate all particles (personal best on strict improvement), global best =
  the first minimum of the personal-best fitness, then updateParticlesWithDuConstrains:
  w = w0 + slope * k (slope = (wf - w0) / maxiter), v = w v + c1 r1 (y - x) + c2 r2 (g - x),
  v clamped to +-max_v, x += v, then stage by stage clamp to x_prev +- du_max and to the box;
- the result is the global best of the last iteration.
The cost is the hardware's (System::nmpc_cost_function_topflow): sum over the Nh predicted states
of Q (Qf at the last) times the squared residual (angles with the one-conditional wrap of
8c61f1b), plus R (z - uss)^2 on the normalized controls of all Nh stages (Nu..Nh-1 hold the last).
Prediction is RK4 (variant A), optionally with a yaw-rate gain on omega (the predictor-gain
option; 'sim-derived' gains are simulator physics, not robot values).
This is a reference for SIL screening, not a timing model of the hardware.
"""

from __future__ import annotations

from dataclasses import dataclass, field
import math
from typing import Callable

import numpy as np

from . import _HUSKY  # noqa: F401  (sets sys.path)
import unicycle_model as um

SCALE = 100.0

# Yaw-rate gains identified IN SIMULATION (plan section 2.1, 2026-10-26): omega_gt / omega_cmd.
# Labelled 'sim-derived'; they are simulator physics, never robot parameters.
SIM_DERIVED_GAIN_IN_PLACE = 0.952
SIM_DERIVED_GAIN_ARC = 0.825
IN_PLACE_SPEED = 0.05  # m/s: |v| below this is classed as in-place motion (PLACEHOLDER_ policy)


@dataclass(frozen=True)
class PsoConfig:
    particles: int
    iterations: int
    w0: float = 0.9  # config.hpp _w0
    wf: float = 0.1  # _wf
    c1: float = 2.1  # _c1
    c2: float = 1.0  # _c2
    max_v: float = 30.0  # _max_v (normalized units)

    def __post_init__(self) -> None:
        if self.particles < 2 or self.iterations < 1:
            raise ValueError("need at least 2 particles (warm start + stable zero) and 1 iteration")

    @property
    def slope(self) -> float:
        return (self.wf - self.w0) / self.iterations

    @staticmethod
    def parse(text: str) -> "PsoConfig":
        """'4x50' -> PsoConfig(4, 50)."""
        particles, iterations = text.lower().split("x")
        return PsoConfig(int(particles), int(iterations))


@dataclass(frozen=True)
class CostConfig:
    """Hardware cost weights. q/qf on [x, y, psi] (1/m^2, 1/m^2, 1/rad^2); r on the normalized
    controls (1/unit^2); uss normalized. All values are PLACEHOLDER_ screening weights."""

    q: tuple[float, float, float] = (1.0, 1.0, 1.0)
    qf: tuple[float, float, float] = (10.0, 10.0, 3.0)
    r: tuple[float, float] = (0.0, 5e-5)  # R_v = 0 (design note 7.3), small yaw-rate effort
    uss: tuple[float, float] | None = None  # None -> (z(v = 0), 0)


def yaw_rate_gain(mode: str) -> Callable[[np.ndarray], np.ndarray]:
    """Predictor gain on omega as a function of v: 'plain' -> 1; 'sim-derived' -> by motion class."""
    if mode == "plain":
        return lambda v: np.ones_like(v)
    if mode == "sim-derived":
        return lambda v: np.where(np.abs(v) < IN_PLACE_SPEED, SIM_DERIVED_GAIN_IN_PLACE, SIM_DERIVED_GAIN_ARC)
    raise ValueError("gain mode must be 'plain' or 'sim-derived'")


def angle_residual(x_hat: np.ndarray, ref: np.ndarray) -> np.ndarray:
    """System::one_step_error for controlled_state == 2: one conditional -/+ 2 pi into (-pi, pi]."""
    e = x_hat - ref
    e = np.where(e > math.pi, e - 2 * math.pi, e)
    return np.where(e <= -math.pi, e + 2 * math.pi, e)


@dataclass
class Problem:
    """One solve, in the local frame (x0 normally zeros) and physical limits."""

    x0: np.ndarray  # (3,)
    xref: np.ndarray  # (Nh, 3)
    u_prev: np.ndarray  # (2,) physical, the applied command
    limits: um.UnicycleLimits
    ts: float
    nu: int
    warm_start: np.ndarray | None = None  # (Nu, 2) physical, must be feasible from u_prev
    gain: Callable[[np.ndarray], np.ndarray] = field(default_factory=lambda: yaw_rate_gain("plain"))

    @property
    def nh(self) -> int:
        return len(self.xref)


@dataclass(frozen=True)
class Solution:
    plan: np.ndarray  # (Nu, 2) physical
    plan_normalized: np.ndarray
    fitness: float
    best_fitness_history: np.ndarray


def _denormalize(z: np.ndarray, limits: um.UnicycleLimits) -> tuple[np.ndarray, np.ndarray]:
    vc, vh = 0.5 * (limits.v_max + limits.v_min), 0.5 * (limits.v_max - limits.v_min)
    return vc + vh * z[..., 0] / SCALE, limits.omega_max * z[..., 1] / SCALE


def predict(problem: Problem, z: np.ndarray) -> np.ndarray:
    """RK4 rollout of all particles: z (P, Nu, 2) normalized -> states (P, Nh, 3) for x_1..x_Nh."""
    p = z.shape[0]
    full = np.concatenate([z, np.repeat(z[:, -1:, :], problem.nh - problem.nu, axis=1)], axis=1)
    v_all, w_all = _denormalize(full, problem.limits)
    w_all = problem.gain(v_all) * w_all
    state = np.tile(np.asarray(problem.x0, dtype=float), (p, 1))
    out = np.empty((p, problem.nh, 3))
    ts = problem.ts

    def f(s: np.ndarray, v: np.ndarray, w: np.ndarray) -> np.ndarray:
        return np.stack([v * np.cos(s[:, 2]), v * np.sin(s[:, 2]), w], axis=1)

    for k in range(problem.nh):
        v, w = v_all[:, k], w_all[:, k]
        k1 = f(state, v, w)
        k2 = f(state + 0.5 * ts * k1, v, w)
        k3 = f(state + 0.5 * ts * k2, v, w)
        k4 = f(state + ts * k3, v, w)
        state = state + ts / 6.0 * (k1 + 2 * k2 + 2 * k3 + k4)
        out[:, k] = state
    return out


def cost(problem: Problem, z: np.ndarray, weights: CostConfig, uss: np.ndarray) -> np.ndarray:
    """Hardware-structured cost of every particle: (P,)."""
    states = predict(problem, z)
    err = states - problem.xref[None, :, :]
    err[:, :, 2] = angle_residual(states[:, :, 2], problem.xref[None, :, 2])
    q = np.tile(np.asarray(weights.q, dtype=float), (problem.nh, 1))
    q[-1] = weights.qf
    state_cost = np.sum(q[None] * err ** 2, axis=(1, 2))
    full = np.concatenate([z, np.repeat(z[:, -1:, :], problem.nh - problem.nu, axis=1)], axis=1)
    u_cost = np.sum(np.asarray(weights.r)[None, None, :] * (full - uss[None, None, :]) ** 2, axis=(1, 2))
    return state_cost + u_cost


def _du_project(x: np.ndarray, z_prev: np.ndarray, du: np.ndarray) -> np.ndarray:
    """Stage by stage: clamp to x_prev +- du, then to the +-100 box (hardware order)."""
    out = np.empty_like(x)
    ant = np.broadcast_to(z_prev, (x.shape[0], 2)).copy()
    for j in range(x.shape[1]):
        stage = np.clip(x[:, j], ant - du, ant + du)
        stage = np.clip(stage, -SCALE, SCALE)
        out[:, j] = stage
        ant = stage
    return out


def solve(problem: Problem, config: PsoConfig, weights: CostConfig, rng: np.random.Generator) -> Solution:
    limits = problem.limits
    du = um.normalized_slew_limits(limits, problem.ts, SCALE)
    z_prev = um.normalize_controls(problem.u_prev, limits, SCALE)[0]
    uss = (um.normalize_controls([0.0, 0.0], limits, SCALE)[0] if weights.uss is None
           else np.asarray(weights.uss, dtype=float))
    p, nu = config.particles, problem.nu
    # initializeParticlesWithDuConstrains: random walk from u_curr inside the du window
    steps = rng.uniform(-1.0, 1.0, (p, nu, 2)) * du
    x = np.empty((p, nu, 2))
    ant = np.tile(z_prev, (p, 1))
    for j in range(nu):
        ant = np.clip(ant + steps[:, j], -SCALE, SCALE)
        x[:, j] = ant
    if problem.warm_start is not None:  # particle 0 = last_best, unprojected (as in hardware)
        x[0] = um.normalize_controls(problem.warm_start, limits, SCALE)
    stable = np.empty((nu, 2))  # particle 1 = stable zero through the du window
    ant1 = z_prev.copy()
    for j in range(nu):
        target = np.clip(uss, ant1 - du, ant1 + du)
        stable[j] = np.clip(target, -SCALE, SCALE)
        ant1 = target  # hardware keeps the pre-box value here (harmless while uss is in the box)
    x[1] = stable
    y = x.copy()
    v = np.full_like(x, 0.1 * config.max_v)
    pbest = np.full(p, np.inf)
    history = np.empty(config.iterations)
    g = x[0].copy()
    for k in range(config.iterations):
        fit = cost(problem, x, weights, uss)
        better = fit < pbest
        y[better] = x[better]
        pbest[better] = fit[better]
        best = int(np.argmin(pbest))
        g = y[best].copy()
        history[k] = pbest[best]
        w = config.w0 + config.slope * k
        r1 = rng.uniform(0.0, 1.0, x.shape)
        r2 = rng.uniform(0.0, 1.0, x.shape)
        v = np.clip(w * v + config.c1 * r1 * (y - x) + config.c2 * r2 * (g[None] - x), -config.max_v, config.max_v)
        x = _du_project(x + v, z_prev, du)
    plan = um.denormalize_controls(g, limits, SCALE)
    return Solution(plan, g, float(history[-1]), history)
