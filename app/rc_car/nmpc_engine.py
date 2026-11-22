"""Emulated NMPC engine of the RC car study: PSO + Ackermann rollout + cost (numpy; offline tested).

SIMULATION ONLY. It emulates the PROPOSED Ackermann engine of docs/ackermann/ackermann_mpc_pso_model.md:
- PSO with the structure of the HLS solver (as mirrored by app/husky/sil/pso.py): normalized
  decision variables in [-100, 100]; particles start as random walks from u_prev inside the du
  window; particle 0 = the host's warm start, particle 1 = "stable zero" (v = 0, kappa = 0)
  through the du window; inertia schedule w0 -> wf, c1, c2, velocity clamp max_v; after each
  update, stage-by-stage clamp to the du window and the box; result = the global best.
- Rollout: the exact-arc chord step (section 5) of the October study.
- Cost: section 9.3 (ackermann_model.mpc_cost): position, heading, speed vs v_ref, curvature vs
  kappa_ref, increments from u_prev, terminal pose.

Arithmetic (`arith`):
- "half": the hardware emulation of app/ackermann/hw_estimates.step_chord_hw, vectorized over
  particles: the model in IEEE half with the 1024-entry table sine (shared index), the states on
  the ap_fixed<32,17> grid (2^-15, saturating at +-65535.99997). Each stage's cost term is
  rounded to that grid and the sum saturates, as an ap_fixed accumulator would. The
  normalized-to-physical control map is done in double before the half model (a simplification).
- "float": float32 chord step with exact sine and cosine (the float spot check).
- "double": double chord step (the test reference; equals mpc_cost on ackermann_model.rollout).

Redesign options (2026-11-22 PSO redesign brief; each OFF by default, and with all of them off the
results are bit-identical to the frozen phase 4 engine, tested against a recorded fixture):
- R1 (Problem.extra_starts, Problem.n_perturb): more seeded particles: extra start sequences at
  particles 2.., then perturbed copies of the warm start (particle 0), each projected; the extra
  random draws come after the original ones.
- R2 (Problem.knots): move blocking: the decision is M knots per input, linearly interpolated over
  the Nh stages; the per-knot rate window is du x (knot spacing), so the interpolated sequence keeps
  the per-step rate limit and the box.
- R3 (Problem.path_frame, Problem.ref_cs): path-frame position cost: the error is rotated by the
  reference heading (host-supplied cos/sin per stage) and weighted laterally and along-track.
- R4: a larger delta-kappa weight: the existing s_dkappa of the section 9.3 cost (controller side).
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from pathlib import Path
import sys

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "ackermann"))
import ackermann_model as am  # noqa: E402
import hw_estimates as hw  # noqa: E402

SCALE = 100.0
F16 = np.float16
LSB = 2.0 ** -15
TOP_MAX = 2.0 ** 16 - LSB  # ap_fixed<32,17> symmetric saturation
ARITHS = ("half", "float", "double")


# ----------------------------------------------------------------------------------------------
# Arithmetic
# ----------------------------------------------------------------------------------------------

def _clip(a, lo, hi):
    """np.clip without its Python-level overhead (numpy 1.21: 80 % of the solve time)."""
    return np.minimum(np.maximum(a, lo), hi)


def fx(a) -> np.ndarray:
    """Round to the ap_fixed<32,17> grid (nearest, ties away from 0) and saturate."""
    a = np.asarray(a, dtype=np.float64)
    q = np.sign(a) * np.floor(np.abs(a) / LSB + 0.5) * LSB
    return _clip(q, -TOP_MAX, TOP_MAX)


def _table_index(angle16: np.ndarray) -> np.ndarray:
    n = (angle16 * hw.INDEX_SCALE).astype(F16)
    rounded = np.where(n >= 0, (n + F16(0.5)).astype(F16), (n - F16(0.5)).astype(F16))
    return np.trunc(rounded.astype(np.float64)).astype(np.int64).astype(np.int16).astype(np.int64)


def _wrap(a: np.ndarray) -> np.ndarray:
    return (a + math.pi) % (2 * math.pi) - math.pi


def step_half(state: np.ndarray, v: np.ndarray, kappa: np.ndarray, ts: float) -> np.ndarray:
    """Vectorized hw_estimates.step_chord_hw (shared index): state (P, 3) on the fx grid."""
    psi16, v16, k16 = state[:, 2].astype(F16), v.astype(F16), kappa.astype(F16)
    distance = (v16 * F16(ts)).astype(F16)
    dpsi = (distance * k16).astype(F16)
    half = (dpsi * F16(0.5)).astype(F16)
    sinc = (F16(1.0) - ((half * half).astype(F16) * F16(1.0 / 6.0)).astype(F16)).astype(F16)
    chord = (distance * sinc).astype(F16)
    phi = (psi16 + half).astype(F16)
    idx = _table_index(phi)
    sin_phi, cos_phi = hw.TABLE[idx & 1023], hw.TABLE[(idx + 256) & 1023]
    dx = (chord * cos_phi).astype(F16).astype(np.float64)
    dy = (chord * sin_phi).astype(F16).astype(np.float64)
    out = np.empty_like(state)
    out[:, 0] = fx(state[:, 0] + fx(dx))
    out[:, 1] = fx(state[:, 1] + fx(dy))
    out[:, 2] = fx(_wrap(state[:, 2] + fx(dpsi.astype(np.float64))))
    return out


def step_exact(state: np.ndarray, v: np.ndarray, kappa: np.ndarray, ts: float, dtype) -> np.ndarray:
    """Chord step (exact arc) in float32 or float64, vectorized."""
    st = state.astype(dtype)
    s = (v * ts).astype(dtype)
    dpsi = (s * kappa.astype(dtype)).astype(dtype)
    h = dpsi * dtype(0.5)
    small = np.abs(h) < 1e-4
    sinc = np.where(small, 1 - h * h / 6, np.sin(h) / np.where(small, 1, h)).astype(dtype)
    chord = s * sinc
    phi = st[:, 2] + h
    out = np.empty_like(st)
    out[:, 0] = st[:, 0] + chord * np.cos(phi)
    out[:, 1] = st[:, 1] + chord * np.sin(phi)
    out[:, 2] = st[:, 2] + dpsi  # no wrap in the local frame (|psi| stays small); cost wraps
    return out.astype(np.float64)


def rollout_sequential(x0: np.ndarray, v: np.ndarray, kappa: np.ndarray, ts: float, arith: str) -> np.ndarray:
    """Step-by-step rollout (the reference for the vectorized one): (P, Nh) -> (P, Nh, 3)."""
    p, nh = v.shape
    state = np.tile(np.asarray(x0, dtype=np.float64), (p, 1))
    if arith == "half":
        state = fx(state)
    out = np.empty((p, nh, 3))
    for k in range(nh):
        if arith == "half":
            state = step_half(state, v[:, k], kappa[:, k], ts)
        elif arith == "float":
            state = step_exact(state, v[:, k], kappa[:, k], ts, np.float32)
        elif arith == "double":
            state = step_exact(state, v[:, k], kappa[:, k], ts, np.float64)
        else:
            raise ValueError(f"arith must be one of {ARITHS}")
        out[:, k] = state
    return out


def rollout(x0: np.ndarray, v: np.ndarray, kappa: np.ndarray, ts: float, arith: str) -> np.ndarray:
    """States x_1..x_Nh of every particle: v, kappa (P, Nh) physical -> (P, Nh, 3).

    Vectorized over the horizon and identical to rollout_sequential (tested): the half model's
    increments do not depend on x and y, and on the ap_fixed grid the position updates
    fx(x + fx(dx)) are exact additions, so x and y are cumulative sums; only the heading needs a
    short loop (its wrap rounds)."""
    if arith not in ARITHS:
        raise ValueError(f"arith must be one of {ARITHS}")
    p, nh = v.shape
    x0 = np.asarray(x0, dtype=np.float64)
    out = np.empty((p, nh, 3))
    if arith == "half":
        x0 = fx(x0)
        v16, k16 = v.astype(F16), kappa.astype(F16)
        distance = (v16 * F16(ts)).astype(F16)
        dpsi = (distance * k16).astype(F16)
        half = (dpsi * F16(0.5)).astype(F16)
        sinc = (F16(1.0) - ((half * half).astype(F16) * F16(1.0 / 6.0)).astype(F16)).astype(F16)
        chord = (distance * sinc).astype(F16)
        dpsi_fx = fx(dpsi.astype(np.float64))
        psi = np.empty((p, nh + 1))
        psi[:, 0] = x0[2]
        for k in range(nh):
            psi[:, k + 1] = fx(_wrap(psi[:, k] + dpsi_fx[:, k]))
        phi = (psi[:, :-1].astype(F16) + half).astype(F16)
        idx = _table_index(phi)
        dx = fx((chord * hw.TABLE[(idx + 256) & 1023]).astype(F16).astype(np.float64))
        dy = fx((chord * hw.TABLE[idx & 1023]).astype(F16).astype(np.float64))
        out[:, :, 0] = x0[0] + np.cumsum(dx, axis=1)
        out[:, :, 1] = x0[1] + np.cumsum(dy, axis=1)
        out[:, :, 2] = psi[:, 1:]
        if np.max(np.abs(out[:, :, :2])) >= TOP_MAX:  # saturation breaks the exact-sum argument
            return rollout_sequential(x0, v, kappa, ts, arith)
        return out
    dtype = np.float32 if arith == "float" else np.float64
    s = (v * ts).astype(dtype)
    dpsi = (s * kappa.astype(dtype)).astype(dtype)
    h = dpsi * dtype(0.5)
    small = np.abs(h) < 1e-4
    sinc = np.where(small, 1 - h * h / 6, np.sin(h) / np.where(small, 1, h)).astype(dtype)
    chord = (s * sinc).astype(dtype)
    psi = np.empty((p, nh + 1), dtype=dtype)
    psi[:, 0] = x0[2]
    psi[:, 1:] = dtype(x0[2]) + np.cumsum(dpsi, axis=1, dtype=dtype)
    phi = (psi[:, :-1] + h).astype(dtype)
    out[:, :, 0] = dtype(x0[0]) + np.cumsum((chord * np.cos(phi)).astype(dtype), axis=1, dtype=dtype)
    out[:, :, 1] = dtype(x0[1]) + np.cumsum((chord * np.sin(phi)).astype(dtype), axis=1, dtype=dtype)
    out[:, :, 2] = psi[:, 1:]
    return out


# ----------------------------------------------------------------------------------------------
# Problem, cost, PSO
# ----------------------------------------------------------------------------------------------

@dataclass(frozen=True)
class PsoConfig:
    particles: int
    iterations: int
    w0: float = 0.9
    wf: float = 0.1
    c1: float = 2.1
    c2: float = 1.0
    max_v: float = 30.0


@dataclass
class Problem:
    """One solve in the LOCAL frame (x0 = 0 normally): references (Nh, 3), reference controls
    (Nh, 2) physical, the applied command u_prev (2,), limits, Ts, Nu, warm start (Nu, 2)."""

    x0: np.ndarray
    xref: np.ndarray
    uref: np.ndarray
    u_prev: np.ndarray
    limits: am.AckermannLimits
    ts: float
    nu: int
    warm_start: np.ndarray | None = None
    knots: tuple | None = None            # R2: knot stage indices (first 0, last Nh - 1)
    path_frame: "PathFrame | None" = None  # R3
    ref_cs: np.ndarray | None = None      # R3: (Nh, 2) cos, sin of the reference heading (host)
    extra_starts: list | None = None      # R1: further start sequences (physical, Nh x 2)
    n_perturb: int = 0                    # R1: perturbed copies of the warm start
    perturb_scale: float = 0.25           # R1: perturbation std, as a fraction of the rate window
    perturb_dist: str = "normal"          # R1: "normal", or "uniform" of the same std (the HLS stream, C checks)

    @property
    def nh(self) -> int:
        return len(self.xref)

    @property
    def dim(self) -> int:
        """Decision stages: Nu, or the number of knots."""
        return len(self.knots) if self.knots is not None else self.nu


@dataclass(frozen=True)
class PathFrame:
    """R3 weights: lateral and along-track position error (stage and terminal)."""

    q_lat: float
    q_lon: float
    qf_lat: float
    qf_lon: float


def interp_matrix(knots: tuple, nh: int) -> np.ndarray:
    """(Nh, M) linear interpolation of the knot values over the stages."""
    k = np.asarray(knots, dtype=float)
    W = np.zeros((nh, len(k)))
    for h in range(nh):
        j = min(int(np.searchsorted(k, h, side="right")) - 1, len(k) - 2)
        a = (h - k[j]) / (k[j + 1] - k[j])
        W[h, j], W[h, j + 1] = 1.0 - a, a
    return W


def make_knots(m: int, nh: int) -> tuple:
    return tuple(int(round(x)) for x in np.linspace(0, nh - 1, m))


def to_full(z: np.ndarray, problem: "Problem") -> np.ndarray:
    """Decision (P, dim, 2) -> per-stage (P, Nh, 2), both normalized."""
    if problem.knots is None:
        return expand(z, problem.nh)
    return np.einsum("hm,pmc->phc", interp_matrix(problem.knots, problem.nh), z)


def to_decision(plan: np.ndarray, problem: "Problem") -> np.ndarray:
    """Physical per-stage plan (Nh or Nu rows) -> normalized decision (dim, 2)."""
    z = am.normalize_controls(plan, problem.limits, SCALE)
    if problem.knots is None:
        return z
    return z[list(problem.knots)]


def rate_windows(problem: "Problem", du: np.ndarray) -> np.ndarray:
    """Per-decision-stage rate window: du (old layout, shape (2,)), or du x knot spacing (dim, 2)."""
    if problem.knots is None:
        return du
    gaps = np.diff(np.concatenate([[-1], np.asarray(problem.knots)]))
    return gaps[:, None] * du[None, :]


def expand(z: np.ndarray, nh: int) -> np.ndarray:
    """(P, Nu, 2) -> (P, Nh, 2), the last input held."""
    return np.concatenate([z, np.repeat(z[:, -1:, :], nh - z.shape[1], axis=1)], axis=1)


def denormalize(z: np.ndarray, limits: am.AckermannLimits) -> tuple[np.ndarray, np.ndarray]:
    vc, vh = 0.5 * (limits.v_max + limits.v_min), 0.5 * (limits.v_max - limits.v_min)
    return vc + vh * z[..., 0] / SCALE, limits.kappa_max * z[..., 1] / SCALE


def cost(problem: Problem, z: np.ndarray, w: am.CostWeights, arith: str) -> np.ndarray:
    """Section 9.3 cost of every particle (P,), z (P, Nu, 2) normalized."""
    v, kappa = denormalize(to_full(z, problem), problem.limits)
    states = rollout(problem.x0, v, kappa, problem.ts, arith)
    pf = problem.path_frame
    if pf is None:
        pos_sq = np.sum((states[:, :, :2] - problem.xref[None, :, :2]) ** 2, axis=2)
    else:  # R3: rotate the error by the reference heading (host cos/sin, on the fx grid in half)
        cs = fx(problem.ref_cs) if arith == "half" else problem.ref_cs
        dx = states[:, :, 0] - problem.xref[None, :, 0]
        dy = states[:, :, 1] - problem.xref[None, :, 1]
        e_lon = cs[None, :, 0] * dx + cs[None, :, 1] * dy
        e_lat = -cs[None, :, 1] * dx + cs[None, :, 0] * dy
    head_sq = _wrap(states[:, :, 2] - problem.xref[None, :, 2]) ** 2
    dv = np.diff(np.concatenate([np.full((len(z), 1), problem.u_prev[0]), v], axis=1), axis=1)
    dk = np.diff(np.concatenate([np.full((len(z), 1), problem.u_prev[1]), kappa], axis=1), axis=1)
    stage = (w.q_v * (v - problem.uref[None, :, 0]) ** 2 + w.r_kappa * (kappa - problem.uref[None, :, 1]) ** 2
             + w.s_dv * dv ** 2 + w.s_dkappa * dk ** 2)
    if pf is None:
        stage[:, :-1] += w.q_pos * pos_sq[:, :-1] + w.q_psi * head_sq[:, :-1]
        stage[:, -1] += w.qf_pos * pos_sq[:, -1] + w.qf_psi * head_sq[:, -1]
    else:
        stage[:, :-1] += pf.q_lat * e_lat[:, :-1] ** 2 + pf.q_lon * e_lon[:, :-1] ** 2 + w.q_psi * head_sq[:, :-1]
        stage[:, -1] += pf.qf_lat * e_lat[:, -1] ** 2 + pf.qf_lon * e_lon[:, -1] ** 2 + w.qf_psi * head_sq[:, -1]
    if arith == "half":
        return _clip(np.sum(fx(stage), axis=1), -TOP_MAX, TOP_MAX)
    return np.sum(stage, axis=1)


def _project_du(x: np.ndarray, z_prev: np.ndarray, du: np.ndarray) -> np.ndarray:
    """Stage by stage: clamp to the previous stage's rate window, then to the box. du is (2,), or
    (dim, 2) per stage (R2)."""
    out = np.empty_like(x)
    ant = np.broadcast_to(z_prev, (x.shape[0], 2)).copy()
    for j in range(x.shape[1]):
        dj = du[j] if du.ndim == 2 else du
        stage = _clip(_clip(x[:, j], ant - dj, ant + dj), -SCALE, SCALE)
        out[:, j] = stage
        ant = stage
    return out


@dataclass(frozen=True)
class Solution:
    plan: np.ndarray        # (Nu, 2) physical (with R2: the interpolated (Nh, 2) plan)
    fitness: float
    history: np.ndarray


def solve(problem: Problem, cfg: PsoConfig, w: am.CostWeights, rng: np.random.Generator, arith: str = "half",
          cost_fn=None) -> Solution:
    """The emulated PSO. `cost_fn(problem, z, w, arith)` replaces `cost` (C checks only: the HLS bit-level
    emulation of app/rc_car/hls_bicycle_emul.py); None = the section 9.3 cost of this module."""
    cost_of = cost if cost_fn is None else cost_fn
    limits = problem.limits
    du = am.normalized_slew_limits(limits, problem.ts, SCALE)
    z_prev = am.normalize_controls(problem.u_prev, limits, SCALE)[0]
    uss = am.normalize_controls([0.0, 0.0], limits, SCALE)[0]
    p, nu = cfg.particles, problem.dim
    du = rate_windows(problem, du)
    steps = rng.uniform(-1.0, 1.0, (p, nu, 2)) * du
    x = np.empty((p, nu, 2))
    ant = np.tile(z_prev, (p, 1))
    for j in range(nu):
        ant = _clip(ant + steps[:, j], -SCALE, SCALE)
        x[:, j] = ant
    if problem.warm_start is not None:
        x[0] = to_decision(problem.warm_start, problem)
    if p > 1:
        stable = np.empty((nu, 2))
        a1 = z_prev.copy()
        for j in range(nu):
            dj = du[j] if du.ndim == 2 else du
            a1 = _clip(_clip(uss, a1 - dj, a1 + dj), -SCALE, SCALE)
            stable[j] = a1
        x[1] = stable
    slot = 2
    for start in (problem.extra_starts or []):  # R1: further seeded particles
        if slot >= p:
            break
        x[slot] = to_decision(start, problem)
        slot += 1
    for _ in range(problem.n_perturb):  # R1: perturbed copies of the warm start
        if slot >= p:
            break
        if problem.perturb_dist == "uniform":  # the HLS R1: uniform noise of the same std
            noise = rng.uniform(-1.0, 1.0, (nu, 2)) * (problem.perturb_scale * math.sqrt(3.0)) * du
        else:
            noise = rng.normal(0.0, problem.perturb_scale, (nu, 2)) * du
        x[slot] = _project_du((x[0] + noise)[None], z_prev, du)[0]
        slot += 1
    y = x.copy()
    vel = np.full_like(x, 0.1 * cfg.max_v)
    pbest = np.full(p, np.inf)
    hist = np.empty(cfg.iterations)
    g = x[0].copy()
    slope = (cfg.wf - cfg.w0) / cfg.iterations
    for k in range(cfg.iterations):
        fit = cost_of(problem, x, w, arith)
        better = fit < pbest
        y[better], pbest[better] = x[better], fit[better]
        best = int(np.argmin(pbest))
        g = y[best].copy()
        hist[k] = pbest[best]
        inertia = cfg.w0 + slope * k
        r1, r2 = rng.uniform(0.0, 1.0, x.shape), rng.uniform(0.0, 1.0, x.shape)
        vel = _clip(inertia * vel + cfg.c1 * r1 * (y - x) + cfg.c2 * r2 * (g[None] - x), -cfg.max_v, cfg.max_v)
        x = _project_du(x + vel, z_prev, du)
    if problem.knots is None:
        plan = am.denormalize_controls(g, limits, SCALE)
    else:
        plan = am.denormalize_controls(to_full(g[None], problem)[0], limits, SCALE)
    return Solution(plan, float(hist[-1]), hist)


def double_cost(problem: Problem, plan: np.ndarray, w: am.CostWeights) -> float:
    """Host-side double-precision cost of a physical plan (warm-start choice, logging). With R2 the
    plan is taken at the knots, as the solver would see it."""
    z = to_decision(plan, problem)[None]
    return float(cost(problem, z, w, "double")[0])
