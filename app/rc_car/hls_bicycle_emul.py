"""Bit-level emulation of the HLS bicycle solver's C code paths (BICYCLE_CONFIG, the PSO_RD_* switches).

C LEVEL / OFFLINE ONLY: an emulation of hw/hls/source (System::nmpc_cost_function_topflow with the
RK4 one_step_prediction and models/hls_bicycle.hpp; the hls_pso.cpp cores in pso_fsm's order), written
to be compared output for output with app/rc_car/hls_bicycle_check.py's g++ builds. Not an RTL model.

Arithmetic, as the C++ types compute it:
- top level ap_fixed<32,17,AP_RND_INF,AP_SAT_SYM>: values are held as integers in LSB units (2^-15).
  A product of ap_fixed values is exact (its width grows), sums are exact, and the only rounding is at
  an assignment to the 32-bit type: round half away from zero, then symmetric saturation
  (+-(2^31 - 1) LSB). Python integers make every intermediate exact.
- the model in IEEE half (numpy float16, round to nearest even per operation), with the 1024-entry
  table sine of local_sin (app/ackermann/hw_estimates.py TABLE and index).
- conversions top -> half (round to nearest even) and half -> top (to the 2^-15 grid).

The random stream: rand() of glibc (TYPE_3 additive feedback), so a C run seeded with srand(s) can be
replayed: r = ap_fixed(rand() / RAND_MAX), as rand_real computes it in C simulation.

Also here: the host-side packing of a Python controller problem (nmpc_engine.Problem) into the C
inputs (x_curr, u_curr, xref, rd_aux, last_best), i.e. what nmpc_overlay.py would have to send.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from pathlib import Path
import sys

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import gen_bicycle_config as gen  # noqa: E402
import nmpc_engine as ne  # noqa: E402
from nmpc_engine import am, hw  # noqa: E402

F16 = np.float16
FRAC = 15
LSB = 2.0 ** -FRAC
TOP_MAX_I = 2 ** 31 - 1          # AP_SAT_SYM: +-(2^16 - 2^-15)
RAND_MAX = 2 ** 31 - 1
SWITCHES = ("R1", "R2", "R3", "R4", "UREF")


# ----------------------------------------------------------------------------------------------
# ap_fixed<32,17> in integers
# ----------------------------------------------------------------------------------------------

def fxi(value) -> int:
    """A double -> ap_fixed<32,17> (AP_RND_INF, AP_SAT_SYM), in LSB units."""
    scaled = float(value) / LSB
    q = int(math.copysign(math.floor(abs(scaled) + 0.5), scaled))
    return max(-TOP_MAX_I, min(TOP_MAX_I, q))


def fxi_array(values) -> np.ndarray:
    a = np.asarray(values, dtype=np.float64) / LSB
    q = np.sign(a) * np.floor(np.abs(a) + 0.5)
    return np.clip(q, -TOP_MAX_I, TOP_MAX_I).astype(np.int64)


def sat(n):
    """Symmetric saturation of an exact integer (or array) in LSB units."""
    if isinstance(n, np.ndarray):
        return np.minimum(np.maximum(n, -TOP_MAX_I), TOP_MAX_I)
    return max(-TOP_MAX_I, min(TOP_MAX_I, n))


def rnd(n, shift: int):
    """Assign an exact value with `shift` extra fraction bits to the 32-bit type: round half away
    from zero, then saturate. Works on Python ints, int64 arrays and object arrays."""
    if shift == 0:
        return sat(n)
    half = 1 << (shift - 1)
    if isinstance(n, np.ndarray):
        q = (np.abs(n) + half) >> shift
        return sat(np.where(n < 0, -q, q))
    q = (abs(n) + half) >> shift
    return sat(-q if n < 0 else q)


def to_float(n) -> np.ndarray:
    return np.asarray(n, dtype=np.float64) * LSB


# ----------------------------------------------------------------------------------------------
# Configuration (the generated BICYCLE_CONFIG values, as the C compiler stores them)
# ----------------------------------------------------------------------------------------------

@dataclass
class Config:
    switches: frozenset
    n_s: int
    maxiter: int
    nh: int
    nu: int
    ts: int
    ts_2: int
    ts_6: int
    du_max: np.ndarray       # (2,) LSB
    u_max: int
    u_min: int
    uss: np.ndarray
    Q: tuple
    Qf: tuple
    R: tuple
    Wr: tuple
    Ws: tuple
    Q_pf: tuple
    Qf_pf: tuple
    knot_gap: np.ndarray | None
    knot_j: np.ndarray | None
    knot_a: np.ndarray | None
    perturb_frac: int
    n_perturb: int
    top_pi: int
    top_two_pi: int
    penalty: int
    h_max: int
    w0: int
    slope: int
    c1: int
    c2: int
    max_v: int
    init_v: int
    state_lim: int
    # the half model's constants
    v_min16: np.float16
    zmin16: np.float16
    vpu16: np.float16
    kpu16: np.float16
    pi_half16: np.float16
    values: dict

    def on(self, s: str) -> bool:
        return s in self.switches

    @property
    def aux_len(self) -> int:
        return 1 + self.nu * 2 + self.nh * 2 + self.nh * 2 if self.switches else 0


def make_config(switches=(), n_s: int = 10, maxiter: int = 40, section: str = "redesign_smooth") -> Config:
    sw = frozenset(switches)
    bad = sw - set(SWITCHES)
    if bad:
        raise ValueError(f"unknown switches {sorted(bad)}")
    v = gen.values(section)
    nh = v["nh"]
    nu = len(v["knots"]) if "R2" in sw else v["nu"]
    ts = fxi(v["ts"])
    k = v["knots"]
    gaps = np.diff(np.concatenate([[-1], k])).astype(np.int64)
    js = np.array([min(int(np.searchsorted(k, h, side="right")) - 1, len(k) - 2) for h in range(nh)])
    W = ne.interp_matrix(k, nh)
    a = np.array([fxi(W[h, js[h] + 1]) for h in range(nh)], dtype=np.int64)
    n_pert = (max(1, n_s // 5) if n_s >= 4 else 0)
    return Config(
        switches=sw, n_s=n_s, maxiter=maxiter, nh=nh, nu=nu,
        ts=ts, ts_2=rnd(ts * fxi(0.5), FRAC), ts_6=rnd(ts * fxi(0.1666666667), FRAC),
        du_max=np.array([fxi(x) for x in v["du"]], dtype=np.int64), u_max=fxi(100), u_min=fxi(-100),
        uss=np.array([fxi(-100.0), fxi(0.0)], dtype=np.int64),
        Q=tuple(fxi(q) for q in v["Q"]), Qf=tuple(fxi(q) for q in v["Qf"]), R=tuple(fxi(r) for r in v["R"]),
        Wr=tuple(fxi(x) for x in v["Wr"]), Ws=tuple(fxi(x) for x in v["Ws"]),
        Q_pf=tuple(fxi(x) for x in v["Q_pf"]), Qf_pf=tuple(fxi(x) for x in v["Qf_pf"]),
        knot_gap=gaps if "R2" in sw else None, knot_j=js if "R2" in sw else None, knot_a=a if "R2" in sw else None,
        perturb_frac=fxi(gen.PERTURB_STD * math.sqrt(3.0)), n_perturb=n_pert,
        top_pi=fxi(3.14159265358979323846), top_two_pi=fxi(6.28318530717958647692),
        penalty=fxi(1e4), h_max=fxi(60000.0),
        w0=fxi(0.9), slope=fxi((0.1 - 0.9) / maxiter), c1=fxi(2.1), c2=fxi(1.0), max_v=fxi(30), init_v=fxi(30 * 0.1),
        state_lim=fxi(1e3),
        v_min16=F16(v["v_min"]), zmin16=F16(-100), vpu16=F16(v["v_per_unit"]), kpu16=F16(v["kappa_per_unit"]),
        pi_half16=F16(1.57079632679), values=v)


# ----------------------------------------------------------------------------------------------
# Model and RK4 (vectorized over particles)
# ----------------------------------------------------------------------------------------------

def _to_half(n: np.ndarray) -> np.ndarray:
    return to_float(n).astype(F16)


def _local_sin(angle16: np.ndarray) -> np.ndarray:
    return hw.TABLE[ne._table_index(angle16) & 1023]


def model(c: Config, state: np.ndarray, control: np.ndarray) -> np.ndarray:
    """System::model with __model (hls_bicycle.hpp): state (P, 3), control (P, 2) in LSB -> (P, 3) LSB."""
    s16, u16 = _to_half(state), _to_half(control)
    psi = s16[:, 2]
    v = (c.v_min16 + (c.vpu16 * (u16[:, 0] - c.zmin16).astype(F16)).astype(F16)).astype(F16)
    kappa = (c.kpu16 * u16[:, 1]).astype(F16)
    cos_psi = _local_sin((psi + c.pi_half16).astype(F16))
    sin_psi = _local_sin(psi)
    sd = np.column_stack([(v * cos_psi).astype(F16), (v * sin_psi).astype(F16), (v * kappa).astype(F16)])
    return fxi_array(sd.astype(np.float64))


def rk4(c: Config, state: np.ndarray, control: np.ndarray) -> np.ndarray:
    """System::one_step_prediction: update_state = state + Ts_local k (one rounding); the final
    update with rk4_middle_sum, rk4_weighted_sum (exact on the grid) and rk4_delta (rounded)."""
    s = state.astype(np.int64)
    k1 = model(c, s, control)
    k2 = model(c, rnd((s << FRAC) + c.ts_2 * k1, FRAC), control)
    k3 = model(c, rnd((s << FRAC) + c.ts_2 * k2, FRAC), control)
    k4 = model(c, rnd((s << FRAC) + c.ts * k3, FRAC), control)
    mid = sat(k2 + k3)
    wsum = sat((k1 + k4) + 2 * mid)
    delta = rnd(c.ts_6 * wsum, FRAC)
    return sat(s + delta)


def stage_u(c: Config, cg: np.ndarray, stage: int) -> np.ndarray:
    """System::rd_stage_u / get_next_uu: cg (P, Nu, 2) LSB -> the stage input (P, 2)."""
    if c.on("R2"):
        j, a = int(c.knot_j[stage]), int(c.knot_a[stage])
        z0, z1 = cg[:, j], cg[:, j + 1]
        return rnd((z0 << FRAC) + a * (z1 - z0), FRAC)
    return cg[:, min(stage, c.nu - 1)]


# ----------------------------------------------------------------------------------------------
# Cost (System::nmpc_cost_function_topflow)
# ----------------------------------------------------------------------------------------------

def _obj(a: np.ndarray) -> np.ndarray:
    return np.array([int(x) for x in np.ravel(a)], dtype=object).reshape(np.shape(a))


_I64_SAFE = float(2 ** 62)


def _fits(*bounds) -> bool:
    """True when every float bound of an intermediate stays well inside int64."""
    return all(float(np.max(np.abs(b))) < _I64_SAFE for b in bounds)


def cost_fast(c: Config, x0, cg, xref, u_curr=None, aux=None):
    """The same cost as `cost_exact`, vectorized over particles in int64. Returns None as soon as a
    float bound says an intermediate might leave int64 range (or a control leaves the box, the
    penalty path), and `cost` then uses the exact path. Equality with `cost_exact` is tested."""
    p = cg.shape[0]
    x = np.tile(np.asarray(x0, dtype=np.int64), (p, 1))
    Ji = np.zeros((p, 3), dtype=np.int64)
    Jui = np.zeros((p, 2), dtype=np.int64)
    u_prev = np.tile(np.asarray(u_curr if u_curr is not None else [0, 0], dtype=np.int64), (p, 1))
    uref_off = 1 + c.nu * 2
    cs_off = uref_off + c.nh * 2
    for i in range(c.nh):
        u = stage_u(c, cg, i)
        if np.any(u < c.u_min) or np.any(u > c.u_max):
            return None
        last = i == c.nh - 1
        for j in range(2):
            uj = u[:, j]
            tmp = uj - (int(aux[uref_off + 2 * i + j]) if c.on("UREF") else int(c.uss[j]))
            if c.on("R4"):
                d = uj - u_prev[:, j]
                if not _fits(float(c.Ws[j]) * d.astype(float)):
                    return None
                du_w = rnd(c.Ws[j] * d, FRAC)
            else:
                du_w = np.zeros(p, dtype=np.int64)
            dw2 = du_w.astype(float) ** 2
            if c.on("UREF"):
                if not _fits(float(c.Wr[j]) * tmp.astype(float)):
                    return None
                e_w = rnd(c.Wr[j] * tmp, FRAC)
                if not _fits(Jui[:, j].astype(float) * 2.0 ** 15 + e_w.astype(float) ** 2 + dw2):
                    return None
                Jui[:, j] = rnd((Jui[:, j] << 15) + e_w * e_w + du_w * du_w, 15)
            else:
                R = c.R[j]
                if not _fits(Jui[:, j].astype(float) * 2.0 ** 30 + float(R) * tmp.astype(float) ** 2 + dw2 * 2.0 ** 15):
                    return None
                Jui[:, j] = rnd((Jui[:, j] << 30) + R * tmp * tmp + ((du_w * du_w) << 15), 30)
        u_prev = u
        x = rk4(c, x, u)
        ref = np.asarray(xref[i], dtype=np.int64)
        if np.any(np.abs(x) > c.state_lim):
            return None
        e = sat(x - ref[None, :])
        h = e[:, 2]
        h = np.where(h > c.top_pi, sat(h - c.top_two_pi), np.where(h <= -c.top_pi, sat(h + c.top_two_pi), h))
        errs = [e[:, 0], e[:, 1], h]
        ws = [c.Qf[j] if last else c.Q[j] for j in range(3)]
        if c.on("R3"):
            cc, ss = int(aux[cs_off + 2 * i]), int(aux[cs_off + 2 * i + 1])
            ex, ey = e[:, 0], e[:, 1]
            errs[0] = rnd(cc * ey - ss * ex, FRAC)
            errs[1] = rnd(cc * ex + ss * ey, FRAC)
            ws[0] = c.Qf_pf[0] if last else c.Q_pf[0]
            ws[1] = c.Qf_pf[1] if last else c.Q_pf[1]
        for j in range(3):
            if not _fits(Ji[:, j].astype(float) * 2.0 ** 30 + float(ws[j]) * errs[j].astype(float) ** 2):
                return None
            Ji[:, j] = rnd((Ji[:, j] << 30) + ws[j] * errs[j] * errs[j], 30)
    a = np.zeros(p, dtype=np.int64)
    for j in range(3):
        a = sat(a + Ji[:, j])
    b = np.zeros(p, dtype=np.int64)
    for j in range(2):
        b = sat(b + Jui[:, j])
    return sat(a + b)


def cost(c: Config, x0: np.ndarray, cg: np.ndarray, xref: np.ndarray, u_curr: np.ndarray | None = None,
         aux: np.ndarray | None = None, trace: bool = False):
    """Per-particle cost J (P,) in LSB: the int64 fast path when it is safe, else the exact path."""
    if not trace:
        J = cost_fast(c, x0, cg, xref, u_curr, aux)
        if J is not None:
            return J
    return cost_exact(c, x0, cg, xref, u_curr, aux, trace)


def cost_exact(c: Config, x0: np.ndarray, cg: np.ndarray, xref: np.ndarray, u_curr: np.ndarray | None = None,
               aux: np.ndarray | None = None, trace: bool = False):
    """Per-particle cost J (P,) in LSB, every intermediate a Python integer (the reference path;
    bit-identical to the C build in hls_bicycle_check.py). x0 (3,), cg (P, Nu, 2), xref (Nh, 3),
    u_curr (2,), aux (len,)."""
    p = cg.shape[0]
    x = np.tile(np.asarray(x0, dtype=np.int64), (p, 1))
    Ji = np.zeros((p, 3), dtype=object)
    Jui = np.zeros((p, 2), dtype=object)
    u_prev = np.tile(np.asarray(u_curr if u_curr is not None else [0, 0], dtype=np.int64), (p, 1))
    uref_off = 1 + c.nu * 2
    cs_off = uref_off + c.nh * 2
    states = []
    for i in range(c.nh):
        u = stage_u(c, cg, i)
        last = i == c.nh - 1
        # --- control error
        uo = _obj(u)
        for j in range(2):
            inbox = (u[:, j] >= c.u_min) & (u[:, j] <= c.u_max)
            if c.on("UREF"):
                tmp = uo[:, j] - int(aux[uref_off + 2 * i + j])
            else:
                tmp = uo[:, j] - int(c.uss[j])
            if c.on("R4"):
                du_w = rnd(c.Ws[j] * (uo[:, j] - _obj(u_prev[:, j])), FRAC)
            else:
                du_w = np.zeros(p, dtype=object)
            out = np.empty(p, dtype=object)
            for q in range(p):
                if c.on("UREF") or c.on("R4"):  # rd_one_step_u_error
                    if inbox[q]:
                        if c.on("UREF"):
                            e_w = rnd(c.Wr[j] * tmp[q], FRAC)
                            out[q] = rnd((Jui[q, j] << 15) + e_w * e_w + du_w[q] * du_w[q], 15)
                        else:
                            out[q] = rnd((Jui[q, j] << 30) + c.R[j] * tmp[q] * tmp[q] + ((du_w[q] * du_w[q]) << 15), 30)
                    else:
                        out[q] = rnd((Jui[q, j] << 30) + c.penalty * tmp[q] * tmp[q] + ((du_w[q] * du_w[q]) << 15), 30)
                else:  # one_step_u_error
                    pen = c.R[j] if inbox[q] else c.penalty
                    out[q] = rnd((Jui[q, j] << 30) + pen * tmp[q] * tmp[q], 30)
            Jui[:, j] = out
        u_prev = u
        # --- prediction
        x = rk4(c, x, u)
        states.append(x.copy())
        # --- state error
        xo = _obj(x)
        ref = [int(r) for r in xref[i]]
        for q in range(p):
            inlim = [(-c.state_lim <= int(x[q, j]) <= c.state_lim) for j in range(3)]
            if c.on("R3"):
                cc, ss = int(aux[cs_off + 2 * i]), int(aux[cs_off + 2 * i + 1])
                ex, ey = sat(xo[q, 0] - ref[0]), sat(xo[q, 1] - ref[1])
                errs = [rnd(cc * ey - ss * ex, FRAC), rnd(cc * ex + ss * ey, FRAC)]
                ws = [c.Qf_pf[0] if last else c.Q_pf[0], c.Qf_pf[1] if last else c.Q_pf[1]]
                js = (0, 1)
            else:
                errs, ws, js = [], [], ()
            for j in range(3):
                if j in js:
                    e, w = errs[j], ws[j]
                else:
                    e = sat(xo[q, j] - ref[j])
                    if j == 2:  # controlled_state 2: wrapped
                        if e > c.top_pi:
                            e = sat(e - c.top_two_pi)
                        elif e <= -c.top_pi:
                            e = sat(e + c.top_two_pi)
                    w = c.Qf[j] if last else c.Q[j]
                pen = w if inlim[j] else c.penalty
                Ji[q, j] = rnd((Ji[q, j] << 30) + pen * e * e, 30)
    J = np.empty(p, dtype=np.int64)
    for q in range(p):  # J_error
        a = 0
        for j in range(3):
            a = sat(a + Ji[q, j])
        b = 0
        for j in range(2):
            b = sat(b + Jui[q, j])
        J[q] = sat(a + b)
    if trace:
        return J, np.stack(states, axis=1)
    return J


# ----------------------------------------------------------------------------------------------
# glibc rand() and the PSO cores
# ----------------------------------------------------------------------------------------------

class GlibcRand:
    """rand() after srand(seed) of glibc (random_r TYPE_3: r[i] = r[i-3] + r[i-31] mod 2^32, out >> 1)."""

    def __init__(self, seed: int) -> None:
        seed = seed or 1
        r = [0] * 34
        r[0] = seed & 0xFFFFFFFF
        word = seed if seed < 2 ** 31 else seed - 2 ** 32
        for i in range(1, 31):
            hi, lo = divmod(word, 127773)
            word = 16807 * lo - 2836 * hi
            if word < 0:
                word += 2147483647
            r[i] = word & 0xFFFFFFFF
        for i in range(31, 34):
            r[i] = r[i - 31]
        self.r = r
        for _ in range(310):
            self._next()

    def _next(self) -> int:
        v = (self.r[-31] + self.r[-3]) & 0xFFFFFFFF
        self.r.append(v)
        if len(self.r) > 64:
            del self.r[:-34]
        return v >> 1

    def __call__(self) -> int:
        return self._next()


class RandStream:
    """rand_real in C simulation: ap_fixed((double)rand() / RAND_MAX), counted."""

    def __init__(self, seed: int) -> None:
        self.g = GlibcRand(seed)
        self.count = 0
        self.values: list[int] = []

    def __call__(self) -> int:
        self.count += 1
        v = fxi(self.g() / RAND_MAX)
        self.values.append(v)
        return v


def _box(c: Config, x: int) -> int:
    return c.u_max if x > c.u_max else (c.u_min if x < c.u_min else x)


def _du(c: Config, k: int, j: int) -> int:
    d = int(c.du_max[k])
    return d * int(c.knot_gap[j]) if c.on("R2") else d


@dataclass
class SolveResult:
    new_best: np.ndarray      # (Nu, 2) LSB
    bestfitness: np.ndarray   # (maxiter,) LSB
    draws: int
    x_init: np.ndarray


def solve(c: Config, x0, u_curr, xref, aux, last_best, rand: RandStream) -> SolveResult:
    """initializeParticles_set, then maxiter x (evaluate, detectGlobalMinimum, update), as main_hls_bicycle_test."""
    n, nu = c.n_s, c.nu
    u_curr = [int(v) for v in u_curr]
    X = np.zeros((n, nu, 2), dtype=np.int64)
    V = np.zeros((n, nu, 2), dtype=np.int64)
    for i in range(n):  # initializeParticlesWithDuConstrains
        ant = list(u_curr)
        for j in range(nu):
            prev = list(ant)
            for k in range(2):
                du = _du(c, k, j)
                r = rand()
                du_tmp = rnd(r * fxi(2.0) * du - (du << 30), 30)
                xt = _box(c, sat(prev[k] + du_tmp))
                X[i, j, k] = xt
                V[i, j, k] = c.init_v
                ant[k] = xt
    Y = X.copy()
    X[0] = np.asarray(last_best, dtype=np.int64).reshape(nu, 2)
    ant_tmp = list(u_curr)  # initializeStableZero (index 1)
    for j in range(nu):
        prev = list(ant_tmp)
        for k in range(2):
            du = _du(c, k, j)
            xt = int(c.uss[k])
            comp = xt - prev[k]
            if comp > du:
                xt = prev[k] + du
            elif comp < -du:
                xt = prev[k] - du
            X[1, j, k] = _box(c, xt)
            ant_tmp[k] = xt
    if c.on("R1"):  # rd_initializeWarmStart
        slot = 2
        if aux[0] > fxi(0.5) and slot < n:
            X[slot] = np.asarray(aux[1:1 + nu * 2], dtype=np.int64).reshape(nu, 2)
            slot += 1
        for _ in range(c.n_perturb):
            if slot >= n:
                break
            ant = list(u_curr)
            for j in range(nu):
                for k in range(2):
                    du = _du(c, k, j)
                    r = rand()
                    noise = rnd((r * fxi(2.0) - (fxi(1.0) << 15)) * c.perturb_frac * du, 45)
                    xt = sat(int(X[0, j, k]) + noise)
                    d = xt - ant[k]
                    if d > du:
                        xt = ant[k] + du
                    elif d < -du:
                        xt = ant[k] - du
                    xt = _box(c, xt)
                    X[slot, j, k] = xt
                    ant[k] = xt
            slot += 1
    x_init = X.copy()
    find = np.full(n, c.h_max, dtype=np.int64)
    best = np.zeros(c.maxiter, dtype=np.int64)
    g = None
    for it in range(c.maxiter):
        fx = cost(c, x0, X, xref, u_curr, aux)
        for i in range(n):
            if fx[i] < find[i]:
                Y[i] = X[i]
                find[i] = fx[i]
        b = 0
        for i in range(1, n):
            if find[i] < find[b]:
                b = i
        best[it] = find[b]
        g = Y[b].copy()
        w = c.w0 + c.slope * it
        for i in range(n):
            ant = list(u_curr)
            for j in range(nu):
                prev = list(ant)
                for k in range(2):
                    r1, r2 = rand(), rand()
                    v, x = int(V[i, j, k]), int(X[i, j, k])
                    vn = rnd(((w * v) << 15) + c.c1 * r1 * (int(Y[i, j, k]) - x) + c.c2 * r2 * (int(g[j, k]) - x), 30)
                    vn = c.max_v if vn > c.max_v else (-c.max_v if vn < -c.max_v else vn)
                    xn = sat(x + vn)
                    du = _du(c, k, j)
                    dx = xn - prev[k]
                    if dx > du:
                        xn = prev[k] + du
                    elif dx < -du:
                        xn = prev[k] - du
                    xn = _box(c, xn)
                    X[i, j, k] = xn
                    V[i, j, k] = vn
                    ant[k] = xn
    return SolveResult(g, best, rand.count, x_init)


# ----------------------------------------------------------------------------------------------
# Host side: a Python controller problem -> the C inputs (what nmpc_overlay.py would send)
# ----------------------------------------------------------------------------------------------

@dataclass
class CInputs:
    x_curr: np.ndarray    # (3,) LSB
    u_curr: np.ndarray    # (2,)
    xref: np.ndarray      # (Nh, 3)
    aux: np.ndarray       # (aux_len,) or empty
    last_best: np.ndarray  # (Nu, 2)


def pack(c: Config, pb: ne.Problem) -> CInputs:
    """Normalize and round as the host would; the cos/sin of the reference heading in double."""
    lim = pb.limits
    z_prev = am.normalize_controls(pb.u_prev, lim, ne.SCALE)[0]
    aux = np.zeros(c.aux_len, dtype=np.int64)
    if c.switches:
        nu = c.nu
        if pb.extra_starts:
            aux[0] = fxi(1.0)
            aux[1:1 + 2 * nu] = fxi_array(ne.to_decision(pb.extra_starts[0], pb)).ravel()
        uref = am.normalize_controls(pb.uref, lim, ne.SCALE)
        aux[1 + 2 * nu:1 + 2 * nu + 2 * c.nh] = fxi_array(uref).ravel()
        cs = pb.ref_cs if pb.ref_cs is not None else np.column_stack([np.cos(pb.xref[:, 2]), np.sin(pb.xref[:, 2])])
        aux[1 + 2 * nu + 2 * c.nh:] = fxi_array(cs).ravel()
    ws = pb.warm_start if pb.warm_start is not None else np.zeros((pb.nu, 2))
    return CInputs(fxi_array(pb.x0), fxi_array(z_prev), fxi_array(pb.xref), aux,
                   fxi_array(ne.to_decision(ws, pb)))


def problem_text(ci: CInputs, header: str, plans: list | None = None) -> str:
    """One problem in main_hls_bicycle_test.cpp's input format (values exact on the grid)."""
    vals = [ci.x_curr, ci.u_curr, ci.xref.ravel(), ci.aux]
    if plans is None:
        vals.append(ci.last_best.ravel())
    else:
        vals += [np.asarray(p).ravel() for p in plans]
    body = " ".join(repr(float(v) * LSB) for a in vals for v in np.ravel(a))
    return f"{header} {body}\n"
