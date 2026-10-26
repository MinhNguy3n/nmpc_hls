"""Accuracy and cycle estimates behind sections 5.4 and 9 of docs/husky_a300/unicycle_mpc_pso_model.md.

Adapted from app/ackermann/hw_estimates.py (Ackermann proposal, worktree agent-a3984fccbf29de483,
2026-10-25): the bit-level arithmetic emulation and the cycle-budget model are reused; the step
functions are rewritten for the unicycle (psidot = omega, no curvature) and the calibration is
updated with the 3x24 board measurements.

1. A bit-level emulation (NOT a C or RTL simulation) of the unicycle step in the hardware
   arithmetic: ap_fixed<32,17> top-level values rounded to 2^-15, a half-precision model and the
   1024-entry table sine of fast_sin_cos.hpp reached through local_sin.
2. A cycle-budget model calibrated on HLS reports and board timing of the quadrotor builds, inside
   one end-to-end cycle-latency budget (C = t_ready - t_tick <= 40 ms, each component counted once).
   The unicycle step cycles are ESTIMATES: no unicycle model has been synthesized.

Run: /usr/bin/python3 -B app/husky/hw_estimates.py
Speed / yaw-rate ranges below are documented controller limits (A300 diff_4wd.yaml: |v|, |omega|
<= 2) or documented robot limits (Create 3: v <= 0.306 m/s, omega <= 1.9 rad/s); see
unicycle_model.PARAMETER_SETS for sources.
"""

from __future__ import annotations

import math
from pathlib import Path
import sys

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import unicycle_model as um  # noqa: E402


# ----------------------------------------------------------------------------------------------
# Hardware arithmetic emulation (from the Ackermann hw_estimates.py)
# ----------------------------------------------------------------------------------------------

F16 = np.float16
FIXED_LSB = 2.0 ** -15  # ap_fixed<32,17>: 15 fractional bits
TABLE = np.sin(2.0 * np.pi * np.arange(1024) / 1024).astype(F16)
INDEX_SCALE = F16(np.float32(512.0) / np.float32(3.14159265358979323846))  # HALF_MAX_CIRCLE_ANGLE_PI in half
HALF_PI = F16(math.pi / 2)


def fx(value: float) -> float:
    """Round to the ap_fixed<32,17> grid (AP_RND_INF: nearest, ties away from zero)."""
    scaled = value / FIXED_LSB
    return float(math.copysign(math.floor(abs(scaled) + 0.5), scaled) * FIXED_LSB)


def table_index(angle: np.float16) -> int:
    """local_sin -> fastsin index: round(angle * 163) computed in half, then int16 truncation."""
    n = F16(angle * INDEX_SCALE)
    rounded = F16(n + F16(0.5)) if n >= 0 else F16(n - F16(0.5))
    return int(np.int16(int(rounded)))


def local_sin(angle: np.float16) -> np.float16:
    return TABLE[table_index(angle) & 1023]


def local_cos_via_sin(angle: np.float16) -> np.float16:
    """cos as the sniffbot model computes it: local_sin(angle + pi/2), the add in half."""
    return local_sin(F16(angle + HALF_PI))


def sincos_shared_index(angle: np.float16) -> tuple[np.float16, np.float16]:
    index = table_index(angle)
    return TABLE[index & 1023], TABLE[(index + 256) & 1023]


def sincos_fixed_index(phi: float, interpolate: bool) -> tuple[np.float16, np.float16]:
    """Table position from the ap_fixed angle; nearest entry or linear interpolation (blend in half)."""
    position = fx(fx(phi) * fx(1024.0 / (2.0 * math.pi)))
    if not interpolate:
        index = int(math.floor(position + 0.5))
        return TABLE[index & 1023], TABLE[(index + 256) & 1023]
    index = int(math.floor(position))
    fraction = F16(position - index)

    def blend(i: int) -> np.float16:
        low, high = TABLE[i & 1023], TABLE[(i + 1) & 1023]
        return F16(low + F16(fraction * F16(high - low)))

    return blend(index), blend(index + 256)


def _wrap_fixed(psi: float) -> float:
    return fx(um.wrap_angle(psi))


def step_rk4_hw(state: np.ndarray, control: np.ndarray, ts: float) -> np.ndarray:
    """System::one_step_prediction with a unicycle model in half (cos via local_sin(x + pi/2))."""
    ts_fixed, ts_2, ts_6 = fx(ts), fx(ts * 0.5), fx(ts * 0.1666666667)
    v16, w16 = F16(control[0]), F16(control[1])

    def model(local_state: np.ndarray) -> np.ndarray:
        psi16 = F16(local_state[2])
        return np.array([fx(float(F16(v16 * local_cos_via_sin(psi16)))),
                         fx(float(F16(v16 * local_sin(psi16)))),
                         fx(float(w16))])

    def update(base: np.ndarray, k: np.ndarray, h: float) -> np.ndarray:
        return np.array([fx(base[i] + fx(h * k[i])) for i in range(3)])

    k1 = model(state)
    k2 = model(update(state, k1, ts_2))
    k3 = model(update(state, k2, ts_2))
    k4 = model(update(state, k3, ts_fixed))
    out = np.array([fx(state[i] + fx(ts_6 * fx(fx(k1[i] + k4[i]) + fx(2.0 * fx(k2[i] + k3[i])))))
                    for i in range(3)])
    out[2] = _wrap_fixed(out[2])
    return out


def step_chord_hw(state: np.ndarray, control: np.ndarray, ts: float, shared_index: bool = True) -> np.ndarray:
    """Chord step, half model, HARDWARE APPROXIMATION sinc(a) ~ 1 - a^2/6 (no division; error
    <= a^4/120, see unicycle_model.sinc_poly2_error_bound): a = omega*Ts/2, chord = v*Ts*(1 - a^2/6),
    (dx, dy) added in ap_fixed."""
    psi16, v16, w16 = F16(state[2]), F16(control[0]), F16(control[1])
    half = F16(w16 * F16(ts * 0.5))
    distance = F16(v16 * F16(ts))
    chord = F16(distance * F16(F16(1.0) - F16(F16(half * half) * F16(1.0 / 6.0))))
    phi = F16(psi16 + half)
    sin_phi, cos_phi = sincos_shared_index(phi) if shared_index else (local_sin(phi), local_cos_via_sin(phi))
    dx, dy = float(F16(chord * cos_phi)), float(F16(chord * sin_phi))
    return np.array([fx(state[0] + fx(dx)), fx(state[1] + fx(dy)), _wrap_fixed(state[2] + fx(float(w16) * ts))])


def step_chord_hw_fixed_angle(state: np.ndarray, control: np.ndarray, ts: float, interpolate: bool) -> np.ndarray:
    """Chord step with dpsi and the table position in ap_fixed; sinc approximated by 1 - a^2/6 (no
    division, error <= a^4/120), blend and products in half."""
    dpsi = fx(fx(control[1]) * fx(ts))
    half = dpsi * 0.5  # exact shift in ap_fixed
    sin_phi, cos_phi = sincos_fixed_index(state[2] + half, interpolate)
    half16 = F16(half)
    chord = F16(F16(fx(fx(control[0]) * fx(ts))) * F16(F16(1.0) - F16(F16(half16 * half16) * F16(1.0 / 6.0))))
    dx, dy = float(F16(chord * cos_phi)), float(F16(chord * sin_phi))
    return np.array([fx(state[0] + fx(dx)), fx(state[1] + fx(dy)), _wrap_fixed(state[2] + dpsi)])


def step_arc_division_hw(state: np.ndarray, control: np.ndarray, ts: float) -> np.ndarray:
    """Literal arc (v/omega)(sin(psi + dpsi) - sin psi) in the same arithmetic: shows the cancellation."""
    psi16, v16, w16 = F16(state[2]), F16(control[0]), F16(control[1])
    dpsi = F16(w16 * F16(ts))
    if abs(float(dpsi)) < 1e-3 or float(w16) == 0.0:
        distance = F16(v16 * F16(ts))
        dx, dy = F16(distance * local_cos_via_sin(psi16)), F16(distance * local_sin(psi16))
    else:
        end = F16(psi16 + dpsi)
        radius = F16(v16 / w16)
        dx = F16(F16(local_sin(end) - local_sin(psi16)) * radius)
        dy = F16(F16(local_cos_via_sin(psi16) - local_cos_via_sin(end)) * radius)
    return np.array([fx(state[0] + fx(float(dx))), fx(state[1] + fx(float(dy))), _wrap_fixed(state[2] + fx(float(dpsi)))])


VARIANTS = {
    "euler (double)": lambda s, u, ts: um.step_euler(s, u, ts),
    "midpoint (double)": lambda s, u, ts: um.step_midpoint(s, u, ts),
    "rk4 (double)": lambda s, u, ts: um.step_rk4(s, u, ts),
    "A: rk4, hw arithmetic": step_rk4_hw,
    "B1: chord, half angle, shared index": lambda s, u, ts: step_chord_hw(s, u, ts, True),
    "B1': chord, half angle, cos via +pi/2": lambda s, u, ts: step_chord_hw(s, u, ts, False),
    "B2: chord, fixed angle, nearest entry": lambda s, u, ts: step_chord_hw_fixed_angle(s, u, ts, False),
    "B3: chord, fixed angle, interpolated": lambda s, u, ts: step_chord_hw_fixed_angle(s, u, ts, True),
    "X: literal arc (division), hw arithmetic": step_arc_division_hw,
}


# Every B variant uses the hardware approximation sinc(a) ~ 1 - a^2/6; the reference ("exact") is
# unicycle_model.step_chord with the exact sinc. "chord_poly (double)" isolates the polynomial error.
VARIANTS = {"chord_poly (double)": lambda s, u, ts: um.step_chord_poly(s, u, ts), **VARIANTS}


def compare_discretizations(v_max: float, omega_max: float, ts: float, horizon: int, local_frame: bool,
                            trials: int = 300, seed: int = 0) -> dict[str, tuple[float, float]]:
    """Worst position error (m) after 1 step and after `horizon` steps vs the double chord step.

    Constant (v, omega) per trial, v in [-v_max, v_max], omega in [-omega_max, omega_max]. With
    local_frame=True the start pose is (0, 0, 0), as the host would send it (design note 6.3);
    otherwise a random pose within 20 m and a random heading.
    """
    rng = np.random.default_rng(seed)
    worst = {name: [0.0, 0.0] for name in VARIANTS}
    for _ in range(trials):
        control = np.array([rng.uniform(-v_max, v_max), rng.uniform(-omega_max, omega_max)])
        if local_frame:
            start = np.zeros(3)
        else:
            start = np.array([fx(rng.uniform(-20, 20)), fx(rng.uniform(-20, 20)), fx(rng.uniform(-math.pi, math.pi))])
        exact = [start]
        for _ in range(horizon):
            exact.append(um.step_chord(exact[-1], control, ts))
        for name, method in VARIANTS.items():
            state = start.copy()
            for k in range(horizon):
                state = method(state, control, ts)
                if k == 0:
                    worst[name][0] = max(worst[name][0], float(np.hypot(*(state[:2] - exact[1][:2]))))
            worst[name][1] = max(worst[name][1], float(np.hypot(*(state[:2] - exact[-1][:2]))))
    return {name: (a, b) for name, (a, b) in worst.items()}


# ----------------------------------------------------------------------------------------------
# Cycle budget (100 MHz PL clock). ESTIMATES ONLY.
# ----------------------------------------------------------------------------------------------

CLOCK_HZ = 100e6
PERIOD_MS = 50.0
# Single end-to-end metric (design note 9.1, plan 3.5): cycle latency C = t_ready - t_tick, from the
# timer tick that freezes the state (latest odometry, predicted forward) to the checked command being
# ready; it is published at the next tick. Budget: C <= 40 ms keeps the existing 20 % margin of the
# 50 ms period (5 ms publish guard + 5 ms unallocated). Every non-solver component is listed once:
CYCLE_BUDGET_MS = 40.0
NON_SOLVER_MS = {
    # measured on this PC in Python/numpy (2026-10-25): local frame 0.21 ms, shifted plan 0.003 ms,
    # one projected candidate 1.1 ms (box limits) to 3.4 ms (with side diamonds); at most one
    # projected candidate on the critical path, the others run while the board solves.
    "host preparation": 4.0,
    "network request + reply (1 GbE, ESTIMATE; measure PING p99)": 1.0,
    # max over 500 solves of the 3x24 board run (the 100-solve run gave 1.9 ms)
    "KV260 PS overhead (encode, flush, start, poll, decode)": 2.35,
    # measured here: check_controls 0.11 ms + rollout and cost 0.53 ms
    "host decode, check, rollout": 1.0,
}
SOLVER_BUDGET_MS = CYCLE_BUDGET_MS - sum(NON_SOLVER_MS.values())  # 31.65 ms, ap_start -> ap_done
# Board vs HLS-report gap: the 3x24 HLS report gives 151,785 cycles per iteration (36.43 ms for
# 24 iterations) and the board measures 38.05 ms median solve: +4.45 %. (fix6 gave +3.7 %.)
UNEXPLAINED = 38.05 / (24 * 151_785 / CLOCK_HZ * 1e3) - 1.0
BRAM_WORDS = 2048  # each of x_mem / y_mem / v_mem (8 KiB) holds n_S * Nu * n_U words
MAX_ITERATIONS = 0x200 // 4  # nmpc_overlay.py: fitness history must end before find_local (0x200)

# Quadrotor calibration (3x24 HLS reports, 2026-10-25): evaluator 147,914 cycles for 3 particles
# (49,305 per particle); update 391 + 1,050 n_S at D = Nu*n_U = 100; detect 330. The D-scaling of
# update and detect below is the Ackermann proposal's assumption (3 cycles per element per pass).


def update_cycles(particles: int, dims: int) -> float:
    return 3 * dims + 91 + particles * 10.5 * dims


def detect_cycles(dims: int) -> float:
    return 3 * dims + 30


def solver_ms(particles: int, iterations: int, eval_per_particle: float, dims: int) -> tuple[float, float]:
    """(lower, upper) solver time ap_start -> ap_done (ms), WITHOUT PS overhead (that is a separate
    budget line). Upper adds the board-vs-report gap. ESTIMATE."""
    cycles = iterations * (particles * eval_per_particle + update_cycles(particles, dims) + detect_cycles(dims))
    base = cycles / CLOCK_HZ * 1e3
    return base, base * (1.0 + UNEXPLAINED)


def cycle_ms(particles: int, iterations: int, eval_per_particle: float, dims: int) -> tuple[float, float]:
    """(lower, upper) cycle latency C = solver + every non-solver budget line (ms)."""
    low, high = solver_ms(particles, iterations, eval_per_particle, dims)
    rest = sum(NON_SOLVER_MS.values())
    return low + rest, high + rest


def max_particles(eval_per_particle: float, dims: int, iterations: int) -> int:
    """Largest n_S whose upper solver estimate fits SOLVER_BUDGET_MS, capped by BRAM (n_S*D <= 2048)."""
    best = 0
    for particles in range(2, BRAM_WORDS // dims + 1):
        if solver_ms(particles, iterations, eval_per_particle, dims)[1] <= SOLVER_BUDGET_MS:
            best = particles
    return best


# name: ((cycles per horizon step low, high), per-call overhead, n_U)
STEP_ESTIMATES = {
    "Q: quadrotor RK4 (measured, D=100)": ((1955, 1955), 430, 4),
    "A: unicycle RK4, model swap only": ((435, 515), 50, 2),
    "B1: chord, half angle, shared index": ((97, 112), 50, 2),
    "B2: chord, fixed angle, nearest entry": ((97, 117), 50, 2),
    "B3: chord, fixed angle, interpolated": ((112, 132), 50, 2),
}
ITERATION_OPTIONS = (24, 30, 50, 100, 128)


def budget_lines(horizon: int = 25) -> list[str]:
    lines = []
    for name, ((low, high), overhead, n_u) in STEP_ESTIMATES.items():
        for nu in ((25,) if n_u == 4 else (25, 10)):
            dims = nu * n_u
            for label, step_cycles in (("low", low), ("high", high)):
                per_particle = horizon * step_cycles + overhead
                opts = ", ".join(f"{max_particles(per_particle, dims, it)}x{it}" for it in ITERATION_OPTIONS)
                lines.append(f"{name:40s} Nu={nu:2d} {label:4s} eval/particle {per_particle:6.0f}  {opts}")
    return lines


def main() -> None:
    print("Cycle-latency budget C = t_ready - t_tick (ms):")
    for item, value in NON_SOLVER_MS.items():
        print(f"  {item:62s} {value:5.2f}")
    print(f"  {'solver ap_start -> ap_done (upper estimate must fit)':62s} {SOLVER_BUDGET_MS:5.2f}")
    print(f"  {'total C budget':62s} {CYCLE_BUDGET_MS:5.2f}  (+5 guard +5 margin = {PERIOD_MS:.0f})")
    q = solver_ms(3, 24, 49_305, 100)
    print(f"Calibration: gap {UNEXPLAINED * 100:.2f} %; quadrotor 3x24 solver {q[0]:.2f}-{q[1]:.2f} ms "
          f"(board median 38.05 ms): it would NOT fit this {SOLVER_BUDGET_MS:.2f} ms solver budget")
    print(f"BRAM particle cap: n_S <= {BRAM_WORDS // 50} at Nu=25, {BRAM_WORDS // 20} at Nu=10; "
          f"iterations <= {MAX_ITERATIONS}")
    print("Largest n_S x iterations whose upper solver estimate fits the budget:")
    for line in budget_lines():
        print(line)
    for n_s, it, nu in ((4, 50, 10), (5, 50, 25), (8, 30, 25), (9, 30, 25), (8, 100, 25), (9, 100, 25),
                        (10, 100, 25), (15, 50, 25), (18, 50, 25)):
        for name in ("A: unicycle RK4, model swap only", "B1: chord, half angle, shared index",
                     "B2: chord, fixed angle, nearest entry", "B3: chord, fixed angle, interpolated"):
            (low, high), overhead, _ = STEP_ESTIMATES[name]
            s_lo = solver_ms(n_s, it, 25 * low + overhead, nu * 2)[0]
            s_hi = solver_ms(n_s, it, 25 * high + overhead, nu * 2)[1]
            c_lo = cycle_ms(n_s, it, 25 * low + overhead, nu * 2)[0]
            c_hi = cycle_ms(n_s, it, 25 * high + overhead, nu * 2)[1]
            print(f"  {n_s}x{it} Nu={nu}: {name:40s} solver {s_lo:6.1f}-{s_hi:6.1f} ms   C {c_lo:6.1f}-{c_hi:6.1f} ms")
    cases = (("A300 controller limits |v|,|omega| <= 2", 2.0, 2.0),
             ("Create 3 safety-on limits v <= 0.306, omega <= 1.9", 0.306, 1.9))
    for local in (True, False):
        for label, v_max, w_max in cases:
            print(f"\nPosition error vs double exact step, {label}, Ts = 0.05 s, Nh = 25, "
                  f"{'local frame (start 0,0,0)' if local else 'world frame (random pose)'}")
            for name, (one, full) in compare_discretizations(v_max, w_max, 0.05, 25, local).items():
                print(f"  {name:42s} 1 step {one:9.2e} m   25 steps {full:9.2e} m")


if __name__ == "__main__":
    main()
