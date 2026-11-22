"""Estimates behind sections 9-10 of docs/ackermann/ackermann_mpc_pso_model.md.

1. A bit-level emulation (not a C simulation) of the Ackermann step in the hardware arithmetic:
   ap_fixed<32,17> top-level values (rounded to 2^-15) and a half-precision model with the
   1024-entry table sine of fast_sin_cos.hpp / local_sin. It compares the existing RK4 path of
   hls_system.hpp with the proposed exact-arc chord step, against the double exact arc.
2. The cycle-budget model, calibrated on the HLS reports of the 3x24 quadrotor build.

Run: python3 app/ackermann/hw_estimates.py
All speeds/curvatures here are PLACEHOLDER ranges for the analysis, not vehicle values.
"""

from __future__ import annotations

import math
from pathlib import Path
import sys

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import ackermann_model as am  # noqa: E402


# ----------------------------------------------------------------------------------------------
# Hardware arithmetic emulation
# ----------------------------------------------------------------------------------------------

F16 = np.float16
FIXED_LSB = 2.0 ** -15  # ap_fixed<32,17>: 15 fractional bits
TABLE = np.sin(2.0 * np.pi * np.arange(1024) / 1024).astype(F16)  # table held as half
# HALF_MAX_CIRCLE_ANGLE_PI = 512 / 3.14159265358979323846f, converted to half -> 163.0
INDEX_SCALE = F16(np.float32(512.0) / np.float32(3.14159265358979323846))
HALF_PI = F16(math.pi / 2)


def fx(value: float) -> float:
    """Round to the ap_fixed<32,17> grid (AP_RND_INF ~ round to nearest, ties away from 0)."""
    scaled = value / FIXED_LSB
    return float(math.copysign(math.floor(abs(scaled) + 0.5), scaled) * FIXED_LSB)


def table_index(angle: np.float16) -> int:
    """fastsin(angle * HALF_MAX_CIRCLE_ANGLE_PI) as local_sin computes it, all in half."""
    n = F16(angle * INDEX_SCALE)
    rounded = F16(n + F16(0.5)) if n >= 0 else F16(n - F16(0.5))
    return int(np.int16(int(rounded)))  # (int16_t) truncates toward zero


def local_sin(angle: np.float16) -> np.float16:
    return TABLE[table_index(angle) & 1023]


def local_cos_via_sin(angle: np.float16) -> np.float16:
    """The sniffbot model's cos: local_sin(angle + pi/2), the add in half."""
    return local_sin(F16(angle + HALF_PI))


def sincos_shared_index(angle: np.float16) -> tuple[np.float16, np.float16]:
    """Proposed helper: one rounded index, cos = table[(i + 256) & 1023] (exact quarter turn)."""
    index = table_index(angle)
    return TABLE[index & 1023], TABLE[(index + 256) & 1023]


def wrap_fixed(psi: float) -> float:
    return fx(am.wrap_angle(psi))


def step_rk4_hw(state: np.ndarray, control: np.ndarray, ts: float) -> np.ndarray:
    """System::one_step_prediction with the kinematic model in half (sniffbot-style cos)."""
    ts_fixed, ts_2, ts_6 = fx(ts), fx(ts * 0.5), fx(ts * 0.1666666667)
    v16, kappa16 = F16(control[0]), F16(control[1])

    def model(local_state: np.ndarray) -> np.ndarray:
        psi16 = F16(local_state[2])
        return np.array([fx(float(F16(v16 * local_cos_via_sin(psi16)))),
                         fx(float(F16(v16 * local_sin(psi16)))),
                         fx(float(F16(v16 * kappa16)))])

    def update(base: np.ndarray, k: np.ndarray, h: float) -> np.ndarray:
        return np.array([fx(base[i] + fx(h * k[i])) for i in range(3)])

    k1 = model(state)
    k2 = model(update(state, k1, ts_2))
    k3 = model(update(state, k2, ts_2))
    k4 = model(update(state, k3, ts_fixed))
    out = np.array([fx(state[i] + fx(ts_6 * fx(fx(k1[i] + k4[i]) + fx(2.0 * fx(k2[i] + k3[i])))))
                    for i in range(3)])
    out[2] = wrap_fixed(out[2])
    return out


def step_chord_hw(state: np.ndarray, control: np.ndarray, ts: float, shared_index: bool = True) -> np.ndarray:
    """Proposed exact-arc chord step: half model returns (dx, dy, dpsi); the add is in ap_fixed."""
    psi16, v16, kappa16 = F16(state[2]), F16(control[0]), F16(control[1])
    distance = F16(v16 * F16(ts))
    dpsi = F16(distance * kappa16)
    half = F16(dpsi * F16(0.5))
    sinc = F16(F16(1.0) - F16(F16(half * half) * F16(1.0 / 6.0)))
    chord = F16(distance * sinc)
    phi = F16(psi16 + half)
    if shared_index:
        sin_phi, cos_phi = sincos_shared_index(phi)
    else:
        sin_phi, cos_phi = local_sin(phi), local_cos_via_sin(phi)
    dx, dy = float(F16(chord * cos_phi)), float(F16(chord * sin_phi))
    return np.array([fx(state[0] + fx(dx)), fx(state[1] + fx(dy)), wrap_fixed(state[2] + fx(float(dpsi)))])


def sincos_fixed_index(phi: float, interpolate: bool) -> tuple[np.float16, np.float16]:
    """Proposed trig: the table position from the ap_fixed angle (not from a half copy of it).

    position = phi * 1024 / (2 pi) in ap_fixed; then either round to the nearest entry or
    interpolate linearly between two entries (the fraction and the blend in half).
    """
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


def step_chord_hw_fixed_angle(state: np.ndarray, control: np.ndarray, ts: float, interpolate: bool) -> np.ndarray:
    """Chord step with s, dpsi and the table position in ap_fixed; sinc, blend and products in half."""
    distance = fx(fx(control[0]) * fx(ts))
    dpsi = fx(distance * fx(control[1]))
    half = dpsi * 0.5  # exact shift in ap_fixed
    sin_phi, cos_phi = sincos_fixed_index(state[2] + half, interpolate)
    half16 = F16(half)
    sinc = F16(F16(1.0) - F16(F16(half16 * half16) * F16(1.0 / 6.0)))
    chord = F16(F16(distance) * sinc)
    dx, dy = float(F16(chord * cos_phi)), float(F16(chord * sin_phi))
    return np.array([fx(state[0] + fx(dx)), fx(state[1] + fx(dy)), wrap_fixed(state[2] + dpsi)])


def step_arc_division_hw(state: np.ndarray, control: np.ndarray, ts: float) -> np.ndarray:
    """The review's division form in the same arithmetic, to show its cancellation."""
    psi16, v16, kappa16 = F16(state[2]), F16(control[0]), F16(control[1])
    dpsi = F16(F16(v16 * F16(ts)) * kappa16)
    if abs(float(dpsi)) < 1e-3 or float(kappa16) == 0.0:
        distance = F16(v16 * F16(ts))
        dx, dy = F16(distance * local_cos_via_sin(psi16)), F16(distance * local_sin(psi16))
    else:
        end = F16(psi16 + dpsi)
        inverse = F16(F16(1.0) / kappa16)
        dx = F16(F16(local_sin(end) - local_sin(psi16)) * inverse)
        dy = F16(F16(local_cos_via_sin(psi16) - local_cos_via_sin(end)) * inverse)
    return np.array([fx(state[0] + fx(float(dx))), fx(state[1] + fx(float(dy))),
                     wrap_fixed(state[2] + fx(float(dpsi)))])


def compare_discretizations(
    v_max: float, kappa_max: float, ts: float, horizon: int, trials: int = 300, seed: int = 0
) -> dict[str, tuple[float, float]]:
    """Worst position error (m) after 1 step and after `horizon` steps vs the double exact arc.

    Random constant (v, kappa) per trial in [0, v_max] x [-kappa_max, kappa_max], random psi0
    and a random start position within 20 m of the origin.
    """
    rng = np.random.default_rng(seed)
    methods = {
        "euler (double)": lambda s, u: am.step_euler(s, u, ts),
        "midpoint (double)": lambda s, u: am.step_midpoint(s, u, ts),
        "rk4 (double)": lambda s, u: am.step_rk4(s, u, ts),
        "rk4, hw arithmetic": lambda s, u: step_rk4_hw(s, u, ts),
        "chord, hw arithmetic, shared index": lambda s, u: step_chord_hw(s, u, ts, True),
        "chord, hw arithmetic, cos via +pi/2": lambda s, u: step_chord_hw(s, u, ts, False),
        "chord, fixed-point angle, nearest entry": lambda s, u: step_chord_hw_fixed_angle(s, u, ts, False),
        "chord, fixed-point angle, interpolated": lambda s, u: step_chord_hw_fixed_angle(s, u, ts, True),
        "arc division form, hw arithmetic": lambda s, u: step_arc_division_hw(s, u, ts),
    }
    worst = {name: [0.0, 0.0] for name in methods}
    for _ in range(trials):
        control = np.array([rng.uniform(0.0, v_max), rng.uniform(-kappa_max, kappa_max)])
        start = np.array([fx(rng.uniform(-20, 20)), fx(rng.uniform(-20, 20)), fx(rng.uniform(-math.pi, math.pi))])
        exact = [start]
        for _ in range(horizon):
            exact.append(am.step_exact_arc_chord(exact[-1], control, ts))
        for name, method in methods.items():
            state = start.copy()
            for k in range(horizon):
                state = method(state, control)
                if k == 0:
                    worst[name][0] = max(worst[name][0], float(np.hypot(*(state[:2] - exact[1][:2]))))
            worst[name][1] = max(worst[name][1], float(np.hypot(*(state[:2] - exact[-1][:2]))))
    return {name: (values[0], values[1]) for name, values in worst.items()}


# ----------------------------------------------------------------------------------------------
# Cycle budget (100 MHz PL clock)
# ----------------------------------------------------------------------------------------------

CLOCK_HZ = 100e6
UNEXPLAINED = 0.037  # board vs HLS-report gap measured on fix6
PS_OVERHEAD_MS = 1.9  # measured max PS-side overhead per solve
TARGET_MS = 40.0  # 50 ms period with 20 % margin

# Calibration from the 3x24 quadrotor HLS reports (2026-10-25) and fix6 (10x100):
# evaluate 48,977 cycles per particle (25 x 1,955 + 102); update 391 + 1,050 per particle for
# D = Nu * n_U = 100 (fit to 3,541 @ 3 and 10,891 @ 10 particles); detect 330 (300 = 3 x D).


def update_cycles(particles: int, decision_dims: int) -> float:
    return 3 * decision_dims + 91 + particles * 10.5 * decision_dims


def detect_cycles(decision_dims: int) -> float:
    return 3 * decision_dims + 30


def solve_ms(particles: int, iterations: int, eval_per_particle: float, decision_dims: int) -> tuple[float, float]:
    """(lower, upper) solve time incl. PS overhead; upper adds the unexplained 3.7 %."""
    cycles = iterations * (particles * eval_per_particle + update_cycles(particles, decision_dims)
                           + detect_cycles(decision_dims))
    base = cycles / CLOCK_HZ * 1e3
    return base + PS_OVERHEAD_MS, base * (1 + UNEXPLAINED) + PS_OVERHEAD_MS


def max_evaluations(eval_per_particle: float, decision_dims: int, iterations: int) -> int:
    """Largest particle count that meets TARGET_MS (upper estimate) at the given iterations."""
    best = 0
    for particles in range(1, 200):
        if solve_ms(particles, iterations, eval_per_particle, decision_dims)[1] <= TARGET_MS:
            best = particles
    return best


STEP_ESTIMATES = {
    # name: (cycles per horizon step low, high), per-call overhead, decision dims
    "quadrotor RK4 (measured)": ((1955, 1955), 102, 100),
    "Ackermann RK4, Nx=3, n_U=2 (estimate)": ((435, 515), 50, 50),
    "Ackermann chord, Nx=3, n_U=2 (estimate)": ((105, 120), 50, 50),
    "Ackermann chord + interpolated trig (estimate)": ((120, 135), 50, 50),
    "Ackermann chord, Nu=10 < Nh=25 (estimate)": ((105, 120), 50, 20),
}


def budget_table(horizon: int = 25) -> list[str]:
    lines = []
    for name, ((low, high), overhead, dims) in STEP_ESTIMATES.items():
        for label, step in (("low", low), ("high", high)):
            per_particle = horizon * step + overhead
            options = []
            for iterations in (24, 30, 50, 100):
                particles = max_evaluations(per_particle, dims, iterations)
                options.append(f"{particles}x{iterations}")
            lines.append(f"{name:48s} {label:4s} eval/particle {per_particle:6.0f}  "
                         f"max particles x iterations within {TARGET_MS:.0f} ms: {', '.join(options)}")
    return lines


def main() -> None:
    print("Check: quadrotor 3x24 ->", ["%.1f ms" % t for t in solve_ms(3, 24, 48977, 100)])
    for line in budget_table():
        print(line)
    for v_max, kappa_max in ((3.0, 0.5), (5.0, 1.0)):
        print(f"\nDiscretization error, PLACEHOLDER range |v| <= {v_max} m/s, |kappa| <= {kappa_max} 1/m, "
              f"Ts = 0.05 s, Nh = 25 (max |dpsi| = {v_max * kappa_max * 0.05:.3f} rad/step)")
        for name, (one, full) in compare_discretizations(v_max, kappa_max, 0.05, 25).items():
            print(f"  {name:42s} 1 step {one:9.2e} m   25 steps {full:9.2e} m")


if __name__ == "__main__":
    main()
