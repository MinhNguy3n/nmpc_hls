"""Emulated command chain of the RC car: command link -> output hold -> VESC speed loop + servo.

Plain Python (no ROS), tested offline (test_actuator_model.py). The ROS node actuator_node.py
feeds it the mux output and publishes the applied (speed, steering angle) to the plant interface.

Stages, in the order of the planned vehicle chain (KV260 -> Pixhawk 4 -> PWM -> VESC + servo):
  1. Command link: a command sent at t reaches the output stage at t + T_link.
  2. Output hold: the ESC and servo outputs are updated as zero-order holds at f_esc_out and
     f_servo_out, each with a random phase per run (one PWM period at 50 Hz = 20 ms). An update
     takes the latest command that has arrived. f = 0 means no hold.
  3. VESC: the speed target (clipped to +-v_max) is followed by a first-order lag tau_v, with the
     change limited by accel_max (speeding up) and decel_max (slowing down).
  4. Servo: the angle target (clipped to +-delta_max) takes effect T_d after the output update
     (dead time), then the angle moves towards it at most at delta_rate_max.

Events (arrivals, hold updates, dead-time releases) are handled at their exact times; the
continuous parts (lag, rate limits) are integrated between events in steps of at most `h_max`.
A non-finite command is replaced by (0, 0).

Paired seeds (phase 3 condition 3): with an integer seed, the two hold phases are a function of
the seed only, and rephase(t_start) places them relative to an episode start, so two runs with
the same seed have identical PWM update times relative to their episode starts.

All profile values are assumed (params/command_chain.yaml); none is a measured vehicle value.
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import math
from pathlib import Path
from typing import Optional

import numpy as np
import yaml

DEFAULT_CHAIN = Path(__file__).resolve().parent / "params" / "command_chain.yaml"
PROFILE_FIELDS = ("T_link", "f_esc_out", "f_servo_out", "tau_v", "accel_max", "decel_max", "T_d", "delta_rate_max")
_EPS = 1e-12


@dataclass(frozen=True)
class ActuatorProfile:
    name: str
    T_link: float
    f_esc_out: float
    f_servo_out: float
    tau_v: float
    accel_max: Optional[float]
    decel_max: Optional[float]
    T_d: float
    delta_rate_max: Optional[float]
    status: str = "placeholder"

    def __post_init__(self) -> None:
        for name in ("T_link", "f_esc_out", "f_servo_out", "tau_v", "T_d"):
            value = getattr(self, name)
            if not (math.isfinite(value) and value >= 0.0):
                raise ValueError(f"{self.name}.{name} must be finite and >= 0, not {value!r}")
        for name in ("accel_max", "decel_max", "delta_rate_max"):
            value = getattr(self, name)
            if value is not None and not (math.isfinite(value) and value > 0.0):
                raise ValueError(f"{self.name}.{name} must be > 0 or null, not {value!r}")


def load_profiles(path: str | Path = DEFAULT_CHAIN) -> dict[str, ActuatorProfile]:
    data = yaml.safe_load(Path(path).read_text())["actuator_profiles"]
    profiles = {}
    for name, entry in data.items():
        missing = [f for f in PROFILE_FIELDS if f not in entry]
        if missing:
            raise ValueError(f"profile {name} lacks {missing}")
        values = {f: (None if entry[f] is None else float(entry[f])) for f in PROFILE_FIELDS}
        for f in ("T_link", "f_esc_out", "f_servo_out", "tau_v", "T_d"):
            if values[f] is None:
                raise ValueError(f"profile {name}: {f} may not be null")
        profiles[name] = ActuatorProfile(name=name, status=str(entry.get("status", "placeholder")), **values)
    return profiles


def load_profile(name: str, path: str | Path = DEFAULT_CHAIN) -> ActuatorProfile:
    profiles = load_profiles(path)
    if name not in profiles:
        raise KeyError(f"unknown actuator profile {name!r}; have {sorted(profiles)}")
    return profiles[name]


def _limit(step: float, limit: Optional[float], dt: float) -> float:
    if limit is None:
        return step
    bound = limit * dt
    return max(-bound, min(bound, step))


class ActuatorModel:
    """The emulated chain. Call command() when a command is sent, advance(t) to move to time t,
    and read .speed and .delta (the applied values at the current time)."""

    def __init__(self, profile: ActuatorProfile, delta_max: float, v_max: float,
                 rng: np.random.Generator | int | None = None, t0: float = 0.0, h_max: float = 2.5e-4) -> None:
        if not (delta_max > 0 and v_max > 0):
            raise ValueError("delta_max and v_max must be > 0")
        self.p, self.delta_max, self.v_max, self.h_max = profile, delta_max, v_max, h_max
        if isinstance(rng, (int, np.integer)):
            rng = np.random.default_rng(int(rng))
        rng = rng if rng is not None else np.random.default_rng()
        self.t = t0
        self.link: deque[tuple[float, float, float]] = deque()  # (arrival, v, delta), arrival-ordered
        self.arrived = (0.0, 0.0)  # latest command at the output stage
        # random hold phases: the first update lies in [t0, t0 + period)
        self._draw_phases(rng)
        self.next_esc = t0 + self.esc_phase if profile.f_esc_out > 0 else math.inf
        self.next_servo = t0 + self.servo_phase if profile.f_servo_out > 0 else math.inf
        self.dead: deque[tuple[float, float]] = deque()  # (release time, delta target), servo dead time
        self.v_target = 0.0
        self.delta_target = 0.0
        self.speed = 0.0
        self.delta = 0.0
        self.servo_updates: list[float] = []  # times of servo output updates (tests, diagnostics)

    def _draw_phases(self, rng: np.random.Generator) -> None:
        p = self.p
        self.esc_phase = float(rng.uniform(0.0, 1.0 / p.f_esc_out)) if p.f_esc_out > 0 else 0.0
        self.servo_phase = float(rng.uniform(0.0, 1.0 / p.f_servo_out)) if p.f_servo_out > 0 else 0.0

    def rephase(self, t_start: float, seed: int | None = None) -> None:
        """Place the hold phases relative to an episode start t_start (>= the model time): the
        next ESC and servo updates are at t_start + phase; with `seed`, the phases are first drawn
        again from that seed. No output update happens between now and then (call it while the
        mux holds the car)."""
        if t_start < self.t - _EPS:
            raise ValueError("rephase: t_start lies in the past")
        if seed is not None:
            self._draw_phases(np.random.default_rng(int(seed)))
        if self.p.f_esc_out > 0:
            self.next_esc = t_start + self.esc_phase
        if self.p.f_servo_out > 0:
            self.next_servo = t_start + self.servo_phase

    # -- inputs ---------------------------------------------------------------------------------
    def command(self, t: float, speed: float, delta: float) -> None:
        """A command sent at time t (t >= the model time; earlier times are taken as now)."""
        if not (math.isfinite(speed) and math.isfinite(delta)):
            speed, delta = 0.0, 0.0
        if t > self.t + _EPS:
            self.advance(t)
        arrival = max(t, self.t) + self.p.T_link
        if self.link and arrival < self.link[-1][0]:
            arrival = self.link[-1][0]  # a serial link does not reorder
        self.link.append((arrival, float(speed), float(delta)))
        self._events_at(self.t)  # T_link = 0: the command arrives (and, without hold, applies) now

    # -- time -----------------------------------------------------------------------------------
    def advance(self, t_end: float) -> tuple[float, float]:
        while self.t < t_end - _EPS:
            t_next = min(t_end, self._next_event())
            self._integrate(t_next)
            self.t = t_next
            self._events_at(self.t)
        return self.speed, self.delta

    def _next_event(self) -> float:
        candidates = [self.next_esc, self.next_servo]
        if self.link:
            candidates.append(self.link[0][0])
        if self.dead:
            candidates.append(self.dead[0][0])
        return min(candidates)

    def _events_at(self, t: float) -> None:
        # 1. arrivals (an arrival and an update at the same instant: the update sees the arrival)
        while self.link and self.link[0][0] <= t + _EPS:
            _, v, d = self.link.popleft()
            self.arrived = (v, d)
            if self.p.f_esc_out == 0:
                self._set_speed_target(v)
            if self.p.f_servo_out == 0:
                self._servo_signal(t, d)
        # 2. hold updates
        if self.next_esc <= t + _EPS:
            self._set_speed_target(self.arrived[0])
            while self.next_esc <= t + _EPS:
                self.next_esc += 1.0 / self.p.f_esc_out
        if self.next_servo <= t + _EPS:
            self._servo_signal(t, self.arrived[1])
            self.servo_updates.append(t)
            while self.next_servo <= t + _EPS:
                self.next_servo += 1.0 / self.p.f_servo_out
        # 3. dead-time releases
        while self.dead and self.dead[0][0] <= t + _EPS:
            self.delta_target = self.dead.popleft()[1]
            if self.p.delta_rate_max is None:
                self.delta = self.delta_target

    def _set_speed_target(self, v: float) -> None:
        self.v_target = max(-self.v_max, min(self.v_max, v))
        if self.p.tau_v == 0 and self.p.accel_max is None and self.p.decel_max is None:
            self.speed = self.v_target

    def _servo_signal(self, t: float, d: float) -> None:
        d = max(-self.delta_max, min(self.delta_max, d))
        if self.p.T_d == 0:
            self.delta_target = d
            if self.p.delta_rate_max is None:
                self.delta = d
        else:
            self.dead.append((t + self.p.T_d, d))

    def _integrate(self, t1: float) -> None:
        span = t1 - self.t
        if span <= 0:
            return
        n = max(1, math.ceil(span / self.h_max - 1e-9))
        h = span / n
        decay = math.exp(-h / self.p.tau_v) if self.p.tau_v > 0 else 0.0
        for _ in range(n):
            v_des = self.v_target + (self.speed - self.v_target) * decay
            dv = v_des - self.speed
            slowing = self.speed != 0.0 and dv * self.speed < 0.0
            self.speed += _limit(dv, self.p.decel_max if slowing else self.p.accel_max, h)
            self.delta += _limit(self.delta_target - self.delta, self.p.delta_rate_max, h)


def servo_start_delays(profile: ActuatorProfile, n: int, rng: np.random.Generator,
                       step: float = 0.25, delta_max: float = 0.45, threshold: float = 1e-3) -> np.ndarray:
    """Monte Carlo of the chain's own delay from sending a steering step to the servo starting to
    move (angle beyond `threshold`), over random hold phases and send times. For the STOP 2 delay
    distribution, next to the delay measured in Gazebo (which adds the plant and sampling)."""
    out = np.empty(n)
    for i in range(n):
        model = ActuatorModel(profile, delta_max, 2.0, rng)
        t_send = float(rng.uniform(0.0, 0.1))
        model.advance(t_send)
        model.command(t_send, 0.0, step)
        t, dt = t_send, 1e-4
        while abs(model.delta) <= threshold:
            t += dt
            model.advance(t)
            if t - t_send > 1.0:
                raise RuntimeError("servo never moved")
        out[i] = t - t_send
    return out
