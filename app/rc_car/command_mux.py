"""Command mux, setpoint watchdog, kill lock and mocap freshness gate of the RC car (plain Python).

The semantics mirror the flight controller planned for the car (Pixhawk 4 with PX4; 2026-11-22
review, section 4), with generic names; the PX4 counterpart is noted for each rule and in
params/command_chain.yaml. The ROS node command_mux_node.py feeds it events; tested offline in
test_command_mux.py.

Modes
    HOLD        disarmed: speed 0, steering held at the last output. Leaving it needs an explicit
                request_arm() (<-> re-entering offboard mode). Start state.
    CONTROLLER  armed: the controller's setpoints pass through (<-> offboard).
    TELEOP      the operator overrides (<-> RC override / the transmitter's mode switch). Teleop has
                top priority: any teleop command takes over from any mode except LOCKED, and the
                controller is disarmed.
    LOCKED      the kill lock is engaged: speed 0, steering held. Releasing the lock gives HOLD.

Rules (fail closed; no automatic resume anywhere)
    * Controller setpoints older than T_wd at arrival (header stamp vs now), or with a stamp not
      newer than the last accepted one, are dropped and counted.
    * CONTROLLER with no accepted setpoint for T_wd -> HOLD ("setpoint_timeout"; <-> COM_OF_LOSS_T,
      then the COM_OBL_RC_ACT action, here Hold). New setpoints do not re-arm.
    * TELEOP with no teleop command for T_teleop -> HOLD ("teleop_released"), never back to
      CONTROLLER by itself.
    * Arming needs: HOLD, the lock released, teleop silent for T_teleop, an accepted controller
      setpoint not older than T_arm_fresh (<-> PX4 accepts offboard only while setpoints stream)
      and, if a freshness gate is attached, a fresh pose. After arming the mux outputs the hold
      until the next setpoint arrives.
    * The freshness gate disarms CONTROLLER when the pose age exceeds T_fresh ("mocap_stale").
      It never re-arms: the operator must call request_arm() again once the pose is fresh.

This replaces app/husky/command_ownership.py for the car. That module infers ownership from the
twist_mux output (stamp matching), because on the Husky the mux is not ours. On the car the mux
is this module: it is the single authority over the command, so no inference is needed; the
fail-closed rules (explicit re-arm, no resume after a takeover, stale -> stop) are kept.

Times are seconds on one clock (sim time in simulation).
"""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
import math
from pathlib import Path
from typing import Optional

import yaml

DEFAULT_CHAIN = Path(__file__).resolve().parent / "params" / "command_chain.yaml"


class Mode(str, Enum):
    HOLD = "hold"
    CONTROLLER = "controller"
    TELEOP = "teleop"
    LOCKED = "locked"


@dataclass(frozen=True)
class MuxConfig:
    T_wd: float = 0.3
    T_teleop: float = 0.3
    T_arm_fresh: float = 0.1

    def __post_init__(self) -> None:
        for name in ("T_wd", "T_teleop", "T_arm_fresh"):
            if not (getattr(self, name) > 0 and math.isfinite(getattr(self, name))):
                raise ValueError(f"{name} must be finite and > 0")


def load_mux_config(path: str | Path = DEFAULT_CHAIN) -> MuxConfig:
    m = yaml.safe_load(Path(path).read_text())["mux"]
    return MuxConfig(T_wd=float(m["T_wd"]), T_teleop=float(m["T_teleop"]), T_arm_fresh=float(m["T_arm_fresh"]))


def load_gate_threshold(path: str | Path = DEFAULT_CHAIN) -> float:
    return float(yaml.safe_load(Path(path).read_text())["freshness_gate"]["T_fresh"])


class FreshnessGate:
    """Pose age = now - capture stamp of the newest pose seen (older, out-of-order poses are ignored).
    fresh(now) is False before the first pose and whenever the age exceeds T_fresh."""

    def __init__(self, T_fresh: float) -> None:
        if not (T_fresh > 0 and math.isfinite(T_fresh)):
            raise ValueError("T_fresh must be finite and > 0")
        self.T_fresh = T_fresh
        self.newest: Optional[float] = None
        self.trips = 0

    def on_pose(self, capture_stamp: float) -> None:
        if math.isfinite(capture_stamp) and (self.newest is None or capture_stamp > self.newest):
            self.newest = capture_stamp

    def age(self, now: float) -> float:
        return math.inf if self.newest is None else now - self.newest

    def fresh(self, now: float) -> bool:
        return self.age(now) <= self.T_fresh


@dataclass(frozen=True)
class Output:
    speed: float
    steering_angle: float
    source: str          # "controller", "teleop" or "hold"
    stamp: Optional[float] = None  # header stamp of the forwarded command (None for the hold)


@dataclass
class Counters:
    dropped_old: int = 0       # controller setpoint older than T_wd at arrival
    dropped_reordered: int = 0  # stamp not newer than the last accepted one
    ignored_disarmed: int = 0  # accepted, but not forwarded (not armed)


@dataclass
class CommandMux:
    config: MuxConfig = field(default_factory=MuxConfig)
    gate: Optional[FreshnessGate] = None
    mode: Mode = Mode.HOLD
    lock_engaged: bool = False
    last_teleop: float = -math.inf       # arrival time
    last_ctrl: float = -math.inf         # arrival time of the last accepted setpoint
    last_ctrl_stamp: float = -math.inf
    ctrl_since_arm: bool = False
    held_angle: float = 0.0
    counters: Counters = field(default_factory=Counters)
    events: list = field(default_factory=list)  # (time, from, to, reason)
    _last: Output = field(default_factory=lambda: Output(0.0, 0.0, "hold"))

    # -- helpers --------------------------------------------------------------------------------
    def _go(self, now: float, mode: Mode, reason: str) -> None:
        if mode != self.mode:
            self.events.append((now, self.mode.value, mode.value, reason))
            self.mode = mode

    def _hold(self) -> Output:
        return Output(0.0, self.held_angle, "hold")

    def output(self) -> Output:
        """The current output when no new command is being forwarded (keep-alive): the hold in
        HOLD and LOCKED and right after arming; otherwise the last forwarded command."""
        if self.mode in (Mode.HOLD, Mode.LOCKED):
            return self._hold()
        if self.mode == Mode.CONTROLLER and not self.ctrl_since_arm:
            return self._hold()
        return self._last

    def _forward(self, out: Output) -> Output:
        self._last = out
        self.held_angle = out.steering_angle
        return out

    # -- inputs ---------------------------------------------------------------------------------
    def on_teleop(self, now: float, speed: float, angle: float, stamp: Optional[float] = None) -> Optional[Output]:
        if self.lock_engaged:
            return None
        self.last_teleop = now
        if self.mode != Mode.TELEOP:
            self._go(now, Mode.TELEOP, "teleop_takeover")
            self.ctrl_since_arm = False
        if not (math.isfinite(speed) and math.isfinite(angle)):
            speed, angle = 0.0, self.held_angle
        return self._forward(Output(float(speed), float(angle), "teleop", stamp))

    def on_controller(self, now: float, speed: float, angle: float, stamp: float) -> Optional[Output]:
        if now - stamp > self.config.T_wd:
            self.counters.dropped_old += 1
            return None
        if stamp <= self.last_ctrl_stamp:
            self.counters.dropped_reordered += 1
            return None
        self.last_ctrl, self.last_ctrl_stamp = now, stamp
        if self.mode != Mode.CONTROLLER:
            self.counters.ignored_disarmed += 1
            return None
        if not (math.isfinite(speed) and math.isfinite(angle)):
            self.disarm(now, "non_finite_setpoint")
            return self._hold()
        self.ctrl_since_arm = True
        return self._forward(Output(float(speed), float(angle), "controller", stamp))

    def on_lock(self, now: float, engaged: bool) -> None:
        if engaged and not self.lock_engaged:
            self.lock_engaged = True
            self.ctrl_since_arm = False
            self._go(now, Mode.LOCKED, "lock_engaged")
        elif not engaged and self.lock_engaged:
            self.lock_engaged = False
            self._go(now, Mode.HOLD, "lock_released")

    def request_arm(self, now: float) -> tuple[bool, str]:
        if self.mode == Mode.CONTROLLER:
            return True, "already armed"
        reasons = []
        if self.mode != Mode.HOLD:
            reasons.append(f"mode is {self.mode.value}")
        if self.lock_engaged:
            reasons.append("lock engaged")
        if now - self.last_teleop <= self.config.T_teleop:
            reasons.append("teleop active")
        if now - self.last_ctrl > self.config.T_arm_fresh:
            reasons.append("no fresh controller setpoint")
        if self.gate is not None and not self.gate.fresh(now):
            reasons.append("pose not fresh")
        if reasons:
            return False, "; ".join(reasons)
        self.ctrl_since_arm = False
        self._go(now, Mode.CONTROLLER, "armed")
        return True, "armed"

    def disarm(self, now: float, reason: str = "operator") -> None:
        if self.mode == Mode.CONTROLLER:
            self.ctrl_since_arm = False
            self._go(now, Mode.HOLD, reason)

    def tick(self, now: float) -> Output:
        """Watchdogs; call periodically. Returns the keep-alive output."""
        if self.mode == Mode.CONTROLLER:
            if now - self.last_ctrl > self.config.T_wd:
                self.disarm(now, "setpoint_timeout")
            elif self.gate is not None and not self.gate.fresh(now):
                self.gate.trips += 1
                self.disarm(now, "mocap_stale")
        elif self.mode == Mode.TELEOP and now - self.last_teleop > self.config.T_teleop:
            self._go(now, Mode.HOLD, "teleop_released")
        return self.output()
