"""Command-ownership state machine and applied-command tracker for the A300 MPC node.

Plain Python, no ROS imports: the ROS node feeds it events and asks it, at every control tick,
whether it may solve and publish. Design: docs/husky_a300/unicycle_mpc_pso_model.md section 7.2;
evidence: plan section 2.1 (V10: twist_mux passes header.stamp through unchanged; cmd_vel_out is
re-stamped by the controller; V11: a single MPC stamp leaked through the mux during a teleop
period; the platform/safety_stop lock latches until released).

States
    DISARMED  not armed (start, explicit disarm, after a foreign takeover, a safety lock or a
              failed arming). Nothing is solved or published. Leaving it needs request_arm().
    ARMING    armed by the operator; the node may publish only a hold probe (the applied command)
              so that the mux output can confirm the channel; no solve. -> OWNED after
              `confirmations` distinct fresh exact matches; -> DISARMED on timeout.
    OWNED     fresh exact stamp+twist match with a recent own command, no foreign source
              selected, no lock engaged: solve and publish.
    UNKNOWN   armed, but the mux output (or the source signal, if configured) is missing or stale:
              no solve, no publish. Fresh matching evidence returns it to OWNED within
              `unknown_grace`; otherwise -> DISARMED.
    FOREIGN   a foreign source is selected, or the mux output does not match an own command:
              no publish, warm start invalidated. When the foreign evidence ends -> DISARMED.

Rules that make it fail closed
    * Ownership needs an exact match of (stamp, twist) to an entry of the bounded recent history
      of own commands, published after the current arming and not older than `max_match_age`.
    * Any mismatch or foreign selection -> FOREIGN at once, from any armed state.
    * Re-arming is only explicit (request_arm), only with no lock engaged, no foreign source
      selected, and a FRESH applied command (cmd_vel_out) inside the MPC envelope. The own-command
      history is cleared on arming, so an old MPC message arriving after a takeover can never
      count as evidence (and in DISARMED/FOREIGN no message re-arms anything).
    * A low-priority (0, 0) publication is never treated as a stop: in every state other than
      ARMING/OWNED the node publishes nothing; real stops come from the independent safety path.

Times are seconds on one clock (sim time in simulation). Stamps are integer nanoseconds
(sec * 10**9 + nanosec) so that matching is exact.
"""

from __future__ import annotations

from collections import OrderedDict
from dataclasses import dataclass
from enum import Enum
from typing import Callable

import numpy as np


class State(str, Enum):
    DISARMED = "disarmed"  # = rearm_required
    ARMING = "arming"
    OWNED = "owned"
    UNKNOWN = "unknown"
    FOREIGN = "foreign"


def stamp_ns(sec: int, nanosec: int) -> int:
    """builtin_interfaces/Time -> integer nanoseconds (exact)."""
    return int(sec) * 1_000_000_000 + int(nanosec)


@dataclass(frozen=True)
class OwnershipConfig:
    """All ages in seconds. Defaults are PLACEHOLDER_ policy values for Ts = 0.05 s, to be tuned
    from the V13 freshness measurements."""

    history_size: int = 20  # own commands kept (1 s at 20 Hz)
    max_match_age: float = 0.15  # an own command older than this is not ownership evidence
    max_mux_age: float = 0.12  # mux output silence tolerated before UNKNOWN (2.4 periods)
    unknown_grace: float = 0.25  # UNKNOWN longer than this -> DISARMED
    confirmations: int = 2  # distinct fresh matches needed in ARMING (hysteresis, V11 leak)
    arming_timeout: float = 0.5
    max_applied_age: float = 0.06  # cmd_vel_out age bound for re-arm and u_prev (3 controller periods)
    use_source_signal: bool = False  # True if the mux exposes its selected input (plan V12)
    max_source_age: float = 0.5
    own_source: str = "external"  # the twist_mux topic name of the MPC input ("external" -> cmd_vel)


@dataclass(frozen=True)
class AppliedSample:
    """A cmd_vel_out sample: the controller-limited command actually applied."""

    stamp: float  # its own header stamp (s), re-stamped by the controller (V10)
    received: float  # node clock (s)
    v: float
    omega: float


class AppliedCommandTracker:
    """u_prev = the latest cmd_vel_out sample whose age is within the bound.

    The sample is ordered and aged by its OWN stamp (the controller re-stamps it, V10); it is never
    aligned to mux stamps. Out-of-order (older) samples are ignored. A sample stamped in the future
    beyond `future_tolerance` is rejected (clock mismatch, e.g. use_sim_time not set).
    """

    def __init__(self, max_age: float, future_tolerance: float = 0.005) -> None:
        if not max_age > 0.0:
            raise ValueError("max_age must be positive")
        self.max_age = max_age
        self.future_tolerance = future_tolerance
        self.latest: AppliedSample | None = None

    def on_sample(self, sample: AppliedSample) -> None:
        if not (np.isfinite(sample.v) and np.isfinite(sample.omega)):
            return
        if self.latest is None or sample.stamp > self.latest.stamp:
            self.latest = sample

    def u_prev(self, now: float) -> np.ndarray | None:
        """[v, omega] of the freshest sample, or None if missing, stale or from the future."""
        if self.latest is None:
            return None
        age = now - self.latest.stamp
        if age > self.max_age or age < -self.future_tolerance or now - self.latest.received > self.max_age:
            return None
        return np.array([self.latest.v, self.latest.omega])


@dataclass(frozen=True)
class Transition:
    time: float
    old: State
    new: State
    reason: str


class OwnershipStateMachine:
    """Explicit ownership state machine; see the module docstring."""

    def __init__(self, config: OwnershipConfig | None = None) -> None:
        self.config = config or OwnershipConfig()
        self.state = State.DISARMED
        self.reason = "start"
        self.transitions: list[Transition] = []
        self._history: OrderedDict[int, tuple[float, float, float]] = OrderedDict()  # stamp -> (v, w, sent)
        self._last_published_stamp: int | None = None
        self._last_mux_received: float | None = None
        self._last_match_time: float | None = None
        self._confirmed: set[int] = set()
        self._entered: float = 0.0
        self._source: tuple[str, float] | None = None  # (selected source, received)
        self._locks: dict[str, bool] = {}
        self._warm_start_invalidated = True

    # ---------------------------------------------------------------- outputs
    @property
    def may_solve(self) -> bool:
        return self.state == State.OWNED

    @property
    def may_publish(self) -> bool:
        """OWNED: publish the MPC command. ARMING: publish only the hold probe (applied command)."""
        return self.state in (State.OWNED, State.ARMING)

    @property
    def lock_engaged(self) -> bool:
        return any(self._locks.values())

    def take_warm_start_reset(self) -> bool:
        """True once after any event that invalidates the warm start (the node then resets it)."""
        flag, self._warm_start_invalidated = self._warm_start_invalidated, False
        return flag

    # ---------------------------------------------------------------- internals
    def _go(self, new: State, now: float, reason: str) -> None:
        if new != self.state:
            self.transitions.append(Transition(now, self.state, new, reason))
            self.state = new
            self._entered = now
        self.reason = reason
        if new in (State.FOREIGN, State.DISARMED, State.UNKNOWN):
            self._warm_start_invalidated = True

    def _armed(self) -> bool:
        return self.state in (State.ARMING, State.OWNED, State.UNKNOWN)

    def _foreign_selected(self, now: float) -> bool | None:
        """True/False from the source signal if configured and fresh; None if not usable."""
        if not self.config.use_source_signal or self._source is None:
            return None
        selected, received = self._source
        if now - received > self.config.max_source_age:
            return None
        return selected != self.config.own_source

    # ---------------------------------------------------------------- events
    def on_published(self, stamp: int, v: float, omega: float, now: float) -> None:
        """Record a command the node published. Stamps must strictly increase (no duplicates)."""
        if self._last_published_stamp is not None and stamp <= self._last_published_stamp:
            raise ValueError("own command stamps must strictly increase (duplicate or reordered stamp)")
        if not self.may_publish:
            raise RuntimeError(f"publishing is not allowed in state {self.state.value}")
        self._last_published_stamp = stamp
        self._history[stamp] = (float(v), float(omega), now)
        while len(self._history) > self.config.history_size:
            self._history.popitem(last=False)

    def on_mux_output(self, stamp: int, v: float, omega: float, now: float) -> None:
        """A platform/cmd_vel (twist_mux output) sample, in receive order."""
        self._last_mux_received = now
        own = self._history.get(stamp)
        if own is None or own[0] != float(v) or own[1] != float(omega):
            # Not one of ours (teleop, RC, markers, an MPC message from before the current arming,
            # or our stamp with another twist): foreign while armed; ignored when already disarmed.
            if self._armed():
                self._go(State.FOREIGN, now, "mux output is not an own command")
            return
        if now - own[2] > self.config.max_match_age:
            return  # delayed own sample: neither evidence nor foreign
        if self.state not in (State.ARMING, State.OWNED, State.UNKNOWN):
            return  # DISARMED / FOREIGN: an own sample never re-arms (V11 leak)
        if self._foreign_selected(now):
            self._go(State.FOREIGN, now, "foreign source selected")
            return
        if self.lock_engaged:
            return
        self._last_match_time = now
        if self.state == State.ARMING:
            self._confirmed.add(stamp)  # a duplicate delivery of one stamp counts once
            if len(self._confirmed) >= self.config.confirmations:
                self._go(State.OWNED, now, "ownership confirmed")
        elif self.state == State.UNKNOWN:
            self._go(State.OWNED, now, "fresh own sample after a gap")

    def on_source(self, selected: str, now: float) -> None:
        """Selected-input signal of the mux, if it exposes one (plan V12)."""
        self._source = (selected, now)
        if self._armed() and selected != self.config.own_source:
            self._go(State.FOREIGN, now, f"source '{selected}' selected")

    def on_lock(self, name: str, engaged: bool, now: float) -> None:
        """Safety locks (platform/safety_stop, platform/emergency_stop): engaged -> not owned."""
        self._locks[name] = bool(engaged)
        if engaged and self.state != State.DISARMED:
            self._go(State.DISARMED, now, f"lock '{name}' engaged")

    def disarm(self, now: float, reason: str = "operator disarm") -> None:
        self._go(State.DISARMED, now, reason)

    def request_arm(self, now: float, applied: AppliedCommandTracker,
                    in_envelope: Callable[[np.ndarray], bool]) -> tuple[bool, str]:
        """Explicit operator re-arm. Accepted only from DISARMED, with every condition met."""
        if self.state != State.DISARMED:
            return False, f"arming only from disarmed (state {self.state.value})"
        if self.lock_engaged:
            return False, "a safety lock is engaged"
        foreign = self._foreign_selected(now)
        if self.config.use_source_signal and foreign is None:
            return False, "source signal missing or stale"
        if foreign:
            return False, "a foreign source is selected"
        u_prev = applied.u_prev(now)
        if u_prev is None:
            return False, "applied command (cmd_vel_out) missing or stale"
        if not in_envelope(u_prev):
            return False, "applied command outside the MPC envelope"
        self._history.clear()  # nothing published before this arming can count as evidence
        self._confirmed.clear()
        self._last_match_time = None
        self._go(State.ARMING, now, "armed by operator")
        return True, "arming"

    def tick(self, now: float) -> State:
        """Evaluate freshness and timeouts at a control tick; returns the state to act on."""
        cfg = self.config
        if self.state == State.FOREIGN:
            foreign = self._foreign_selected(now)
            mux_quiet = self._last_mux_received is None or now - self._last_mux_received > cfg.max_mux_age
            if not foreign and (mux_quiet or foreign is False):
                self._go(State.DISARMED, now, "foreign takeover ended: re-arm required")
            return self.state
        if not self._armed():
            return self.state
        if self.lock_engaged:
            self._go(State.DISARMED, now, "lock engaged")
            return self.state
        if self._foreign_selected(now):
            self._go(State.FOREIGN, now, "foreign source selected")
            return self.state
        source_missing = cfg.use_source_signal and self._foreign_selected(now) is None
        if self.state == State.ARMING:
            if now - self._entered > cfg.arming_timeout:
                self._go(State.DISARMED, now, "arming not confirmed in time")
            elif source_missing:
                self._go(State.UNKNOWN, now, "source signal missing or stale")
            return self.state
        evidence_fresh = self._last_match_time is not None and now - self._last_match_time <= cfg.max_mux_age
        if self.state == State.OWNED and (not evidence_fresh or source_missing):
            self._go(State.UNKNOWN, now, "mux output or source signal missing or stale")
        elif self.state == State.UNKNOWN:
            if evidence_fresh and not source_missing:
                self._go(State.OWNED, now, "evidence fresh again")
            elif now - self._entered > cfg.unknown_grace:
                self._go(State.DISARMED, now, "unknown for too long: re-arm required")
        return self.state
