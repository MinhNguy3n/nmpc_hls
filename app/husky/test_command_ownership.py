"""Tests of app/husky/command_ownership.py (offline, no ROS).

Run: /usr/bin/python3 -m pytest app/husky -q -p no:cacheprovider
Times are seconds; the node publishes every TS = 0.05 s. Config values are the module's
PLACEHOLDER_ policy defaults.
"""

from __future__ import annotations

from pathlib import Path
import sys

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent))
from command_ownership import (  # noqa: E402
    AppliedCommandTracker, AppliedSample, OwnershipConfig, OwnershipStateMachine, State, stamp_ns,
)

TS = 0.05
ENVELOPE = 0.3  # PLACEHOLDER_ operating envelope (m/s and rad/s) for the re-arm check


def in_envelope(u: np.ndarray) -> bool:
    return bool(abs(u[0]) <= ENVELOPE and abs(u[1]) <= ENVELOPE)


def ns(t: float) -> int:
    return int(round(t * 1e9))


class Harness:
    """Drives the state machine like the node: publish at a tick, mux echoes it shortly after."""

    def __init__(self, config: OwnershipConfig | None = None) -> None:
        self.sm = OwnershipStateMachine(config)
        self.applied = AppliedCommandTracker(max_age=self.sm.config.max_applied_age)
        self.t = 0.0

    def applied_sample(self, v: float = 0.0, w: float = 0.0, age: float = 0.0) -> None:
        self.applied.on_sample(AppliedSample(stamp=self.t - age, received=self.t - age, v=v, omega=w))

    def arm(self) -> tuple[bool, str]:
        self.applied_sample()
        return self.sm.request_arm(self.t, self.applied, in_envelope)

    def publish(self, v: float = 0.1, w: float = 0.0) -> int:
        stamp = ns(self.t)
        self.sm.on_published(stamp, v, w, self.t)
        return stamp

    def echo(self, stamp: int, v: float = 0.1, w: float = 0.0, delay: float = 0.003) -> None:
        self.sm.on_mux_output(stamp, v, w, self.t + delay)

    def cycle(self, echo: bool = True, v: float = 0.1) -> State:
        """One period: tick, publish if allowed, mux echo, advance time."""
        state = self.sm.tick(self.t)
        if self.sm.may_publish:
            stamp = self.publish(v)
            if echo:
                self.echo(stamp, v)
        self.t += TS
        return state

    def to_owned(self) -> None:
        assert self.arm()[0]
        for _ in range(3):
            self.cycle()
        assert self.sm.tick(self.t) == State.OWNED


def test_stamp_ns_is_exact():
    assert stamp_ns(12345, 678) == 12345 * 10**9 + 678


def test_arming_handshake_needs_distinct_fresh_matches():
    h = Harness()
    assert h.sm.state == State.DISARMED and not h.sm.may_publish and not h.sm.may_solve
    assert h.arm() == (True, "arming")
    assert h.sm.state == State.ARMING and h.sm.may_publish and not h.sm.may_solve
    first = h.publish()
    h.echo(first)
    h.echo(first)  # duplicate delivery of one stamp counts once
    assert h.sm.state == State.ARMING
    h.t += TS
    h.echo(h.publish())
    assert h.sm.state == State.OWNED and h.sm.may_solve


def test_message_loss_tolerance_unknown_and_disarm():
    h = Harness()
    h.to_owned()
    h.cycle(echo=False)  # one lost mux sample: still owned at the next tick
    assert h.cycle() == State.OWNED
    h.cycle(echo=False)
    h.cycle(echo=False)
    h.cycle(echo=False)
    assert h.sm.state == State.UNKNOWN and not h.sm.may_publish and not h.sm.may_solve
    for _ in range(6):  # no publications in UNKNOWN: nothing can confirm -> disarmed
        h.cycle()
    assert h.sm.state == State.DISARMED and h.sm.reason.startswith("unknown for too long")


def test_unknown_recovers_only_on_a_fresh_own_sample_within_grace():
    h = Harness()
    h.to_owned()
    last = h.publish()  # published, but its echo is delayed
    h.t += 0.13
    assert h.sm.tick(h.t) == State.UNKNOWN
    h.sm.on_mux_output(last, 0.1, 0.0, h.t + 0.01)  # sent 0.14 s ago (<= max_match_age 0.15)
    assert h.sm.state == State.OWNED
    h.t += 0.2  # a sample of a command older than max_match_age is no evidence
    assert h.sm.tick(h.t) == State.UNKNOWN
    h.sm.on_mux_output(last, 0.1, 0.0, h.t)
    assert h.sm.state == State.UNKNOWN


def test_delayed_and_reordered_own_samples():
    h = Harness()
    h.to_owned()
    a = h.publish()
    h.t += TS
    b = h.publish()
    h.echo(b)
    h.echo(a)  # reordered: older own stamp after a newer one -> still ours, no foreign
    assert h.sm.tick(h.t + 0.01) == State.OWNED
    h.t += 1.0
    h.sm.on_mux_output(a, 0.1, 0.0, h.t)  # very late own sample: no evidence, not foreign
    assert h.sm.state == State.OWNED
    assert h.sm.tick(h.t) == State.UNKNOWN


def test_v11_single_leaked_mpc_stamp_never_rearms():
    h = Harness()
    h.to_owned()
    leaked = h.publish()  # an MPC command already in flight
    teleop = stamp_ns(999, 0)
    h.sm.on_mux_output(teleop, 0.0, 0.5, h.t + 0.002)  # joystick takes over
    assert h.sm.state == State.FOREIGN and not h.sm.may_publish
    assert h.sm.take_warm_start_reset()
    h.t += TS
    h.sm.on_mux_output(leaked, 0.1, 0.0, h.t)  # the one-message leak inside the teleop period
    assert h.sm.state == State.FOREIGN
    for _ in range(4):
        h.sm.on_mux_output(teleop + 1, 0.0, 0.5, h.t)
        h.t += TS
        h.sm.tick(h.t)
    assert h.sm.state == State.FOREIGN
    h.t += 0.2  # teleop stops; the mux falls quiet (the node publishes nothing)
    assert h.sm.tick(h.t) == State.DISARMED
    h.sm.on_mux_output(leaked, 0.1, 0.0, h.t)  # an old MPC message arriving after the takeover
    assert h.sm.tick(h.t) == State.DISARMED and not h.sm.may_publish
    assert h.arm()[0]  # only an explicit re-arm restarts it, with a cleared history
    h.sm.on_mux_output(leaked, 0.1, 0.0, h.t + 0.001)  # pre-arm stamp is no longer ours
    assert h.sm.state == State.FOREIGN


def test_duplicate_stamps():
    h = Harness()
    h.to_owned()
    stamp = h.publish()
    with pytest.raises(ValueError):
        h.sm.on_published(stamp, 0.1, 0.0, h.t)  # duplicate own stamp
    with pytest.raises(ValueError):
        h.sm.on_published(stamp - 1, 0.1, 0.0, h.t)  # reordered own stamp
    h.sm.on_mux_output(stamp, 0.1, 0.2, h.t)  # our stamp with another twist: not ours
    assert h.sm.state == State.FOREIGN


def test_transitions_at_timer_deadlines():
    cfg = OwnershipConfig()
    h = Harness(cfg)
    h.to_owned()
    stamp = h.publish()
    h.sm.on_mux_output(stamp, 0.1, 0.0, h.t)
    assert h.sm.tick(h.t + cfg.max_mux_age) == State.OWNED  # exactly at the bound: fresh
    assert h.sm.tick(h.t + cfg.max_mux_age + 1e-9) == State.UNKNOWN  # just past it
    entered = h.sm.transitions[-1].time
    assert h.sm.tick(entered + cfg.unknown_grace - 1e-9) == State.UNKNOWN
    assert h.sm.tick(entered + cfg.unknown_grace + 1e-9) == State.DISARMED
    g = Harness(cfg)
    assert g.arm()[0]
    assert g.sm.tick(g.t + cfg.arming_timeout) == State.ARMING
    assert g.sm.tick(g.t + cfg.arming_timeout + 1e-9) == State.DISARMED
    with pytest.raises(RuntimeError):
        g.sm.on_published(ns(g.t + 1.0), 0.0, 0.0, g.t + 1.0)  # publishing not allowed now


def test_stale_or_bad_applied_command_blocks_rearm():
    h = Harness()
    assert h.sm.request_arm(h.t, h.applied, in_envelope) == (False, "applied command (cmd_vel_out) missing or stale")
    h.t = 10.0
    h.applied_sample(age=0.2)  # stale cmd_vel_out
    assert h.sm.request_arm(h.t, h.applied, in_envelope)[0] is False
    h.applied.on_sample(AppliedSample(stamp=h.t + 0.5, received=h.t, v=0.0, omega=0.0))  # future stamp
    assert h.applied.u_prev(h.t) is None
    assert h.sm.request_arm(h.t, h.applied, in_envelope)[0] is False
    fresh = Harness()
    fresh.t = 20.0
    fresh.applied_sample(v=1.0)  # fresh, but the robot is moving outside the envelope
    assert fresh.sm.request_arm(fresh.t, fresh.applied, in_envelope) == (False, "applied command outside the MPC envelope")
    fresh.t += 0.02  # the robot has stopped: a newer cmd_vel_out sample
    fresh.applied_sample(v=0.0)
    assert fresh.sm.request_arm(fresh.t, fresh.applied, in_envelope) == (True, "arming")


def test_applied_tracker_uses_its_own_stamps():
    tracker = AppliedCommandTracker(max_age=0.06)
    tracker.on_sample(AppliedSample(stamp=5.000, received=5.001, v=0.2, omega=0.1))
    tracker.on_sample(AppliedSample(stamp=4.979, received=5.002, v=9.0, omega=9.0))  # older, reordered
    assert np.allclose(tracker.u_prev(5.010), [0.2, 0.1])
    assert tracker.u_prev(5.061) is None  # stale by its own stamp
    tracker.on_sample(AppliedSample(stamp=5.021, received=5.022, v=float("nan"), omega=0.0))
    assert np.allclose(tracker.u_prev(5.030), [0.2, 0.1])  # non-finite sample ignored


def test_safety_lock_latch_and_release():
    h = Harness()
    h.to_owned()
    h.sm.on_lock("safety_stop", True, h.t)
    assert h.sm.state == State.DISARMED and not h.sm.may_publish
    assert h.arm() == (False, "a safety lock is engaged")
    h.t += 1.0
    h.sm.on_lock("safety_stop", False, h.t)
    assert h.sm.tick(h.t) == State.DISARMED  # release alone does not re-arm
    h.to_owned()
    h.sm.on_lock("emergency_stop", True, h.t)
    assert h.sm.state == State.DISARMED


def test_source_signal_when_the_mux_exposes_it():
    cfg = OwnershipConfig(use_source_signal=True)
    h = Harness(cfg)
    assert h.arm() == (False, "source signal missing or stale")
    h.sm.on_source("external", h.t)
    h.to_owned()
    h.sm.on_source("joy", h.t)  # foreign selection: immediate, even before any mux sample
    assert h.sm.state == State.FOREIGN
    h.t += 0.1
    h.sm.on_source("external", h.t)
    assert h.sm.tick(h.t) == State.DISARMED
    h.sm.on_source("external", h.t)
    h.to_owned()
    h.t += cfg.max_source_age + 0.01  # the source signal goes stale
    h.sm.on_mux_output(h.publish(), 0.1, 0.0, h.t)
    assert h.sm.tick(h.t) == State.UNKNOWN
