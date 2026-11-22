"""Offline tests of the mux, watchdog, lock and freshness gate: takeover, lock, stale setpoint,
no automatic resume (STOP 1 review, section 4.1 item 2)."""

from __future__ import annotations

import math

import pytest

import command_mux as cm

CFG = cm.MuxConfig(T_wd=0.3, T_teleop=0.3, T_arm_fresh=0.1)


def stream(mux: cm.CommandMux, t0: float, t1: float, speed=1.0, angle=0.2, period=0.02, delay=0.0):
    """Controller setpoints every `period` from t0 to t1, arriving `delay` after their stamp,
    with mux ticks in between; returns the outputs forwarded."""
    out = []
    t = t0
    while t <= t1 + 1e-9:
        o = mux.on_controller(t + delay, speed, angle, stamp=t)
        if o is not None:
            out.append(o)
        mux.tick(t + delay)
        t += period
    return out


def armed_mux(gate: cm.FreshnessGate | None = None) -> cm.CommandMux:
    mux = cm.CommandMux(CFG, gate)
    stream(mux, 0.0, 0.1)
    ok, why = mux.request_arm(0.1)
    assert ok, why
    return mux


def test_config_loads_from_the_chain_file():
    c = cm.load_mux_config()
    assert (c.T_wd, c.T_teleop, c.T_arm_fresh) == (0.3, 0.3, 0.1)
    assert cm.load_gate_threshold() == 0.1


def test_starts_disarmed_and_ignores_setpoints():
    mux = cm.CommandMux(CFG)
    assert mux.mode == cm.Mode.HOLD
    assert stream(mux, 0.0, 0.5) == []
    assert mux.tick(0.5) == cm.Output(0.0, 0.0, "hold")
    assert mux.counters.ignored_disarmed > 0


def test_arming_needs_a_fresh_setpoint():
    mux = cm.CommandMux(CFG)
    ok, why = mux.request_arm(0.0)
    assert not ok and "no fresh controller setpoint" in why
    stream(mux, 0.0, 0.1)
    ok, why = mux.request_arm(0.35)  # last setpoint at 0.1 s: 0.25 s old
    assert not ok and mux.mode == cm.Mode.HOLD
    assert mux.request_arm(0.15)[0] and mux.mode == cm.Mode.CONTROLLER


def test_armed_forwards_with_the_stamp_and_holds_until_the_first_setpoint():
    mux = armed_mux()
    assert mux.tick(0.105).source == "hold"  # armed, no setpoint yet since arming
    out = mux.on_controller(0.11, 0.8, -0.1, stamp=0.109)
    assert out == cm.Output(0.8, -0.1, "controller", 0.109)
    assert mux.tick(0.115) == out  # keep-alive repeats the last forwarded command


def test_stale_setpoint_stops_and_holds_without_auto_resume():
    mux = armed_mux()
    stream(mux, 0.12, 0.5, angle=0.25)
    # the controller stops publishing at 0.5 s
    for k in range(1, 40):
        mux.tick(0.5 + 0.01 * k)
    assert mux.mode == cm.Mode.HOLD
    t_trip, _, to, reason = mux.events[-1]
    assert (to, reason) == ("hold", "setpoint_timeout")
    assert 0.5 + CFG.T_wd < t_trip <= 0.5 + CFG.T_wd + 0.011
    assert mux.tick(0.9) == cm.Output(0.0, 0.25, "hold")  # speed 0, steering held
    # setpoints come back: they are accepted but not forwarded; no re-arm by itself
    assert stream(mux, 1.0, 2.0) == [] and mux.mode == cm.Mode.HOLD
    assert mux.request_arm(2.0)[0] and mux.mode == cm.Mode.CONTROLLER


def test_old_and_reordered_setpoints_are_dropped():
    mux = armed_mux()
    assert mux.on_controller(1.0, 1.0, 0.0, stamp=0.6) is None  # 0.4 s old > T_wd
    assert mux.counters.dropped_old == 1
    assert mux.on_controller(0.21, 1.0, 0.0, stamp=0.2) is not None
    assert mux.on_controller(0.22, 1.0, 0.0, stamp=0.19) is None  # older stamp than the last
    assert mux.on_controller(0.23, 1.0, 0.0, stamp=0.2) is None   # same stamp
    assert mux.counters.dropped_reordered == 2
    # delayed but in-time setpoints keep it armed; too-old ones count as silence
    stream(mux, 0.24, 0.6, delay=0.25)
    assert mux.mode == cm.Mode.CONTROLLER
    stream(mux, 0.62, 1.2, delay=0.35)  # every setpoint arrives older than T_wd
    assert mux.mode == cm.Mode.HOLD and mux.events[-1][3] == "setpoint_timeout"


def test_teleop_takes_over_and_never_hands_back_by_itself():
    mux = armed_mux()
    stream(mux, 0.12, 0.3)
    out = mux.on_teleop(0.31, -0.3, 0.1)
    assert out == cm.Output(-0.3, 0.1, "teleop") and mux.mode == cm.Mode.TELEOP
    assert mux.events[-1][3] == "teleop_takeover"
    # the controller keeps publishing: nothing of it is forwarded during the override
    for k in range(10):
        t = 0.32 + 0.02 * k
        assert mux.on_controller(t, 1.0, 0.2, stamp=t) is None
        mux.on_teleop(t, -0.3, 0.1)
        assert mux.tick(t).source == "teleop"
    # teleop goes silent (last at 0.50): stop and hold, not back to the controller
    stream(mux, 0.52, 1.2)
    assert mux.mode == cm.Mode.HOLD and mux.events[-1][3] == "teleop_released"
    assert mux.tick(1.2) == cm.Output(0.0, 0.1, "hold")
    assert mux.request_arm(1.2)[0]  # only an explicit arm hands control back


def test_arming_refused_while_teleop_is_active():
    mux = cm.CommandMux(CFG)
    stream(mux, 0.0, 0.1)
    mux.on_teleop(0.1, 0.0, 0.0)
    ok, why = mux.request_arm(0.15)
    assert not ok and "teleop active" in why and "mode is teleop" in why


def test_lock_latches_over_everything_and_release_needs_rearm():
    mux = armed_mux()
    stream(mux, 0.12, 0.3, angle=-0.2)
    mux.on_lock(0.31, True)
    assert mux.mode == cm.Mode.LOCKED and mux.tick(0.31) == cm.Output(0.0, -0.2, "hold")
    assert mux.on_teleop(0.32, 1.0, 0.0) is None and mux.mode == cm.Mode.LOCKED  # teleop too
    assert stream(mux, 0.33, 0.6) == []
    ok, why = mux.request_arm(0.6)
    assert not ok and "lock engaged" in why
    mux.on_lock(0.61, False)
    assert mux.mode == cm.Mode.HOLD and mux.events[-1][3] == "lock_released"
    assert stream(mux, 0.62, 0.8) == []  # no auto resume
    assert mux.request_arm(0.8)[0]


def test_non_finite_setpoint_disarms():
    mux = armed_mux()
    out = mux.on_controller(0.12, math.nan, 0.0, stamp=0.12)
    assert out.source == "hold" and mux.mode == cm.Mode.HOLD
    assert mux.events[-1][3] == "non_finite_setpoint"


def test_freshness_gate_age_and_order():
    g = cm.FreshnessGate(0.1)
    assert not g.fresh(0.0) and g.age(0.0) == math.inf
    g.on_pose(1.0)
    g.on_pose(0.95)  # older, out of order: ignored
    assert g.age(1.05) == pytest.approx(0.05) and g.fresh(1.0999) and not g.fresh(1.1001)
    with pytest.raises(ValueError):
        cm.FreshnessGate(0.0)


def test_stale_pose_disarms_and_does_not_rearm():
    gate = cm.FreshnessGate(0.1)
    mux = cm.CommandMux(CFG, gate)
    stream(mux, 0.0, 0.1)
    ok, why = mux.request_arm(0.1)
    assert not ok and "pose not fresh" in why
    for k in range(11):
        gate.on_pose(0.01 * k)
    assert mux.request_arm(0.1)[0]
    # poses stop at 0.10 s; setpoints continue
    stream(mux, 0.12, 0.4)
    assert mux.mode == cm.Mode.HOLD and mux.events[-1][3] == "mocap_stale" and gate.trips == 1
    trip = mux.events[-1][0]
    assert 0.2 < trip <= 0.22
    # poses return: still HOLD until an explicit arm
    for k in range(10):
        gate.on_pose(0.41 + 0.01 * k)
        stream(mux, 0.41 + 0.01 * k, 0.41 + 0.01 * k)
    assert mux.mode == cm.Mode.HOLD
    assert mux.request_arm(0.5)[0]


def test_operator_disarm():
    mux = armed_mux()
    mux.disarm(0.2)
    assert mux.mode == cm.Mode.HOLD and mux.events[-1][3] == "operator"
    mux.disarm(0.3)  # no effect in HOLD
    assert len([e for e in mux.events if e[3] == "operator"]) == 1
