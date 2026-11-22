"""Offline tests of the emulated command chain (step responses per profile, the output hold)."""

from __future__ import annotations

from dataclasses import replace
import math

import numpy as np
import pytest

import actuator_model as am

PROFILES = am.load_profiles()
DMAX, VMAX = 0.45, 2.0


def run(model: am.ActuatorModel, t_end: float, dt: float = 1e-4):
    """Sample (t, speed, delta) from the model time to t_end."""
    rows = []
    t = model.t
    while t < t_end - 1e-12:
        t = min(t_end, t + dt)
        rows.append((t, *model.advance(t)))
    return np.array(rows)


def first_motion(rows: np.ndarray, column: int, threshold: float = 1e-9) -> float:
    idx = np.nonzero(np.abs(rows[:, column]) > threshold)[0]
    return float(rows[idx[0], 0]) if len(idx) else math.nan


def test_profiles_load_and_are_labelled():
    assert set(PROFILES) == {"ideal", "realistic_guess", "fast_servo"}
    assert PROFILES["ideal"].status == "design"
    assert PROFILES["realistic_guess"].status == "placeholder" == PROFILES["fast_servo"].status
    # fast_servo differs from realistic_guess only in the servo update rate
    rg, fs = PROFILES["realistic_guess"], PROFILES["fast_servo"]
    assert replace(fs, name=rg.name, f_servo_out=rg.f_servo_out) == rg
    assert (rg.f_servo_out, fs.f_servo_out) == (50.0, 333.0)


def test_invalid_profile_values_rejected():
    with pytest.raises(ValueError):
        replace(PROFILES["ideal"], tau_v=-0.1)
    with pytest.raises(ValueError):
        replace(PROFILES["ideal"], accel_max=0.0)


def test_ideal_profile_is_a_pass_through_with_saturation():
    m = am.ActuatorModel(PROFILES["ideal"], DMAX, VMAX, np.random.default_rng(0))
    m.advance(0.1)
    m.command(0.1, 1.2, 0.3)
    assert (m.speed, m.delta) == (1.2, 0.3)  # at the same instant
    m.command(0.2, 5.0, -1.0)
    assert (m.speed, m.delta) == (VMAX, -DMAX)
    m.command(0.3, math.nan, 0.2)
    assert (m.speed, m.delta) == (0.0, 0.0)


def test_command_link_delay_without_hold():
    p = replace(PROFILES["ideal"], T_link=0.004)
    m = am.ActuatorModel(p, DMAX, VMAX, np.random.default_rng(0))
    m.command(0.0, 1.0, 0.2)
    rows = run(m, 0.01, dt=1e-5)
    assert first_motion(rows, 1) == pytest.approx(0.004, abs=2e-5)
    assert first_motion(rows, 2) == pytest.approx(0.004, abs=2e-5)


@pytest.mark.parametrize("seed", range(8))
def test_hold_samples_the_latest_arrived_command(seed):
    """f_servo_out = 50 Hz, no other lag: the servo moves at the first update at or after the arrival."""
    p = replace(PROFILES["ideal"], T_link=0.004, f_servo_out=50.0)
    m = am.ActuatorModel(p, DMAX, VMAX, np.random.default_rng(seed))
    phase = m.servo_phase
    assert 0.0 <= phase < 0.02
    t_send = 0.0137
    m.advance(t_send)
    m.command(t_send, 0.0, 0.3)
    rows = run(m, 0.1, dt=1e-5)
    arrival = t_send + 0.004
    k = math.ceil((arrival - phase) / 0.02 - 1e-9)
    expected = phase + k * 0.02
    assert first_motion(rows, 2) == pytest.approx(expected, abs=2e-5)
    assert expected - arrival < 0.02
    # updates are periodic at 20 ms from the phase
    updates = np.array(m.servo_updates)
    assert np.allclose(np.diff(updates), 0.02) and updates[0] == pytest.approx(phase)


def test_hold_keeps_only_the_latest_of_several_commands():
    p = replace(PROFILES["ideal"], f_servo_out=50.0)
    m = am.ActuatorModel(p, DMAX, VMAX, np.random.default_rng(1))
    m.advance(m.servo_phase + 0.001)  # just after an update
    for i, d in enumerate((0.1, 0.2, 0.3)):  # three commands within one PWM period
        m.command(m.t + 0.002 * i, 0.0, d)
    m.advance(m.servo_phase + 0.0205)
    assert m.delta == 0.3


def test_speed_step_first_order_lag_unlimited():
    p = replace(PROFILES["ideal"], tau_v=0.1)
    m = am.ActuatorModel(p, DMAX, VMAX)
    m.command(0.0, 1.0, 0.0)
    for t in (0.05, 0.1, 0.3):
        m.advance(t)
        assert m.speed == pytest.approx(1.0 - math.exp(-t / 0.1), rel=1e-9)


def test_speed_step_accel_and_decel_limits():
    p = replace(PROFILES["ideal"], tau_v=0.1, accel_max=3.0, decel_max=4.0)
    m = am.ActuatorModel(p, DMAX, VMAX)
    m.command(0.0, 2.0, 0.0)  # a 2 m/s step wants 20 m/s^2 at first: the 3 m/s^2 limit binds
    rows = run(m, 0.2, dt=1e-3)
    slope = np.diff(rows[:50, 1]) / np.diff(rows[:50, 0])
    assert np.allclose(slope, 3.0, rtol=1e-6)
    m.advance(3.0)
    assert m.speed == pytest.approx(2.0, abs=1e-6)
    m.command(3.0, 0.0, 0.0)  # slowing: the 4 m/s^2 limit
    rows = run(m, 3.2, dt=1e-3)
    slope = np.diff(rows[:50, 1]) / np.diff(rows[:50, 0])
    assert np.allclose(slope, -4.0, rtol=1e-6)
    # reversing through zero: slowing limit down to 0, then the speeding-up limit
    m.advance(6.0)
    m.command(6.0, -0.2, 0.0)
    rows = run(m, 6.2, dt=1e-4)
    assert rows[-1, 1] < 0 and np.all(np.diff(rows[:, 1]) <= 1e-12)


def test_servo_dead_time_rate_limit_and_saturation():
    p = replace(PROFILES["ideal"], T_d=0.01, delta_rate_max=5.0)
    m = am.ActuatorModel(p, DMAX, VMAX)
    m.command(0.0, 0.0, 1.0)  # beyond delta_max
    rows = run(m, 0.3, dt=1e-4)
    assert first_motion(rows, 2) == pytest.approx(0.01, abs=2e-4)
    moving = rows[(rows[:, 0] > 0.0105) & (rows[:, 0] < 0.09)]
    rate = np.diff(moving[:, 2]) / np.diff(moving[:, 0])
    assert np.allclose(rate, 5.0, rtol=1e-6)
    assert rows[-1, 2] == pytest.approx(DMAX)  # reached at 0.01 + 0.45 / 5 = 0.1 s
    assert np.max(rows[:, 2]) <= DMAX + 1e-12


@pytest.mark.parametrize("name", ["ideal", "realistic_guess", "fast_servo"])
def test_step_responses_per_profile(name):
    """Steering and speed steps through each whole profile: the start delay lies inside the bounds
    set by T_link, the hold period and T_d, and the final values are reached."""
    p = PROFILES[name]
    for seed in range(5):
        m = am.ActuatorModel(p, DMAX, VMAX, np.random.default_rng(seed))
        m.advance(0.05)
        m.command(0.05, 1.0, 0.25)
        rows = run(m, 1.5, dt=5e-5)
        hold_s = 1.0 / p.f_servo_out if p.f_servo_out else 0.0
        hold_v = 1.0 / p.f_esc_out if p.f_esc_out else 0.0
        d_start = first_motion(rows, 2) - 0.05
        v_start = first_motion(rows, 1) - 0.05
        assert p.T_link + p.T_d - 1e-4 <= d_start <= p.T_link + p.T_d + hold_s + 1e-4
        assert p.T_link - 1e-4 <= v_start <= p.T_link + hold_v + 1e-4
        assert rows[-1, 2] == pytest.approx(0.25, abs=1e-9)
        assert rows[-1, 1] == pytest.approx(1.0, abs=1e-4)


def test_monte_carlo_servo_delay_matches_the_hold():
    rng = np.random.default_rng(3)
    rg = am.servo_start_delays(PROFILES["realistic_guess"], 400, rng)
    fs = am.servo_start_delays(PROFILES["fast_servo"], 400, rng)
    base = 0.004 + 0.010  # T_link + T_d (+ the rate-limited time to 1 mrad: 0.2 ms)
    assert rg.min() >= base and rg.max() <= base + 0.02 + 0.0005
    assert fs.min() >= base and fs.max() <= base + 1 / 333 + 0.0005
    assert rg.mean() == pytest.approx(base + 0.010 + 0.0002, abs=0.0015)  # uniform hold wait, mean 10 ms
    assert np.all(am.servo_start_delays(PROFILES["ideal"], 10, rng) < 1e-3)


def test_integer_seed_fixes_the_phases_and_rephase_aligns_them():
    p = PROFILES["realistic_guess"]
    a = am.ActuatorModel(p, DMAX, VMAX, 7, t0=3.217)
    b = am.ActuatorModel(p, DMAX, VMAX, 7, t0=10.0)
    c = am.ActuatorModel(p, DMAX, VMAX, 8, t0=0.0)
    assert (a.esc_phase, a.servo_phase) == (b.esc_phase, b.servo_phase) != (c.esc_phase, c.servo_phase)
    # different histories, then the same episode-relative updates
    a.command(3.3, 0.5, 0.1)
    a.advance(5.0)
    b.advance(10.05)
    a.rephase(5.1)
    b.rephase(12.0)
    a.advance(5.2)
    b.advance(12.1)
    ua = np.array([t for t in a.servo_updates if t >= 5.1]) - 5.1
    ub = np.array([t for t in b.servo_updates if t >= 12.0]) - 12.0
    assert len(ua) == len(ub) > 3 and np.allclose(ua, ub, atol=1e-9)
    with pytest.raises(ValueError):
        a.rephase(1.0)
