"""Paired seeds: for a given seed, two runs get identical mocap noise / jitter / spike / dropout
streams per frame and identical PWM hold phases, relative to their episode starts, whatever ran
before (phase 3 condition 3)."""

from __future__ import annotations

import numpy as np
import pytest

import actuator_model as am
import episode
import mocap_emulator as me

MOCAP = me.load_profiles()["realistic_guess"]
ACT = am.load_profiles()["realistic_guess"]


def run_mocap(seed: int, launch_seed: int, pre_frames: int, t0: float, offset: float, n: int = 3000):
    """A node launched with `launch_seed`, `pre_frames` frames before the episode; then an episode
    with `seed` starting at t0 (captures every 10 ms, `offset` after t0)."""
    em = me.MocapEmulator(MOCAP, launch_seed)
    for i in range(pre_frames):
        em.on_truth(t0 - 0.01 * (pre_frames - i), 0.0, 0.0, 0.0)
    em.due(1e9)
    ep = episode.parse(episode.Episode(t0, seed).to_json())
    em.reset_episode(ep.t_start, ep.seed)
    em.on_truth(t0 - 0.0005, 0.0, 0.0, 0.0)  # a late pre-episode frame: must not shift the streams
    em.due(1e9)
    out = []
    for k in range(n):
        capture = t0 + offset + 0.01 * k
        em.on_truth(capture, 1.0, 2.0, 0.5)
        frames = em.due(1e9)
        out.append(None if not frames else (frames[0].x - 1.0, frames[0].y - 2.0, frames[0].yaw - 0.5,
                                            frames[0].arrival - capture, frames[0].spike))
    return out, em.draws


def test_mocap_streams_are_paired_by_seed():
    a, da = run_mocap(5, launch_seed=1, pre_frames=0, t0=20.0, offset=0.003)
    b, db = run_mocap(5, launch_seed=9, pre_frames=777, t0=61.37, offset=0.008)
    assert da == db
    assert [x is None for x in a] == [x is None for x in b]  # identical dropout pattern
    assert sum(x is None for x in a) > 10 and sum(1 for x in a if x and x[4]) > 10  # streams exercised
    for x, y in zip(a, b):
        if x is not None:
            assert x == pytest.approx(y, abs=1e-12)
    c, _ = run_mocap(6, launch_seed=1, pre_frames=0, t0=20.0, offset=0.003)
    assert [x is None for x in a] != [x is None for x in c]


def test_actuator_hold_phases_are_paired_by_seed():
    a = am.ActuatorModel(ACT, 0.45, 2.0, 1, t0=0.0)
    b = am.ActuatorModel(ACT, 0.45, 2.0, 2, t0=4.321)
    a.advance(7.0)
    b.advance(30.0)
    a.rephase(7.5, seed=5)
    b.rephase(30.2, seed=5)
    a.advance(8.0)
    b.advance(30.7)
    ua = np.array([t for t in a.servo_updates if t >= 7.5]) - 7.5
    ub = np.array([t for t in b.servo_updates if t >= 30.2]) - 30.2
    assert len(ua) == len(ub) == 25 and np.allclose(ua, ub, atol=1e-9)
    assert a.esc_phase == b.esc_phase
    c = am.ActuatorModel(ACT, 0.45, 2.0, 1, t0=0.0)
    c.rephase(0.0, seed=6)
    assert c.servo_phase != a.servo_phase


def test_episode_message():
    ep = episode.parse('{"t_start": 12.5, "seed": 3}')
    assert ep == episode.Episode(12.5, 3) and episode.parse(ep.to_json()) == ep
    with pytest.raises(ValueError):
        episode.parse('{"t_start": "nan", "seed": 1}')
