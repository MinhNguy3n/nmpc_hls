"""Offline tests of the OptiTrack emulator: stamps, the two delay parts, spikes, noise, dropouts."""

from __future__ import annotations

from dataclasses import replace
import math

import numpy as np
import pytest

import mocap_emulator as me

PROFILES = me.load_profiles()


def feed(em: me.MocapEmulator, n: int, rate: float = 100.0, yaw=lambda t: 0.3):
    for i in range(n):
        t = i / rate
        em.on_truth(t, 1.0 + t, -2.0, yaw(t))
    return em.due(math.inf)


def test_profiles_load_and_are_labelled():
    assert set(PROFILES) == {"ideal", "realistic_guess"}
    assert PROFILES["ideal"].status == "design" and PROFILES["realistic_guess"].status == "placeholder"
    with pytest.raises(ValueError):
        replace(PROFILES["ideal"], spike_min=0.05, spike_max=0.01)


def test_ideal_is_exact_and_immediate():
    em = me.MocapEmulator(PROFILES["ideal"], 0)
    em.on_truth(1.5, 2.0, 3.0, 0.4)
    frames = em.due(1.5)
    assert len(frames) == 1
    f = frames[0]
    assert (f.capture, f.arrival, f.x, f.y, f.yaw) == (1.5, 1.5, 2.0, 3.0, 0.4)


def test_capture_stamp_kept_and_delay_split():
    p = replace(PROFILES["ideal"], T_mocap=0.005, wifi_base=0.003)
    em = me.MocapEmulator(p, 0)
    em.on_truth(2.0, 0.0, 0.0, 0.0)
    assert em.due(2.0079) == []
    (f,) = em.due(2.008 + 1e-9)
    assert f.capture == 2.0 and f.arrival == pytest.approx(2.008)
    assert (f.mocap_delay, f.wifi_delay) == (0.005, pytest.approx(0.003))


def test_decimation_to_the_frame_rate():
    em = me.MocapEmulator(replace(PROFILES["ideal"], rate_hz=50.0), 0)
    frames = feed(em, 200)  # 2 s of truth at 100 Hz
    assert len(frames) == 100
    assert np.allclose(np.diff([f.capture for f in frames]), 0.02)


def test_realistic_delay_distribution_and_spikes():
    p = PROFILES["realistic_guess"]
    em = me.MocapEmulator(replace(p, drop_prob=0.0), 1)
    frames = feed(em, 20000)
    delay = np.array([f.arrival - f.capture for f in frames])
    spikes = np.array([f.spike for f in frames])
    assert delay.min() >= p.T_mocap + p.wifi_base
    no_spike = delay[~spikes] - p.T_mocap - p.wifi_base
    assert no_spike.mean() == pytest.approx(p.wifi_jitter_mean, rel=0.05)  # exponential jitter
    assert spikes.mean() == pytest.approx(p.spike_prob, abs=0.003)
    spike_part = delay[spikes] - p.T_mocap - p.wifi_base
    assert spike_part.min() >= p.spike_min and spike_part.mean() == pytest.approx(
        0.5 * (p.spike_min + p.spike_max) + p.wifi_jitter_mean, rel=0.1)
    assert all(f.mocap_delay == p.T_mocap for f in frames)


def test_arrival_order_and_overtaking():
    p = replace(PROFILES["ideal"], spike_prob=1.0, spike_min=0.05, spike_max=0.05)
    em = me.MocapEmulator(p, 0)
    em.on_truth(0.00, 0, 0, 0)  # spiked: arrives at 0.05
    em.p = replace(p, spike_prob=0.0)
    em.on_truth(0.01, 0, 0, 0)  # arrives at 0.01: overtakes the first
    frames = em.due(1.0)
    assert [f.capture for f in frames] == [0.01, 0.0]
    assert [f.arrival for f in frames] == sorted(f.arrival for f in frames)


def test_noise_statistics_and_yaw_wrap():
    p = replace(PROFILES["ideal"], noise_xy=0.001, noise_yaw=0.01)
    em = me.MocapEmulator(p, 2)
    frames = feed(em, 20000, yaw=lambda t: math.pi - 1e-3)
    ex = np.array([f.x - (1.0 + f.capture) for f in frames])
    assert ex.std() == pytest.approx(0.001, rel=0.05) and abs(ex.mean()) < 5e-5
    yaw = np.array([f.yaw for f in frames])
    assert np.all((yaw >= -math.pi) & (yaw < math.pi))
    eyaw = np.angle(np.exp(1j * (yaw - (math.pi - 1e-3))))
    assert eyaw.std() == pytest.approx(0.01, rel=0.05)


def test_dropout_bursts():
    p = replace(PROFILES["ideal"], drop_prob=0.02, drop_burst_max=3)
    em = me.MocapEmulator(p, 4)
    frames = feed(em, 50000)
    lost = em.stats["dropped"]
    assert lost + len(frames) == 50000
    # renewal cycles: a kept frame (prob 1 - p) or a burst of mean length 2 (prob p)
    assert lost / 50000 == pytest.approx(0.02 * 2 / (0.98 + 0.02 * 2), rel=0.1)
    gaps = np.round(np.diff([f.capture for f in frames]) * 100).astype(int) - 1
    assert set(np.unique(gaps)) >= {0, 1, 2, 3}
    assert np.mean(gaps > 3) < 1e-3  # longer gaps only when two bursts touch
