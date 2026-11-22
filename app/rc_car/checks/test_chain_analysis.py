"""Offline tests of the chain-check analysis."""

from __future__ import annotations

import math
from pathlib import Path
import sys

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent))
import chain_analysis as ca  # noqa: E402


def test_motion_start_after_a_dead_time():
    t = np.arange(0.0, 1.0, 0.005)
    value = np.where(t < 0.537, 0.2, 0.2 - 5.0 * (t - 0.537))  # moves down from 0.537 s at 5 rad/s
    d = ca.motion_start(t, value, 0.5, threshold=2e-3)
    assert d == pytest.approx(0.040, abs=1e-9)  # the first 5 ms sample after 0.537 s: 0.540 s, 15 mrad off
    assert math.isnan(ca.motion_start(t, np.full_like(t, 0.2), 0.5, 2e-3))
    assert math.isnan(ca.motion_start(t, value, 0.0, 2e-3))  # no baseline before t = 0


def test_summary_and_histogram():
    s = [0.010, 0.012, 0.014, math.nan]
    r = ca.summary(s)
    assert (r["n"], r["n_nan"]) == (3, 1)
    assert r["median_ms"] == pytest.approx(12.0) and r["max_ms"] == pytest.approx(14.0)
    assert ca.summary([math.nan]) == {"n": 0, "n_nan": 1}
    assert ca.histogram([0.0101, 0.0105, 0.0139], 2.0) == {"10": 2, "12": 1}


def test_first_transition():
    ev = [(1.0, "controller", "armed"), (1.4, "hold", "setpoint_timeout"), (2.0, "hold", "x")]
    assert ca.first_transition(ev, 1.1, "hold") == (1.4, "setpoint_timeout")
    assert ca.first_transition(ev, 2.5, "hold") is None
