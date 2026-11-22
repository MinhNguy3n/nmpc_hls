"""Offline tests of the tracking metrics."""

from __future__ import annotations

import math

import numpy as np
import pytest

import paths
import tracking_metrics as tm

AREA = paths.load_area()


def follow(path: paths.Path2D, offset: float = 0.0, dyaw: float = 0.0, v: float = 1.0, rate: float = 100.0):
    """Poses along the path at speed v, shifted `offset` to the left, heading off by dyaw."""
    s = np.arange(0.0, path.length, v / rate)
    x, y, yaw = (np.interp(s, path.s, a) for a in (path.x, path.y, np.unwrap(path.yaw)))
    return s / v, x - offset * np.sin(yaw), y + offset * np.cos(yaw), yaw + dyaw


@pytest.mark.parametrize("name", paths.EVALUATION)
def test_exact_following_gives_zero_error(name):
    p = paths.build(name, AREA)
    t, x, y, yaw = follow(p)
    r = tm.track(p, t, x, y, yaw)
    assert r["cross_track_max_m"] < 1e-3 and r["heading_max_rad"] < 0.02
    assert r["reached_end"] and r["completion"] == pytest.approx(1.0)
    assert r["mean_progress_speed_mps"] == pytest.approx(1.0, rel=0.01)


def test_constant_offset_sign_and_heading_error():
    p = paths.build("circle", AREA)
    t, x, y, yaw = follow(p, offset=0.05, dyaw=0.02)
    r = tm.track(p, t, x, y, yaw)
    assert r["cross_track_rmse_m"] == pytest.approx(0.05, abs=2e-3)
    ser = tm.series(p, t, x, y, yaw)
    assert np.all(ser["e_y"][:-50] > 0.045)  # left of the path is positive
    assert r["heading_rmse_rad"] == pytest.approx(0.02, abs=2e-3)


def test_figure_eight_crossing_and_circle_laps_stay_on_branch():
    """The match moves monotonically: at the crossing and on lap 2 it never jumps."""
    for name in ("figure_eight", "circle", "arcs_and_lines"):
        p = paths.build(name, AREA)
        t, x, y, yaw = follow(p, offset=0.03)
        ser = tm.series(p, t, x, y, yaw)
        assert np.all(np.diff(ser["s"]) >= -1e-9)
        assert ser["s"][-1] > p.length - 0.05


def test_incomplete_run():
    p = paths.build("line", AREA)
    t, x, y, yaw = follow(p)
    r = tm.track(p, t[: len(t) // 2], x[: len(t) // 2], y[: len(t) // 2], yaw[: len(t) // 2])
    assert not r["reached_end"] and r["completion"] == pytest.approx(0.5, abs=0.01)


def test_window_ends_at_the_end_margin():
    p = paths.build("line", AREA)
    t, x, y, yaw = follow(p)
    y = y + np.where(x > p.x[-1] - 0.2, 1.0, 0.0)  # a wild end inside the last 0.3 m is not scored
    assert tm.track(p, t, x, y, yaw)["cross_track_max_m"] < 1e-3


def test_goal_segments_cover_the_path_with_overlap():
    import sys
    from pathlib import Path
    sys.path.insert(0, str(Path(__file__).resolve().parent / "checks"))
    import track_run as tr
    for length in (0.5, 2.0, 4.2, 14.77):
        segs = tr.segments(length)
        assert segs[0][0] == 0.0 and segs[-1][1] == length
        assert all(b[0] - a[0] == tr.SEG_STEP for a, b in zip(segs, segs[1:]))
        assert all(s1 - s0 <= tr.SEG_LEN for s0, s1 in segs)
        # while a segment is active (until the next one starts) its end lies >= SEG_LEN - SEG_STEP ahead
        assert all(a[1] - b[0] >= tr.SEG_LEN - tr.SEG_STEP - 1e-9 or a[1] == length for a, b in zip(segs, segs[1:]))
