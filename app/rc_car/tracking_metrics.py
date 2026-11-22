"""Path-tracking metrics of the RC car runs (numpy only; tested in test_tracking_metrics.py).

The pose is the Gazebo truth of the rear-axle midpoint (never the mocap pose). The reference
point is found by a LOCAL search that only moves forward along the path (a window of `ahead_m`
ahead and `back_m` behind the previous match), so the figure-eight crossing and the second lap
of the circle are never matched to the wrong branch.

    e_y      signed cross-track error at the matched point (left of the path positive), m
    e_psi    heading error, wrapped, rad
    s        matched arc length, m

Scoring window: from the first sample until the match reaches length - end_margin (the end of
the path, where the controllers stop, is excluded the same way for both arms).
"""

from __future__ import annotations

import math

import numpy as np

from paths import Path2D


def match(path: Path2D, x, y, ahead_m: float = 1.0, back_m: float = 0.1, first_window_m: float = 1.0) -> np.ndarray:
    """Index of the matched path point for every pose."""
    x, y = np.asarray(x, dtype=float), np.asarray(y, dtype=float)
    ds = float(path.s[1] - path.s[0])
    ahead, back = int(round(ahead_m / ds)), int(round(back_m / ds))
    idx = np.empty(len(x), dtype=int)
    lo, hi = 0, min(len(path.s), int(round(first_window_m / ds)) + 1)
    for k in range(len(x)):
        d2 = (path.x[lo:hi] - x[k]) ** 2 + (path.y[lo:hi] - y[k]) ** 2
        i = lo + int(np.argmin(d2))
        idx[k] = i
        lo, hi = max(0, i - back), min(len(path.s), i + ahead + 1)
    return idx


def errors(path: Path2D, x, y, yaw, idx: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    x, y, yaw = (np.asarray(a, dtype=float) for a in (x, y, yaw))
    yr = path.yaw[idx]
    e_y = -(x - path.x[idx]) * np.sin(yr) + (y - path.y[idx]) * np.cos(yr)
    e_psi = np.angle(np.exp(1j * (yaw - yr)))
    return e_y, e_psi, path.s[idx]


def track(path: Path2D, t, x, y, yaw, end_margin: float = 0.3) -> dict:
    t = np.asarray(t, dtype=float)
    idx = match(path, x, y)
    e_y, e_psi, s = errors(path, x, y, yaw, idx)
    s_stop = path.length - end_margin
    reached = np.nonzero(s >= s_stop)[0]
    n = int(reached[0]) + 1 if len(reached) else len(t)
    e_y, e_psi, s_w, t_w = e_y[:n], e_psi[:n], s[:n], t[:n]
    duration = float(t_w[-1] - t_w[0]) if n > 1 else 0.0
    rms = lambda a: float(np.sqrt(np.mean(a ** 2)))  # noqa: E731
    return {"cross_track_rmse_m": rms(e_y), "cross_track_max_m": float(np.max(np.abs(e_y))),
            "heading_rmse_rad": rms(e_psi), "heading_max_rad": float(np.max(np.abs(e_psi))),
            "completion": float(min(1.0, (s_w.max() + end_margin) / path.length)) if len(reached) else
            float(s_w.max() / path.length), "reached_end": bool(len(reached)),
            "duration_s": duration, "mean_progress_speed_mps": float((s_w[-1] - s_w[0]) / duration) if duration else 0.0,
            "samples": n, "end_margin_m": end_margin}


def series(path: Path2D, t, x, y, yaw) -> dict:
    """Per-sample e_y, e_psi, s (for plots)."""
    idx = match(path, x, y)
    e_y, e_psi, s = errors(path, x, y, yaw, idx)
    return {"t": np.asarray(t, dtype=float), "e_y": e_y, "e_psi": e_psi, "s": s}


def wrap(a: float) -> float:
    return (a + math.pi) % (2 * math.pi) - math.pi
