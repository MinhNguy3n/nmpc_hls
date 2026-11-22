"""Analysis functions of the RC car bring-up checks C1-C6 (numpy only; tested offline).

Poses are the Gazebo truth of the model, i.e. the rear-axle midpoint (description/), as arrays
t (sim s), x, y (m), yaw (rad).
"""

from __future__ import annotations

import math
from typing import Sequence

import numpy as np

# Pass thresholds (design; stated in the report with every result)
C1_MAX_DRIFT_M = 1e-3       # xy displacement over the rest window
C1_MAX_YAW_RAD = 1e-3
C2_MAX_SPEED_ERR = 0.02     # relative
C2_MAX_HEADING_RAD = 0.01   # heading change over the measured stretch
C3_MAX_RADIUS_ERR = 0.03    # relative, |R_fit / (L / tan delta) - 1|
C4_MAX_STEER_ERR_RAD = 2e-3 # steady state, per wheel
C5_MIN_RATE_FRAC = 0.9      # measured rate >= 90 % of nominal
C5_MAX_STAMP_LAG_S = 0.05   # joint_states header stamp vs /clock


def unwrap(yaw: Sequence[float]) -> np.ndarray:
    return np.unwrap(np.asarray(yaw, dtype=float))


def drift(t, x, y, yaw) -> dict:
    """C1: displacement of the pose over a rest window."""
    x, y, yaw = (np.asarray(a, dtype=float) for a in (x, y, yaw))
    d = np.hypot(x - x[0], y - y[0])
    dyaw = np.abs(unwrap(yaw) - unwrap(yaw)[0])
    return {"duration_s": float(t[-1] - t[0]), "max_xy_m": float(d.max()), "max_yaw_rad": float(dyaw.max()),
            "final_xy_m": float(d[-1])}


def straight_line(t, x, y, yaw) -> dict:
    """C2: speed by least squares of travelled distance vs time; heading change; lateral spread."""
    t, x, y = (np.asarray(a, dtype=float) for a in (t, x, y))
    s = np.concatenate([[0.0], np.cumsum(np.hypot(np.diff(x), np.diff(y)))])
    speed = float(np.polyfit(t, s, 1)[0])
    yu = unwrap(yaw)
    direction = math.atan2(y[-1] - y[0], x[-1] - x[0])
    # perpendicular distance of every point from the chord start -> end
    lateral = -(x - x[0]) * math.sin(direction) + (y - y[0]) * math.cos(direction)
    return {"speed_mps": speed, "heading_change_rad": float(yu[-1] - yu[0]),
            "lateral_max_m": float(np.abs(lateral).max()), "distance_m": float(s[-1])}


def fit_circle(x, y) -> tuple[float, float, float, float]:
    """Algebraic (Kasa) least-squares circle: (xc, yc, R, rms radial residual)."""
    x, y = np.asarray(x, dtype=float), np.asarray(y, dtype=float)
    a = np.column_stack([x, y, np.ones_like(x)])
    b = x * x + y * y
    (c0, c1, c2), *_ = np.linalg.lstsq(a, b, rcond=None)
    xc, yc = 0.5 * c0, 0.5 * c1
    radius = math.sqrt(c2 + xc * xc + yc * yc)
    residual = np.hypot(x - xc, y - yc) - radius
    return float(xc), float(yc), radius, float(np.sqrt(np.mean(residual ** 2)))


def body_velocities(t, x, y, yaw) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Forward speed, lateral speed (slip indicator at the rear axle) and yaw rate, by central
    differences of the truth."""
    t, x, y = (np.asarray(a, dtype=float) for a in (t, x, y))
    yu = unwrap(yaw)
    vx, vy, w = np.gradient(x, t), np.gradient(y, t), np.gradient(yu, t)
    c, s = np.cos(yu), np.sin(yu)
    return c * vx + s * vy, -s * vx + c * vy, w


def circle(t, x, y, yaw, wheelbase: float, delta: float, speed_cmd: float | None = None) -> dict:
    """C3: fitted radius vs the kinematic L / tan(delta); turn direction vs steering sign.

    With `speed_cmd`, also the rear-axle forward speed error (mean measured / commanded - 1).
    Reported from 2026-11-22 on, next to the lateral speed: an over-constrained 4WD that scrubs
    shows up as a rear axle running faster than commanded at low speed (STOP 1 review, section 2)."""
    xc, yc, radius, rms = fit_circle(x, y)
    expected = wheelbase / math.tan(abs(delta))
    v_fwd, v_lat, w = body_velocities(t, x, y, yaw)
    return {"radius_fit_m": radius, "radius_expected_m": expected, "radius_rel_err": radius / expected - 1.0,
            "fit_rms_m": rms, "yaw_rate_mean": float(np.mean(w)), "speed_mean": float(np.mean(v_fwd)),
            "lateral_speed_mean": float(np.mean(v_lat)), "lateral_speed_max": float(np.max(np.abs(v_lat))),
            "turn_sign_ok": bool(np.sign(np.mean(w)) == np.sign(delta) * np.sign(np.mean(v_fwd))),
            "speed_rel_err": (float(np.mean(v_fwd)) / speed_cmd - 1.0) if speed_cmd else float("nan")}


def rise_time(t, value, start: float, target: float) -> float:
    """10-90 % rise time of a step from `start` to `target`; nan if the 90 % level is never reached."""
    t, value = np.asarray(t, dtype=float), np.asarray(value, dtype=float)
    span = target - start
    frac = (value - start) / span
    above10 = np.nonzero(frac >= 0.1)[0]
    above90 = np.nonzero(frac >= 0.9)[0]
    if not len(above10) or not len(above90):
        return float("nan")
    return float(t[above90[0]] - t[above10[0]])


def rate(stamps: Sequence[float]) -> dict:
    """C5: mean rate and the largest gap of a sequence of receive (or header) times."""
    s = np.asarray(stamps, dtype=float)
    if len(s) < 2:
        return {"count": int(len(s)), "rate_hz": 0.0, "max_gap_s": float("inf")}
    gaps = np.diff(s)
    return {"count": int(len(s)), "rate_hz": float((len(s) - 1) / (s[-1] - s[0])), "max_gap_s": float(gaps.max())}


def real_time_factor(sim: Sequence[float], wall: Sequence[float]) -> float:
    """C6: slope of sim time vs wall time."""
    return float(np.polyfit(np.asarray(wall, dtype=float), np.asarray(sim, dtype=float), 1)[0])
