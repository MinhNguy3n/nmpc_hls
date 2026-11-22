"""Reference paths of the RC car study, sized to the tracked area (params/lab_area.yaml).

All paths lie inside the geofence (the tracked area minus the margin), centred on the origin.
Each is a dense polyline (x, y, yaw, s, kappa) at spacing `ds`, driven at a constant speed.

Core evaluation set (EVALUATION):
  `line`          straight, across the usable length;
  `circle`        R = min(1.5 m, 0.4 x the usable short side), two laps, counter-clockwise;
  `figure_eight`  a SMOOTH lemniscate of Bernoulli (lab facts 2026-11-22: it fits the 8 x 8 m
                  usable area), half-width a = min(3.5 m, 0.4375 x the usable length), one loop;
                  peak curvature 3 / a at the tips;
  `lane_change`   a quintic lateral shift of min(1.0 m, 0.25 x the usable width) over
                  min(3.0 m, 0.5 x the usable length).
Stress path (STRESS, the speed sweep): `arcs_and_lines` = two arcs joined by lines: circles of
  radius 1.2 m centred at (+-2.0, 0) joined by the two straight lines through the origin that are
  tangent to both. Its curvature JUMPS between 0 and +-1/R at four joins (by design). Called "two
  arcs joined by lines", never a smooth figure-eight.
Tuning only (TUNING, never evaluated): `s_curve`, with the FIXED geometry the baseline was tuned
  on (2026-11-22, the 6 x 4 m placeholder area): x from -2.5 to 2.0 m, y = 0.3 sin(2 pi (x + 2.5)
  / 3.6) m. It does not scale with the area, so the NMPC is tuned on the same path as the baseline.

Feasibility: every path's peak curvature is at most PEAK_FRACTION = 70 % of kappa_max (lab facts
3.3: radius >= 1.06 m at kappa_max = 1.352 1/m), so both controllers have room to correct. Checked
in the tests; `build()` raises if a path breaks it or the area is too small.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from pathlib import Path

import numpy as np
import yaml

DEFAULT_AREA = Path(__file__).resolve().parent / "params" / "lab_area.yaml"
EVALUATION = ("line", "circle", "figure_eight", "lane_change")
STRESS = ("arcs_and_lines",)
TUNING = ("s_curve",)
ALL = EVALUATION + STRESS + TUNING
PEAK_FRACTION = 0.7
STOP_ROOM = 0.5  # m left free at the end of open paths (the geofence margin comes on top)


@dataclass(frozen=True)
class Area:
    length: float
    width: float
    margin: float

    @property
    def usable(self) -> tuple[float, float]:
        return self.length - 2 * self.margin, self.width - 2 * self.margin


def load_area(path: str | Path = DEFAULT_AREA) -> Area:
    d = yaml.safe_load(Path(path).read_text())
    return Area(float(d["length"]["value"]), float(d["width"]["value"]), float(d["margin"]["value"]))


@dataclass(frozen=True)
class Path2D:
    name: str
    x: np.ndarray
    y: np.ndarray
    yaw: np.ndarray
    s: np.ndarray
    kappa: np.ndarray
    closed: bool

    @property
    def length(self) -> float:
        return float(self.s[-1])

    @property
    def start(self) -> tuple[float, float, float]:
        return float(self.x[0]), float(self.y[0]), float(self.yaw[0])


def _from_points(name: str, x: np.ndarray, y: np.ndarray, ds: float, closed: bool) -> Path2D:
    """Heading and curvature on the dense parametric curve (second-order differences, also at the
    ends), then resampling of everything to arc-length spacing ds."""
    dx, dy = np.gradient(x, edge_order=2), np.gradient(y, edge_order=2)
    ddx, ddy = np.gradient(dx, edge_order=2), np.gradient(dy, edge_order=2)
    yaw_raw = np.unwrap(np.arctan2(dy, dx))
    kappa_raw = (dx * ddy - dy * ddx) / np.maximum((dx * dx + dy * dy) ** 1.5, 1e-300)
    seg = np.hypot(np.diff(x), np.diff(y))
    s_raw = np.concatenate([[0.0], np.cumsum(seg)])
    s = np.arange(0.0, s_raw[-1] + 1e-12, ds)
    yaw = np.interp(s, s_raw, yaw_raw)
    yaw = (yaw + math.pi) % (2 * math.pi) - math.pi
    return Path2D(name, np.interp(s, s_raw, x), np.interp(s, s_raw, y), yaw, s,
                  np.interp(s, s_raw, kappa_raw), closed)


def build(name: str, area: Area | None = None, kappa_max: float | None = None, ds: float = 0.01) -> Path2D:
    area = area or load_area()
    lu, wu = area.usable
    n = 40001
    if name == "line":
        x = np.linspace(-lu / 2, lu / 2 - STOP_ROOM, n)
        path = _from_points(name, x, np.zeros(n), ds, False)
    elif name == "circle":
        r = min(1.5, 0.4 * min(lu, wu))
        t = np.linspace(0.0, 4 * math.pi, 2 * n)  # two laps, CCW, start at the bottom heading +x
        path = _from_points(name, r * np.sin(t), -r * np.cos(t), ds, True)
    elif name == "figure_eight":
        a = min(3.5, 0.4375 * lu)
        if a / (2 * math.sqrt(2)) > wu / 2:
            raise ValueError("figure_eight: the area is too narrow")
        t = np.linspace(-math.pi / 2, 1.5 * math.pi, 2 * n)  # start at the centre crossing
        den = 1.0 + np.sin(t) ** 2
        path = _from_points(name, a * np.cos(t) / den, a * np.sin(t) * np.cos(t) / den, ds, True)
    elif name == "lane_change":
        x = np.linspace(-lu / 2, lu / 2 - STOP_ROOM, n)
        span = min(3.0, 0.5 * lu)
        x1, x2 = -span / 2, span / 2
        d = min(1.0, 0.25 * wu)  # lateral offset
        tau = np.clip((x - x1) / (x2 - x1), 0.0, 1.0)
        y = -d / 2 + d * (10 * tau ** 3 - 15 * tau ** 4 + 6 * tau ** 5)
        path = _from_points(name, x, y, ds, False)
    elif name == "arcs_and_lines":
        path = _arcs_and_lines(area, ds, r=1.2, c=2.0)
    elif name == "s_curve":  # FIXED geometry (the baseline's tuning path), independent of the area
        x = np.linspace(-2.5, 2.0, n)
        path = _from_points(name, x, 0.3 * np.sin(2 * math.pi * (x + 2.5) / 3.6), ds, False)
        if not inside(path, area):
            raise ValueError("s_curve: the area is smaller than the tuning path")
    else:
        raise KeyError(f"unknown path {name!r}; have {ALL}")
    if kappa_max is not None and np.max(np.abs(path.kappa)) > PEAK_FRACTION * kappa_max + 1e-9:
        raise ValueError(f"{name}: peak curvature {np.max(np.abs(path.kappa)):.3f} > {PEAK_FRACTION} kappa_max = "
                         f"{PEAK_FRACTION * kappa_max:.3f}")
    return path


def _arcs_and_lines(area: Area, ds: float, r: float, c: float) -> Path2D:
    """Two arcs joined by lines: circles of radius r centred at (+-c, 0), joined by the two
    straight lines through the origin that are tangent to both. Start at the origin heading
    up-right; the right circle is driven clockwise, the left one counter-clockwise. Built exactly
    by arc length."""
    lu, wu = area.usable
    if c + r > lu / 2 or r > wu / 2 or c < 1.1 * r:
        raise ValueError("arcs_and_lines: does not fit the area")
    alpha = math.asin(r / c)
    d = math.sqrt(c * c - r * r)  # origin -> tangent point
    sweep = math.pi + 2 * alpha
    s = np.arange(0.0, 4 * d + 2 * sweep * r + 1e-12, ds)
    x, y, yaw, kappa = (np.empty_like(s) for _ in range(4))
    bounds = np.cumsum([d, sweep * r, 2 * d, sweep * r, d])
    for i, si in enumerate(s):
        if si <= bounds[0]:  # origin -> T1, heading alpha
            u = si
            x[i], y[i], yaw[i], kappa[i] = u * math.cos(alpha), u * math.sin(alpha), alpha, 0.0
        elif si <= bounds[1]:  # right circle, clockwise
            phi = alpha + math.pi / 2 - (si - bounds[0]) / r
            x[i], y[i], yaw[i], kappa[i] = c + r * math.cos(phi), r * math.sin(phi), phi - math.pi / 2, -1.0 / r
        elif si <= bounds[2]:  # T2 -> origin -> T3, heading pi - alpha
            u = si - bounds[1] - d  # signed distance past the origin
            h = math.pi - alpha
            x[i], y[i], yaw[i], kappa[i] = u * math.cos(h), u * math.sin(h), h, 0.0
        elif si <= bounds[3]:  # left circle, counter-clockwise
            phi = -alpha + math.pi / 2 + (si - bounds[2]) / r
            x[i], y[i], yaw[i], kappa[i] = -c + r * math.cos(phi), r * math.sin(phi), phi + math.pi / 2, 1.0 / r
        else:  # T4 -> origin, heading alpha
            u = si - bounds[4]  # negative until the origin
            x[i], y[i], yaw[i], kappa[i] = u * math.cos(alpha), u * math.sin(alpha), alpha, 0.0
    yaw = (yaw + math.pi) % (2 * math.pi) - math.pi
    return Path2D("arcs_and_lines", x, y, yaw, s, kappa, True)


def inside(path: Path2D, area: Area) -> bool:
    lu, wu = area.usable
    return bool(np.all(np.abs(path.x) <= lu / 2 + 1e-9) and np.all(np.abs(path.y) <= wu / 2 + 1e-9))
