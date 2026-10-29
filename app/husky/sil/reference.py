"""Reference paths for the SIL runs, defined RELATIVE to the pose at which the MPC takes ownership.

Frame: x forward, y left of the start pose; headings CCW. Sizes are small so that the paths fit an
open area of the warehouse world; the user spawns or drives the robot to an open area first and
checks clearance in Gazebo (plan section 4.1, stage 3). Defaults (PLACEHOLDER_ screening values):
- line:      3.0 m straight ahead;
- circle:    one CCW lap of radius 1.0 m (centre 1.0 m to the left of the start);
- figure8:   a CCW lap (centre at +R left) then a CW lap (centre at -R right), radius 1.0 m;
  footprint 2 m x 4 m centred on the start, which is the crossing point.
Speed 0.3 m/s (the manual's slow-mode level). After the path ends the reference holds the end pose.
"""

from __future__ import annotations

from dataclasses import dataclass
import math

import numpy as np

from . import _HUSKY  # noqa: F401
import unicycle_model as um


@dataclass(frozen=True)
class PathSpec:
    kind: str = "circle"  # line | circle | figure8
    speed: float = 0.3  # m/s
    size: float = 1.0  # m: radius (circle, figure8) or length / 3 (line uses 3 * size)

    def __post_init__(self) -> None:
        if self.kind not in ("line", "circle", "figure8"):
            raise ValueError("kind must be line, circle or figure8")
        if not (self.speed > 0 and self.size > 0):
            raise ValueError("speed and size must be positive")

    @property
    def length(self) -> float:
        if self.kind == "line":
            return 3.0 * self.size
        laps = 1 if self.kind == "circle" else 2
        return laps * 2 * math.pi * self.size

    @property
    def duration(self) -> float:
        return self.length / self.speed

    def pose(self, tau: float) -> np.ndarray:
        """Relative pose [x, y, psi] at time tau >= 0 after the start (psi unwrapped)."""
        s = min(max(tau, 0.0), self.duration) * self.speed
        r = self.size
        if self.kind == "line":
            return np.array([s, 0.0, 0.0])
        if self.kind == "circle" or s <= 2 * math.pi * r:
            theta = s / r  # CCW around (0, r)
            return np.array([r * math.sin(theta), r * (1 - math.cos(theta)), theta])
        theta = (s - 2 * math.pi * r) / r  # CW around (0, -r), starting at the origin heading +x
        return np.array([r * math.sin(theta), -r * (1 - math.cos(theta)), 2 * math.pi - theta])

    def poses(self, times: np.ndarray) -> np.ndarray:
        return np.array([self.pose(float(t)) for t in times])

    def polyline(self, spacing: float = 0.01) -> np.ndarray:
        """Dense points of the whole path (for cross-track error)."""
        n = max(int(self.length / spacing), 2)
        return self.poses(np.linspace(0.0, self.duration, n + 1))[:, :2]


def compose(origin: np.ndarray, relative: np.ndarray) -> np.ndarray:
    """World poses of relative poses (rows) attached to the origin pose."""
    rel = np.atleast_2d(relative)
    c, s = math.cos(origin[2]), math.sin(origin[2])
    out = np.column_stack([origin[0] + c * rel[:, 0] - s * rel[:, 1],
                           origin[1] + s * rel[:, 0] + c * rel[:, 1],
                           origin[2] + rel[:, 2] if rel.shape[1] > 2 else np.zeros(len(rel))])
    return out


def reference_window(spec: PathSpec, origin: np.ndarray, tau0: float, horizon: int, ts: float,
                     state_at_actuation: np.ndarray) -> np.ndarray:
    """Nh reference poses for x_1..x_Nh after the actuation time, in the local frame of the
    predicted state (headings unwrapped from 0; design note 7.3)."""
    rel = spec.poses(tau0 + ts * np.arange(1, horizon + 1))
    return um.to_local_frame(state_at_actuation, compose(origin, rel))


def cross_track(points: np.ndarray, polyline: np.ndarray) -> np.ndarray:
    """Distance of each point to the nearest segment of the polyline."""
    a, b = polyline[:-1], polyline[1:]
    ab = b - a
    denom = np.maximum(np.sum(ab * ab, axis=1), 1e-18)
    out = np.empty(len(points))
    for i, p in enumerate(np.atleast_2d(points)):
        t = np.clip(np.sum((p - a) * ab, axis=1) / denom, 0.0, 1.0)
        out[i] = float(np.min(np.hypot(*(a + t[:, None] * ab - p).T)))
    return out
