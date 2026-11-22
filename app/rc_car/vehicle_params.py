"""Load the RC car parameters (params/reely_new1.yaml) and derive the steering-joint geometry.

Plain Python (PyYAML + the tested Ackermann model); no ROS. Every entry carries a status
(`spec_unverified`, `placeholder`, `measured`, `design`) so reports can list what is still a
PLACEHOLDER_.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from pathlib import Path
import sys
from typing import Any

import yaml

_HERE = Path(__file__).resolve().parent
_ACKERMANN = str(_HERE.parent / "ackermann")
if _ACKERMANN not in sys.path:
    sys.path.insert(0, _ACKERMANN)
import ackermann_model as am  # noqa: E402

DEFAULT_PARAMS = _HERE / "params" / "reely_new1.yaml"
STATUSES = ("spec_unverified", "placeholder", "measured_approx", "measured", "design")
REQUIRED = ("wheelbase", "front_track", "rear_track", "wheel_radius", "delta_max", "v_max_sim", "plant_timeout")


def load_raw(path: str | Path = DEFAULT_PARAMS) -> dict[str, dict[str, Any]]:
    with open(path, encoding="utf-8") as stream:
        raw = yaml.safe_load(stream)
    for name, entry in raw.items():
        if not isinstance(entry, dict) or not {"value", "unit", "status", "source"} <= entry.keys():
            raise ValueError(f"{name}: needs value, unit, status and source")
        if entry["status"] not in STATUSES:
            raise ValueError(f"{name}: unknown status {entry['status']!r}")
        if not isinstance(entry["value"], (int, float)) or not math.isfinite(entry["value"]):
            raise ValueError(f"{name}: value must be a finite number")
    missing = [name for name in REQUIRED if name not in raw]
    if missing:
        raise ValueError(f"missing parameters: {missing}")
    return raw


@dataclass(frozen=True)
class CarGeometry:
    wheelbase: float
    front_track: float
    rear_track: float
    wheel_radius: float
    delta_max: float  # bicycle-equivalent full lock, symmetric (asymmetric travel: later, from sheet item 4)

    def __post_init__(self) -> None:
        if min(self.wheelbase, self.front_track, self.rear_track, self.wheel_radius) <= 0.0:
            raise ValueError("lengths must be positive")
        if not 0.0 < self.delta_max < 0.5 * math.pi:
            raise ValueError("delta_max must be in (0, pi/2)")
        if self.kappa_max * 0.5 * self.front_track >= 1.0:
            raise ValueError("full lock puts the ICR inside the front track")

    @property
    def kappa_max(self) -> float:
        return am.kappa_limit_from_steering(self.delta_max, self.wheelbase)

    @property
    def r_min(self) -> float:
        """Rear-axle-midpoint turning radius at full lock: L / tan(delta_max)."""
        return 1.0 / self.kappa_max

    @property
    def inner_wheel_max(self) -> float:
        """Inner front wheel angle at full lock: the steering joint limit needed (> delta_max)."""
        return am.inner_outer_angles(self.kappa_max, self.wheelbase, self.front_track)[0]


def load_geometry(path: str | Path = DEFAULT_PARAMS) -> CarGeometry:
    raw = load_raw(path)
    return CarGeometry(*(float(raw[name]["value"]) for name in
                         ("wheelbase", "front_track", "rear_track", "wheel_radius", "delta_max")))


def placeholders(path: str | Path = DEFAULT_PARAMS) -> list[str]:
    """Names of the entries that are still PLACEHOLDER_."""
    return [name for name, entry in load_raw(path).items() if entry["status"] == "placeholder"]
