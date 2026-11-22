"""Controller pose input of the RC car arms: the newest capture stamp wins (phase 3 condition 3).

Both arms (the baseline through mocap_bridge_node.py, the NMPC in phase 4) read the same
/mocap/<body>/pose topic through this class. A pose whose capture stamp is not newer than the
newest one accepted (a frame overtaken by a later one, e.g. after a Wi-Fi spike) is ignored and
counted, as in the freshness gate (command_mux.FreshnessGate).
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Optional


@dataclass(frozen=True)
class Pose:
    stamp: float  # capture time, s
    x: float
    y: float
    yaw: float


class NewestPose:
    def __init__(self) -> None:
        self.pose: Optional[Pose] = None
        self.accepted = 0
        self.ignored_old = 0
        self.ignored_bad = 0

    def update(self, stamp: float, x: float, y: float, yaw: float) -> bool:
        """True if the pose was accepted (newer than every pose before)."""
        if not all(math.isfinite(v) for v in (stamp, x, y, yaw)):
            self.ignored_bad += 1
            return False
        if self.pose is not None and stamp <= self.pose.stamp:
            self.ignored_old += 1
            return False
        self.pose = Pose(stamp, x, y, yaw)
        self.accepted += 1
        return True

    def age(self, now: float) -> float:
        return math.inf if self.pose is None else now - self.pose.stamp


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
