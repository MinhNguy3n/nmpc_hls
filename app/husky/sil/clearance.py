"""Collision clearance of a SIL path in the Clearpath `warehouse` world (simulation only).

    python3 -m sil.clearance --path figure8 --size 2 --start 0 3.9 1.5708

Obstacles are axis-aligned rectangles in world x/y at robot height, taken from
clearpath_gz/worlds/warehouse.sdf (model poses) and the Fuel models' collision geometry
(checked 2026-11-19):

    shelf        box 3.6 x 0.6 (MovAi/shelf/1)          shelf_big   box 2.1 x 18 (MovAi/shelf_big/2)
    person 1     mesh MaleVisitorStatic_Col.obj, local x -0.11..0.34, y -0.30..0.20, yaw 1.57
    warehouse    pillars 0.5 x 0.5 at x = +-7.18..7.67, y = -15, -7.5, 0, 7.5, 15; outer walls
                 at |x| = 15, |y| = 25 (warehouse_colision.stl, surfaces between 0.3 and 1 m)
    far objects  (barriers, chairs, table, other people, pallet box): generous 1.5 m squares

The robot is a circle of ROBOT_RADIUS around base_link: the A300's collision geometry is a
0.74 x 0.42 m top plate and four wheels of radius 0.165 m on a 0.68 m outer track, inside
0.9 x 0.75 m, whose half-diagonal is 0.59 m. The swept footprint is that circle along the path
(any heading). The reported clearance is the minimum distance from that circle to an obstacle.
"""

from __future__ import annotations

import argparse
import math
import sys

import numpy as np

from . import _HUSKY  # noqa: F401
from .reference import PathSpec

ROBOT_RADIUS = 0.6


def _box(cx: float, cy: float, sx: float, sy: float, yaw: float = 0.0) -> tuple[float, float, float, float]:
    if abs(math.sin(yaw)) > 0.7:  # rotated by about +-90 deg
        sx, sy = sy, sx
    return (cx - sx / 2, cy - sy / 2, cx + sx / 2, cy + sy / 2)


OBSTACLES: dict[str, tuple[float, float, float, float]] = {
    "shelf_3": _box(13.5, 4.5, 3.6, 0.6), "shelf_4": _box(10.0, 4.5, 3.6, 0.6),
    "shelf_5": _box(13.5, -21.0, 3.6, 0.6), "shelf_6": _box(13.5, -15.0, 3.6, 0.6),
    "shelf_7": _box(0.4, -2.0, 3.6, 0.6),
    "shelf_0": _box(-10.0, 21.5, 3.6, 0.6, 1.57), "shelf_1": _box(-7.0, 23.6, 3.6, 0.6, 1.57),
    "shelf_2": _box(-4.0, 21.5, 3.6, 0.6, 1.57),
    "shelf_big_0": _box(-8.5, -13.0, 2.1, 18.0), "shelf_big_1": _box(6.5, -13.0, 2.1, 18.0),
    "shelf_big_2": _box(-1.5, -13.0, 2.1, 18.0),
    "shelf_big_3": _box(3.5, 9.5, 2.1, 18.0, 1.57), "shelf_big_4": _box(-1.3, 18.5, 2.1, 18.0, 1.57),
    "person_1": (1.0 - 0.20, -1.0 - 0.11, 1.0 + 0.30, -1.0 + 0.34),
    **{f"pillar_{side}{y:+g}": (sx - 0.25 if side == "e" else -7.67, y - 0.25, 7.67 if side == "e" else -7.18,
                                y + 0.25)
       for side, sx in (("e", 7.43), ("w", -7.43)) for y in (-15.0, -7.5, 0.0, 7.5, 15.0)},
    "wall_e": (15.0, -25.0, 16.0, 25.0), "wall_w": (-16.0, -25.0, -15.0, 25.0),
    "wall_n": (-15.0, 25.0, 15.0, 26.0), "wall_s": (-15.0, -26.0, 15.0, -25.0),
    **{name: _box(x, y, 1.5, 1.5) for name, x, y in (
        ("pallet_box_0", -4.0, 12.0), ("chair_0", 14.3, -5.5), ("chair_1", 14.3, -4.0),
        ("fchair_0", -11.5, 6.4), ("fchair1", -14.0, 6.5), ("table0", -12.7, 6.5),
        ("person_0", 14.65, -10.0), ("person_2", -12.0, 15.0))},
    **{f"barrier_{k}": _box(-10.4, y, 4.2, 1.0, 1.57) for k, y in enumerate((14.75, 10.5, 6.5))},
    "barrier_3": _box(-12.85, 4.85, 4.2, 1.0),
}


def path_points(spec: PathSpec, start: tuple[float, float, float], step: float = 0.02) -> np.ndarray:
    """World x/y of the reference path attached at `start` (x, y, yaw)."""
    t = np.arange(0.0, spec.duration + step, step)
    rel = spec.poses(t)
    c, s = math.cos(start[2]), math.sin(start[2])
    return np.column_stack([start[0] + c * rel[:, 0] - s * rel[:, 1], start[1] + s * rel[:, 0] + c * rel[:, 1]])


def box_distance(points: np.ndarray, box: tuple[float, float, float, float]) -> np.ndarray:
    dx = np.maximum(np.maximum(box[0] - points[:, 0], 0.0), points[:, 0] - box[2])
    dy = np.maximum(np.maximum(box[1] - points[:, 1], 0.0), points[:, 1] - box[3])
    return np.hypot(dx, dy)


def clearance(spec: PathSpec, start: tuple[float, float, float], robot_radius: float = ROBOT_RADIUS) -> dict:
    pts = path_points(spec, start)
    per = {name: float(box_distance(pts, box).min()) - robot_radius for name, box in OBSTACLES.items()}
    nearest = min(per, key=per.get)
    return {"path": spec.kind, "size": spec.size, "speed": spec.speed, "start": list(start),
            "robot_radius": robot_radius, "extent_x": [float(pts[:, 0].min()), float(pts[:, 0].max())],
            "extent_y": [float(pts[:, 1].min()), float(pts[:, 1].max())],
            "min_clearance": per[nearest], "nearest": nearest}


def main(argv: list[str] | None = None) -> int:
    p = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    p.add_argument("--path", choices=("line", "circle", "figure8"), required=True)
    p.add_argument("--size", type=float, required=True)
    p.add_argument("--speed", type=float, default=0.2)
    p.add_argument("--start", type=float, nargs=3, default=(0.0, 0.0, 0.0), metavar=("X", "Y", "YAW"))
    p.add_argument("--min", type=float, default=1.0, help="required clearance beyond the robot circle (m)")
    args = p.parse_args(argv)
    r = clearance(PathSpec(args.path, args.speed, args.size), tuple(args.start))
    ok = r["min_clearance"] >= args.min
    print(f"{r['path']} r={r['size']} start={r['start']}: x {r['extent_x'][0]:.2f}..{r['extent_x'][1]:.2f}, "
          f"y {r['extent_y'][0]:.2f}..{r['extent_y'][1]:.2f}; min clearance {r['min_clearance']:.2f} m "
          f"to {r['nearest']} (robot circle {r['robot_radius']} m) -> {'OK' if ok else 'TOO CLOSE'} (>= {args.min} m)")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
