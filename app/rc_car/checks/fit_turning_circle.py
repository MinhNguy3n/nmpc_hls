"""Turning-circle fit from a Motive CSV export: which reading of "about 25 deg full lock" is right?

Drive the car slowly at full lock in a steady circle (both directions, several laps each) with the
OptiTrack rigid body tracked, export the take as CSV from Motive, and run:

    python3 app/rc_car/checks/fit_turning_circle.py take.csv [--body NAME] [--plane xz|xy]
        [--offset OX OY] [--wheelbase 0.275] [--track 0.30]

What it does:
1. Reads the rigid body's position (Motive's multi-row CSV header: Type / Name / ID / Rotation or
   Position / X Y Z). `--plane xz` (default) is Motive's Y-up ground plane; `--plane xy` for Z-up.
2. Splits the track into turning directions by the sign of the path curvature (smoothed), and fits
   a least-squares circle to each direction's points (algebraic fit, then geometric refinement).
3. Offset (`--offset OX OY`, metres, car frame: x forward, y left): if the rigid body's pivot is not
   at the rear-axle midpoint, its circle is concentric with the rear axle's, with radius
   R_m = sqrt(OX^2 + (R - OY)^2) for a left turn (centre at y = +R) and
   R_m = sqrt(OX^2 + (R + OY)^2) for a right turn, so R = +-OY + sqrt(R_m^2 - OX^2) (driving forward).
4. Reports, per direction: the rear-axle radius R, the bicycle angle delta = atan(L / R), the inner
   and outer wheel angles atan(L / (R -+ W/2)), the fit residual, the angle covered, and which
   reading of "25 deg" it supports (inner wheel: R ~ 0.74 m; bicycle: ~0.59 m; outer wheel: ~0.44 m
   with L = 0.275 m, W = 0.30 m).

The rear-axle radius assumes no wheel slip: drive SLOWLY (e.g. < 0.3 m/s).
"""

from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path
import sys

import numpy as np


# ----------------------------------------------------------------------------------------------
# Motive CSV
# ----------------------------------------------------------------------------------------------

def read_motive_csv(path: str | Path, body: str | None = None, plane: str = "xz") -> tuple[np.ndarray, np.ndarray, str]:
    """(t, positions (N, 2) in the ground plane, body name) of one rigid body."""
    rows = list(csv.reader(Path(path).read_text(errors="replace").splitlines()))
    header_i = next(i for i, r in enumerate(rows) if r and r[0].strip().lower() == "frame")
    hdr = rows[header_i]
    above = rows[max(0, header_i - 5):header_i]

    def find(label: str) -> list[str] | None:
        for r in above:
            if r and r[0].strip().lower() == label:
                return r
            if len(r) > 1 and r[1].strip().lower() == label:
                return r
        return None

    types, names, kinds = find("type"), find("name"), None
    for r in above:  # the Rotation/Position row has no label in column 0 in recent Motive versions
        if any(c.strip().lower() == "position" for c in r):
            kinds = r
    if names is None or kinds is None:
        raise ValueError("not a Motive CSV export: no Name / Position header rows found")
    cols = {}
    for j, axis in enumerate(hdr):
        if j >= len(kinds) or kinds[j].strip().lower() != "position":
            continue
        if types is not None and j < len(types) and types[j].strip().lower() not in ("rigid body", ""):
            continue  # markers of the rigid body, etc.
        name = names[j].strip() if j < len(names) else ""
        cols.setdefault(name, {})[axis.strip().upper()] = j
    if not cols:
        raise ValueError("no rigid-body position columns found")
    if body is None:
        if len(cols) > 1:
            raise ValueError(f"several rigid bodies {sorted(cols)}: choose one with --body")
        body = next(iter(cols))
    if body not in cols:
        raise ValueError(f"rigid body {body!r} not found; have {sorted(cols)}")
    a, b = ("X", "Z") if plane == "xz" else ("X", "Y")
    t_col = next(j for j, c in enumerate(hdr) if c.strip().lower().startswith("time"))
    t, p = [], []
    for r in rows[header_i + 1:]:
        try:
            ti, x, y = float(r[t_col]), float(r[cols[body][a]]), float(r[cols[body][b]])
        except (ValueError, IndexError, KeyError):
            continue  # untracked frame
        t.append(ti)
        p.append((x, y))
    if plane == "xz":  # Y up: (x, z) is left-handed seen from above; use (x, -z) so CCW stays CCW
        p = [(x, -z) for x, z in p]
    return np.array(t), np.array(p), body


# ----------------------------------------------------------------------------------------------
# Geometry
# ----------------------------------------------------------------------------------------------

def fit_circle(p: np.ndarray, iterations: int = 20) -> tuple[float, float, float, float]:
    """(xc, yc, R, rms residual): Kasa algebraic fit, then Gauss-Newton on the geometric distance."""
    x, y = p[:, 0], p[:, 1]
    A = np.column_stack([x, y, np.ones_like(x)])
    c, *_ = np.linalg.lstsq(A, x * x + y * y, rcond=None)
    xc, yc = 0.5 * c[0], 0.5 * c[1]
    r = math.sqrt(c[2] + xc * xc + yc * yc)
    for _ in range(iterations):
        dx, dy = x - xc, y - yc
        d = np.hypot(dx, dy)
        res = d - r
        J = np.column_stack([-dx / d, -dy / d, -np.ones_like(d)])
        step, *_ = np.linalg.lstsq(J, -res, rcond=None)
        xc, yc, r = xc + step[0], yc + step[1], r + step[2]
        if np.max(np.abs(step)) < 1e-12:
            break
    res = np.hypot(x - xc, y - yc) - r
    return float(xc), float(yc), float(r), float(np.sqrt(np.mean(res ** 2)))


def turn_sign(p: np.ndarray, window_m: float = 0.10) -> np.ndarray:
    """+1 left (CCW), -1 right (CW), 0 undecided, per sample: the sign of the heading change over
    about `window_m` of path (robust to marker noise much larger than the sample spacing)."""
    n = len(p)
    step = np.hypot(*np.diff(p, axis=0).T)
    d = float(np.median(step)) if len(step) else 0.0
    k = max(3, int(round(window_m / d))) if d > 0 else 3
    sign = np.zeros(n)
    if n < 2 * k + 1:
        return sign
    head = np.arctan2(p[k:, 1] - p[:-k, 1], p[k:, 0] - p[:-k, 0])  # heading of the chord i -> i+k
    dpsi = np.angle(np.exp(1j * (head[k:] - head[:-k])))           # change over the next k samples
    chord = np.hypot(p[k:, 0] - p[:-k, 0], p[k:, 1] - p[:-k, 1])
    ok = (chord[:-k] > 0.5 * window_m) & (chord[k:] > 0.5 * window_m) & (np.abs(dpsi) > 0.02)
    sign[k: k + len(dpsi)] = np.where(ok, np.sign(dpsi), 0.0)
    # keep only runs of one sign longer than 3 windows (isolated flips are noise)
    i = 0
    while i < n:
        j = i
        while j < n and sign[j] == sign[i]:
            j += 1
        if sign[i] != 0 and j - i < 3 * k:
            sign[i:j] = 0
        i = j
    return sign


def rear_axle_radius(r_marker: float, offset: tuple[float, float], left: bool) -> float:
    ox, oy = offset
    if r_marker < abs(ox):
        raise ValueError("marker radius smaller than the forward offset: check --offset")
    root = math.sqrt(r_marker * r_marker - ox * ox)
    return oy + root if left else -oy + root


def angles(R: float, wheelbase: float, track: float) -> dict:
    return {"delta_bicycle_rad": math.atan(wheelbase / R), "inner_wheel_rad": math.atan(wheelbase / (R - track / 2)),
            "outer_wheel_rad": math.atan(wheelbase / (R + track / 2))}


def reading(R: float, wheelbase: float, track: float, deg: float = 25.0) -> dict:
    """Rear-axle radius predicted by each reading of 'deg degrees full lock'."""
    t = math.tan(math.radians(deg))
    return {"inner_wheel": wheelbase / t + track / 2, "bicycle": wheelbase / t, "outer_wheel": wheelbase / t - track / 2}


def analyse(p: np.ndarray, offset=(0.0, 0.0), wheelbase=0.275, track=0.30, min_points=50) -> list[dict]:
    sign = turn_sign(p)
    out = []
    for s, label in ((1, "left"), (-1, "right")):
        sel = p[sign == s]
        if len(sel) < min_points:
            continue
        xc, yc, rm, rms = fit_circle(sel)
        for _ in range(3):  # reject outliers (e.g. points of the other turn) and refit
            res = np.abs(np.hypot(sel[:, 0] - xc, sel[:, 1] - yc) - rm)
            keep = res <= max(5 * rms, 0.01) if rms > 0 else res <= 0.01
            if keep.all() or keep.sum() < min_points:
                break
            sel = sel[keep]
            xc, yc, rm, rms = fit_circle(sel)
        ang = np.unwrap(np.arctan2(sel[:, 1] - yc, sel[:, 0] - xc))
        R = rear_axle_radius(rm, offset, s > 0)
        pred = reading(R, wheelbase, track)
        best = min(pred, key=lambda k: abs(pred[k] - R))
        out.append({"direction": label, "points": int(len(sel)), "marker_radius_m": rm, "fit_rms_m": rms,
                    "turns_covered": float(abs(ang[-1] - ang[0]) / (2 * math.pi)), "centre": (xc, yc),
                    "rear_axle_radius_m": R, **angles(R, wheelbase, track),
                    "predicted_radius_by_reading_m": pred, "closest_reading": best})
    return out


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("csv")
    ap.add_argument("--body", default=None)
    ap.add_argument("--plane", choices=("xz", "xy"), default="xz", help="ground plane: xz = Motive Y-up (default)")
    ap.add_argument("--offset", type=float, nargs=2, default=(0.0, 0.0), metavar=("OX", "OY"),
                    help="rigid-body pivot relative to the rear-axle midpoint, car frame (x fwd, y left), m")
    ap.add_argument("--wheelbase", type=float, default=0.275)
    ap.add_argument("--track", type=float, default=0.30)
    a = ap.parse_args(argv)
    t, p, body = read_motive_csv(a.csv, a.body, a.plane)
    print(f"{body}: {len(p)} tracked frames over {t[-1] - t[0]:.1f} s")
    for r in analyse(p, tuple(a.offset), a.wheelbase, a.track):
        print(f"{r['direction']:5s}: rear-axle R = {r['rear_axle_radius_m']:.3f} m (marker circle {r['marker_radius_m']:.3f} m, "
              f"fit rms {1e3 * r['fit_rms_m']:.1f} mm, {r['turns_covered']:.1f} turns) -> delta_bicycle "
              f"{math.degrees(r['delta_bicycle_rad']):.1f} deg, inner {math.degrees(r['inner_wheel_rad']):.1f} deg, "
              f"outer {math.degrees(r['outer_wheel_rad']):.1f} deg; closest reading: {r['closest_reading']} "
              f"(predicted R: " + ", ".join(f"{k} {v:.2f}" for k, v in r["predicted_radius_by_reading_m"].items()) + ")")
    return 0


if __name__ == "__main__":
    sys.exit(main())
