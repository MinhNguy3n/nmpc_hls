"""Offline tests of the turning-circle fit on synthetic circles."""

from __future__ import annotations

import math
from pathlib import Path
import sys

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent))
import fit_turning_circle as ftc  # noqa: E402

L, W = 0.275, 0.30


def drive(R: float, left: bool, laps: float = 3, offset=(0.0, 0.0), noise=0.0, n=3000, c=(1.0, -2.0), seed=0):
    """Positions of a marker at `offset` (car frame) while the rear axle drives a circle of radius R."""
    rng = np.random.default_rng(seed)
    th = np.linspace(0, 2 * math.pi * laps, n) * (1 if left else -1)
    # rear axle on the circle; heading tangent (forward driving)
    cx, cy = c
    if left:
        rear = np.column_stack([cx + R * np.sin(th), cy - R * np.cos(th)])
        yaw = th
    else:
        rear = np.column_stack([cx - R * np.sin(th), cy + R * np.cos(th)])
        yaw = th
    ox, oy = offset
    m = rear + np.column_stack([ox * np.cos(yaw) - oy * np.sin(yaw), ox * np.sin(yaw) + oy * np.cos(yaw)])
    return m + rng.normal(0, noise, m.shape)


@pytest.mark.parametrize("left", [True, False])
def test_fit_exact_circle(left):
    p = drive(0.74, left, noise=0.0)
    xc, yc, r, rms = ftc.fit_circle(p)
    assert r == pytest.approx(0.74, abs=1e-9) and rms < 1e-9


def test_both_directions_noise_and_readings():
    p = np.vstack([drive(0.74, True, noise=0.002, seed=1), drive(0.75, False, noise=0.002, c=(4.0, 1.0), seed=2)])
    res = {r["direction"]: r for r in ftc.analyse(p)}
    assert set(res) == {"left", "right"}
    assert res["left"]["rear_axle_radius_m"] == pytest.approx(0.74, abs=0.003)
    assert res["right"]["rear_axle_radius_m"] == pytest.approx(0.75, abs=0.003)
    assert res["left"]["closest_reading"] == "inner_wheel"
    a = ftc.angles(0.74, L, W)
    assert math.degrees(a["inner_wheel_rad"]) == pytest.approx(25.0, abs=0.1)  # the lab-facts table
    assert math.degrees(a["delta_bicycle_rad"]) == pytest.approx(20.4, abs=0.1)


@pytest.mark.parametrize("left", [True, False])
@pytest.mark.parametrize("offset", [(0.10, 0.0), (0.12, 0.03), (-0.05, -0.04)])
def test_offset_correction(left, offset):
    p = drive(0.59, left, offset=offset, noise=0.0)
    (r,) = ftc.analyse(p, offset=offset)
    assert r["rear_axle_radius_m"] == pytest.approx(0.59, abs=1e-6)
    assert r["closest_reading"] == "bicycle"
    (uncorrected,) = ftc.analyse(p)
    if offset != (0.0, 0.0):
        assert abs(uncorrected["rear_axle_radius_m"] - 0.59) > 1e-3


def test_motive_csv_parsing(tmp_path):
    """A Motive-style export, Y up: ground plane x-z; the CCW sense is kept by using (x, -z)."""
    p = drive(0.44, True, noise=0.0, n=600)
    lines = ["Format Version,1.23,Take Name,turn,Capture Frame Rate,120.000000,Export Frame Rate,120.000000",
             "",
             "Type,,Rigid Body,Rigid Body,Rigid Body,Rigid Body,Rigid Body,Rigid Body,Rigid Body,Rigid Body Marker",
             "Name,,car,car,car,car,car,car,car,car:Marker1",
             "ID,,1,1,1,1,1,1,1,2",
             ",,Rotation,Rotation,Rotation,Rotation,Position,Position,Position,Position",
             "Frame,Time (Seconds),X,Y,Z,W,X,Y,Z,X"]
    for i, (x, y) in enumerate(p):
        z = -y  # Y-up export of the ground point (x, y)
        if i == 10:
            lines.append(f"{i},{i / 120:.6f},,,,,,,,")  # an untracked frame
            continue
        lines.append(f"{i},{i / 120:.6f},0,0,0,1,{x:.6f},0.05,{z:.6f},0")
    f = tmp_path / "take.csv"
    f.write_text("\n".join(lines))
    t, q, body = ftc.read_motive_csv(f)
    assert body == "car" and len(q) == len(p) - 1
    (r,) = ftc.analyse(q)
    assert r["direction"] == "left" and r["rear_axle_radius_m"] == pytest.approx(0.44, abs=1e-5)
    assert r["closest_reading"] == "outer_wheel"
