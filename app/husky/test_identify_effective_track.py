"""Offline tests of the analysis part of app/husky/identify_effective_track.py (synthetic data).

The ROS part is not exercised here (it needs the Jazzy container and a running simulation).
"""

from __future__ import annotations

import math
from pathlib import Path
import sys

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent))
import identify_effective_track as ide  # noqa: E402

CONTROLLER_TRACK = 0.562 * 1.75  # the controller's W*m (Jazzy default; simulation V6)
PLACEHOLDER_TRUE_TRACK = 1.20  # m, synthetic "physics" effective track for the test only


def synthetic_rows(true_track: float, noise: float, seed: int = 0) -> list[dict]:
    """Script of segments; wheels follow the controller mapping, the body turns at diff/true_track."""
    rng = np.random.default_rng(seed)
    rows, t, yaw, x, y = [], 0.0, 0.3, 0.0, 0.0
    for index, (duration, v, w_cmd) in enumerate(ide.SEQUENCE):
        rows.append({"source": "seg", "t": t, "seg": index, "v_cmd": v, "omega_cmd": w_cmd, "t_end": t + duration})
        diff = w_cmd * CONTROLLER_TRACK
        w_true = diff / true_track
        steps = int(round(duration / 0.02))
        for _ in range(steps):
            t += 0.02
            yaw += w_true * 0.02
            x += v * 0.02 * math.cos(yaw)
            y += v * 0.02 * math.sin(yaw)
            rows.append({"source": "gt", "t": t, "t_recv": t + 0.001, "x": x, "y": y,
                         "yaw": math.atan2(math.sin(yaw), math.cos(yaw)) + rng.normal(0, noise)})
            rows.append({"source": "joints", "t": t, "t_recv": t + 0.001,
                         "v_left": v - diff / 2 + rng.normal(0, noise), "v_right": v + diff / 2 + rng.normal(0, noise)})
            rows.append({"source": "odom", "t": t, "t_recv": t + 0.002, "v": v, "omega": w_cmd})
    return rows


def test_effective_track_recovered_with_uncertainty(tmp_path):
    rows = synthetic_rows(PLACEHOLDER_TRUE_TRACK, noise=1e-3)
    path = tmp_path / "track.csv"
    ide.write_csv(path, rows)
    result = ide.analyze(ide.read_csv(path))
    assert len(result.segments) == 6  # 4 in-place turns + 2 arcs (stops are skipped)
    assert abs(result.track_eff - PLACEHOLDER_TRUE_TRACK) <= max(3 * result.track_eff_se, 1e-3)
    assert result.track_eff_se < 0.01
    assert result.multiplier_implied == pytest.approx(PLACEHOLDER_TRUE_TRACK / 0.562, rel=1e-2)
    assert result.ratio_gt_cmd_mean == pytest.approx(CONTROLLER_TRACK / PLACEHOLDER_TRUE_TRACK, rel=1e-2)
    for segment in result.segments:
        assert np.sign(segment.omega_gt) == np.sign(segment.omega_cmd)
        assert segment.omega_odom == pytest.approx(segment.omega_cmd)


def test_noise_widens_the_uncertainty_and_cli_runs(tmp_path, capsys):
    quiet = ide.analyze(synthetic_rows(CONTROLLER_TRACK, noise=1e-4))
    noisy = ide.analyze(synthetic_rows(CONTROLLER_TRACK, noise=2e-2, seed=1))
    assert quiet.track_eff == pytest.approx(CONTROLLER_TRACK, rel=1e-3)
    assert noisy.track_eff_se > quiet.track_eff_se
    path = tmp_path / "t.csv"
    ide.write_csv(path, synthetic_rows(CONTROLLER_TRACK, noise=1e-3))
    assert ide.main(["analyze", str(path)]) == 0
    assert "pooled W_eff" in capsys.readouterr().out


def test_analysis_rejects_missing_data():
    rows = [r for r in synthetic_rows(1.0, 1e-3) if r["source"] != "gt"]
    with pytest.raises(ValueError):
        ide.analyze(rows)


def test_freshness_statistics(tmp_path, capsys):
    stamps = np.arange(0.0, 2.0, 0.021)  # 47.6 Hz as observed in simulation (V7)
    received = stamps + 0.002 + 0.001 * (np.arange(len(stamps)) % 3)
    stats = ide.freshness_stats(stamps, received)
    assert stats.rate_hz == pytest.approx(1 / 0.021, rel=1e-6)
    assert stats.age_median == pytest.approx(0.003, abs=1e-9) and stats.age_max == pytest.approx(0.004)
    assert stats.non_monotonic == 0
    swapped = stamps.copy()
    swapped[[10, 11]] = swapped[[11, 10]]  # one reordering
    assert ide.freshness_stats(swapped, received).non_monotonic == 1
    rows = [{"source": "odom", "t": s, "t_recv": r} for s, r in zip(stamps, received)]
    path = tmp_path / "f.csv"
    ide.write_csv(path, rows)
    assert ide.main(["freshness-report", str(path)]) == 0
    assert "odom" in capsys.readouterr().out


def test_record_refuses_without_ground_truth_frame():
    with pytest.raises(SystemExit):
        ide.main(["record", "--sim"])
    assert "rclpy" not in sys.modules  # the analysis module does not import ROS
