"""Offline tests of mux_timing.mux_report on a synthetic capture with known delays, a lost message,
a reordered pair, a duplicate, a joy takeover with one leaked cmd_vel sample, and a lock."""

from __future__ import annotations

from pathlib import Path
import sys

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent))
import mux_timing as mt  # noqa: E402


def ns(t: float) -> int:
    return int(round(t * 1e9))


def synthetic_capture() -> list[dict]:
    rng = np.random.default_rng(0)
    rows, mux = [], []
    joy_windows = ((10.0, 12.0),)
    lock = (20.0, 21.0)
    for k in range(int(30 / 0.05)):
        t = 1.0 + 0.05 * k
        rows.append({"topic": mt.INPUT, "stamp_ns": ns(t), "received": t + 0.001, "tag": 0.001, "v": 0.0, "omega": 0.0})
        joy_recent = any(a + 0.002 <= t <= b + mt.MUX_TIMEOUT for a, b in joy_windows)  # after the first joy arrives
        locked = lock[0] <= t < lock[1]
        if abs(t - 5.0) < 1e-9 or joy_recent or locked:
            continue  # lost at 5.0 s, suppressed during the takeover and the lock
        mux.append((t, t + 0.001 + 0.002 + rng.uniform(0, 0.001), mt.INPUT))
    for k in range(int(2 / 0.1) + 1):  # joy at 10 Hz from 10 to 12 s
        t = 10.0 + 0.1 * k + 0.0003
        rows.append({"topic": mt.JOY, "stamp_ns": ns(t), "received": t + 0.001, "tag": 0.002, "v": 0.0, "omega": 0.0})
        mux.append((t, t + 0.004, mt.JOY))
    mux.append((11.0, 11.0 + 0.004, mt.INPUT))  # the V11-style leak inside the takeover
    mux.append((7.0, 7.0 + 0.004, mt.INPUT))  # duplicate delivery of an earlier stamp
    mux.sort(key=lambda m: m[1])
    i = next(j for j, m in enumerate(mux) if abs(m[0] - 8.0) < 1e-9)  # reorder 8.0 and 8.05
    mux[i], mux[i + 1] = (mux[i + 1][0], mux[i][1], mux[i + 1][2]), (mux[i][0], mux[i + 1][1], mux[i][2])
    for stamp, received, topic in mux:
        rows.append({"topic": mt.MUX, "stamp_ns": ns(stamp), "received": received, "tag": mt.TAGS[topic],
                     "v": 0.0, "omega": 0.0})
    rows.append({"topic": "platform/safety_stop", "received": lock[0], "data": True})
    rows.append({"topic": "platform/safety_stop", "received": lock[1], "data": False})
    rows.append({"topic": mt.MUX, "stamp_ns": 0, "received": 25.0, "tag": 0.0, "v": 0.0, "omega": 0.0})  # zero stamp
    return rows


def test_mux_report_finds_delays_losses_reordering_leaks_and_locks(tmp_path):
    path = tmp_path / "capture.csv"
    mt.write_capture(path, synthetic_capture())
    rep = mt.mux_report(mt.read_capture(path))
    delay = rep["echo_delay"][mt.INPUT]
    assert 0.002 <= delay["median"] <= 0.004 and delay["max"] < 0.06  # the reordered sample arrives one period late
    assert rep["lost"][mt.INPUT]["unexpected"] == 1 and rep["lost"][mt.INPUT]["times"][0] == pytest.approx(5.001)
    assert rep["lost"][mt.INPUT]["suppressed_by_higher_priority_or_lock"] > 20
    assert rep["reordered"][mt.INPUT] == 1 and rep["duplicates"][mt.INPUT] == 1
    (takeover,) = rep["takeovers"]
    assert takeover["takeover_latency"] == pytest.approx(0.003, abs=1e-6)
    assert takeover["handback_after_last_joy"] == pytest.approx(mt.MUX_TIMEOUT, abs=0.06)
    (leak,) = [l for l in rep["leaks"] if not l["locks"]]
    assert leak["received"] == pytest.approx(11.004) and leak["last_joy_age"] < 0.1
    assert leak["previous_output"] == mt.JOY and leak["next_output"] == mt.JOY
    (lock_event,) = rep["locks"]["platform/safety_stop"]
    assert lock_event["mux_outputs_while_engaged"] == 0
    assert rep["unattributed_mux_outputs"] == 1
    assert rep["freshness"][mt.INPUT]["rate_hz"] == pytest.approx(20.0)


def test_report_cli(tmp_path, capsys):
    path = tmp_path / "c.csv"
    mt.write_capture(path, synthetic_capture())
    assert mt.main(["report", str(path), "--out", str(tmp_path / "r.json")]) == 0
    assert (tmp_path / "r.json").exists() and "echo_delay" in capsys.readouterr().out
    with pytest.raises(SystemExit):
        mt.main(["scenario", "active"])  # --out is required; ROS is never reached
    assert "rclpy" not in sys.modules
