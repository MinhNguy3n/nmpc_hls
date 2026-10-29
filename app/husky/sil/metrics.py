"""Offline scoring of a SIL run directory. Each result family is reported SEPARATELY:

1. tracking against Gazebo ground truth (scoring only), per owned episode;
2. constraint violations: of the solutions (rejected ones included) and of the published commands;
3. the deadline C, in simulation time (the SIL deadline) and wall-clock time (Python timing only);
4. fallback events by kind;
5. ownership transitions.
A diagnostic comparison (feedback vs ground truth, wheel odometry vs ground truth) is added, but
never mixed into the tracking score.

Run (inside the container, from the read-only tree; writes only the report path):
    cd <app/husky> && python3 -m sil.metrics ~/logs/sil/<run> [--out ~/logs/sil/<run>/report.json]
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import sys
from typing import Any

import numpy as np

from . import _HUSKY  # noqa: F401
from .logs import read_rows
from .reference import PathSpec, compose, cross_track
import unicycle_model as um


def _interp_pose(rows: list[dict[str, Any]], t: float) -> np.ndarray | None:
    times = np.array([r["stamp"] for r in rows])
    if len(times) < 2 or t < times[0] or t > times[-1]:
        return None
    yaw = np.unwrap([r["yaw"] for r in rows])
    return np.array([np.interp(t, times, [r["x"] for r in rows]), np.interp(t, times, [r["y"] for r in rows]),
                     np.interp(t, times, yaw)])


def tracking(spec: PathSpec, episodes: list[dict[str, Any]], truth: list[dict[str, Any]],
             settle: float = 2.0) -> list[dict[str, Any]]:
    """Per episode: position error vs the time-indexed reference, heading error and cross-track
    error, all from ground truth; the reference is attached to the TRUE pose at tau0."""
    truth = sorted(truth, key=lambda r: r["stamp"])
    results = []
    for ep in episodes:
        tau0 = ep["tau0"]
        start = _interp_pose(truth, tau0)
        if start is None:
            results.append({"episode": ep["episode"], "error": "no ground truth at tau0"})
            continue
        sel = [r for r in truth if tau0 <= r["stamp"] <= tau0 + spec.duration + settle]
        if len(sel) < 2:
            results.append({"episode": ep["episode"], "error": "no ground truth in the episode"})
            continue
        t = np.array([r["stamp"] for r in sel])
        actual = np.array([[r["x"], r["y"], r["yaw"]] for r in sel])
        ref = compose(start, spec.poses(t - tau0))
        pos = np.hypot(actual[:, 0] - ref[:, 0], actual[:, 1] - ref[:, 1])
        head = np.abs(um.angle_error(actual[:, 2], ref[:, 2]))
        poly = compose(start, np.column_stack([spec.polyline(), np.zeros(len(spec.polyline()))]))[:, :2]
        xt = cross_track(actual[:, :2], poly)
        results.append({"episode": ep["episode"], "samples": len(sel), "duration": float(t[-1] - t[0]),
                        "pos_rmse": float(np.sqrt(np.mean(pos ** 2))), "pos_max": float(pos.max()),
                        "pos_final": float(pos[-1]), "heading_rmse": float(np.sqrt(np.mean(head ** 2))),
                        "heading_max": float(head.max()), "cross_track_rmse": float(np.sqrt(np.mean(xt ** 2))),
                        "cross_track_max": float(xt.max())})
    return results


def violations(ticks: list[dict[str, Any]], limits: um.UnicycleLimits, ts: float, tol: float = 1e-9) -> dict[str, Any]:
    keys = ("speed", "omega", "linear_slew", "angular_slew", "side_speed", "side_accel")
    solved = [r for r in ticks if r.get("solved")]
    of_solutions = {k: {"max": max([r[f"viol_{k}"] or 0.0 for r in solved], default=0.0),
                        "ticks": sum(1 for r in solved if (r[f"viol_{k}"] or 0.0) > tol)} for k in keys}
    published = {k: 0.0 for k in ("speed", "omega", "linear_slew", "angular_slew")}
    count = 0
    previous = None  # consecutive published commands; the first of a run is checked against cmd_vel_out
    for r in ticks:
        if r.get("publish_v") is None or r.get("publish_kind") == "probe":
            previous = None if r.get("publish_kind") != "probe" else (r["publish_v"], r["publish_w"])
            continue
        command = (r["publish_v"], r["publish_w"])
        base = previous if previous is not None else (r.get("u_prev_v"), r.get("u_prev_w"))
        previous = command
        if base[0] is None:
            continue
        rep = um.check_controls([command], base, limits, ts)
        for k in published:
            published[k] = max(published[k], rep[k])
        count += int(not rep["ok"])
    return {"solutions": of_solutions, "published": published, "published_ticks_violating": count,
            "note": "each published command is checked against the previous published one (the first "
                    "after a gap or probe against the probe / cmd_vel_out)"}


def deadline(ticks: list[dict[str, Any]], budget: float) -> dict[str, Any]:
    solved = [r for r in ticks if r.get("solved")]
    if not solved:
        return {"solved": 0}
    sim = np.array([r["c_sim_total"] for r in solved])
    wall = np.array([r["c_wall_total"] for r in solved])
    solve_wall = np.array([r["c_wall_solve"] for r in solved])
    return {"solved": len(solved), "budget": budget, "misses_sim": int(np.sum(sim > budget)),
            "c_sim": {"median": float(np.median(sim)), "p95": float(np.percentile(sim, 95)), "max": float(sim.max())},
            "c_wall": {"median": float(np.median(wall)), "p95": float(np.percentile(wall, 95)), "max": float(wall.max())},
            "solve_wall": {"median": float(np.median(solve_wall)), "max": float(solve_wall.max())},
            "note": "simulation-time C is the SIL deadline; wall-clock C is Python timing, not a hardware claim"}


def fallbacks(ticks: list[dict[str, Any]]) -> dict[str, int]:
    out: dict[str, int] = {}
    for r in ticks:
        if r.get("fallback"):
            key = str(r["fallback"]).split(":")[0] + ":" + ("stop" if "stop" in str(r["fallback"]) else "shift")
            out[key] = out.get(key, 0) + 1
    out["published_kinds"] = {k: sum(1 for r in ticks if r.get("publish_kind") == k)  # type: ignore[assignment]
                              for k in ("plan", "fallback", "stop", "probe")}
    return out


def signal_vs_truth(signal: list[dict[str, Any]], truth: list[dict[str, Any]]) -> dict[str, float]:
    """Diagnostic: pose difference of a signal against ground truth at the signal's stamps, after
    removing the initial offset (odom frames start where the robot starts)."""
    truth = sorted(truth, key=lambda r: r["stamp"])
    pairs = [(r, _interp_pose(truth, r["stamp"])) for r in signal]
    pairs = [(r, p) for r, p in pairs if p is not None]
    if len(pairs) < 2:
        return {}
    r0, p0 = pairs[0]
    rel_sig = um.to_local_frame([r0["x"], r0["y"], r0["yaw"]], [[r["x"], r["y"], r["yaw"]] for r, _ in pairs])
    rel_gt = um.to_local_frame(p0, [p for _, p in pairs])
    dpos = np.hypot(*(rel_sig[:, :2] - rel_gt[:, :2]).T)
    dyaw = np.abs(um.angle_error(rel_sig[:, 2], rel_gt[:, 2]))
    ages = [r["received"] - r["stamp"] for r, _ in pairs if r.get("received") is not None]
    return {"samples": len(pairs), "drift_pos_final": float(dpos[-1]), "drift_pos_max": float(dpos.max()),
            "drift_yaw_final": float(dyaw[-1]), "age_median": float(np.median(ages)) if ages else math.nan}


def report(run: Path) -> dict[str, Any]:
    meta = json.loads((run / "meta.json").read_text())
    spec = PathSpec(meta["path"]["kind"], meta["path"]["speed"], meta["path"]["size"])
    limits = um.UnicycleLimits(**meta["limits"])
    ticks = read_rows(run / "ticks.csv")
    truth = read_rows(run / "truth.csv")
    return {
        "run": meta.get("run_id"), "config": {k: meta.get(k) for k in ("pso", "gain_mode", "seed", "control_horizon")},
        "tracking_vs_ground_truth": tracking(spec, read_rows(run / "episodes.csv"), truth),
        "constraint_violations": violations(ticks, limits, meta["ts"]),
        "deadline": deadline(ticks, meta["deadline"]),
        "fallbacks": fallbacks(ticks),
        "ownership_transitions": read_rows(run / "transitions.csv"),
        "diagnostic_feedback_vs_truth": signal_vs_truth(read_rows(run / "feedback.csv"), truth),
        "diagnostic_wheel_odom_vs_truth": signal_vs_truth(read_rows(run / "wheel_odom.csv"), truth),
    }


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Score a SIL run directory")
    parser.add_argument("run")
    parser.add_argument("--out", default=None, help="write the JSON report here (default: print only)")
    args = parser.parse_args(argv)
    result = report(Path(args.run).expanduser())
    text = json.dumps(result, indent=2, default=str)
    print(text)
    if args.out:
        Path(args.out).expanduser().write_text(text)
    return 0


if __name__ == "__main__":
    sys.exit(main())
