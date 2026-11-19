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


def _dist(a: np.ndarray) -> dict[str, float]:
    return {"median": float(np.median(a)), "p95": float(np.percentile(a, 95)), "max": float(a.max())}


def deadline(ticks: list[dict[str, Any]], budget: float, ts: float = 0.05) -> dict[str, Any]:
    """Two separate results. Simulation time: C_sim (wall time since the tick start x the measured
    RTF) against the budget: the SIL deadline at that RTF. Wall clock, NOT scaled: the Python
    implementation's own cost against the same budget and the sample period, as a property of
    this Python code only (not of the C model or the hardware)."""
    solved = [r for r in ticks if r.get("solved")]
    if not solved:
        return {"solved": 0}
    sim = np.array([r["c_sim_total"] for r in solved])
    wall = np.array([r["c_wall_total"] for r in solved])
    solve_wall = np.array([r["c_wall_solve"] for r in solved])
    return {"solved": len(solved), "budget": budget, "period": ts,
            "misses_sim": int(np.sum(sim > budget)), "c_sim": _dist(sim),
            "c_wall": _dist(wall), "c_wall_over_budget": int(np.sum(wall > budget)),
            "c_wall_over_period": int(np.sum(wall > ts)), "solve_wall": _dist(solve_wall),
            "note": "C_sim is RTF-scaled (the SIL deadline); C_wall is unscaled Python timing, not a hardware claim"}


def executor(ticks: list[dict[str, Any]], ts: float) -> dict[str, Any]:
    """Callback backlog drained per tick and late timer ticks (simulation time)."""
    events = np.array([r["events"] for r in ticks if r.get("events") is not None])
    t = np.array([r["t"] for r in ticks])
    gaps = np.diff(t) if len(t) > 1 else np.array([])
    out: dict[str, Any] = {"ticks": len(ticks), "late_ticks": int(np.sum(gaps > 1.5 * ts)),
                           "tick_interval_max": float(gaps.max()) if len(gaps) else None}
    if len(events):
        out["events_per_tick"] = _dist(events)
    return out


# PLACEHOLDER_ tracking thresholds of the plan (section 4.3), to be agreed at the SIL review
CROSS_TRACK_RMSE_MAX, CROSS_TRACK_MAX = 0.10, 0.25
END_REASONS = ("run finished", "shutdown")


def acceptance(rep: dict[str, Any]) -> dict[str, Any]:
    """Candidate acceptance for one run: every reason listed; an empty list means accepted.
    Hard rejections: published-command constraint violations, loss of ownership, a tipped robot,
    a stop fallback, simulation-time deadline misses above 1 %, tracking above the thresholds."""
    reasons = []
    transitions = rep["ownership_transitions"]
    if not any(t.get("new") == "owned" for t in transitions):
        reasons.append("never owned")
    lost = [t for t in transitions if t.get("old") == "owned" and t.get("reason") not in END_REASONS]
    if lost:
        reasons.append(f"ownership lost: {lost[0].get('reason')}")
    if rep["validity"]["valid"] is not True:
        reasons.append("robot tipped" if rep["validity"]["valid"] is False else "validity unknown")
    if rep["constraint_violations"]["published_ticks_violating"]:
        reasons.append("published constraint violation")
    d = rep["deadline"]
    if d.get("solved", 0) == 0:
        reasons.append("no solves")
    elif d["misses_sim"] > 0.01 * d["solved"]:
        reasons.append("C_sim misses > 1 %")
    if rep["fallbacks"]["published_kinds"]["stop"]:
        reasons.append("stop fallback")
    tracks = rep["tracking_vs_ground_truth"]
    if not tracks or "error" in tracks[0]:
        reasons.append("no tracking result")
    elif tracks[0]["cross_track_rmse"] > CROSS_TRACK_RMSE_MAX or tracks[0]["cross_track_max"] > CROSS_TRACK_MAX:
        reasons.append("tracking above the provisional thresholds")
    return {"accepted": not reasons, "reasons": reasons,
            "note": "C_wall is reported, not judged: Python timing is not the implementation's latency"}


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


MAX_TILT = 0.35  # rad (20 deg): beyond this the robot has climbed something or tipped over


def validity(truth: list[dict[str, Any]]) -> dict[str, Any]:
    """Whether the robot stayed upright (ground truth). A tipped robot's wheels and odometry keep
    turning, so feedback-based numbers look normal; such a run is not scored."""
    tilts = [(r["stamp"], r["tilt"]) for r in truth if r.get("tilt") is not None]
    if not tilts:
        return {"valid": None, "reason": "no tilt in the ground truth (log written before 2026-11-19)"}
    worst = max(tilts, key=lambda st: st[1])
    first = next((st[0] for st in tilts if st[1] > MAX_TILT), None)
    return {"valid": first is None, "tilt_max": worst[1], "tilt_max_at": worst[0], "max_tilt": MAX_TILT,
            "tipped_at": first, "reason": "" if first is None else "robot tilted beyond max_tilt: run invalid"}


def report(run: Path) -> dict[str, Any]:
    meta = json.loads((run / "meta.json").read_text())
    spec = PathSpec(meta["path"]["kind"], meta["path"]["speed"], meta["path"]["size"])
    limits = um.UnicycleLimits(**meta["limits"])
    ticks = read_rows(run / "ticks.csv")
    truth = read_rows(run / "truth.csv")
    result = {
        "run": meta.get("run_id"), "config": {k: meta.get(k) for k in ("pso", "gain_mode", "seed", "control_horizon")},
        "validity": validity(truth),
        "tracking_vs_ground_truth": tracking(spec, read_rows(run / "episodes.csv"), truth),
        # the controller's own tracking: the same metric on the feedback it closes the loop on; the
        # difference to the ground-truth result is the estimator's share (diagnostic, not acceptance)
        "tracking_vs_feedback": tracking(spec, read_rows(run / "episodes.csv"), read_rows(run / "feedback.csv")),
        "constraint_violations": violations(ticks, limits, meta["ts"]),
        "deadline": deadline(ticks, meta["deadline"], meta["ts"]),
        "executor": executor(ticks, meta["ts"]),
        "fallbacks": fallbacks(ticks),
        "ownership_transitions": read_rows(run / "transitions.csv"),
        "diagnostic_feedback_vs_truth": signal_vs_truth(read_rows(run / "feedback.csv"), truth),
        "diagnostic_wheel_odom_vs_truth": signal_vs_truth(read_rows(run / "wheel_odom.csv"), truth),
        "feedback_source": {k: meta.get(k) for k in ("feedback_topic", "imu_topic", "estimator")},
    }
    result["acceptance"] = acceptance(result)
    return result


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
