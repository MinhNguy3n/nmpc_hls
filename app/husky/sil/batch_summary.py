"""One summary row per SIL run: every acceptance criterion and the metrics behind it.

    python3 -m sil.batch_summary <run>/report.json <batch>/batch_summary.csv

Appends the row to the CSV (header on first use) and prints a one-line summary that starts with
ACCEPT or REJECT and lists every rejection reason (run_batch.sh stops on ownership or tilt ones).
"""

from __future__ import annotations

import csv
import json
import math
from pathlib import Path
import sys
from typing import Any

FIELDS = ("run", "pso", "gain_mode", "seed", "estimator", "verdict", "reasons", "owned", "valid", "tilt_max",
          "pos_rmse", "pos_max", "cross_track_rmse", "cross_track_max", "heading_rmse",
          "fb_pos_rmse", "fb_cross_track_rmse", "fb_cross_track_max", "feedback_drift_pos_max", "feedback_drift_yaw_final",
          "published_violations", "solution_violation_ticks", "solved", "misses_sim",
          "c_sim_median", "c_sim_p95", "c_sim_max", "c_wall_median", "c_wall_p95", "c_wall_max",
          "c_wall_over_budget", "c_wall_over_period", "fallback_ticks", "stop_ticks", "probe_ticks",
          "late_ticks", "events_per_tick_max", "cmd_v_median", "cmd_kappa_median", "at_v_max", "at_omega_max",
          "applied_v_median", "truth_speed_median", "startup_armed_wall", "startup_settled_t",
          "ownership_transitions")


def _motion_and_startup(rep: dict[str, Any]) -> dict[str, Any]:
    m, su = rep.get("motion") or {}, rep.get("startup") or {}
    cmd = m.get("commanded") or {}
    return {"cmd_v_median": (cmd.get("v") or {}).get("median"), "cmd_kappa_median": (cmd.get("kappa") or {}).get("median"),
            "at_v_max": cmd.get("at_v_max"), "at_omega_max": cmd.get("at_omega_max"),
            "applied_v_median": ((m.get("applied") or {}).get("v") or {}).get("median"),
            "truth_speed_median": (m.get("truth_speed") or {}).get("median"),
            "startup_armed_wall": (su.get("armed") or {}).get("wall"),
            "startup_settled_t": (su.get("settled") or {}).get("t")}


def summary_row(rep: dict[str, Any]) -> dict[str, Any]:
    track = (rep.get("tracking_vs_ground_truth") or [{}])[0]
    own = (rep.get("tracking_vs_feedback") or [{}])[0]
    drift = rep.get("diagnostic_feedback_vs_truth") or {}
    d, cv, fb = rep["deadline"], rep["constraint_violations"], rep["fallbacks"]
    acc = rep["acceptance"]
    nan = math.nan
    return {
        "run": rep["run"], **{k: rep["config"].get(k) for k in ("pso", "gain_mode", "seed")},
        "estimator": (rep.get("feedback_source") or {}).get("feedback_topic"),
        "verdict": "ACCEPT" if acc["accepted"] else "REJECT", "reasons": "; ".join(acc["reasons"]),
        "owned": any(t.get("new") == "owned" for t in rep["ownership_transitions"]),
        "valid": rep["validity"]["valid"], "tilt_max": rep["validity"].get("tilt_max"),
        **{k: track.get(k, nan) for k in ("pos_rmse", "pos_max", "cross_track_rmse", "cross_track_max", "heading_rmse")},
        "fb_pos_rmse": own.get("pos_rmse", nan), "fb_cross_track_rmse": own.get("cross_track_rmse", nan),
        "fb_cross_track_max": own.get("cross_track_max", nan),
        "feedback_drift_pos_max": drift.get("drift_pos_max", nan), "feedback_drift_yaw_final": drift.get("drift_yaw_final", nan),
        "published_violations": cv["published_ticks_violating"],
        "solution_violation_ticks": sum(v["ticks"] for v in cv["solutions"].values()),
        "solved": d.get("solved", 0), "misses_sim": d.get("misses_sim"),
        **{f"c_sim_{k}": d.get("c_sim", {}).get(k, nan) for k in ("median", "p95", "max")},
        **{f"c_wall_{k}": d.get("c_wall", {}).get(k, nan) for k in ("median", "p95", "max")},
        "c_wall_over_budget": d.get("c_wall_over_budget"), "c_wall_over_period": d.get("c_wall_over_period"),
        "fallback_ticks": fb["published_kinds"]["fallback"], "stop_ticks": fb["published_kinds"]["stop"],
        "probe_ticks": fb["published_kinds"]["probe"],
        "late_ticks": (rep.get("executor") or {}).get("late_ticks"),
        "events_per_tick_max": ((rep.get("executor") or {}).get("events_per_tick") or {}).get("max"),
        **_motion_and_startup(rep),
        "ownership_transitions": " | ".join(f"{t.get('old')}->{t.get('new')} ({t.get('reason')})"
                                            for t in rep["ownership_transitions"]),
    }


def one_line(row: dict[str, Any]) -> str:
    def f(x: Any, fmt: str) -> str:
        return "nan" if x is None or (isinstance(x, float) and math.isnan(x)) else format(x, fmt)
    text = (f"{row['verdict']} xt_rmse={f(row['cross_track_rmse'], '.3f')} xt_max={f(row['cross_track_max'], '.3f')} "
            f"pos_rmse={f(row['pos_rmse'], '.3f')} fb_xt_rmse={f(row['fb_cross_track_rmse'], '.3f')} "
            f"est_drift={f(row['feedback_drift_pos_max'], '.2f')}m viol={row['published_violations']} "
            f"misses_sim={row['misses_sim']}/{row['solved']} "
            f"c_wall_ms p50/p95/max={f(1e3 * row['c_wall_median'], '.1f')}/{f(1e3 * row['c_wall_p95'], '.1f')}/"
            f"{f(1e3 * row['c_wall_max'], '.1f')} (>{'40'}ms:{row['c_wall_over_budget']}, >50ms:{row['c_wall_over_period']}) "
            f"fallback={row['fallback_ticks']} stop={row['stop_ticks']} tilt_max={f(row['tilt_max'], '.3f')} "
            f"kappa_cmd={f(row['cmd_kappa_median'], '.2f')} at_vmax={f(row['at_v_max'], '.2f')} "
            f"v_truth={f(row['truth_speed_median'], '.3f')}")
    return text + (f" [{row['reasons']}]" if row["reasons"] else "")


def main(argv: list[str] | None = None) -> int:
    argv = sys.argv[1:] if argv is None else argv
    row = summary_row(json.loads(Path(argv[0]).read_text()))
    out = Path(argv[1])
    new = not out.exists()
    with open(out, "a", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=FIELDS)
        if new:
            writer.writeheader()
        writer.writerow(row)
    print(one_line(row))
    return 0


if __name__ == "__main__":
    sys.exit(main())
