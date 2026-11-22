"""Select the frozen baseline lookahead from the tuning grid (SIMULATION ONLY).

Rule (stated in docs/rc_car/simulation_plan.md section 7.3, written down after the first grid
put its minimum on the grid edge and the extension showed an instability cliff):
  1. a grid point is ACCEPTABLE if every tuning seed completed the path without a timeout or a
     safety event and its cross-track RMSE is at most 3x the best mean RMSE of the grid (points
     beyond 3x are "rejected": either oscillating, or merely much worse, e.g. long lookaheads);
  2. a point is ELIGIBLE if it and all of its existing neighbours along both grid axes are
     acceptable (a margin of one grid step from any oscillation);
  3. the frozen value is the eligible point with the lowest mean cross-track RMSE over the seeds.

    python3 app/rc_car/checks/tuning_select.py DIR [DIR ...]   (track_run.py summary.json files)
"""

from __future__ import annotations

import json
from pathlib import Path
import sys

import numpy as np


def select(rows: list[dict], factor: float = 3.0, keys: tuple[str, str] | None = None) -> dict:
    """keys: the two grid axes; default: the baseline's (lookahead_time, min_lookahead_dist), or the
    NMPC's (q_pos, q_psi) when the rows carry those."""
    if keys is None:
        keys = ("q_pos", "q_psi") if "q_pos" in rows[0]["grid"] else ("lookahead_time", "min_lookahead_dist")
    points: dict[tuple[float, float], list[dict]] = {}
    for r in rows:
        key = (float(r["grid"][keys[0]]), float(r["grid"][keys[1]]))
        points.setdefault(key, []).append(r)
    mean = {k: float(np.mean([r["cross_track_rmse_m"] for r in v])) for k, v in points.items()}
    best = min(mean.values())
    stable = {k: all(r["end_reason"] == "arm_reported_end" and r["safety_event_total"] == 0 and r["reached_end"]
                     and r["cross_track_rmse_m"] <= factor * best for r in v) for k, v in points.items()}
    ts, ms = sorted({k[0] for k in points}), sorted({k[1] for k in points})

    def neighbours(k):
        i, j = ts.index(k[0]), ms.index(k[1])
        cand = [(ts[i + d], k[1]) for d in (-1, 1) if 0 <= i + d < len(ts)]
        cand += [(k[0], ms[j + d]) for d in (-1, 1) if 0 <= j + d < len(ms)]
        return [c for c in cand if c in points]

    eligible = {k: stable[k] and all(stable[n] for n in neighbours(k)) for k in points}
    chosen = min((k for k in points if eligible[k]), key=lambda k: mean[k])
    table = [{keys[0]: k[0], keys[1]: k[1], "mean_rmse_mm": 1e3 * mean[k],
              "seeds": len(points[k]), "stable": stable[k], "eligible": eligible[k]} for k in sorted(points)]
    best_key = min(mean, key=mean.get)
    on_edge = best_key[0] in (ts[0], ts[-1]) or best_key[1] in (ms[0], ms[-1])
    return {"keys": list(keys), "chosen": {keys[0]: chosen[0], keys[1]: chosen[1], "mean_rmse_mm": 1e3 * mean[chosen]},
            "best_unconstrained": dict(zip(keys, best_key)), "best_on_grid_edge": bool(on_edge),
            "best_mean_rmse_mm": 1e3 * best, "table": table}


def main(argv: list[str]) -> int:
    rows = [e for d in argv for e in json.loads((Path(d) / "summary.json").read_text())["episodes"]]
    res = select(rows)
    k0, k1 = res["keys"]
    for r in res["table"]:
        print(f"{k0}={r[k0]:<6} {k1}={r[k1]:<6} mean RMSE {r['mean_rmse_mm']:7.1f} mm "
              f"(n={r['seeds']}) {'acceptable' if r['stable'] else 'rejected'}{', eligible' if r['eligible'] else ''}")
    print("chosen:", res["chosen"], "| unconstrained best:", res["best_unconstrained"],
          "| best on the grid edge (extension allowed once):", res["best_on_grid_edge"])
    print(json.dumps(res))
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
