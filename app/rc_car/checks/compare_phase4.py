"""Phase 4 comparison tables from the batch logs (SIMULATION ONLY; assumed chain values,
PLACEHOLDER_ / measured_approx vehicle values).

Reads every episode JSON written by track_run.py under the given block directories and reports, per
configuration (arm x swarm size x compensation x arithmetic x profile x path x speed x contact):
- the median and range (min-max) over the seeds of each metric, each reported SEPARATELY:
  cross-track RMSE and max, heading RMSE and max, completion, safety events, deadline misses,
  stale poses at the tick, fallbacks, overruns, busy ticks, steering angle / rate on their limits,
  rear-axle lateral speed (slip), final cost (NMPC);
- paired differences against the baseline on the SAME seeds (NMPC - baseline), median and range.
No significance claims from 5 seeds (STOP 3 review, condition 7).

    python3 app/rc_car/checks/compare_phase4.py ~/nmpc_sim/logs/rc_car_phase4_batch/B2 B3 B4 --out DIR
"""

from __future__ import annotations

import argparse
from collections import defaultdict
import json
from pathlib import Path
import sys

import numpy as np

METRICS = [  # (name, getter, scale, unit)
    ("xte_rmse", lambda r: r.get("tracking", {}).get("cross_track_rmse_m"), 1e3, "mm"),
    ("xte_max", lambda r: r.get("tracking", {}).get("cross_track_max_m"), 1e3, "mm"),
    ("head_rmse", lambda r: r.get("tracking", {}).get("heading_rmse_rad"), 1e3, "mrad"),
    ("head_max", lambda r: r.get("tracking", {}).get("heading_max_rad"), 1e3, "mrad"),
    ("completion", lambda r: r.get("tracking", {}).get("completion"), 1.0, ""),
    ("safety", lambda r: r.get("safety_event_total"), 1.0, ""),
    ("misses", lambda r: r.get("nmpc", {}).get("deadline_misses"), 1.0, ""),
    ("stale", lambda r: r.get("nmpc", {}).get("stale_pose"), 1.0, ""),
    ("fallbacks", lambda r: r.get("nmpc", {}).get("fallbacks"), 1.0, ""),
    ("overruns", lambda r: r.get("nmpc", {}).get("overruns"), 1.0, ""),
    ("busy", lambda r: r.get("nmpc", {}).get("python_busy"), 1.0, ""),
    ("steer_lim", lambda r: r.get("applied", {}).get("steer_at_limit_frac"), 100.0, "%"),
    ("rate_lim", lambda r: r.get("applied", {}).get("steer_rate_at_limit_frac"), 100.0, "%"),
    ("slip_rms", lambda r: r.get("slip_rear_lateral_mps", {}).get("rms"), 1e3, "mm/s"),
    ("cost_final", lambda r: r.get("nmpc", {}).get("cost_final"), 1.0, ""),
]


def config_of(r: dict, run_dir: str) -> tuple:
    contact = "10mm" if "contact10mm" in run_dir else "70mm"
    if r["arm"] == "baseline":
        arm = "baseline"
        comp = arith = "-"
    else:
        n = r.get("nmpc", {})
        arm = f"nmpc{n.get('size')}" + ("" if n.get("variant", "phase4") == "phase4" else f"_{n.get('variant')}")
        comp = "on" if n.get("compensation") else "off"
        arith = n.get("arith", "?")
    return (arm, comp, arith, r["profile"], r["path"], float(r["speed"]), contact)


def load(dirs: list[Path]) -> list[tuple[tuple, int, dict, str]]:
    out = []
    for d in dirs:
        for f in sorted(d.glob("*/*.json")):
            if f.name == "summary.json" or f.parent.name.startswith(("rtfcheck", "smoke", "b0_", "nmpc_tuning", "diag")):
                continue
            r = json.loads(f.read_text())
            if "arm" not in r:
                continue
            out.append((config_of(r, f.parent.name), int(r["seed"]), r, str(f)))
    return out


def stat(values: list) -> dict | None:
    v = np.array([x for x in values if x is not None], dtype=float)
    if not len(v):
        return None
    return {"n": int(len(v)), "median": float(np.median(v)), "min": float(v.min()), "max": float(v.max())}


def fmt(s: dict | None, digits: int = 1) -> str:
    if s is None:
        return "-"
    if s["min"] == s["max"]:
        return f"{s['median']:.{digits}f}"
    return f"{s['median']:.{digits}f} ({s['min']:.{digits}f}-{s['max']:.{digits}f})"


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("dirs", nargs="+")
    ap.add_argument("--out", required=True)
    args = ap.parse_args(argv)
    rows = load([Path(d).expanduser() for d in args.dirs])
    by_cfg: dict[tuple, dict[int, dict]] = defaultdict(dict)
    for cfg, seed, r, _ in rows:
        by_cfg[cfg][seed] = r
    result = {"note": "SIMULATION ONLY; assumed chain values; PLACEHOLDER_/measured_approx vehicle values; "
                      "median (range) over seeds; no significance claims", "configs": []}
    lines = ["| arm | comp | arith | profile | path | v | contact | seeds | " +
             " | ".join(f"{m} [{u}]" if u else m for m, _, _, u in METRICS) + " |",
             "|" + "---|" * (8 + len(METRICS))]
    for cfg in sorted(by_cfg):
        seeds = by_cfg[cfg]
        stats = {m: stat([g(seeds[s]) * k if g(seeds[s]) is not None else None for s in sorted(seeds)])
                 for m, g, k, _ in METRICS}
        entry = {"config": dict(zip(("arm", "comp", "arith", "profile", "path", "speed", "contact"), cfg)),
                 "seeds": sorted(seeds), "stats": stats}
        if cfg[0] != "baseline":  # paired differences against the baseline on the same seeds
            base = by_cfg.get(("baseline", "-", "-") + cfg[3:])
            if base:
                common = sorted(set(seeds) & set(base))
                entry["paired_vs_baseline"] = {"seeds": common, **{
                    m: stat([(g(seeds[s]) - g(base[s])) * k for s in common
                             if g(seeds[s]) is not None and g(base[s]) is not None])
                    for m, g, k, _ in METRICS[:4] + METRICS[11:14]}}
        result["configs"].append(entry)
        digits = {"completion": 2, "safety": 0, "misses": 0, "stale": 0, "fallbacks": 0, "overruns": 0, "busy": 0}
        lines.append("| " + " | ".join(str(c) for c in cfg) + f" | {len(seeds)} | " +
                     " | ".join(fmt(stats[m], digits.get(m, 1)) for m, _, _, _ in METRICS) + " |")
    paired = ["| arm | comp | arith | profile | path | v | contact | seeds | d xte_rmse [mm] | d xte_max [mm] | "
              "d head_rmse [mrad] | d head_max [mrad] | d steer_lim [%] | d rate_lim [%] | d slip_rms [mm/s] |",
              "|" + "---|" * 15]
    for e in result["configs"]:
        p = e.get("paired_vs_baseline")
        if p:
            c = e["config"]
            paired.append("| " + " | ".join(str(c[k]) for k in ("arm", "comp", "arith", "profile", "path", "speed", "contact"))
                          + f" | {len(p['seeds'])} | " + " | ".join(fmt(p[m], 2) for m in (
                              "xte_rmse", "xte_max", "head_rmse", "head_max", "steer_lim", "rate_lim", "slip_rms")) + " |")
    out = Path(args.out).expanduser()
    out.mkdir(parents=True, exist_ok=True)
    (out / "comparison.json").write_text(json.dumps(result, indent=2))
    (out / "comparison.md").write_text("# Phase 4 comparison (SIMULATION ONLY)\n\nMedian (range) over seeds.\n\n"
                                       + "\n".join(lines) + "\n\n## Paired differences, NMPC - baseline (same seeds)\n\n"
                                       + "\n".join(paired) + "\n")
    print(f"{len(rows)} episodes, {len(by_cfg)} configurations -> {out / 'comparison.md'}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
