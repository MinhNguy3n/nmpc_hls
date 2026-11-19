"""Paired comparison of SIL predictor variants from a batch_summary.csv (run_batch.sh).

    python3 -m sil.compare <batch>/batch_summary.csv [--a plain --b sim-derived] [--out report.md]

Runs are paired by (path, PSO size, seed): same path, initial pose (run_batch.sh resets it) and
seed, differing only in the predictor gain. Per path: each variant's mean, min and max over the
seeds, and the paired difference b - a per seed, for the decision metrics (ground-truth tracking,
hard constraints, verdicts) and the attribution metrics (tracking against the feedback, feedback
drift, timing, fallbacks). A spread is reported, never a single winner from one seed.
"""

from __future__ import annotations

import argparse
import csv
from pathlib import Path
import statistics
import sys
from typing import Any

DECISION = ("cross_track_rmse", "cross_track_max", "pos_rmse", "heading_rmse", "published_violations")
ATTRIBUTION = ("fb_cross_track_rmse", "fb_cross_track_max", "fb_pos_rmse", "feedback_drift_pos_max",
               "fallback_ticks", "stop_ticks", "misses_sim", "c_sim_max", "c_wall_median", "c_wall_p95",
               "c_wall_max", "c_wall_over_budget", "c_wall_over_period")


def load(path: str | Path) -> list[dict[str, Any]]:
    rows = []
    with open(path, newline="") as handle:
        for raw in csv.DictReader(handle):
            row: dict[str, Any] = dict(raw)
            row["path"] = raw["run"].split("_")[0]
            for key in DECISION + ATTRIBUTION:
                try:
                    row[key] = float(raw[key])
                except (KeyError, ValueError):
                    row[key] = float("nan")
            rows.append(row)
    return rows


def compare(rows: list[dict[str, Any]], a: str, b: str) -> dict[str, Any]:
    out: dict[str, Any] = {}
    for path in sorted({r["path"] for r in rows}):
        sel = [r for r in rows if r["path"] == path]
        per_variant = {}
        for gain in (a, b):
            runs = [r for r in sel if r["gain_mode"] == gain]
            per_variant[gain] = {
                "n": len(runs), "verdicts": [r["verdict"] for r in runs],
                **{k: {"mean": statistics.fmean(r[k] for r in runs), "min": min(r[k] for r in runs),
                       "max": max(r[k] for r in runs)} for k in DECISION + ATTRIBUTION if runs}}
        pairs = []
        for ra in (r for r in sel if r["gain_mode"] == a):
            rb = next((r for r in sel if r["gain_mode"] == b and r["seed"] == ra["seed"] and r["pso"] == ra["pso"]),
                      None)
            if rb is not None:
                pairs.append({"seed": ra["seed"], "pso": ra["pso"],
                              **{k: rb[k] - ra[k] for k in DECISION + ATTRIBUTION}})
        out[path] = {"variants": per_variant, "pairs": pairs}
    return out


def to_markdown(result: dict[str, Any], a: str, b: str) -> str:
    lines = []
    for path, item in result.items():
        lines.append(f"### {path}\n")
        lines.append(f"| Metric | {a} mean [min, max] | {b} mean [min, max] | paired {b} - {a} per seed |")
        lines.append("| --- | --- | --- | --- |")
        for key in DECISION + ATTRIBUTION:
            cells = []
            for gain in (a, b):
                s = item["variants"][gain].get(key)
                cells.append("-" if s is None else f"{s['mean']:.4g} [{s['min']:.4g}, {s['max']:.4g}]")
            diffs = ", ".join(f"{p[key]:+.4g}" for p in item["pairs"])
            lines.append(f"| {key} | {cells[0]} | {cells[1]} | {diffs} |")
        verdicts = {g: ", ".join(item["variants"][g]["verdicts"]) for g in (a, b)}
        lines.append(f"\nVerdicts: {a}: {verdicts[a]}; {b}: {verdicts[b]}\n")
    return "\n".join(lines)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("summary")
    parser.add_argument("--a", default="plain")
    parser.add_argument("--b", default="sim-derived")
    parser.add_argument("--out", default=None)
    args = parser.parse_args(argv)
    text = to_markdown(compare(load(args.summary), args.a, args.b), args.a, args.b)
    print(text)
    if args.out:
        Path(args.out).write_text(text)
    return 0


if __name__ == "__main__":
    sys.exit(main())
