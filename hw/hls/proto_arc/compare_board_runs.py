#!/usr/bin/env python3
"""Compare board results of an engine overlay with a baseline overlay (ARC 2027).

  compare_board_runs.py dump  <engine 06_validate_*.json> <baseline 06_validate_*.json>
  compare_board_runs.py loop  <engine run folder> [<baseline run folder>]

dump: the step5_validate.sh solves use fixed inputs, and the hardware RNG restarts when the PL is
      programmed, so an engine build with the same S x K must give bit-identical raw outputs
      (new_best_raw, bestfitness_raw) to the baseline build. Prints the solve times of both.
loop: notebook closed-loop run folders (closed_loop_histories.npz, solver_timing.csv). With a
      baseline folder, every history array must be identical (same S x K only); always prints the
      timing distribution (solver ap_start->ap_done, and solver + PS overhead) per step.
Run with /usr/bin/python3 (needs numpy).
"""
import csv
import json
import sys
from pathlib import Path

import numpy as np


def compare_dump(engine: Path, base: Path) -> int:
    e, b = json.loads(engine.read_text()), json.loads(base.read_text())
    ok = e["inputs_raw"] == b["inputs_raw"]
    print(f"inputs identical: {ok}")
    for re_, rb in zip(e["runs"], b["runs"]):
        same = re_["new_best_raw"] == rb["new_best_raw"] and re_["bestfitness_raw"] == rb["bestfitness_raw"]
        ok &= same and re_["iterations"] == rb["iterations"]
        print(f"run {re_['run']}: iterations {re_['iterations']}/{rb['iterations']}, outputs "
              f"{'IDENTICAL' if same else 'DIFFERENT'}; solve {1e3 * re_['elapsed_seconds']:.3f} ms "
              f"(engine) vs {1e3 * rb['elapsed_seconds']:.3f} ms (baseline)")
    print("DUMP MATCH" if ok and len(e["runs"]) == len(b["runs"]) else "DUMP MISMATCH")
    return 0 if ok else 1


def timing(folder: Path) -> None:
    rows = list(csv.DictReader((folder / "solver_timing.csv").open()))
    lat = np.array([float(r["latency_seconds"]) for r in rows]) * 1e3
    tot = np.array([float(r["solve_call_seconds"]) for r in rows]) * 1e3
    tmo = sum(r["timeout"] == "True" for r in rows)
    for name, v in (("solver (ap_start->ap_done)", lat), ("solver + PS overhead", tot)):
        p = np.percentile(v, [50, 95, 99])
        print(f"  {name}: n={v.size} median {p[0]:.2f} ms, p95 {p[1]:.2f}, p99 {p[2]:.2f}, max {v.max():.2f}"
              f"; > 40 ms: {(v > 40).sum()}, > 50 ms: {(v > 50).sum()}")
    print(f"  timeouts: {tmo}")


def compare_loop(engine: Path, base: Path | None) -> int:
    print(f"engine run {engine.name}:")
    timing(engine)
    if base is None:
        return 0
    print(f"baseline run {base.name}:")
    timing(base)
    he, hb = np.load(engine / "closed_loop_histories.npz"), np.load(base / "closed_loop_histories.npz")
    ok = sorted(he.files) == sorted(hb.files)
    for k in sorted(set(he.files) & set(hb.files)):
        same = he[k].shape == hb[k].shape and np.array_equal(he[k], hb[k])
        ok &= same
        print(f"  {k}: {'IDENTICAL' if same else 'DIFFERENT'} {he[k].shape}")
    print("HISTORIES MATCH" if ok else "HISTORIES DIFFER")
    return 0 if ok else 1


if __name__ == "__main__":
    if len(sys.argv) >= 4 and sys.argv[1] == "dump":
        sys.exit(compare_dump(Path(sys.argv[2]), Path(sys.argv[3])))
    if len(sys.argv) >= 3 and sys.argv[1] == "loop":
        sys.exit(compare_loop(Path(sys.argv[2]), Path(sys.argv[3]) if len(sys.argv) > 3 else None))
    print(__doc__)
    sys.exit(2)
