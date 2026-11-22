"""RTF-invariance check before the phase 4b batch (STOP 4a review section 3; SIMULATION ONLY).

The same episode (figure-eight, 1.0 m/s, seed 1) at two real-time factors per arm:
baseline at RTF 1.0 and 0.4, NMPC 10x40 at RTF 0.5 and 0.4. PASS if, for each arm, the
cross-track RMSE pair agrees within 0.2 mm and the event counts are equal (safety events; for the
NMPC also deadline misses, stale poses at the tick, fallbacks, overruns and busy ticks).
If not, every arm (baseline included) runs at one common factor, 0.35.

    python3 app/rc_car/checks/rtf_check.py BLOCK_LOG_DIR      # writes BLOCK_LOG_DIR/rtf_decision.json
"""

from __future__ import annotations

import json
from pathlib import Path
import sys

PAIRS = {"baseline": ("rtfcheck_baseline_rtf1.0", "rtfcheck_baseline_rtf0.4"),
         "nmpc10x40": ("rtfcheck_nmpc10x40_rtf0.5", "rtfcheck_nmpc10x40_rtf0.4")}
TOL_M = 0.2e-3
NMPC_EVENTS = ("deadline_misses", "stale_pose", "fallbacks", "overruns", "python_busy")


def episode(d: Path) -> dict:
    files = sorted(d.glob("*figure_eight*_v1_seed1*.json"))
    if len(files) != 1:
        raise FileNotFoundError(f"{d}: expected one figure-eight episode, found {len(files)}")
    return json.loads(files[0].read_text())


def events(r: dict) -> dict:
    e = {"safety_event_total": r["safety_event_total"], "end_reason": r["end_reason"]}
    if "nmpc" in r:
        e.update({k: r["nmpc"].get(k) for k in NMPC_EVENTS})
    return e


def main(argv: list[str]) -> int:
    log = Path(argv[0])
    result = {"note": "SIMULATION ONLY", "tolerance_mm": TOL_M * 1e3, "arms": {}}
    ok = True
    for arm, (a, b) in PAIRS.items():
        ra, rb = episode(log / a), episode(log / b)
        da = abs(ra["tracking"]["cross_track_rmse_m"] - rb["tracking"]["cross_track_rmse_m"])
        same = events(ra) == events(rb)
        passed = da <= TOL_M and same
        ok &= passed
        result["arms"][arm] = {"rtf": [ra.get("rtf"), rb.get("rtf")],
                               "rmse_mm": [1e3 * ra["tracking"]["cross_track_rmse_m"], 1e3 * rb["tracking"]["cross_track_rmse_m"]],
                               "rmse_diff_mm": 1e3 * da, "events": [events(ra), events(rb)], "pass": passed}
        print(f"{arm}: RTF {ra.get('rtf'):.3f} vs {rb.get('rtf'):.3f}: RMSE {1e3 * ra['tracking']['cross_track_rmse_m']:.2f} vs "
              f"{1e3 * rb['tracking']['cross_track_rmse_m']:.2f} mm (diff {1e3 * da:.2f}), events equal: {same} -> "
              f"{'PASS' if passed else 'FAIL'}")
    result["pass"] = ok
    result["decision"] = "per-arm factors (baseline 1.0; NMPC 3x24 0.35, 10x40 0.5, 10x60 0.4)" if ok else \
        "common factor 0.35 for every arm, baseline included"
    (log / "rtf_decision.json").write_text(json.dumps(result, indent=2))
    print("RTF check:", "PASS" if ok else "FAIL", "->", result["decision"])
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
