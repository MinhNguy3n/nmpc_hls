"""Ablation table of the PSO redesign (OFFLINE, kinematic model, emulated solver).

    python3 app/rc_car/checks/redesign_table.py [ABLATION_DIR] > table.md
"""
from __future__ import annotations
import json, sys
from pathlib import Path
import numpy as np

ORDER = [("pp", "pure pursuit (reference, Python)"), ("frozen", "frozen phase 4 NMPC"), ("R1", "+R1"),
         ("R1R2", "+R1+R2"), ("R1R2R3", "+R1+R2+R3 (q_lat 3160, q_lon 316)"), ("R1R2R3R4", "+R1+R2+R3+R4 (s_dkappa 10)"),
         ("r1,r3=3160:316", "+R1+R3 (no R2)"), ("r1,r3=3160:316,r4=10", "+R1+R3+R4 (no R2)"),
         ("r1,r3=1000:31.6,r4=10", "TUNED: R1+R3 (q_lat 1000, q_lon 31.6)+R4 (s_dkappa 10)")]


def mr(v, d=2):
    v = np.asarray(v, float)
    return f"{np.median(v):.{d}f} ({v.min():.{d}f}–{v.max():.{d}f})"


def main(argv):
    root = Path(argv[0] if argv else Path.home() / "nmpc_sim/logs/rc_car_redesign_20261122/ablation").expanduser()
    print("| Variant | runs | cross-track RMSE [mm] | figure-eight / circle median [mm] | heading RMSE [mrad] | "
          "J_PSO/J_opt (ticks) | RMS Δκ per tick [1/m] | on the rate limit [%] | completed |")
    print("|---|---|---|---|---|---|---|---|---|")
    for key, label in ORDER:
        d = root / key
        recs = [json.loads(f.read_text()) for f in sorted(d.glob("*.json"))]
        if not recs:
            continue
        m = lambda k: [r["metrics"][k] for r in recs]  # noqa: E731
        f8 = np.median([r["metrics"]["xte_rmse_mm"] for r in recs if r["path"] == "figure_eight"])
        ci = np.median([r["metrics"]["xte_rmse_mm"] for r in recs if r["path"] == "circle"])
        ratios = [t["ratio"] for r in recs for t in r["d1"]]
        rs = f"{np.median(ratios):.2f} (p10–p90 {np.percentile(ratios, 10):.2f}–{np.percentile(ratios, 90):.2f}; n={len(ratios)})" if ratios else "—"
        print(f"| {label} | {len(recs)} | {mr(m('xte_rmse_mm'))} | {f8:.2f} / {ci:.2f} | {mr(m('head_rmse_mrad'), 1)} | {rs} | "
              f"{mr(m('dkappa_rms'), 3)} | {mr(m('rate_lim_pct'), 1)} | {sum(m('done'))}/{len(recs)} |")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
