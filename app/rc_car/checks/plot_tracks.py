"""Plots of track_run.py episodes: path vs reference, and the cross-track error over arc length.
SIMULATION ONLY. Needs matplotlib (run in the container: the host's matplotlib does not import).

    python3 app/rc_car/checks/plot_tracks.py DIR [DIR ...] --out FIGDIR
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))
import paths as P  # noqa: E402
import tracking_metrics as tm  # noqa: E402


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("dirs", nargs="+")
    parser.add_argument("--out", required=True)
    parser.add_argument("--pattern", default="baseline_*.json")
    args = parser.parse_args()
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    area = P.load_area()
    lu, wu = area.usable
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    files = sorted(f for d in args.dirs for f in Path(d).glob(args.pattern))
    for f in files:
        r = json.loads(f.read_text())
        path = P.build(r["path"], area)
        tr = np.array(r["series"]["truth"])
        if len(tr) < 5:
            continue
        ser = tm.series(path, tr[:, 0], tr[:, 1], tr[:, 2], tr[:, 3])
        fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(11, 4.2), gridspec_kw={"width_ratios": [1.5, 1]})
        ax1.add_patch(plt.Rectangle((-area.length / 2, -area.width / 2), area.length, area.width, fill=False,
                                    ls=":", color="0.6", label="tracked area (PLACEHOLDER_)"))
        ax1.add_patch(plt.Rectangle((-lu / 2, -wu / 2), lu, wu, fill=False, ls="--", color="0.75", label="geofence"))
        ax1.plot(path.x, path.y, color="0.2", lw=1.0, label="reference")
        ax1.plot(tr[:, 1], tr[:, 2], color="tab:orange", lw=1.4, label="car (Gazebo truth, rear axle)")
        ax1.plot(*path.start[:2], "ko", ms=4)
        ax1.set_aspect("equal")
        ax1.set_xlabel("x (m)")
        ax1.set_ylabel("y (m)")
        ax1.legend(fontsize=7, loc="upper right")
        t = r.get("tracking", {})
        ax1.set_title(f"{r['arm']} | {r['path']} | {r['speed']:g} m/s | {r['profile']} | seed {r['seed']}", fontsize=9)
        ax2.plot(ser["s"], 1e3 * ser["e_y"], color="tab:orange", lw=1.0)
        ax2.axvline(path.length - 0.5, color="0.6", ls=":", lw=0.8)
        ax2.axhline(0, color="0.5", lw=0.6)
        ax2.set_xlabel("arc length s (m)")
        ax2.set_ylabel("cross-track error (mm, left +)")
        ax2.set_title(f"RMSE {1e3 * t.get('cross_track_rmse_m', np.nan):.1f} mm, max {1e3 * t.get('cross_track_max_m', np.nan):.1f} mm; "
                      f"safety events {r['safety_event_total']}", fontsize=9)
        fig.suptitle("SIMULATION ONLY (Gazebo; assumed chain values, PLACEHOLDER_ vehicle values)", fontsize=8, color="0.4")
        fig.tight_layout()
        fig.savefig(out / f"{f.stem}.png", dpi=110)
        plt.close(fig)
        print(out / f"{f.stem}.png")
    return 0


if __name__ == "__main__":
    sys.exit(main())
