"""Apply the section 7.7 / 7.10 selection rule to offline redesign tuning runs (OFFLINE, kinematic
model, emulated solver).

    python3 app/rc_car/checks/redesign_select.py DIR KEY1 [KEY2]
"""

from __future__ import annotations

import json
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parent))
import tuning_select as ts  # noqa: E402


def rows(d: Path, keys: list[str]) -> list[dict]:
    out = []
    for f in sorted(d.glob("**/*.json")):
        r = json.loads(f.read_text())
        g = {k: float(r["grid"][k]) for k in keys}
        if len(keys) == 1:
            g["_"] = 0.0
        m = r["metrics"]
        out.append({"grid": g, "cross_track_rmse_m": m["xte_rmse_mm"] / 1e3,
                    "end_reason": "arm_reported_end" if m["done"] else "timeout",
                    "safety_event_total": int(r.get("fallbacks") or 0), "reached_end": bool(m["done"])})
    return out


def main(argv: list[str]) -> int:
    d, keys = Path(argv[0]).expanduser(), argv[1:]
    k2 = tuple(keys) if len(keys) == 2 else (keys[0], "_")
    res = ts.select(rows(d, keys), keys=k2)
    for r in res["table"]:
        print(f"{k2[0]}={r[k2[0]]:<8g} {k2[1]}={r[k2[1]]:<6g} mean xte {r['mean_rmse_mm']:.3f} mm "
              f"{'acceptable' if r['stable'] else 'rejected'}{', eligible' if r['eligible'] else ''}")
    print("chosen:", res["chosen"], "| unconstrained best:", res["best_unconstrained"], "| on edge:", res["best_on_grid_edge"])
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
