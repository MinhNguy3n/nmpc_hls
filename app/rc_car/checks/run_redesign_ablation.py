"""Offline ablation / tuning runner of the PSO redesign (plan section 7.10). OFFLINE, KINEMATIC MODEL,
EMULATED SOLVER. One process, run it with `nice -n 19`; it waits while a Vivado/Vitis/xsim/csim
job is running. Resumable: one JSON per episode; existing ones are skipped.

    nice -n 19 python3 app/rc_car/checks/run_redesign_ablation.py VARIANT [--paths ...] [--seeds ...]
        [--out DIR] [--d1-every 40] [--grid q_lat=..,.. q_lon=..,..] [--r4 10]

VARIANT: pp | frozen | R1 | R1R2 | R1R2R3 | R1R2R3R4 | a custom 'r1,r2=5,r3=3160:316,r4=10'.
"""

from __future__ import annotations

import argparse
import itertools
import json
from pathlib import Path
import subprocess
import sys
import time

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parent))
import nmpc_controller as nc  # noqa: E402
import paths as P  # noqa: E402
import redesign_eval as rev  # noqa: E402

ABLATION = {  # plan section 7.10, fixed before any run
    "frozen": nc.Redesign(),
    "R1": nc.Redesign(r1=True),
    "R1R2": nc.Redesign(r1=True, r2=5),
    "R1R2R3": nc.Redesign(r1=True, r2=5, r3=(3160.0, 316.0)),
    "R1R2R3R4": nc.Redesign(r1=True, r2=5, r3=(3160.0, 316.0), r4=10.0),
}


def parse_variant(text: str) -> nc.Redesign | None:
    if text == "pp":
        return None
    if text in ABLATION:
        return ABLATION[text]
    if text in ("redesign", "redesign_smooth"):
        return nc.load_redesign(section=text)
    kw = {}
    for part in text.split(","):
        k, _, v = part.partition("=")
        if k == "r1":
            kw["r1"] = True
        elif k == "r2":
            kw["r2"] = int(v)
        elif k == "r3":
            a, b = v.split(":")
            kw["r3"] = (float(a), float(b))
        elif k == "r4":
            kw["r4"] = float(v)
        elif k == "max_v":
            kw["max_v"] = float(v)
    return nc.Redesign(**kw)


def wait_for_heavy_jobs() -> None:
    while subprocess.run(["pgrep", "-f", "[v]ivado|[v]itis|[x]sim|[c]sim"], capture_output=True).returncode == 0:
        print(f"{time.strftime('%H:%M:%S')} a Vivado/HLS/cosim job is running: paused", flush=True)
        time.sleep(60)


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("variant")
    ap.add_argument("--paths", nargs="+", default=["figure_eight", "circle"])
    ap.add_argument("--seeds", type=int, nargs="+", default=[1, 2, 3, 4, 5])
    ap.add_argument("--speed", type=float, default=1.0)
    ap.add_argument("--size", default="10x40")
    ap.add_argument("--arith", default="half")
    ap.add_argument("--d1-every", type=int, default=40)
    ap.add_argument("--grid", nargs="*", default=[], help="tuning: q_lat=..,.. q_lon=..,.. or r4=..,..")
    ap.add_argument("--out", default=str(Path.home() / "nmpc_sim/logs/rc_car_redesign_20261122/ablation"))
    a = ap.parse_args(argv)
    base = parse_variant(a.variant)
    keys, values = [], []
    for g in a.grid:
        k, _, v = g.partition("=")
        keys.append(k)
        values.append([float(x) for x in v.split(",")])
    points = [dict(zip(keys, c)) for c in itertools.product(*values)] or [{}]
    out = Path(a.out).expanduser() / (a.variant + ("" if a.size == "10x40" else f"_{a.size}"))
    out.mkdir(parents=True, exist_ok=True)
    for pt, name, seed in itertools.product(points, a.paths, a.seeds):
        rd = base
        if pt:
            import dataclasses
            kw = {}
            if "q_lat" in pt or "q_lon" in pt:
                kw["r3"] = (pt.get("q_lat", (base.r3 or (0, 0))[0]), pt.get("q_lon", (base.r3 or (0, 0))[1]))
            if "r4" in pt:
                kw["r4"] = pt["r4"]
            if "max_v" in pt:
                kw["max_v"] = pt["max_v"]
            rd = dataclasses.replace(base, **kw)
        tag = f"{name}_v{a.speed:g}_seed{seed}" + "".join(f"_{k}{v:g}" for k, v in pt.items())
        f = out / f"{tag}.json"
        if f.exists():
            continue
        wait_for_heavy_jobs()
        path = P.build(name, kappa_max=rev.CFG.limits.kappa_max)
        t0 = time.time()
        ep = rev.run("pp" if rd is None else "nmpc", path, a.speed, seed, rd, a.size, a.arith,
                     d1_every=0 if rd is None else a.d1_every)
        m = rev.metrics(ep, path)
        d1 = [rev.j_opt(pb, w, plan) for pb, plan, w in ep.d1]
        rec = {"note": "OFFLINE, kinematic model, emulated solver", "variant": a.variant,
               "redesign": None if rd is None else rd.__dict__, "label": "pure pursuit" if rd is None else rd.label,
               "path": name, "speed": a.speed, "seed": seed, "grid": pt, "size": a.size, "arith": a.arith,
               "metrics": m, "d1": d1, "fallbacks": None if ep.stats is None else ep.stats.fallbacks,
               "deadline_misses": None if ep.stats is None else ep.stats.deadline_misses,
               "wall_s": time.time() - t0}
        f.write_text(json.dumps(rec, default=float))
        ratio = np.median([d["ratio"] for d in d1]) if d1 else float("nan")
        print(f"{a.variant} {tag}: xte {m['xte_rmse_mm']:.2f} mm, head {m['head_rmse_mrad']:.1f} mrad, "
              f"J_PSO/J_opt median {ratio:.2f}, dkappa_rms {m['dkappa_rms']:.3f}, rate_lim {m['rate_lim_pct']:.1f} %, "
              f"done {m['done']} ({time.time() - t0:.0f} s)", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
