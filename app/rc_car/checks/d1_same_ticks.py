"""D1 on the SAME sampled ticks for several controller variants (STOP R review, item 1).
OFFLINE, KINEMATIC MODEL, EMULATED SOLVER.

1. Re-runs the frozen phase 4 controller offline (10x40, half, figure-eight and circle at 1.0 m/s,
   `realistic_guess`, seeds 1-5) and keeps every 40th tick's problem: predicted state, reference,
   reference controls, u_prev, warm start (as in the ablation's D1).
2. Solves each of those identical problems with each variant's PSO (10x40, half; 3 PSO seeds per
   tick, the same seeds for every variant) and with SLSQP on that variant's cost
   (redesign_eval.j_opt: same decision space and rate windows, multi-start).
3. Reports J_PSO / J_opt per variant: how far the swarm stays from the optimum of its own cost.

The variants differ in their cost (R3, R4) and search (R1, R2), so each ratio is against the
optimum of the variant's OWN cost; the absolute costs are not comparable between variants.

    nice -n 19 python3 app/rc_car/checks/d1_same_ticks.py [--out FILE]
"""

from __future__ import annotations

import argparse
import dataclasses
import json
from pathlib import Path
import sys

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parent))
import nmpc_controller as nc  # noqa: E402
import nmpc_engine as ne  # noqa: E402
import paths as P  # noqa: E402
import redesign_eval as rev  # noqa: E402
from nmpc_engine import am  # noqa: E402

import dataclasses as _dc

_SMOOTH = nc.load_redesign(section="redesign_smooth")
SETS = {  # name -> {variant label: (Redesign, (particles, iterations))}
    "redesign": {
        "frozen": (nc.Redesign(), (10, 40)),
        "chosen R1+R3+R4": (nc.load_redesign(section="redesign"), (10, 40)),
        "smooth R1+R2+R3+R4 (not selected)": (_SMOOTH, (10, 40)),
    },
    "gap": {  # plan section 7.13 (b): does a larger budget, or the re-tuned max_v, close the gap?
        "frozen 10x40": (nc.Redesign(), (10, 40)),
        "smooth 10x40, max_v 30": (_SMOOTH, (10, 40)),
        "smooth 10x40, max_v 60": (_dc.replace(_SMOOTH, max_v=60.0), (10, 40)),
        "smooth 10x60, max_v 30": (_SMOOTH, (10, 60)),
        "smooth 10x60, max_v 60": (_dc.replace(_SMOOTH, max_v=60.0), (10, 60)),
    },
}
VARIANTS = {k: v[0] for k, v in SETS["redesign"].items()}


def variant_problem(pb: ne.Problem, rd: nc.Redesign, w: am.CostWeights, particles: int):
    q = dataclasses.replace(pb)
    weights = dataclasses.replace(w, s_dkappa=float(rd.r4)) if rd.r4 is not None else w
    if rd.r2:
        q.knots = ne.make_knots(int(rd.r2), q.nh)
    if rd.r3 is not None:
        ratio = w.qf_pos / w.q_pos
        q.path_frame = ne.PathFrame(rd.r3[0], rd.r3[1], ratio * rd.r3[0], ratio * rd.r3[1])
        q.ref_cs = np.column_stack([np.cos(q.xref[:, 2]), np.sin(q.xref[:, 2])])
    if rd.r1:
        ff = am.project_controls(am.feedforward_from_reference(q.xref[: q.nu], np.zeros(3), q.ts), q.u_prev,
                                 q.limits, q.ts)
        q.extra_starts = [ff]
        q.n_perturb = max(1, particles // 5) if particles >= 4 else 0
    return q, weights


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--out", default=str(Path.home() / "nmpc_sim/logs/rc_car_redesign_20261122/d1_same_ticks.json"))
    ap.add_argument("--pso-seeds", type=int, default=3)
    ap.add_argument("--set", choices=sorted(SETS), default="redesign")
    a = ap.parse_args(argv)
    variants = SETS[a.set]
    pso = ne.PsoConfig(10, 40, **{k: float(v) for k, v in rev.CFG.pso.items()})
    ticks = []
    for name in ("figure_eight", "circle"):
        path = P.build(name, kappa_max=rev.CFG.limits.kappa_max)
        for seed in (1, 2, 3, 4, 5):
            ep = rev.run("nmpc", path, 1.0, seed, nc.Redesign(), "10x40", "half", d1_every=40)
            ticks += [(name, seed, i, pb, w) for i, (pb, _plan, w) in enumerate(ep.d1)]
    print(f"{len(ticks)} ticks from the frozen controller's offline runs", flush=True)
    res = {v: [] for v in variants}
    for k, (name, seed, i, pb, w) in enumerate(ticks):
        for v, (rd, (np_, ni)) in variants.items():
            kw = {kk: float(vv) for kk, vv in rev.CFG.pso.items()}
            if rd.max_v is not None:
                kw["max_v"] = float(rd.max_v)
            pso = ne.PsoConfig(np_, ni, **kw)
            q, wv = variant_problem(pb, rd, w, pso.particles)
            ratios, best_plan, best_j = [], None, np.inf
            for s in range(a.pso_seeds):
                sol = ne.solve(q, pso, wv, np.random.default_rng([k, s, 4242]), "half")
                j = ne.double_cost(q, sol.plan, wv)
                if j < best_j:
                    best_j, best_plan = j, sol.plan
                ratios.append(j)
            opt = rev.j_opt(q, wv, best_plan)
            res[v].append({"path": name, "seed": seed, "tick": i, "j_pso": ratios, "j_opt": opt["j_opt"],
                           "ratio_median": float(np.median(ratios) / opt["j_opt"])})
        if k % 20 == 0:
            print(f"  tick {k + 1}/{len(ticks)}", flush=True)
    summary = {}
    for v, rows in res.items():
        r = np.array([x["ratio_median"] for x in rows])
        summary[v] = {"n_ticks": len(r), "median": float(np.median(r)), "p10": float(np.percentile(r, 10)),
                      "p90": float(np.percentile(r, 90)), "share_within_1.5x": float(np.mean(r <= 1.5))}
        print(f"{v:36s} J_PSO/J_opt median {summary[v]['median']:.2f} (p10-p90 {summary[v]['p10']:.2f}-{summary[v]['p90']:.2f}), "
              f"within 1.5x: {100 * summary[v]['share_within_1.5x']:.0f} % of ticks")
    Path(a.out).write_text(json.dumps({"note": "OFFLINE, kinematic model, emulated solver; same ticks for every variant",
                                       "summary": summary, "ticks": res}, default=float, indent=1))
    print("->", a.out)
    return 0


if __name__ == "__main__":
    sys.exit(main())
