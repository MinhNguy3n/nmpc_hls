"""Offline closed loop of the candidate controller with the HLS solver's own settings (STOP H review, item 1).
OFFLINE, KINEMATIC MODEL, EMULATED SOLVER. Gazebo stays paused.

The episode set and plant are those of the offline ablation (redesign_eval.run: figure-eight and circle at
1.0 m/s, seeds 1-5, `realistic_guess` actuator and mocap, the Gazebo term, 10x40). Arms, paired by seed:

    candidate   the Python candidate (`redesign_smooth`, half, chord rollout, normal R1 noise), rerun now;
                also compared with the stored ablation run (R1R2R3R4) as a reproducibility check
    hls         the same controller host side (warm start, references, checks, timing model), with the
                solve replaced by the bit-level emulation of the HLS solver with PSO_RD_ALL
                (app/rc_car/hls_bicycle_emul.py; bit-identical to the C build in hls_bicycle_check.py):
                RK4 rollout, fixed-point PSO, uniform R1 noise; the random stream is free-running over
                the episode (seeded from the episode seed), as the board's generator is between solves
    py_uniform  diagnostic: the Python candidate with uniform R1 noise only (separates the noise change
                from the RK4 / fixed-point change)

PASS RULE (written before the run, 2026-11-22, from the STOP H review): the median over the 10 episodes of
the paired cross-track RMSE difference (hls - candidate) is within +-0.5 mm. Reported also: per path,
heading RMSE, RMS delta-kappa per tick, share on the steering-rate limit, completion, fallbacks.

    nice -n 19 /usr/bin/python3 -B app/rc_car/checks/hls_closed_loop.py [--out DIR] [--arms ...]

One process; waits while a Vivado/HLS/cosim job runs; resumable (one JSON per episode).
"""

from __future__ import annotations

import argparse
import dataclasses
import json
from pathlib import Path
import sys
import time

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parent))
import hls_bicycle_check as K  # noqa: E402
import hls_bicycle_emul as E  # noqa: E402
import nmpc_controller as nc  # noqa: E402
import nmpc_engine as ne  # noqa: E402
import paths as P  # noqa: E402
import redesign_eval as rev  # noqa: E402
from run_redesign_ablation import wait_for_heavy_jobs  # noqa: E402

PATHS = ("figure_eight", "circle")
SEEDS = (1, 2, 3, 4, 5)
STORED = Path.home() / "nmpc_sim/logs/rc_car_redesign_20261122/ablation/R1R2R3R4"
RULE_MM = 0.5
HLS_CFG = E.make_config(E.SWITCHES, 10, 40)

_orig_solve = nc.NmpcController.solve
_orig_start = nc.NmpcController.start_episode
_orig_prepare = nc.NmpcController.prepare


def _hls_start(self, path, speed, seed):
    _orig_start(self, path, speed, seed)
    self._hls_rand = E.RandStream(10_000 + int(seed))


def _hls_solve(self, plan):
    pb = plan.problem
    ci = E.pack(HLS_CFG, pb)
    r = E.solve(HLS_CFG, ci.x_curr, ci.u_curr, ci.xref, ci.aux, ci.last_best.ravel(), self._hls_rand)
    hist = r.bestfitness.astype(float) * E.LSB
    return ne.Solution(K.plan_of(HLS_CFG, pb, r.new_best), float(hist[-1]), hist)


def _uniform_prepare(self, t_tick, pose):
    tp = _orig_prepare(self, t_tick, pose)
    if tp is not None:
        tp.problem.perturb_dist = "uniform"
    return tp


def set_arm(arm: str) -> None:
    nc.NmpcController.solve = _hls_solve if arm == "hls" else _orig_solve
    nc.NmpcController.start_episode = _hls_start if arm == "hls" else _orig_start
    nc.NmpcController.prepare = _uniform_prepare if arm == "py_uniform" else _orig_prepare


def run_episode(arm: str, name: str, seed: int) -> dict:
    set_arm(arm)
    rd = nc.load_redesign(section="redesign_smooth")
    path = P.build(name, kappa_max=rev.CFG.limits.kappa_max)
    t0 = time.time()
    ep = rev.run("nmpc", path, 1.0, seed, rd, "10x40", "half")
    m = rev.metrics(ep, path)
    return {"note": "OFFLINE, kinematic model, emulated solver", "arm": arm, "path": name, "seed": seed,
            "metrics": m, "fallbacks": ep.stats.fallbacks, "deadline_misses": ep.stats.deadline_misses,
            "ticks": ep.stats.ticks, "wall_s": time.time() - t0}


def summarize(out: Path, arms) -> dict:
    rec = {(a, n, s): json.loads((out / f"{a}_{n}_seed{s}.json").read_text())
           for a in arms for n in PATHS for s in SEEDS if (out / f"{a}_{n}_seed{s}.json").exists()}
    res: dict = {"arms": {}, "paired": {}}
    keys = ("xte_rmse_mm", "head_rmse_mrad", "dkappa_rms", "rate_lim_pct")
    for a in arms:
        rows = [rec[(a, n, s)] for n in PATHS for s in SEEDS if (a, n, s) in rec]
        res["arms"][a] = {
            "episodes": len(rows), "done": sum(r["metrics"]["done"] for r in rows),
            "fallbacks": sum(r["fallbacks"] for r in rows), "deadline_misses": sum(r["deadline_misses"] for r in rows),
            **{n: {k: float(np.median([r["metrics"][k] for r in rows if r["path"] == n])) for k in keys}
               for n in PATHS}}
    for a in arms:
        if a == "candidate":
            continue
        d = {k: [rec[(a, n, s)]["metrics"][k] - rec[("candidate", n, s)]["metrics"][k]
                 for n in PATHS for s in SEEDS if (a, n, s) in rec and ("candidate", n, s) in rec] for k in keys}
        per_path = {n: {k: float(np.median([rec[(a, n, s)]["metrics"][k] - rec[("candidate", n, s)]["metrics"][k]
                                            for s in SEEDS if (a, n, s) in rec and ("candidate", n, s) in rec]))
                        for k in keys} for n in PATHS}
        res["paired"][a] = {"n": len(d["xte_rmse_mm"]),
                            **{k: {"median": float(np.median(v)), "min": float(np.min(v)), "max": float(np.max(v))}
                               for k, v in d.items()}, "per_path": per_path}
    if "hls" in res["paired"]:
        med = res["paired"]["hls"]["xte_rmse_mm"]["median"]
        res["rule"] = {"median_paired_xte_mm": med, "bound_mm": RULE_MM, "pass": bool(abs(med) <= RULE_MM)}
    repro = []
    for n in PATHS:
        for s in SEEDS:
            f = STORED / f"{n}_v1_seed{s}.json"
            if ("candidate", n, s) in rec and f.exists():
                stored = json.loads(f.read_text())["metrics"]["xte_rmse_mm"]
                repro.append(abs(stored - rec[("candidate", n, s)]["metrics"]["xte_rmse_mm"]))
    res["candidate_vs_stored_ablation_max_abs_mm"] = float(max(repro)) if repro else None
    return res


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--out", default=str(Path.home() / "nmpc_sim/logs/rc_car_hls_closed_loop_20261122"))
    ap.add_argument("--arms", nargs="+", default=["candidate", "py_uniform", "hls"])
    a = ap.parse_args(argv)
    out = Path(a.out)
    out.mkdir(parents=True, exist_ok=True)
    for arm in a.arms:
        for name in PATHS:
            for seed in SEEDS:
                f = out / f"{arm}_{name}_seed{seed}.json"
                if f.exists():
                    continue
                wait_for_heavy_jobs()
                r = run_episode(arm, name, seed)
                f.write_text(json.dumps(r, default=float))
                m = r["metrics"]
                print(f"{time.strftime('%H:%M:%S')} {arm} {name} seed {seed}: xte {m['xte_rmse_mm']:.3f} mm, head "
                      f"{m['head_rmse_mrad']:.2f} mrad, dkappa_rms {m['dkappa_rms']:.3f}, rate_lim {m['rate_lim_pct']:.1f} %, "
                      f"done {m['done']}, fallbacks {r['fallbacks']} ({r['wall_s']:.0f} s)", flush=True)
    s = summarize(out, a.arms)
    (out / "summary.json").write_text(json.dumps(s, indent=1))
    print(json.dumps(s, indent=1))
    return 0


if __name__ == "__main__":
    sys.exit(main())
