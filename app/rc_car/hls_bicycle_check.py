"""C-level check of the HLS bicycle model and the PSO redesign switches (BICYCLE_CONFIG, PSO_RD_*).

    nice -n 19 /usr/bin/python3 -B app/rc_car/hls_bicycle_check.py [--quick]

C LEVEL / OFFLINE ONLY: g++ builds of hw/hls/source (no Vitis HLS, no synthesis, no cosim, no board),
one process at a time. Build outputs and the summary go to hw/hls/build/bicycle_check/ (ignored by git).

Builds (as app/husky/hls_unicycle_check.py and app/kria-pynq/validate_hw_run.py do them):
    fixed  -DHW_ARITH_CSIM -DUSE_FAST_SIN_COS: ap_fixed<32,17> top level, float model (fast sin table)
    half   the same plus -DHW_MODEL_HALF_CSIM from a header copy without `volatile`, linked to the fpo
           bit-accurate floating-point model: the hardware's own arithmetic
    _n_S / _maxiter are patched in the header copy (10 x 40, the accepted candidate's size).

References:
    app/ackermann  ackermann_model.step_exact_arc (double, exact), step_rk4 (double), and
                   hw_estimates.step_rk4_hw (the October bit-level emulation of the RK4 path);
    app/rc_car/hls_bicycle_emul.py   a bit-level emulation of these exact C code paths;
    the Python controller            nmpc_engine.cost / solve on problems logged from the candidate
                   controller's own offline closed loop (redesign_eval.run, `redesign_smooth`, 10x40,
                   half, figure-eight and circle at 1.0 m/s, seeds 1-5, every 40th tick).

PASS CRITERIA (written before the first full run, 2026-11-22):
  Model (switches off, BICYCLE_CONFIG):
    M1  half build == emulation on every output of 300 one-step cases and 100 25-step rollouts
    M2  worst position error vs the exact arc, both builds, at 1 and 25 steps <= 1.5 x the October
        emulation's (hw_estimates.step_rk4_hw) worst error on the same cases
    M4  gen_headers.sh dry run on hls_bicycle.hpp: RENAME-OK, PRAGMAS-OK; __model == __model_pipe on
        every one-step input (both builds)
  Cost:
    C1  half build == emulation for every plan, in each build: off, R1, R2, R3, R4, UREF, ALL
    C2  (ALL) against the controller's double-precision cost of the same plans, the C cost's absolute
        deviation (median and p90) <= 1.5 x the Python controller's half emulation's (chord step)
  Solver (10 x 40):
    S1  half build == emulation (new_best, the best fitness of every iteration, the number of random
        draws) on every problem solved in C, in each build
    S2  (report only) same random stream as the C run: first-control difference to the Python
        controller's PSO (native cost; and with the HLS cost plugged in)
    S3  over --seeds seeds per problem: median over problems of
        median J_double(C plan) / median J_double(Python plan) <= 1.10 (J_double: the controller's own
        double-precision cost; C plans from the emulation, after S1)
  Switches off:
    O1  g++ -E -P token streams of hls_pso.cpp, hls_nonlinear_solver.cpp and the validation drivers are
        identical to the sources before this task (HEAD solver files + config.hpp/hls_model.hpp without
        the generated BICYCLE blocks) for SNIFFBOT_CONFIG and UNICYCLE_CONFIG, float/fixed/half
    O2  outputs byte-identical: main_hls_unicycle_test (fixed, half), main_hls_model_unit_test
        (SNIFFBOT), main_hls_validate solve (SNIFFBOT and UNICYCLE, fixed and half, 3 seeds; its "J"
        line excluded: execute() returns an uninitialized value there, a pre-existing bug)
    O3  git diff of the seven solver/config files has no removed line (pure additions)
"""

from __future__ import annotations

import argparse
import dataclasses
import json
import math
import os
from pathlib import Path
import pickle
import re
import shutil
import subprocess
import sys
import tempfile
import time

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE / "checks"))
import hls_bicycle_emul as E  # noqa: E402
import nmpc_controller as nc  # noqa: E402
import nmpc_engine as ne  # noqa: E402
from nmpc_engine import am, hw  # noqa: E402

REPO = HERE.parents[1]
HLS = REPO / "hw/hls/source"
BUILD = REPO / "hw/hls/build/bicycle_check"
HW_FLAGS = ["-DHW_ARITH_CSIM", "-DUSE_FAST_SIN_COS"]
SOLVER_FILES = ["include/hls_system.hpp", "include/hls_pso.hpp", "include/hls_nonlinear_solver.hpp",
                "src/hls_pso.cpp", "src/hls_nonlinear_solver.cpp"]
DIFF_FILES = SOLVER_FILES + ["include/config.hpp", "include/hls_model.hpp"]
BUILDS = {"off": (), "R1": ("R1",), "R2": ("R2",), "R3": ("R3",), "R4": ("R4",), "UREF": ("UREF",),
          "ALL": E.SWITCHES}
GENERATOR = r"""set -e
src={src}; tmp={tmp}
sed -E -e '/^[[:space:]]*#pragma HLS/d' -e 's/\bvolatile //g' \
       -e 's/HLS_BICYCLE_HPP/HLS_BICYCLE_PIPE_HPP/g' -e 's/>void __model\(/>void __model_pipe(/' \
       $src/models/hls_bicycle.hpp > $tmp/model_pipe.hpp
grep -q '__model_pipe(' $tmp/model_pipe.hpp && echo RENAME-OK || echo "FAIL: rename"
grep -nE 'pragma HLS.*[;{{}}]|[;{{}}].*pragma HLS' $src/models/hls_bicycle.hpp && echo "FAIL: pragma shares a line" || echo PRAGMAS-OK
"""


def log(msg: str) -> None:
    print(f"{time.strftime('%H:%M:%S')} {msg}", flush=True)


def wait_for_heavy_jobs() -> None:
    while subprocess.run(["pgrep", "-f", "[v]ivado|[v]itis|[x]sim|[c]sim"], capture_output=True).returncode == 0:
        log("a Vivado/HLS/cosim job is running: paused")
        time.sleep(60)


# ------------------------------------------------------------------------------------------ builds
def fpo_libs(xilinx: Path) -> list[str]:
    fpo = xilinx / "lnx64/tools/fpo_v7_1"
    return [f"-L{fpo}", "-lIp_floating_point_v7_1_bitacc_cmodel", "-Wl,--disable-new-dtags", f"-Wl,-rpath,{fpo}"]


def header_copy(src_include: Path, dest: Path, half: bool, n_s: int | None, maxiter: int | None) -> Path:
    shutil.rmtree(dest, ignore_errors=True)
    shutil.copytree(src_include, dest)
    if half:
        for path in dest.rglob("*.hpp"):
            path.write_bytes(re.sub(rb"\bvolatile ", b"", path.read_bytes()))
    cfg = dest / "config.hpp"
    text = cfg.read_bytes()
    for name, value in (("_n_S", n_s), ("_maxiter", maxiter)):
        if value is not None:
            text, count = re.subn(rb"(#define %s[ \t]+)\d+" % name.encode(), rb"\g<1>%d" % value, text)
            assert count == 1, name
    cfg.write_bytes(text)
    return dest


def build_bicycle(tag: str, switches, mode: str, xilinx: Path, n_s=10, maxiter=40, pipe_dir: Path | None = None) -> Path:
    inc = header_copy(HLS / "include", BUILD / f"inc_{tag}_{mode}", mode == "half", n_s, maxiter)
    flags = list(HW_FLAGS) + [f"-DPSO_RD_{s}" for s in switches]
    if mode == "half":
        flags.append("-DHW_MODEL_HALF_CSIM")
    if pipe_dir is not None:
        flags += ["-DBICYCLE_PIPE_CHECK", f"-I{pipe_dir}"]
    binary = BUILD / f"bicycle_{tag}_{mode}"
    cmd = ["nice", "-n", "19", "g++", "-std=c++11", "-O0", "-w", "-D__VITIS__", "-DPSO_CONFIG", "-DBICYCLE_CONFIG",
           f"-I{inc}", f"-I{inc / 'models'}", f"-I{xilinx / 'Vitis/include'}", *flags, "-o", str(binary),
           str(HLS / "src/main_hls_bicycle_test.cpp"), str(HLS / "src/hls_pso.cpp"), str(HLS / "src/aux_functions.cpp"),
           *fpo_libs(xilinx)]  # the fixed build does not need fpo; linking it is harmless
    subprocess.run(cmd, check=True)
    return binary


def run(binary: Path, *args) -> str:
    r = subprocess.run(["nice", "-n", "19", str(binary), *map(str, args)], capture_output=True, text=True)
    if r.returncode != 0:
        raise RuntimeError(f"{binary.name} {args[0]}: rc {r.returncode}\n{r.stdout}\n{r.stderr}")
    return r.stdout


def generator_dry_run() -> tuple[Path, list[str]]:
    tmp = Path(tempfile.mkdtemp(prefix="bicycle_pipe_", dir=BUILD))
    out = subprocess.run(["bash", "-c", GENERATOR.format(src=HLS / "include", tmp=tmp)],
                         capture_output=True, text=True, check=True).stdout.split()
    return tmp, out


# ------------------------------------------------------------------------------------------ model
def model_check(xilinx: Path, n: int, seed: int) -> dict:
    c = E.make_config()
    v = c.values
    rng = np.random.default_rng(seed)
    edge = [(-100.0, 0.0), (-100.0, 100.0), (100.0, 100.0), (100.0, -100.0), (0.0, 0.0), (100.0, 0.0), (37.5, -100.0)]
    steps = [[E.fxi(rng.uniform(-0.5, 0.5)), E.fxi(rng.uniform(-0.5, 0.5)), E.fxi(rng.uniform(-2.5, 2.5)),
              E.fxi(rng.uniform(-100, 100)), E.fxi(rng.uniform(-100, 100))] for _ in range(n - len(edge))]
    steps += [[E.fxi(rng.uniform(-0.5, 0.5)), E.fxi(rng.uniform(-0.5, 0.5)), E.fxi(rng.uniform(-2.5, 2.5)),
               E.fxi(a), E.fxi(b)] for a, b in edge]
    rolls = [[E.fxi(rng.uniform(-100, 100)), E.fxi(rng.uniform(-100, 100))] for _ in range(n // 3 - len(edge))]
    rolls += [[E.fxi(a), E.fxi(b)] for a, b in edge]
    cases = BUILD / "model_cases.txt"
    cases.write_text("".join("S " + " ".join(repr(x * E.LSB) for x in s) + "\n" for s in steps)
                     + "".join("R " + " ".join(repr(x * E.LSB) for x in r) + "\n" for r in rolls))
    pipe_dir, gen_out = generator_dry_run()
    out = {"generator": gen_out, "cases": len(steps), "rollouts": len(rolls)}
    got = {}
    for mode in ("fixed", "half"):
        b = build_bicycle("model", (), mode, xilinx, pipe_dir=pipe_dir)
        f = BUILD / f"model_out_{mode}.txt"
        out[f"stdout_{mode}"] = run(b, "model", cases, f).strip().splitlines()
        S, R = [], []
        for line in f.read_text().splitlines():
            parts = line.split()
            (S if parts[0] == "S" else R).append([float(x) for x in parts[2 if parts[0] == "S" else 3:]])
        got[mode] = (np.array(S), np.array(R).reshape(len(rolls), c.nh, 3))
    st = np.array([s[:3] for s in steps], dtype=np.int64)
    us = np.array([s[3:] for s in steps], dtype=np.int64)
    em_s = E.rk4(c, st, us)
    x = np.zeros((len(rolls), 3), dtype=np.int64)
    ur = np.array(rolls, dtype=np.int64)
    em_r = []
    for _ in range(c.nh):
        x = E.rk4(c, x, ur)
        em_r.append(x)
    em_r = np.stack(em_r, axis=1)
    half_s, half_r = (E.fxi_array(a) for a in got["half"])
    out["M1_differ"] = int(np.count_nonzero(half_s != em_s) + np.count_nonzero(half_r != em_r))
    out["M1_outputs"] = int(em_s.size + em_r.size)

    def phys(z):
        return np.array([v["v_min"] + v["v_per_unit"] * (z[0] * E.LSB + 100), v["kappa_per_unit"] * z[1] * E.LSB])

    def refs(state, u, k):
        r = {"exact": [], "rk4": [], "october": []}
        s = {name: np.array(state, dtype=float) for name in r}
        for _ in range(k):
            s["exact"] = am.step_exact_arc(s["exact"], u, v["ts"], wrap=False)
            s["rk4"] = am.step_rk4(s["rk4"], u, v["ts"], wrap=False)
            s["october"] = hw.step_rk4_hw(s["october"], u, v["ts"])
            for name in r:
                r[name].append(s[name].copy())
        return r

    r1 = [refs(np.array(s[:3]) * E.LSB, phys(s[3:]), 1) for s in steps]
    r25 = [refs(np.zeros(3), phys(r), c.nh) for r in rolls]
    res = {}
    oct1 = max(math.hypot(*(r["october"][-1][:2] - r["exact"][-1][:2])) for r in r1)
    oct25 = max(math.hypot(*(r["october"][k][:2] - r["exact"][k][:2])) for r in r25 for k in range(c.nh))
    res["october_vs_exact"] = {"1": oct1, "25": oct25}
    for mode in ("fixed", "half"):
        gs, gr = got[mode]
        e1 = [math.hypot(*(gs[i][:2] - r1[i]["exact"][-1][:2])) for i in range(len(steps))]
        e25 = [math.hypot(*(gr[i][k][:2] - r25[i]["exact"][k][:2])) for i in range(len(rolls)) for k in range(c.nh)]
        rk = [math.hypot(*(gr[i][-1][:2] - r25[i]["rk4"][-1][:2])) for i in range(len(rolls))]
        psi = [abs(am.wrap_angle(gr[i][-1][2] - r25[i]["exact"][-1][2])) for i in range(len(rolls))]
        res[mode] = {"1_max": max(e1), "1_p99": float(np.percentile(e1, 99)), "25_max": max(e25),
                     "25_p99": float(np.percentile(e25, 99)), "25_vs_rk4_max": max(rk), "25_psi_max": max(psi)}
    out["errors"] = res
    out["M2"] = {mode: bool(res[mode]["1_max"] <= 1.5 * oct1 and res[mode]["25_max"] <= 1.5 * oct25)
                 for mode in ("fixed", "half")}
    out["M4"] = bool(gen_out == ["RENAME-OK", "PRAGMAS-OK"]
                     and all(any("pipe:" in l and l.endswith(" 0 differ") for l in out[f"stdout_{m}"]) for m in ("fixed", "half")))
    return out


# ------------------------------------------------------------------------------------------ problems
def logged_problems(every: int) -> list:
    """Problems logged from the candidate controller's own offline closed loop (cached)."""
    cache = BUILD / f"problems_every{every}.pkl"
    if cache.exists():
        return pickle.loads(cache.read_bytes())
    import paths as P
    import redesign_eval as rev
    rd = nc.load_redesign(section="redesign_smooth")
    probs = []
    for name in ("figure_eight", "circle"):
        path = P.build(name, kappa_max=rev.CFG.limits.kappa_max)
        for seed in (1, 2, 3, 4, 5):
            wait_for_heavy_jobs()
            ep = rev.run("nmpc", path, 1.0, seed, rd, "10x40", "half", d1_every=every)
            probs += [{"path": name, "seed": seed, "tick": i, "pb": pb, "plan": plan, "w": w}
                      for i, (pb, plan, w) in enumerate(ep.d1)]
            log(f"  logged {name} seed {seed}: {len(ep.d1)} ticks")
    cache.write_bytes(pickle.dumps(probs))
    return probs


def for_build(c: E.Config, pb: ne.Problem) -> ne.Problem:
    """The candidate's problem as a build with fewer switches sees it (R2 off: the 25-stage decision)."""
    return pb if c.on("R2") or pb.knots is None else dataclasses.replace(pb, knots=None)


def test_plans(c: E.Config, pb: ne.Problem, plan: np.ndarray, rng: np.random.Generator) -> list[np.ndarray]:
    ci = E.pack(c, pb)
    plans = [ci.last_best, E.fxi_array(ne.to_decision(plan, pb))]
    if pb.extra_starts:
        plans.append(E.fxi_array(ne.to_decision(pb.extra_starts[0], pb)))
    for _ in range(3):  # random plans around the warm start, in the box
        plans.append(np.clip(ci.last_best + E.fxi_array(rng.normal(0, 8, ci.last_best.shape)), -E.fxi(100), E.fxi(100)))
    return plans


# ------------------------------------------------------------------------------------------ cost
def cost_check(xilinx: Path, probs: list, quick: bool) -> dict:
    out = {}
    rng = np.random.default_rng(20261122)
    for tag, sw in BUILDS.items():
        c = E.make_config(sw)
        b = build_bicycle(f"cost_{tag}", sw, "half", xilinx)
        txt, expected, meta = "", [], []
        for k, p in enumerate(probs):
            pb = for_build(c, p["pb"])
            plans = test_plans(c, pb, p["plan"], rng)
            ci = E.pack(c, pb)
            txt += E.problem_text(ci, f"P {k} {len(plans)}", plans)
            J = E.cost(c, ci.x_curr, np.array(plans), ci.xref, ci.u_curr, ci.aux)
            expected += [(k, j, int(J[j])) for j in range(len(plans))]
            meta.append((pb, plans))
        f_in, f_out = BUILD / f"cost_{tag}_in.txt", BUILD / f"cost_{tag}_out.txt"
        f_in.write_text(txt)
        run(b, "cost", f_in, f_out)
        got = {}
        for line in f_out.read_text().splitlines():
            parts = line.split()
            got[(int(parts[1]), int(parts[2]))] = E.fxi(float(parts[3]))
        differ = [(k, j, got[(k, j)], e) for k, j, e in expected if got[(k, j)] != e]
        out[tag] = {"plans": len(expected), "differ": len(differ), "examples": differ[:5]}
        log(f"  cost {tag}: {len(expected)} plans, {len(differ)} differ from the emulation")
        if tag == "ALL":  # C2: against the Python controller's cost of the same plans
            rows = []
            for k, (pb, plans) in enumerate(meta):
                w = probs[k]["w"]
                for j, z in enumerate(plans):
                    zf = np.asarray(z, dtype=float)[None] * E.LSB
                    rows.append({"c": got[(k, j)] * E.LSB, "py_half": float(ne.cost(pb, zf, w, "half")[0]),
                                 "py_float": float(ne.cost(pb, zf, w, "float")[0]),
                                 "double": float(ne.cost(pb, zf, w, "double")[0])})
            dc = np.array([abs(r["c"] - r["double"]) for r in rows])
            dp = np.array([abs(r["py_half"] - r["double"]) for r in rows])
            rel_c = np.array([abs(r["c"] - r["double"]) / r["double"] for r in rows])
            rel_p = np.array([abs(r["py_half"] - r["double"]) / r["double"] for r in rows])
            cvp = np.array([abs(r["c"] - r["py_half"]) / r["double"] for r in rows])
            out["C2"] = {
                "n": len(rows),
                "abs_dev_c": {"median": float(np.median(dc)), "p90": float(np.percentile(dc, 90)), "max": float(dc.max())},
                "abs_dev_py_half": {"median": float(np.median(dp)), "p90": float(np.percentile(dp, 90)), "max": float(dp.max())},
                "rel_dev_c": {"median": float(np.median(rel_c)), "p90": float(np.percentile(rel_c, 90))},
                "rel_dev_py_half": {"median": float(np.median(rel_p)), "p90": float(np.percentile(rel_p, 90))},
                "rel_c_vs_py_half": {"median": float(np.median(cvp)), "p90": float(np.percentile(cvp, 90))},
                "J_double": {"median": float(np.median([r["double"] for r in rows])),
                             "p10": float(np.percentile([r["double"] for r in rows], 10)),
                             "p90": float(np.percentile([r["double"] for r in rows], 90))},
                "pass": bool(np.median(dc) <= 1.5 * np.median(dp) and np.percentile(dc, 90) <= 1.5 * np.percentile(dp, 90)),
            }
    return out


# ------------------------------------------------------------------------------------------ solver
class StreamRng:
    """A numpy Generator stand-in replaying the C stream in the order nmpc_engine.solve draws it: the
    initialization and R1 perturbations in sequence; each iteration's r1 and r2 interleaved per
    element in C (r1, r2 per particle, stage, input), separate arrays in Python."""

    def __init__(self, values) -> None:
        self.v = np.asarray(values, dtype=float)
        self.i = 0
        self.pending = None

    def _take(self, n: int) -> np.ndarray:
        if self.i + n > len(self.v):
            raise RuntimeError("stream exhausted")
        out = self.v[self.i:self.i + n]
        self.i += n
        return out

    def uniform(self, low, high, size):
        n = int(np.prod(size))
        if low == 0.0 and high == 1.0:
            if self.pending is None:
                vals = self._take(2 * n)
                self.pending = vals[1::2].reshape(size)
                return vals[0::2].reshape(size)
            r2, self.pending = self.pending, None
            return r2
        return low + (high - low) * self._take(n).reshape(size)

    def normal(self, *a, **k):
        raise RuntimeError("the C stream is uniform: set perturb_dist='uniform'")


def plan_of(c: E.Config, pb: ne.Problem, new_best: np.ndarray) -> np.ndarray:
    z = np.asarray(new_best, dtype=float).reshape(c.nu, 2) * E.LSB
    full = ne.to_full(z[None], pb)[0] if pb.knots is not None else z
    return am.denormalize_controls(full, pb.limits, ne.SCALE)


def solve_check(xilinx: Path, probs: list, n_c: int, n_c_each: int) -> dict:
    out = {}
    for tag, sw in BUILDS.items():
        if tag == "off":
            continue  # the switches-off solver is the deployed one: O1/O2
        c = E.make_config(sw)
        n = n_c if tag == "ALL" else n_c_each
        idx = np.linspace(0, len(probs) - 1, n).round().astype(int)
        b = build_bicycle(f"solve_{tag}", sw, "half", xilinx)
        txt = "".join(E.problem_text(E.pack(c, for_build(c, probs[k]["pb"])), f"P {k}") for k in idx)
        f_in, f_out = BUILD / f"solve_{tag}_in.txt", BUILD / f"solve_{tag}_out.txt"
        f_in.write_text(txt)
        seed0 = 1000
        t0 = time.time()
        run(b, "solve", f_in, f_out, seed0)
        wall = time.time() - t0
        got = {}
        for line in f_out.read_text().splitlines():
            parts = line.split()
            got[(parts[0], int(parts[1]))] = parts[2:]
        rows = []
        for k in idx:
            pb = for_build(c, probs[k]["pb"])
            ci = E.pack(c, pb)
            rs = E.RandStream(seed0 + int(k))
            r = E.solve(c, ci.x_curr, ci.u_curr, ci.xref, ci.aux, ci.last_best.ravel(), rs)
            nb = np.array([E.fxi(float(x)) for x in got[("B", k)]])
            bf = np.array([E.fxi(float(x)) for x in got[("F", k)]])
            nxt = int(got[("N", k)][0])
            rows.append({"k": int(k), "new_best": bool(np.array_equal(nb, r.new_best.ravel())),
                         "bestfitness": bool(np.array_equal(bf, r.bestfitness)), "draws": r.draws,
                         "next_rand": bool(nxt == rs.g()), "J": float(bf[-1] * E.LSB)})
        ok = all(x["new_best"] and x["bestfitness"] and x["next_rand"] for x in rows)
        out[tag] = {"problems": len(rows), "identical": ok, "c_wall_s_per_solve": wall / len(rows), "rows": rows}
        log(f"  solve {tag}: {len(rows)} problems, C == emulation: {ok} ({wall / len(rows):.1f} s per C solve)")
    return out


def emul_cost_fn(c: E.Config, ci: E.CInputs):
    def f(problem, z, w, arith):
        return E.cost(c, ci.x_curr, E.fxi_array(z), ci.xref, ci.u_curr, ci.aux).astype(float) * E.LSB
    return f


def compare_check(probs: list, seeds: int) -> dict:
    """S2 (matched stream) and S3 (distributions) for the ALL build, emulation = C (S1)."""
    c = E.make_config(E.SWITCHES)
    cfg = nc.load_config()
    pso = ne.PsoConfig(10, 40, **{k: float(v) for k, v in cfg.pso.items()})
    s2, s3 = [], []
    for k, p in enumerate(probs):
        if k % 10 == 0:
            wait_for_heavy_jobs()
        pb, w = p["pb"], p["w"]
        ci = E.pack(c, pb)
        # S2: the C stream, replayed
        rs = E.RandStream(5000 + k)
        r = E.solve(c, ci.x_curr, ci.u_curr, ci.xref, ci.aux, ci.last_best.ravel(), rs)
        plan_c = plan_of(c, pb, r.new_best)
        stream = np.array(rs.values, dtype=float) * E.LSB
        pbu = dataclasses.replace(pb, perturb_dist="uniform")
        sol_py = ne.solve(pbu, pso, w, StreamRng(stream), "half")
        sol_py_hls = ne.solve(pbu, pso, w, StreamRng(stream), "half", cost_fn=emul_cost_fn(c, ci))
        row = {"k": k, "u0_c": plan_c[0].tolist(), "u0_py": sol_py.plan[0].tolist(), "u0_py_hls": sol_py_hls.plan[0].tolist(),
               "Jd_c": ne.double_cost(pb, plan_c, w), "Jd_py": ne.double_cost(pb, sol_py.plan, w),
               "Jd_py_hls": ne.double_cost(pb, sol_py_hls.plan, w)}
        s2.append(row)
        # S3: distributions over seeds (C: the emulation; Python: the controller as simulated, normal perturbation)
        jc, jp = [], []
        for s in range(seeds):
            rr = E.solve(c, ci.x_curr, ci.u_curr, ci.xref, ci.aux, ci.last_best.ravel(), E.RandStream(9000 + 100 * k + s))
            jc.append(ne.double_cost(pb, plan_of(c, pb, rr.new_best), w))
            sp = ne.solve(pb, pso, w, np.random.default_rng([k, s, 4242]), "half")
            jp.append(ne.double_cost(pb, sp.plan, w))
        s3.append({"k": k, "path": p["path"], "seed": p["seed"], "Jd_c": jc, "Jd_py": jp,
                   "ratio": float(np.median(jc) / np.median(jp))})
        if k % 10 == 0:
            log(f"  compare {k + 1}/{len(probs)}")

    def du(rows, key):
        d = np.abs(np.array([r[key] for r in rows]) - np.array([r["u0_c"] for r in rows]))
        return {"dv_median": float(np.median(d[:, 0])), "dv_p90": float(np.percentile(d[:, 0], 90)),
                "dkappa_median": float(np.median(d[:, 1])), "dkappa_p90": float(np.percentile(d[:, 1], 90)),
                "share_within_0.01": float(np.mean((d[:, 0] <= 0.01) & (d[:, 1] <= 0.01))),
                "share_identical_1e-9": float(np.mean(np.all(d <= 1e-9, axis=1)))}

    ratios = np.array([r["ratio"] for r in s3])
    jcm = np.array([np.median(r["Jd_c"]) for r in s3])
    jpm = np.array([np.median(r["Jd_py"]) for r in s3])
    return {
        "S2": {"n": len(s2), "vs_python_native": du(s2, "u0_py"), "vs_python_with_hls_cost": du(s2, "u0_py_hls"),
               "Jd_ratio_c_over_py_median": float(np.median([r["Jd_c"] / r["Jd_py"] for r in s2])),
               "Jd_ratio_c_over_py_hls_median": float(np.median([r["Jd_c"] / r["Jd_py_hls"] for r in s2])), "rows": s2},
        "S3": {"n": len(s3), "seeds": seeds, "ratio_median": float(np.median(ratios)),
               "ratio_p10": float(np.percentile(ratios, 10)), "ratio_p90": float(np.percentile(ratios, 90)),
               "share_c_better": float(np.mean(ratios < 1.0)),
               "Jd_c_median": float(np.median(jcm)), "Jd_py_median": float(np.median(jpm)),
               "pass": bool(np.median(ratios) <= 1.10), "rows": s3},
    }


# ------------------------------------------------------------------------------------------ switches off
def reference_tree() -> Path:
    """The sources before this task: HEAD solver files; config.hpp and hls_model.hpp without the
    generated BICYCLE blocks (they keep the uncommitted Task A unicycle additions)."""
    ref = BUILD / "ref_source"
    shutil.rmtree(ref, ignore_errors=True)
    shutil.copytree(HLS, ref, ignore=shutil.ignore_patterns("*.o"))
    for f in SOLVER_FILES:
        data = subprocess.run(["git", "show", f"HEAD:hw/hls/source/{f}"], cwd=REPO, capture_output=True, check=True).stdout
        (ref / f).write_bytes(data)
    cfg = (ref / "include/config.hpp").read_bytes().decode()
    eol = "\r\n"
    for begin, end in (("// BEGIN PSO redesign switches", "// END PSO redesign switches" + eol + eol),
                       ("#elif defined(BICYCLE_CONFIG)", "// END BICYCLE_CONFIG (generated by app/rc_car/gen_bicycle_config.py)" + eol)):
        i = cfg.index(begin)
        j = cfg.index(end, i) + len(end)
        if begin.startswith("#elif"):
            i = cfg.rindex(eol, 0, i) + len(eol)
        cfg = cfg[:i] + cfg[j:]
    (ref / "include/config.hpp").write_bytes(cfg.encode())
    mdl = (ref / "include/hls_model.hpp").read_bytes()
    mdl = mdl.replace(b'#elif defined(BICYCLE_CONFIG)\r\n#include "models/hls_bicycle.hpp"\r\n', b"")
    (ref / "include/hls_model.hpp").write_bytes(mdl)
    (ref / "include/models/hls_bicycle.hpp").unlink()
    return ref


def _tokens(text: str) -> list[str]:
    return text.replace("\r", "").split()


def off_check(xilinx: Path) -> dict:
    ref = reference_tree()
    trees = {"ref": ref, "new": HLS}
    # the reference config must differ from HEAD only by the Task A unicycle additions
    head_cfg = subprocess.run(["git", "show", "HEAD:hw/hls/source/include/config.hpp"], cwd=REPO, capture_output=True,
                              check=True).stdout.decode()
    ref_cfg = (ref / "include/config.hpp").read_bytes().decode()
    import difflib
    removed = [l for l in difflib.unified_diff(head_cfg.splitlines(), ref_cfg.splitlines(), lineterm="", n=0)
               if l.startswith("-") and not l.startswith("---")]
    out = {"ref_config_removed_vs_head": len(removed)}
    # O1: preprocessed token streams
    o1 = []
    tus = ["src/hls_pso.cpp", "src/hls_nonlinear_solver.cpp", "src/main_hls_validate.cpp", "src/main_hls_unicycle_test.cpp",
           "src/main_hls_model_unit_test.cpp"]
    for config in ("SNIFFBOT_CONFIG", "UNICYCLE_CONFIG"):
        for mode in ("float", "fixed", "half"):
            incs = {}
            for name, tree in trees.items():
                incs[name] = header_copy(tree / "include", BUILD / f"pp_{name}_{config}_{mode}", mode == "half", None, None)
            for tu in tus:
                if tu.endswith("unicycle_test.cpp") and (config != "UNICYCLE_CONFIG" or mode == "float"):
                    continue
                if tu.endswith("model_unit_test.cpp") and (config != "SNIFFBOT_CONFIG" or mode == "float"):
                    continue
                streams = {}
                for name, tree in trees.items():
                    flags = [] if mode == "float" else list(HW_FLAGS) + (["-DHW_MODEL_HALF_CSIM"] if mode == "half" else [])
                    r = subprocess.run(["nice", "-n", "19", "g++", "-std=c++11", "-E", "-P", "-w", "-D__VITIS__", "-DPSO_CONFIG",
                                        f"-D{config}", f"-I{incs[name]}", f"-I{incs[name] / 'models'}",
                                        f"-I{xilinx / 'Vitis/include'}", *flags, str(tree / tu)],
                                       capture_output=True, text=True, check=True)
                    streams[name] = _tokens(r.stdout)
                o1.append({"config": config, "mode": mode, "tu": tu, "tokens": len(streams["new"]),
                           "identical": streams["ref"] == streams["new"]})
    out["O1"] = o1
    log(f"  O1: {sum(x['identical'] for x in o1)} of {len(o1)} preprocessed translation units identical")
    # O2: outputs
    o2 = []
    sys.path.insert(0, str(REPO / "app/husky"))
    import hls_unicycle_check as uc
    steps, rolls = uc.make_cases(200, 20261121)
    ucases = BUILD / "unicycle_cases.txt"
    ucases.write_text("".join(f"S {x!r} {y!r} {psi!r} {zv!r} {zw!r}\n" for x, y, psi, zv, zw in steps)
                      + "".join(f"R {zv!r} {zw!r}\n" for zv, zw in rolls))
    rng = np.random.default_rng(7)
    for config, nx, nu_ in (("SNIFFBOT_CONFIG", 12, 4), ("UNICYCLE_CONFIG", 3, 2)):
        vals = [*rng.uniform(-0.2, 0.2, nx), *rng.uniform(-0.5, 0.5, nx * 25), *rng.uniform(-20, 20, nu_),
                *rng.uniform(-20, 20, nu_ * 25)]
        validate_in = " ".join(f"{v:.9g}" for v in vals)
        for mode in ("fixed", "half"):
            outputs = {}
            for name, tree in trees.items():
                inc = header_copy(tree / "include", BUILD / f"o2_{name}_{config}_{mode}", mode == "half", None, None)
                flags = list(HW_FLAGS) + (["-DHW_MODEL_HALF_CSIM"] if mode == "half" else [])
                common = ["nice", "-n", "19", "g++", "-std=c++11", "-O0", "-w", "-D__VITIS__", "-DPSO_CONFIG", f"-D{config}",
                          f"-I{inc}", f"-I{inc / 'models'}", f"-I{xilinx / 'Vitis/include'}", *flags]
                res = {}
                vb = BUILD / f"validate_{name}_{config}_{mode}"
                subprocess.run([*common, "-o", str(vb), *(str(tree / "src" / f) for f in
                                ("main_hls_validate.cpp", "hls_pso.cpp", "aux_functions.cpp", "hls_nonlinear_solver.cpp")),
                                *fpo_libs(xilinx)], check=True)
                for seed in (1, 2, 3):
                    r = subprocess.run(["nice", "-n", "19", str(vb), "solve", str(seed)], input=validate_in,
                                       capture_output=True, text=True, check=True)
                    res[f"validate_seed{seed}"] = "\n".join(l for l in r.stdout.splitlines() if not l.startswith("J "))
                if config == "UNICYCLE_CONFIG":
                    ub = BUILD / f"unitest_{name}_{mode}"
                    subprocess.run([*common, "-o", str(ub), str(tree / "src/main_hls_unicycle_test.cpp"),
                                    str(tree / "src/aux_functions.cpp"), *fpo_libs(xilinx)], check=True)
                    uo = BUILD / f"unitest_{name}_{mode}.txt"
                    r = subprocess.run(["nice", "-n", "19", str(ub), str(ucases), str(uo)], capture_output=True, text=True)
                    res["unicycle_test"] = uo.read_text() + r.stdout.replace(str(uo), "OUT")
                if config == "SNIFFBOT_CONFIG" and mode == "fixed":
                    mb = BUILD / f"modelunit_{name}"
                    subprocess.run([*common, "-o", str(mb), str(tree / "src/main_hls_model_unit_test.cpp"),
                                    *fpo_libs(xilinx)], check=True)
                    r = subprocess.run(["nice", "-n", "19", str(mb)], capture_output=True, text=True)
                    res["model_unit_test"] = f"rc {r.returncode}\n{r.stdout}"
                outputs[name] = res
            for key in outputs["ref"]:
                o2.append({"config": config, "mode": mode, "output": key, "bytes": len(outputs["new"][key]),
                           "identical": outputs["ref"][key] == outputs["new"][key]})
            log(f"  O2 {config} {mode}: {[(x['output'], x['identical']) for x in o2 if x['config'] == config and x['mode'] == mode]}")
    out["O2"] = o2
    # O3: pure additions
    o3 = {}
    for f in DIFF_FILES:
        d = subprocess.run(["git", "diff", "HEAD", "--", f"hw/hls/source/{f}"], cwd=REPO, capture_output=True, text=True).stdout
        o3[f] = {"added": sum(1 for l in d.splitlines() if l.startswith("+") and not l.startswith("+++")),
                 "removed": sum(1 for l in d.splitlines() if l.startswith("-") and not l.startswith("---"))}
    out["O3"] = o3
    return out


# ------------------------------------------------------------------------------------------ main
def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--xilinx", default=os.environ.get("XILINX", "/tools/Xilinx/2025.2"))
    ap.add_argument("--every", type=int, default=40, help="log every N-th tick of the offline runs")
    ap.add_argument("--n-solve", type=int, default=12, help="problems solved in C, ALL build")
    ap.add_argument("--n-solve-each", type=int, default=3, help="problems solved in C, single-switch builds")
    ap.add_argument("--seeds", type=int, default=10, help="S3: seeds per problem")
    ap.add_argument("--quick", action="store_true", help="a smoke run: few problems, 2 seeds")
    ap.add_argument("--only", nargs="*", default=["model", "cost", "solve", "compare", "off"])
    a = ap.parse_args(argv)
    xilinx = Path(a.xilinx)
    BUILD.mkdir(parents=True, exist_ok=True)
    wait_for_heavy_jobs()
    t0 = time.time()
    summary: dict = {"note": "C LEVEL / OFFLINE: g++ builds of hw/hls/source, emulation, Python controller; no synthesis",
                     "args": vars(a)}
    gen = subprocess.run([sys.executable, "-B", str(HERE / "gen_bicycle_config.py"), "--check"], capture_output=True, text=True)
    summary["generator_check"] = gen.stdout.strip()
    probs = None
    if {"cost", "solve", "compare"} & set(a.only):
        probs = logged_problems(a.every)
        if a.quick:
            probs = probs[::8]
        log(f"{len(probs)} logged problems")
    if "model" in a.only:
        log("model check")
        summary["model"] = model_check(xilinx, 300, 20261122)
    if "cost" in a.only:
        log("cost check")
        summary["cost"] = cost_check(xilinx, probs, a.quick)
    if "solve" in a.only:
        log("solve check (C, half)")
        summary["solve"] = solve_check(xilinx, probs, 3 if a.quick else a.n_solve, 1 if a.quick else a.n_solve_each)
    if "compare" in a.only:
        log("compare with the Python controller")
        summary["compare"] = compare_check(probs, 2 if a.quick else a.seeds)
    if "off" in a.only:
        log("switches off")
        summary["off"] = off_check(xilinx)
    summary["wall_s"] = time.time() - t0

    v = {"generator --check": summary["generator_check"] == "generated blocks match"}
    if "model" in summary:
        m = summary["model"]
        v["M1 half == emulation (model)"] = m["M1_differ"] == 0
        for mode, ok in m["M2"].items():
            v[f"M2 {mode}: position error <= 1.5 x October emulation"] = ok
        v["M4 generator dry run and __model == __model_pipe"] = m["M4"]
    if "cost" in summary:
        for tag in BUILDS:
            v[f"C1 cost {tag}: half == emulation"] = summary["cost"][tag]["differ"] == 0
        v["C2 cost deviation vs double <= 1.5 x Python half"] = summary["cost"]["C2"]["pass"]
    if "solve" in summary:
        for tag, r in summary["solve"].items():
            v[f"S1 solve {tag}: half == emulation"] = r["identical"]
    if "compare" in summary:
        v["S3 median J ratio C / Python <= 1.10"] = summary["compare"]["S3"]["pass"]
    if "off" in summary:
        o = summary["off"]
        v["O1 preprocessed streams identical"] = all(x["identical"] for x in o["O1"])
        v["O2 outputs identical"] = all(x["identical"] for x in o["O2"])
        v["O3 no removed line"] = all(x["removed"] == 0 for x in o["O3"].values())
    summary["verdicts"] = v
    (BUILD / ("summary_quick.json" if a.quick else "summary.json")).write_text(json.dumps(summary, indent=1, default=float))
    print("\n=== Verdicts ===")
    for name, ok in v.items():
        print(f"{'PASS' if ok else 'FAIL'}  {name}")
    print(f"summary: {BUILD / ('summary_quick.json' if a.quick else 'summary.json')} ({summary['wall_s']:.0f} s)")
    return 0 if all(v.values()) else 1


if __name__ == "__main__":
    sys.exit(main())
