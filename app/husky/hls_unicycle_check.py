"""C check of the HLS unicycle model (UNICYCLE_CONFIG) against the Python references.

    /usr/bin/python3 -B app/husky/hls_unicycle_check.py [--cases 200] [--seed 20261121]

Builds hw/hls/source/src/main_hls_unicycle_test.cpp with g++ (no Vitis HLS, no synthesis) in two
modes, the logic of app/kria-pynq/validate_hw_run.py:build() (that file is not changed):

    fixed  -DHW_ARITH_CSIM -DUSE_FAST_SIN_COS: ap_fixed<32,17> top level, float model
    half   the same plus -DHW_MODEL_HALF_CSIM, from a header copy without `volatile`, linked to the
           fpo bit-accurate floating-point model

and runs:

    Test 1  one RK4 step (System::one_step_prediction) for random states and controls inside the
            PROVISIONAL A300 SIL envelope, local frame (|x|, |y| <= 0.4 m, |psi| <= 2.5 rad);
    Test 2  25-step rollouts from (0, 0, 0) with constant controls;
    Test 3  cost terms on hand cases (inside the C program);
    Test 4  the generator dry-run of gen_headers.sh (pragma lines deleted, renamed to
            __model_pipe) and __model == __model_pipe on every Test 1 input (inside the C program);
    and the quadrotor regression: main_hls_model_unit_test.cpp with -DSNIFFBOT_CONFIG returns 0.

References for Tests 1-2: unicycle_model.step_chord (double, exact: the reference of the model
note's section 5.4 table), unicycle_model.step_rk4 (double RK4, the same scheme) and
hw_estimates.step_rk4_hw (the bit-level emulation of variant A, NOT a C or RTL simulation).

Pass criteria (model note 5.4, variant A, A300 limits, local frame):
    P1  worst one-step position error vs the exact step <= 2.1e-4 m      (both modes)
    P2  worst 25-step position error vs the exact step  <= 2.9e-3 m      (both modes)
    P3  half mode within the emulation's own spread: the worst |half - emulation| position
        difference <= the emulation's worst error vs the exact step, at 1 and at 25 steps
Build outputs go to hw/hls/build/unicycle_check/ (ignored by git).
"""

from __future__ import annotations

import argparse
import json
import math
import os
from pathlib import Path
import re
import shutil
import subprocess
import sys
import tempfile

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import hw_estimates as hw  # noqa: E402
import unicycle_model as um  # noqa: E402

REPO = Path(__file__).resolve().parents[2]
HLS = REPO / "hw/hls/source"
BUILD = REPO / "hw/hls/build/unicycle_check"
HW_FLAGS = ["-DHW_ARITH_CSIM", "-DUSE_FAST_SIN_COS"]
TS, NH = 0.05, 25
# PROVISIONAL: the A300 Gazebo SIL envelope (sil/controller.py CoreConfig), as in config.hpp and the model
LIMITS = um.limits_from_parameter_set("husky_a300", v_min=0.0, v_max=0.3, omega_max=0.75)
BOUND_1, BOUND_25 = 2.1e-4, 2.9e-3  # model note 5.4, variant A, A300 limits, local frame

# gen_headers.sh's transformation, verbatim from the ARC prompt (Test 4)
GENERATOR = r"""set -e
src={src}; tmp={tmp}
sed -E -e '/^[[:space:]]*#pragma HLS/d' -e 's/\bvolatile //g' \
       -e 's/HLS_UNICYCLE_HPP/HLS_UNICYCLE_PIPE_HPP/g' -e 's/>void __model\(/>void __model_pipe(/' \
       $src/models/hls_unicycle.hpp > $tmp/model_pipe.hpp
grep -q '__model_pipe(' $tmp/model_pipe.hpp && echo RENAME-OK || echo "FAIL: rename (check the template header style)"
grep -nE 'pragma HLS.*[;{{}}]|[;{{}}].*pragma HLS' $src/models/hls_unicycle.hpp && echo "FAIL: pragma shares a line" || echo PRAGMAS-OK
"""


def fpo_libs(xilinx: Path) -> list[str]:
    fpo = xilinx / "lnx64/tools/fpo_v7_1"
    return [f"-L{fpo}", "-lIp_floating_point_v7_1_bitacc_cmodel", "-Wl,--disable-new-dtags", f"-Wl,-rpath,{fpo}"]


def build(mode: str, xilinx: Path, pipe_dir: Path | None) -> tuple[Path, list[str]]:
    """g++ build of the unicycle test, as validate_hw_run.py:build() does it for the quadrotor."""
    BUILD.mkdir(parents=True, exist_ok=True)
    include = HLS / "include"
    flags = list(HW_FLAGS)
    if mode == "half":
        include = BUILD / "include_half"
        shutil.rmtree(include, ignore_errors=True)
        shutil.copytree(HLS / "include", include)
        for path in include.rglob("*.hpp"):  # the C++ half emulation has no volatile operators
            path.write_bytes(re.sub(rb"\bvolatile ", b"", path.read_bytes()))
        flags.append("-DHW_MODEL_HALF_CSIM")
    if pipe_dir is not None:
        flags += ["-DUNICYCLE_PIPE_CHECK", f"-I{pipe_dir}"]
    binary = BUILD / f"unicycle_test_{mode}"
    command = ["g++", "-std=c++11", "-O0", "-w", "-D__VITIS__", "-DPSO_CONFIG", "-DUNICYCLE_CONFIG",
               f"-I{include}", f"-I{include / 'models'}", f"-I{xilinx / 'Vitis/include'}", *flags,
               "-o", str(binary), str(HLS / "src/main_hls_unicycle_test.cpp"), str(HLS / "src/aux_functions.cpp"),
               *fpo_libs(xilinx)]
    subprocess.run(command, check=True)
    return binary, command


def quadrotor_regression(xilinx: Path) -> tuple[int, str, list[str]]:
    """The command in main_hls_model_unit_test.cpp's header (-DSNIFFBOT_CONFIG) plus
    -Wl,--disable-new-dtags: with the header's plain -rpath the fpo library's own libgmp is not
    found at run time on this PC (validate_hw_run.py:build() uses the same flag)."""
    binary = BUILD / "model_unit_test_sniffbot"
    command = ["g++", "-std=c++11", "-O0", "-D__VITIS__", "-DPSO_CONFIG", "-DSNIFFBOT_CONFIG", "-DHW_ARITH_CSIM",
               "-DUSE_FAST_SIN_COS", f"-I{HLS / 'include'}", f"-I{HLS / 'include/models'}",
               f"-I{xilinx / 'Vitis/include'}", str(HLS / "src/main_hls_model_unit_test.cpp"), "-w",
               f"-L{xilinx / 'lnx64/tools/fpo_v7_1'}", "-lIp_floating_point_v7_1_bitacc_cmodel", "-Wl,--disable-new-dtags",
               f"-Wl,-rpath,{xilinx / 'lnx64/tools/fpo_v7_1'}", "-o", str(binary)]
    subprocess.run(command, check=True)
    run = subprocess.run([str(binary)], capture_output=True, text=True)
    last = (run.stdout.strip().splitlines() or [run.stderr.strip()])[-1]
    return run.returncode, last, command


def generator_dry_run() -> tuple[Path, list[str]]:
    tmp = Path(tempfile.mkdtemp(prefix="unicycle_pipe_"))
    out = subprocess.run(["bash", "-c", GENERATOR.format(src=HLS / "include", tmp=tmp)],
                         capture_output=True, text=True, check=True).stdout.split()
    return tmp, out


# ------------------------------------------------------------------------------------------ cases
def grid(value: float) -> float:
    return hw.fx(value)  # ap_fixed<32,17> grid, so the C program reads the same numbers exactly


def make_cases(n: int, seed: int) -> tuple[list[tuple], list[tuple]]:
    rng = np.random.default_rng(seed)
    edge_z = [(-100.0, 0.0), (-100.0, 100.0), (100.0, 100.0), (100.0, -100.0), (0.0, 0.0), (100.0, 0.0)]
    steps = [(grid(rng.uniform(-0.4, 0.4)), grid(rng.uniform(-0.4, 0.4)), grid(rng.uniform(-2.5, 2.5)),
              grid(rng.uniform(-100, 100)), grid(rng.uniform(-100, 100))) for _ in range(n - len(edge_z))]
    steps += [(grid(rng.uniform(-0.4, 0.4)), grid(rng.uniform(-0.4, 0.4)), grid(rng.uniform(-2.5, 2.5)), zv, zw)
              for zv, zw in edge_z]
    rolls = [(grid(rng.uniform(-100, 100)), grid(rng.uniform(-100, 100))) for _ in range(n - len(edge_z))] + edge_z
    return steps, rolls


def physical(z_v: float, z_w: float) -> np.ndarray:
    return um.denormalize_controls([z_v, z_w], LIMITS)[0]


def references(state: np.ndarray, control: np.ndarray, steps: int) -> dict[str, list[np.ndarray]]:
    out = {"exact": [], "rk4": [], "emulation": []}
    s = {k: np.array(state, dtype=float) for k in out}
    for _ in range(steps):
        s["exact"] = um.step_chord(s["exact"], control, TS, wrap=False)
        s["rk4"] = um.step_rk4(s["rk4"], control, TS, wrap=False)
        s["emulation"] = hw.step_rk4_hw(s["emulation"], control, TS)
        for k in out:
            out[k].append(s[k].copy())
    return out


def read_output(path: Path) -> tuple[dict[int, np.ndarray], dict[int, list[np.ndarray]]]:
    steps, rolls = {}, {}
    for line in path.read_text().splitlines():
        f = line.split()
        if f[0] == "S":
            steps[int(f[1])] = np.array([float(v) for v in f[2:5]])
        else:
            rolls.setdefault(int(f[1]), []).append(np.array([float(v) for v in f[3:6]]))
    return steps, rolls


def stats(values: list[float]) -> dict[str, float]:
    a = np.abs(np.array(values))
    return {"max": float(a.max()), "p99": float(np.percentile(a, 99))}


def compare(steps: list[tuple], rolls: list[tuple], outputs: dict[str, Path]) -> dict:
    refs_1 = [references(np.array(c[:3]), physical(*c[3:]), 1) for c in steps]
    refs_25 = [references(np.zeros(3), physical(*c), NH) for c in rolls]
    result: dict = {"emulation_vs_exact": {}}
    for horizon, refs in (("1", refs_1), ("25", refs_25)):
        result["emulation_vs_exact"][horizon] = stats(
            [math.hypot(*(r["emulation"][-1][:2] - r["exact"][-1][:2])) for r in refs])
    for mode, path in outputs.items():
        got_1, got_25 = read_output(path)
        res: dict = {}
        for horizon, refs, got in (("1", refs_1, [got_1[i] for i in range(len(steps))]),
                                   ("25", refs_25, [got_25[i][-1] for i in range(len(rolls))])):
            per_ref = {}
            for name in ("exact", "rk4", "emulation"):
                ends = [r[name][-1] for r in refs]
                per_ref[name] = {
                    "position": stats([math.hypot(*(g[:2] - e[:2])) for g, e in zip(got, ends)]),
                    **{state: stats([g[i] - e[i] for g, e in zip(got, ends)]) for i, state in enumerate(("x", "y", "psi"))}}
            res[horizon] = per_ref
        # worst position error over every step of the rollouts (not only the last)
        res["25_any_step_vs_exact"] = stats([math.hypot(*(got_25[i][k][:2] - refs_25[i]["exact"][k][:2]))
                                             for i in range(len(rolls)) for k in range(NH)])
        result[mode] = res
    return result


def verdicts(result: dict) -> dict[str, bool]:
    v = {}
    for mode in ("fixed", "half"):
        v[f"P1 {mode}: 1-step position vs exact <= {BOUND_1:g} m"] = result[mode]["1"]["exact"]["position"]["max"] <= BOUND_1
        v[f"P2 {mode}: 25-step position vs exact <= {BOUND_25:g} m"] = (
            result[mode]["25_any_step_vs_exact"]["max"] <= BOUND_25)
    for horizon in ("1", "25"):
        spread = result["emulation_vs_exact"][horizon]["max"]
        v[f"P3 half vs emulation <= emulation spread ({horizon} step(s), {spread:.3g} m)"] = (
            result["half"][horizon]["emulation"]["position"]["max"] <= spread)
    return v


def main(argv: list[str] | None = None) -> int:
    p = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    p.add_argument("--cases", type=int, default=200)
    p.add_argument("--seed", type=int, default=20261121)
    p.add_argument("--xilinx", default=os.environ.get("XILINX", "/tools/Xilinx/2025.2"))
    args = p.parse_args(argv)
    xilinx = Path(args.xilinx)
    BUILD.mkdir(parents=True, exist_ok=True)

    pipe_dir, gen_out = generator_dry_run()
    print(f"Test 4 generator dry-run ({pipe_dir}): {' '.join(gen_out)}")
    steps, rolls = make_cases(args.cases, args.seed)
    cases = BUILD / "cases.txt"
    cases.write_text("".join(f"S {x!r} {y!r} {psi!r} {zv!r} {zw!r}\n" for x, y, psi, zv, zw in steps)
                     + "".join(f"R {zv!r} {zw!r}\n" for zv, zw in rolls))

    outputs, commands, c_ok = {}, {}, True
    for mode in ("fixed", "half"):
        binary, commands[mode] = build(mode, xilinx, pipe_dir)
        outputs[mode] = BUILD / f"out_{mode}.txt"
        run = subprocess.run([str(binary), str(cases), str(outputs[mode])], capture_output=True, text=True)
        print(f"\n=== mode {mode} ===\n{run.stdout.strip()}")
        c_ok &= run.returncode == 0
    result = compare(steps, rolls, outputs)
    rc, last, commands["quadrotor"] = quadrotor_regression(xilinx)
    v = verdicts(result)
    v["Test 3 + Test 4 (C program, both modes)"] = c_ok
    v["Test 4 generator: RENAME-OK and PRAGMAS-OK"] = gen_out == ["RENAME-OK", "PRAGMAS-OK"]
    v[f"quadrotor regression (SNIFFBOT_CONFIG unit test returns 0: {last})"] = rc == 0

    print("\n=== Tests 1-2: position and per-state |error| (max / p99), m and rad ===")
    for mode in ("fixed", "half"):
        for horizon in ("1", "25"):
            for name in ("exact", "rk4", "emulation"):
                r = result[mode][horizon][name]
                cells = "  ".join(f"{k} {r[k]['max']:.3e}/{r[k]['p99']:.3e}" for k in ("position", "x", "y", "psi"))
                print(f"{mode:5s} {horizon:>2s} step vs {name:9s}: {cells}")
        print(f"{mode:5s} 25-step rollout, worst over every step vs exact: {result[mode]['25_any_step_vs_exact']['max']:.3e}")
    for horizon in ("1", "25"):
        e = result["emulation_vs_exact"][horizon]
        print(f"emulation vs exact, {horizon} step(s): position max {e['max']:.3e}  p99 {e['p99']:.3e}")
    print("\n=== Verdicts ===")
    for name, ok in v.items():
        print(f"{'PASS' if ok else 'FAIL'}  {name}")
    summary = {"seed": args.seed, "cases": args.cases, "pipe_dir": str(pipe_dir), "generator": gen_out,
               "commands": commands, "result": result, "verdicts": v}
    (BUILD / "summary.json").write_text(json.dumps(summary, indent=2))
    print(f"\nsummary: {BUILD / 'summary.json'}")
    return 0 if all(v.values()) else 1


if __name__ == "__main__":
    sys.exit(main())
