"""Numerically validate hardware NMPC solves dumped by the board validation script.

For every hardware run in the JSON dump, three results are reported separately:

(a) Implementation consistency: the hardware's final bestfitness against J(new_best)
    recomputed with the hardware's own arithmetic (ap_fixed<32,17> top level, half model,
    fast sin) in C. This checks the FPGA implementation, not the model.
(b) Model gap: J and per-state trajectory deviation of the hardware model against a
    double-precision reference (REF: double arithmetic, std::sin). REF uses the same model
    equations, so this measures numerical fidelity to those equations only; it is not a
    physical validation of the plant model.
(c) Plant constraint: the true roll/pitch limit +-pi/4 checked on the REF trajectory. The
    tightened cost-function limit (pi/4 - _angle_limit_margin) is reported for information.
    Total cost alone cannot classify safety: the hard 1e4 penalty is discontinuous at the
    cost limit.

Control constraints (u bounds, du_max steps from u_curr, non-increasing bestfitness) are
checked as well. With --seeds N the hardware cost is also placed among N software PSO solves.

Element-wise equality with a software solve is not expected: the hardware random stream reaches
the workers through AXIS register slices and the any-ready fan-out, so its sequence differs from
the software RNG. Component-level bit-exactness is covered by the HLS cosim testbenches.

Usage: python3 validate_hw_run.py <validate.json> [--seeds N] [--xilinx /tools/Xilinx/2025.2]
"""
from __future__ import annotations

import argparse
import json
import math
import re
import shutil
import subprocess
import sys
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parents[2]
HLS_SOURCE = REPO / "hw/hls/source"
BUILD_DIR = REPO / "hw/hls/nmpc_solver_validate_build"

NX, NU, HORIZON = 12, 4, 25
U_MIN, U_MAX, DU_MAX = -100.0, 100.0, 20.0
PLANT_ANGLE_LIMIT = math.pi / 4
CONSISTENCY_TOLERANCE = 1e-3  # relative, hardware vs half model
HW_FLAGS = ["-DHW_ARITH_CSIM", "-DUSE_FAST_SIN_COS"]


def angle_margin() -> float:
    text = (HLS_SOURCE / "include/config.hpp").read_text()
    match = re.search(r"_angle_limit_margin\s*=\s*([0-9.eE+-]+)", text)
    return float(match.group(1)) if match else 0.0


def build(mode: str, xilinx: Path, overrides: dict[str, int] | None = None) -> Path:
    """Build the validation tool; overrides replace `#define NAME value` lines of config.hpp
    (e.g. {"_n_S": 5, "_maxiter": 25}) in an isolated header copy, never in the source tree."""
    overrides = overrides or {}
    tag = mode + "".join(f"_{name.strip('_')}{value}" for name, value in sorted(overrides.items()))
    BUILD_DIR.mkdir(parents=True, exist_ok=True)
    binary = BUILD_DIR / f"validate_{tag}"
    sources = [HLS_SOURCE / "src" / f for f in
               ("main_hls_validate.cpp", "hls_pso.cpp", "aux_functions.cpp", "hls_nonlinear_solver.cpp")]
    headers = list((HLS_SOURCE / "include").rglob("*.hpp"))
    newest = max(p.stat().st_mtime for p in [*sources, *headers])
    if binary.exists() and binary.stat().st_mtime > newest:
        return binary
    include = HLS_SOURCE / "include"
    flags, libs = [], []
    if mode in ("hw", "half"):
        flags = list(HW_FLAGS)
    if mode == "half" or overrides:
        include = BUILD_DIR / f"include_{tag}"
        shutil.rmtree(include, ignore_errors=True)
        shutil.copytree(HLS_SOURCE / "include", include)
        config = include / "config.hpp"
        text = config.read_bytes()
        for name, value in overrides.items():
            text, count = re.subn(rb"(#define %s[ \t]+)\S+" % name.encode(), rb"\g<1>%d" % value, text)
            if count != 1:
                raise ValueError(f"config.hpp defines {name} {count} times, expected once")
        config.write_bytes(text)
    if mode == "half":
        # The C++ half emulation has no volatile operators: build from a copy without them.
        for name in ("hls_system.hpp", "models/hls_sniffbot.hpp"):
            path = include / name
            path.write_bytes(re.sub(rb"\bvolatile ", b"", path.read_bytes()))
        flags.append("-DHW_MODEL_HALF_CSIM")
        fpo = xilinx / "lnx64/tools/fpo_v7_1"
        # RPATH (not RUNPATH) so the library's own gmp/mpfr dependencies resolve from fpo too.
        libs = [f"-L{fpo}", "-lIp_floating_point_v7_1_bitacc_cmodel", "-Wl,--disable-new-dtags",
                f"-Wl,-rpath,{fpo}"]
    # -O0: host GCC miscompiles the multi-word ap_private paths used by >64-bit ap_fixed
    # intermediates at -O1/-O2 (garbage costs); Vitis csim builds without optimisation too.
    command = ["g++", "-std=c++11", "-O0", "-w", "-D__VITIS__", "-DPSO_CONFIG", "-DSNIFFBOT_CONFIG",
               f"-I{include}", f"-I{include / 'models'}", f"-I{xilinx / 'Vitis/include'}",
               *flags, "-o", str(binary), *map(str, sources), *libs]
    subprocess.run(command, check=True)
    return binary


def run_tool(binary: Path, args: list[str], values: list[float]) -> dict[str, list]:
    result = subprocess.run([str(binary), *args], input=" ".join(f"{v:.9g}" for v in values),
                            capture_output=True, text=True, check=True)
    parsed: dict[str, list] = {"S": []}
    for line in result.stdout.splitlines():
        key, *rest = line.split() or [""]
        if key == "S":
            parsed["S"].append([float(v) for v in rest[1:]])
        elif key in ("J", "Jrecomputed", "u"):  # skip HLS simulation library messages
            parsed[key] = [float(v) for v in rest]
    return parsed


def trajectory(binary: Path, base: list[float], u: np.ndarray) -> np.ndarray:
    steps = np.asarray(run_tool(binary, ["trajectory"], [*base, *u.reshape(-1)])["S"])
    return steps  # columns: x[0..NX-1], accumulated state cost, accumulated control cost


def control_problems(new_best: np.ndarray, u_curr: np.ndarray, bestfitness: np.ndarray) -> list[str]:
    problems = []
    if np.any(new_best < U_MIN) or np.any(new_best > U_MAX):
        problems.append("control outside [u_min, u_max]")
    worst = float(np.max(np.abs(np.diff(np.vstack([u_curr, new_best]), axis=0))))
    if worst > DU_MAX + 1e-3:
        problems.append(f"step of {worst:.4f} exceeds du_max {DU_MAX}")
    rises = np.flatnonzero(np.diff(bestfitness) > 0)
    if rises.size:
        problems.append(f"bestfitness increases at iterations {rises[:5].tolist()}")
    return problems


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("dump", type=Path)
    parser.add_argument("--seeds", type=int, default=0, help="software PSO solves to compare against")
    parser.add_argument("--xilinx", type=Path, default=Path("/tools/Xilinx/2025.2"))
    args = parser.parse_args()

    record = json.loads(args.dump.read_text())
    inputs = record["inputs"]
    x_curr = np.asarray(inputs["x_curr"])
    xref = np.asarray(inputs["xref"]).reshape(-1)
    u_curr = np.asarray(inputs["u_curr"])
    last_best = np.asarray(inputs["last_best"]).reshape(-1)
    base = [*x_curr, *xref]
    tools = {mode: build(mode, args.xilinx) for mode in ("float", "hw", "half")}
    cost_limit = PLANT_ANGLE_LIMIT - angle_margin()

    print(f"Overlay: {record.get('overlay')}   runs: {len(record['runs'])}")
    print(f"Limits: plant |roll|,|pitch| <= {PLANT_ANGLE_LIMIT:.5f}; cost function <= {cost_limit:.5f}")
    software = {}
    for mode in ("float", "hw") if args.seeds else ():
        costs = [run_tool(tools[mode], ["solve", str(seed)], [*base, *u_curr, *last_best])["Jrecomputed"][0]
                 for seed in range(1, args.seeds + 1)]
        software[mode] = np.asarray(costs)
        print(f"Software PSO ({mode}, {args.seeds} seeds): J min {min(costs):.2f}  "
              f"median {np.median(costs):.2f}  max {max(costs):.2f}")

    failures = 0
    for run in record["runs"]:
        new_best = np.asarray(run["new_best"]).reshape(HORIZON, NU)
        bestfitness = np.asarray(run["bestfitness"])
        j_hw = float(bestfitness[-1])
        print(f"\nRun {run['run']}: iterations {run['iterations']}, {run['elapsed_seconds'] * 1e3:.1f} ms, "
              f"hardware J {j_hw:.4f}")

        j_half = run_tool(tools["half"], ["cost"], [*base, *new_best.reshape(-1)])["J"][0]
        consistent = abs(j_half - j_hw) <= CONSISTENCY_TOLERANCE * abs(j_half)
        print(f"  (a) implementation consistency: half-model J {j_half:.4f}, "
              f"rel diff {abs(j_half - j_hw) / abs(j_half):.4%} -> {'PASS' if consistent else 'FAIL'}")

        ref = trajectory(tools["float"], base, new_best)
        hw = trajectory(tools["half"], base, new_best)
        j_ref = ref[-1, NX] + ref[-1, NX + 1]
        print(f"  (b) model gap vs REF (fidelity to the model equations, not physical validation): "
              f"REF J {j_ref:.4f} ({(j_hw - j_ref) / j_ref:+.2%}); max |hw-REF| roll/pitch "
              f"{np.abs(hw[:, 3:5] - ref[:, 3:5]).max():.5f} rad, position {np.abs(hw[:, 0:3] - ref[:, 0:3]).max():.4f} m")

        tilt_ref = np.abs(ref[:, 3:5]).max(axis=1)
        tilt_hw = np.abs(hw[:, 3:5]).max(axis=1)
        plant_steps = np.flatnonzero(tilt_ref > PLANT_ANGLE_LIMIT).tolist()
        print(f"  (c) plant constraint on REF: peak |roll/pitch| {tilt_ref.max():.5f}, "
              f"steps beyond +-pi/4 {plant_steps or 'none'} -> {'FAIL' if plant_steps else 'PASS'}; "
              f"cost-limit crossings REF {np.flatnonzero(tilt_ref > cost_limit).tolist() or 'none'}, "
              f"hardware model {np.flatnonzero(tilt_hw > cost_limit).tolist() or 'none'}")

        problems = control_problems(new_best, u_curr, bestfitness)
        print("  control constraints: " + ("OK (bounds, du_max steps, non-increasing bestfitness)"
                                           if not problems else "; ".join(problems)))
        for mode, costs in software.items():
            print(f"  software PSO ({mode}): hardware J at percentile {np.mean(costs <= j_hw) * 100:.0f}")
        failures += (not consistent) + bool(plant_steps) + bool(problems)

    if len(record["runs"]) > 1:
        a, b = (np.asarray(r["new_best"]) for r in record["runs"][:2])
        print(f"\nConsecutive solves: new_best max |diff| {np.abs(a - b).max():.4f}; "
              f"J {record['runs'][0]['bestfitness'][-1]:.4f} vs {record['runs'][1]['bestfitness'][-1]:.4f} "
              f"(expected to differ: the free-running RNG continues between solves)")
    print(f"\n{'VALIDATION PASS' if not failures else f'VALIDATION FAIL ({failures})'}")
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
