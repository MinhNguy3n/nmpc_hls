"""Numerically validate a hardware NMPC solve dumped by the board validation script.

Builds hw/hls/source/src/main_hls_validate.cpp twice (float reference, and the hardware
top-level arithmetic via -DHW_ARITH_CSIM) and, for every hardware run in the JSON dump:

* recomputes J(new_best) with the C evaluator and compares it with the hardware's final
  bestfitness (absolute / relative error),
* checks the constraints per element (u bounds, du_max steps from u_curr) and that
  bestfitness never increases,
* places the hardware cost within the distribution of software PSO solves (same inputs,
  several seeds, one process per seed so update's static inertia weight starts fresh).

Element-wise equality with a software solve is not expected: the hardware random stream is
shared through an any-ready fan-out with AXIS register slices, so its sequence differs from
the software RNG. Component-level bit-exactness is covered by the HLS cosim testbenches.

Usage: python3 validate_hw_run.py <validate.json> [--seeds N]
"""
from __future__ import annotations

import argparse
import json
import subprocess
import sys
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parents[2]
HLS_SOURCE = REPO / "hw/hls/source"
BUILD_DIR = REPO / "hw/hls/nmpc_solver_validate_build"
XILINX_INCLUDE = Path("/tools/Xilinx/2025.2/Vitis/include")

NX, NU, HORIZON = 12, 4, 25
U_MIN, U_MAX, DU_MAX = -100.0, 100.0, 20.0
MODES = {"float": [], "hw": ["-DHW_ARITH_CSIM", "-DUSE_FAST_SIN_COS"]}


def build(mode: str) -> Path:
    BUILD_DIR.mkdir(parents=True, exist_ok=True)
    binary = BUILD_DIR / f"validate_{mode}"
    sources = [HLS_SOURCE / "src" / f for f in
               ("main_hls_validate.cpp", "hls_pso.cpp", "aux_functions.cpp", "hls_nonlinear_solver.cpp")]
    newest = max(p.stat().st_mtime for p in [*sources, *(HLS_SOURCE / "include").rglob("*.hpp")])
    if binary.exists() and binary.stat().st_mtime > newest:
        return binary
    command = ["g++", "-std=c++11", "-O2", "-w", "-D__VITIS__", "-DPSO_CONFIG", "-DSNIFFBOT_CONFIG",
               f"-I{HLS_SOURCE / 'include'}", f"-I{HLS_SOURCE / 'include/models'}", f"-I{XILINX_INCLUDE}",
               *MODES[mode], "-o", str(binary), *map(str, sources)]
    subprocess.run(command, check=True)
    return binary


def run_tool(binary: Path, args: list[str], values: list[float]) -> dict[str, list[float]]:
    result = subprocess.run([str(binary), *args], input=" ".join(f"{v:.9g}" for v in values),
                            capture_output=True, text=True, check=True)
    parsed = {}
    for line in result.stdout.splitlines():
        key, *rest = line.split() or [""]
        if key in ("J", "Jrecomputed", "u"):  # skip HLS simulation library messages
            parsed[key] = [float(v) for v in rest]
    return parsed


def constraint_report(new_best: np.ndarray, u_curr: np.ndarray, bestfitness: np.ndarray) -> list[str]:
    problems = []
    if np.any(new_best < U_MIN) or np.any(new_best > U_MAX):
        problems.append("control outside [u_min, u_max]")
    steps = np.diff(np.vstack([u_curr, new_best]), axis=0)
    worst = float(np.max(np.abs(steps)))
    if worst > DU_MAX + 1e-3:
        problems.append(f"step of {worst:.4f} exceeds du_max {DU_MAX}")
    rises = np.flatnonzero(np.diff(bestfitness) > 0)
    if rises.size:
        problems.append(f"bestfitness increases at iterations {rises[:5].tolist()}")
    return problems


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("dump", type=Path)
    parser.add_argument("--seeds", type=int, default=20)
    args = parser.parse_args()

    record = json.loads(args.dump.read_text())
    inputs = record["inputs"]
    x_curr = np.asarray(inputs["x_curr"])
    xref = np.asarray(inputs["xref"]).reshape(-1)
    u_curr = np.asarray(inputs["u_curr"])
    last_best = np.asarray(inputs["last_best"]).reshape(-1)
    binaries = {mode: build(mode) for mode in MODES}

    print(f"Overlay: {record.get('overlay')}   runs: {len(record['runs'])}")
    software = {}
    for mode, binary in binaries.items():
        costs = []
        for seed in range(1, args.seeds + 1):
            out = run_tool(binary, ["solve", str(seed)], [*x_curr, *xref, *u_curr, *last_best])
            costs.append(out["Jrecomputed"][0])
        software[mode] = np.asarray(costs)
        c = software[mode]
        print(f"Software PSO ({mode}, {args.seeds} seeds): J min {c.min():.2f}  median {np.median(c):.2f}  "
              f"max {c.max():.2f}")

    failures = 0
    for run in record["runs"]:
        new_best = np.asarray(run["new_best"]).reshape(HORIZON, NU)
        bestfitness = np.asarray(run["bestfitness"])
        j_hw = float(bestfitness[-1])
        print(f"\nRun {run['run']}: iterations {run['iterations']}, {run['elapsed_seconds'] * 1e3:.1f} ms, "
              f"hardware J {j_hw:.4f}")
        for mode, binary in binaries.items():
            j = run_tool(binary, ["cost"], [*x_curr, *xref, *new_best.reshape(-1)])["J"][0]
            rank = float(np.mean(software[mode] <= j_hw)) * 100
            print(f"  J(new_best) recomputed ({mode}): {j:.4f}   abs err {abs(j - j_hw):.4f}   "
                  f"rel err {abs(j - j_hw) / abs(j):.3%}   hardware J at software percentile {rank:.0f}")
        problems = constraint_report(new_best, u_curr, bestfitness)
        print("  constraints: " + ("OK (bounds, du_max steps, non-increasing bestfitness)" if not problems
                                   else "; ".join(problems)))
        failures += bool(problems)

    if len(record["runs"]) > 1:
        a, b = (np.asarray(r["new_best"]) for r in record["runs"][:2])
        diff = np.abs(a - b)
        print(f"\nConsecutive solves: new_best max |diff| {diff.max():.4f}, mean {diff.mean():.4f}; "
              f"J {record['runs'][0]['bestfitness'][-1]:.4f} vs {record['runs'][1]['bestfitness'][-1]:.4f} "
              f"(expected to differ: the RNG and update's static w carry over between solves)")
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
