"""Screen PSO particle/iteration settings in a PC closed loop before any hardware rebuild.

Each candidate (particles x iterations) is compiled from an isolated copy of the HLS headers with
`_n_S`/`_maxiter` overridden (validate_hw_run.build); the fix6 sources are never edited. The binary
reports its compile-time settings, which are checked before use. Arithmetic modes (as in
validate_hw_run.py): "half" is the hardware's own arithmetic (ap_fixed<32,17> top level, half
model, fast sin) and the reference for selection; "hw" and "float" are faster surrogates.

The closed loop matches the notebook (kv260_nmpc_demo.ipynb): RK4 software plant, spiral reference,
initial state [7, 10, 0...], zero initial control and warm start, warm start shifted each step,
and quadrotor.tracking_metrics for every metric.

RNG: the PC uses the C library rand(). Each trial calls srand(seed) once and its stream continues
across the trial's solves. The board uses the hardware generator through AXIS register slices and
the any-ready fan-out, and its trials share one continuing stream, so compare distributions over
seeds, not individual trajectories.

Trials are written to <out>/<mode>_<S>x<K>_seed<N>.json and .npz as they finish; existing trials
are reused, so an interrupted screen resumes where it stopped.

Usage: python3 screen_pso_config.py --mode half --configs 10x100,5x25 --seeds 1-5 [--steps 100]
           [--jobs 12] [--out DIR] [--board validation_logs/<run>]
"""
from __future__ import annotations

import argparse
import json
import subprocess
import sys
import time
from concurrent.futures import ThreadPoolExecutor, as_completed
from pathlib import Path

import numpy as np

from quadrotor import DEFAULT_TS, NU, NX, quadrotor_step, reference_horizon, tracking_metrics
from validate_hw_run import build

HORIZON = 25
INITIAL_STATE = np.array([7.0, 10.0] + [0.0] * (NX - 2))
DEFAULT_OUT = Path(__file__).resolve().parent / "validation_logs" / "pc_screen"
SUMMARY_KEYS = ["position_rmse_m", "position_max_error_m", "final_error_m", "peak_roll_pitch_rad",
                "max_control_step", "steps_at_du_max"]


def parse_configs(text: str) -> list[tuple[int, int]]:
    configs = []
    for item in text.split(","):
        particles, iterations = (int(v) for v in item.lower().split("x"))
        if particles < 2 or iterations < 1:
            raise ValueError(f"{item}: need >= 2 particles (warm start and stable zero) and >= 1 iteration")
        configs.append((particles, iterations))
    return configs


def parse_seeds(text: str) -> list[int]:
    if "-" in text:
        first, last = (int(v) for v in text.split("-"))
        return list(range(first, last + 1))
    return [int(v) for v in text.split(",")]


def run_trial(binary: Path, particles: int, iterations: int, seed: int, steps: int) -> dict:
    process = subprocess.Popen([str(binary), "serve"], stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                               text=True, bufsize=1)
    lines = iter(process.stdout.readline, "")
    config = next(line.split() for line in lines if line.startswith("config "))
    if [int(v) for v in config[1:]] != [particles, iterations, HORIZON, HORIZON]:
        process.kill()
        raise RuntimeError(f"{binary.name} reports {config}, expected {particles}x{iterations}")

    state = INITIAL_STATE.copy()
    control = np.zeros(NU)
    warm_start = np.zeros((HORIZON, NU))
    states, controls, costs, solve_seconds = [state.copy()], [], [], []
    for step in range(steps):
        xref = reference_horizon(step * DEFAULT_TS, HORIZON, DEFAULT_TS)
        record = [seed if step == 0 else 0, *state, *xref.reshape(-1), *control, *warm_start.reshape(-1)]
        started = time.monotonic()
        process.stdin.write(" ".join(f"{v:.9g}" for v in record) + "\n")
        process.stdin.flush()
        reply = {}
        for line in lines:  # the HLS simulation libraries may print other lines
            key, *values = line.split() or [""]
            if key == "end":
                break
            if key in ("u", "Jrecomputed"):
                reply[key] = [float(v) for v in values]
        else:
            raise RuntimeError(f"{binary.name} exited during step {step}")
        solve_seconds.append(time.monotonic() - started)
        sequence = np.asarray(reply["u"]).reshape(HORIZON, NU)
        control = sequence[0].copy()
        state = quadrotor_step(state, control, DEFAULT_TS, method="rk4")
        warm_start = np.vstack([sequence[1:], sequence[-1]])
        states.append(state.copy())
        controls.append(control)
        costs.append(reply["Jrecomputed"][0])
    process.stdin.close()
    process.wait()

    states, controls = np.asarray(states), np.asarray(controls)
    metrics = tracking_metrics(states, controls, np.zeros(NU), DEFAULT_TS)
    metrics["final_error_m"] = float(np.linalg.norm(metrics["final_position_error_m"]))
    return {
        "particles": particles, "iterations": iterations, "evaluations": particles * iterations,
        "seed": seed, "steps": steps, "plant_integrator": "rk4", "binary": binary.name,
        "rng": "C rand(), srand(seed) once per trial, stream continues across solves",
        "pc_solve_seconds_mean": float(np.mean(solve_seconds)),
        "final_cost": costs[-1], "metrics": metrics,
        "_histories": {"states": states, "controls": controls, "costs": np.asarray(costs)},
    }


def summarize(trials: list[dict]) -> list[dict]:
    rows = []
    for key in sorted({(t["mode"], t["particles"], t["iterations"]) for t in trials},
                      key=lambda k: (k[0], -k[1] * k[2], -k[1])):
        group = [t for t in trials if (t["mode"], t["particles"], t["iterations"]) == key]
        row = {"mode": key[0], "config": f"{key[1]}x{key[2]}", "evaluations": key[1] * key[2],
               "seeds": len(group)}
        for name in SUMMARY_KEYS:
            values = np.asarray([t["metrics"][name] for t in group], dtype=float)
            row[f"{name}_mean"] = float(values.mean())
            row[f"{name}_std"] = float(values.std(ddof=1)) if len(values) > 1 else float("nan")
            row[f"{name}_worst"] = float(values.max())
        row["du_violations"] = sum(t["metrics"]["du_violations"] for t in group)
        row["bound_violations"] = sum(t["metrics"]["bound_violations"] for t in group)
        row["plant_limit_ok"] = all(t["metrics"]["plant_limit_ok"] for t in group)
        row["pc_solve_seconds_mean"] = float(np.mean([t["pc_solve_seconds_mean"] for t in group]))
        rows.append(row)
    return rows


def board_summary(run_dir: Path) -> dict | None:
    report = run_dir / "validation_report.json"
    if not report.is_file():
        return None
    metrics = json.loads(report.read_text())["closed_loop_metrics"]
    per_trial = metrics.get("per_trial", [metrics])  # single-run exports store the metrics flat
    row = {"mode": "board", "config": "10x100", "evaluations": 1000, "seeds": len(per_trial)}
    for name in SUMMARY_KEYS:
        if name == "final_error_m":
            values = np.asarray([np.linalg.norm(t["final_position_error_m"]) for t in per_trial])
        elif name in per_trial[0]:
            values = np.asarray([t[name] for t in per_trial], dtype=float)
        else:
            continue
        row[f"{name}_mean"] = float(values.mean())
        row[f"{name}_std"] = float(values.std(ddof=1)) if len(values) > 1 else float("nan")
        row[f"{name}_worst"] = float(values.max())
    return row


def print_table(rows: list[dict]) -> None:
    header = (f"{'mode':6} {'config':>7} {'E':>5} {'n':>2}  {'RMSE m':>19}  {'max err m':>19}  "
              f"{'final err m':>19}  {'peak tilt rad':>19}  {'du/bound viol':>13}  {'PC s/solve':>10}")
    print(header)
    print("-" * len(header))

    def cell(row: dict, name: str) -> str:
        if f"{name}_mean" not in row:
            return f"{'-':>19}"
        return f"{row[f'{name}_mean']:.3f}±{row[f'{name}_std']:.3f} w{row[f'{name}_worst']:.3f}".rjust(19)

    for row in rows:
        violations = f"{row.get('du_violations', '-')}/{row.get('bound_violations', '-')}"
        if row.get("plant_limit_ok") is False:
            violations += " TILT"
        seconds = f"{row['pc_solve_seconds_mean']:.2f}" if "pc_solve_seconds_mean" in row else "-"
        print(f"{row['mode']:6} {row['config']:>7} {row['evaluations']:>5} {row['seeds']:>2}  "
              f"{cell(row, 'position_rmse_m')}  {cell(row, 'position_max_error_m')}  "
              f"{cell(row, 'final_error_m')}  {cell(row, 'peak_roll_pitch_rad')}  "
              f"{violations:>13}  {seconds:>10}")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--mode", choices=("half", "hw", "float"), default="half")
    parser.add_argument("--configs", default="10x100", help="comma-separated particles x iterations")
    parser.add_argument("--seeds", default="1-5", help="e.g. 1-5 or 1,2,7")
    parser.add_argument("--steps", type=int, default=100)
    parser.add_argument("--jobs", type=int, default=12)
    parser.add_argument("--out", type=Path, default=DEFAULT_OUT)
    parser.add_argument("--board", type=Path, help="board run directory with validation_report.json")
    parser.add_argument("--xilinx", type=Path, default=Path("/tools/Xilinx/2025.2"))
    args = parser.parse_args()

    configs, seeds = parse_configs(args.configs), parse_seeds(args.seeds)
    args.out.mkdir(parents=True, exist_ok=True)
    binaries = {c: build(args.mode, args.xilinx, {"_n_S": c[0], "_maxiter": c[1]}) for c in configs}

    trials, pending = [], []
    for (particles, iterations) in configs:
        for seed in seeds:
            path = args.out / f"{args.mode}_{particles}x{iterations}_seed{seed}.json"
            if path.is_file():
                trial = json.loads(path.read_text())
                if trial["steps"] == args.steps:
                    trials.append(trial)
                    continue
            pending.append((particles, iterations, seed, path))
    pending.sort(key=lambda job: -job[0] * job[1])  # longest first for load balance
    print(f"{len(trials)} trials reused, {len(pending)} to run with {args.jobs} jobs", flush=True)

    with ThreadPoolExecutor(max_workers=args.jobs) as pool:
        futures = {pool.submit(run_trial, binaries[(p, k)], p, k, s, args.steps): (p, k, s, path)
                   for p, k, s, path in pending}
        for future in as_completed(futures):
            particles, iterations, seed, path = futures[future]
            trial = future.result()
            trial["mode"] = args.mode
            np.savez(path.with_suffix(".npz"), **trial.pop("_histories"))
            path.write_text(json.dumps(trial, indent=1) + "\n")
            trials.append(trial)
            m = trial["metrics"]
            print(f"{args.mode} {particles}x{iterations} seed {seed}: RMSE {m['position_rmse_m']:.3f} m, "
                  f"peak tilt {m['peak_roll_pitch_rad']:.3f} rad, "
                  f"{trial['pc_solve_seconds_mean']:.1f} s/solve", flush=True)

    trials = [t for t in trials if (t["particles"], t["iterations"]) in configs and t["seed"] in seeds]
    rows = summarize(trials)
    board = board_summary(args.board) if args.board else None
    if board:
        rows.insert(0, board)
    (args.out / f"summary_{args.mode}.json").write_text(json.dumps(rows, indent=1) + "\n")
    print("\nmean±std, w = worst (largest) seed; E = particle evaluations per solve")
    print_table(rows)
    return 0


if __name__ == "__main__":
    sys.exit(main())
