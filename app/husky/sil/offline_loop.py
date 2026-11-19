"""Offline closed loop of the SIL controller on a SCREENING plant (no ROS, no Gazebo).

    python3 -m sil.offline_loop --paths circle figure8 --r-omega 5e-5 5e-6 0 --seeds 1 2 3 \
        --plant-yaw-gain 1.0 0.92 --out ~/nmpc_sim/logs/offline_romega_20261119

The controller is the real sil.controller.MpcCore, driven exactly as the node drives it:
arming request, hold probes until the (echoed) mux output confirms ownership, then per tick
begin_tick -> publish -> finish_tick, so the objective, reference window, warm start and its
projection, the one-tick actuation delay, the host-side constraint checks, fallbacks and the
seeded PSO are the node's own code, not a copy.

The plant is an abstraction for screening controller settings, NOT a model of the A300:
    ideal unicycle         x' = v cos psi, y' = v sin psi, psi' = omega
    gain-scaled unicycle   the same with v and omega multiplied by constant factors
                           (labelled; e.g. the simulated yaw-rate ratio 0.92 at kappa 0.5)
    actuation delay        the command published at t acts from t + plant_delay (the previous
                           one until then). MpcCore predicts t -> t + Ts with the applied command
                           (cmd_vel_out) only, i.e. as if the delay were >= Ts; in the Gazebo SIL
                           runs cmd_vel_out showed the new command 3 / 12 / 33 ms (p5 / median /
                           p95) after the tick (circle plain seed 1, 2026-11-19).
It isolates objective bias and solver behaviour. It does not stand in for Gazebo contact
dynamics, the C model or the board.

Feedback is the plant state itself (stamped at the tick, twist = the body motion), the applied
command (cmd_vel_out) is the published one, and the mux echoes every published command at once.
The simulation clock does not advance during a solve (C_sim = 0), so deadline misses and
deadline fallbacks are not modelled; C_wall is measured and reported for information only.
Every run is deterministic for a given configuration and seed.
"""

from __future__ import annotations

import argparse
import csv
from dataclasses import asdict, dataclass, field, replace
import datetime
import json
import math
from multiprocessing import Pool
from pathlib import Path
import subprocess
import sys
import time
from typing import Any

import numpy as np

from . import _HUSKY  # noqa: F401  (sets sys.path)
from . import metrics
from . import pso as pso_mod
from .controller import CoreConfig, MpcCore, OdomSample
from .reference import PathSpec, compose
import command_ownership as co
import unicycle_model as um

SETTLE = 2.0  # s scored after the path ends (as metrics.tracking)
SUBSTEPS = 10  # plant integration steps per tick
PLANT_LABEL = "screening abstraction (ideal / gain-scaled unicycle), not the A300 and not Gazebo"


@dataclass(frozen=True)
class LoopConfig:
    path: str = "circle"
    size: float = 2.0
    speed: float = 0.2
    pso: str = "7x30"
    nu: int = 25
    gain_mode: str = "plain"
    r_v: float = 0.0
    r_omega: float = 5e-5
    seed: int = 1
    plant_speed_gain: float = 1.0
    plant_yaw_gain: float = 1.0
    plant_delay: float = 0.0  # s from the publishing tick until the plant executes the new command
    v_max: float = 0.3
    omega_max: float = 0.75

    def core_config(self) -> CoreConfig:
        limits = um.limits_from_parameter_set("husky_a300", v_min=0.0, v_max=self.v_max, omega_max=self.omega_max)
        cost = replace(pso_mod.CostConfig(), r=(self.r_v, self.r_omega))
        return CoreConfig(limits=limits, pso=pso_mod.PsoConfig.parse(self.pso), control_horizon=self.nu,
                          gain_mode=self.gain_mode, path=PathSpec(self.path, self.speed, self.size), cost=cost)

    @property
    def run_id(self) -> str:
        return (f"{self.path}_r{self.size:g}_v{self.speed:g}_{self.pso}_nu{self.nu}_{self.gain_mode}_"
                f"romega{self.r_omega:g}_plant{self.plant_speed_gain:g}x{self.plant_yaw_gain:g}_d{self.plant_delay * 1e3:g}ms_"
                f"seed{self.seed}")


@dataclass
class LoopResult:
    config: LoopConfig
    ticks: list[dict[str, Any]] = field(default_factory=list)  # t, state, publish kind and command, fallback
    truth: list[dict[str, Any]] = field(default_factory=list)  # plant state (stamp, x, y, yaw)
    tau0: float | None = None
    origin: np.ndarray | None = None
    ended: str = ""


def run_loop(cfg: LoopConfig, start: tuple[float, float, float] = (0.0, 0.0, 0.0), t0: float = 10.0) -> LoopResult:
    """One closed-loop episode through MpcCore; ends SETTLE s after the path or on lost ownership."""
    core_cfg = cfg.core_config()
    core = MpcCore(core_cfg, seed=cfg.seed)
    ts = core_cfg.ts
    res = LoopResult(cfg)
    pose = np.array(start, dtype=float)
    u = np.zeros(2)  # latest published command (= cmd_vel_out once it takes effect)
    u_old = np.zeros(2)  # the command executed before the latest one takes effect
    t = t0
    body = np.zeros(2)

    def feed() -> None:
        core.on_odom(OdomSample(t, t, pose[0], pose[1], pose[2], body[0], body[1]))
        core.on_imu(t)
        core.on_applied(co.AppliedSample(t, t, float(u_old[0]), float(u_old[1])))

    feed()
    ok, why = core.request_arm(t)
    if not ok:
        res.ended = f"arming refused: {why}"
        return res
    clock = {"t": t}
    end = None
    owned_once = False
    for _ in range(100000):
        feed()
        clock["t"] = t
        stamp = int(round(t * 1e9))
        record = core.begin_tick(t, stamp, time.perf_counter)
        u_old = u
        if record.publish is not None:
            core.on_mux(stamp, record.publish[0], record.publish[1], t)
            u = np.array(record.publish)
        core.finish_tick(record, lambda: clock["t"], time.perf_counter)
        if core.tau0 is not None and res.tau0 is None:
            res.tau0, res.origin = core.tau0, core.origin.copy()
            end = res.tau0 + core_cfg.path.duration + SETTLE
        owned_once = owned_once or record.state == co.State.OWNED.value
        res.ticks.append({"t": t, "state": record.state, "kind": record.publish_kind,
                          "v": None if record.publish is None else record.publish[0],
                          "w": None if record.publish is None else record.publish[1],
                          "fallback": record.fallback, "accepted": record.accepted, "solved": record.solved,
                          "c_wall": record.c_wall.get("total"), "estimate": record.estimate})
        if owned_once and record.state != co.State.OWNED.value:
            res.ended = f"ownership lost: {record.note or record.state}"
            return res
        # plant over [t, t + ts]: the previous command until t + plant_delay, then the new one
        dt = ts / SUBSTEPS
        for k in range(SUBSTEPS):
            res.truth.append({"stamp": t, "x": float(pose[0]), "y": float(pose[1]), "yaw": float(pose[2])})
            cmd = u_old if (k + 0.5) * dt < cfg.plant_delay else u
            body = np.array([cfg.plant_speed_gain * cmd[0], cfg.plant_yaw_gain * cmd[1]])
            # exact arc of the unicycle over dt (constant v, omega)
            v, w = body
            if abs(w) < 1e-9:
                pose = pose + np.array([v * math.cos(pose[2]) * dt, v * math.sin(pose[2]) * dt, 0.0])
            else:
                h1 = pose[2] + w * dt
                pose = np.array([pose[0] + v / w * (math.sin(h1) - math.sin(pose[2])),
                                 pose[1] - v / w * (math.cos(h1) - math.cos(pose[2])), h1])
            t = round(t + dt, 9)
        u_old = u  # by the next tick the latest command is the applied one
        if end is not None and t > end + 1e-9:
            res.ended = "run finished"
            return res
    res.ended = "tick limit"
    return res


# ---------------------------------------------------------------------------------- metrics
def published_violations(commands: np.ndarray, limits: um.UnicycleLimits, ts: float) -> dict[str, Any]:
    """Each published command against the previous published one (as metrics.violations)."""
    keys = ("speed", "omega", "linear_slew", "angular_slew")
    worst = {k: 0.0 for k in keys}
    count = 0
    for prev, cur in zip(commands[:-1], commands[1:]):
        rep = um.check_controls([cur], prev, limits, ts)
        for k in keys:
            worst[k] = max(worst[k], rep[k])
        count += int(not rep["ok"])
    return {"max": worst, "ticks_violating": count}


def summarize(res: LoopResult) -> dict[str, Any]:
    cfg = res.config
    core_cfg = cfg.core_config()
    spec, ts, limits = core_cfg.path, core_cfg.ts, core_cfg.limits
    out: dict[str, Any] = {"run": cfg.run_id, **asdict(cfg), "ended": res.ended, "plant": PLANT_LABEL}
    if res.tau0 is None:
        return out
    trk = metrics.tracking(spec, [{"episode": 1, "tau0": res.tau0}], res.truth)[0]
    out.update({k: trk[k] for k in ("pos_rmse", "pos_max", "heading_rmse", "cross_track_rmse", "cross_track_max")
                if k in trk})
    if spec.kind == "circle":  # signed radial offset of the plant from the reference circle (+ = outside)
        start = metrics._interp_pose(res.truth, res.tau0)
        centre = compose(start, np.array([[0.0, spec.size, 0.0]]))[0, :2]
        sel = [r for r in res.truth if res.tau0 <= r["stamp"] <= res.tau0 + spec.duration]
        radial = np.hypot([r["x"] - centre[0] for r in sel], [r["y"] - centre[1] for r in sel]) - spec.size
        out["radial_offset_mean"] = float(radial.mean())
        out["radial_offset_last_eighth"] = float(radial[-max(len(radial) // 8, 1):].mean())
    window = [r for r in res.ticks if res.tau0 - ts - 1e-9 <= r["t"] <= res.tau0 + spec.duration
              and r["v"] is not None and r["kind"] != "probe"]
    cmd = np.array([[r["v"], r["w"]] for r in window])
    if len(cmd):
        moving = np.abs(cmd[:, 0]) > 0.02
        out["v_cmd_median"] = float(np.median(cmd[:, 0]))
        out["kappa_cmd_median"] = float(np.median(np.abs(cmd[moving, 1] / cmd[moving, 0]))) if moving.any() else None
        out["at_v_max"] = float(np.mean(cmd[:, 0] >= limits.v_max - 1e-3))
        out["at_omega_max"] = float(np.mean(np.abs(cmd[:, 1]) >= limits.omega_max - 1e-3))
        viol = published_violations(cmd, limits, ts)
        out["published_violations"] = viol["ticks_violating"]
        out["violation_max"] = viol["max"]
        out["smoothness"] = metrics.smoothness(cmd, limits, ts)
    owned = [r for r in res.ticks if r["solved"]]
    out["solves"] = len(owned)
    out["fallback_ticks"] = sum(1 for r in owned if r["fallback"])
    out["stop_ticks"] = sum(1 for r in res.ticks if r["kind"] == "stop")
    cw = np.array([r["c_wall"] for r in owned if r["c_wall"] is not None])
    if len(cw):
        out["c_wall_info"] = {"median": float(np.median(cw)), "p95": float(np.percentile(cw, 95)), "max": float(cw.max())}
    return out


# -------------------------------------------------------------------------------------- CLI
CSV_FIELDS = ("run", "path", "size", "speed", "pso", "nu", "gain_mode", "r_v", "r_omega", "seed", "plant_speed_gain",
              "plant_yaw_gain", "plant_delay", "ended", "cross_track_rmse", "cross_track_max", "pos_rmse", "pos_max", "heading_rmse",
              "radial_offset_mean", "radial_offset_last_eighth", "v_cmd_median", "kappa_cmd_median", "at_v_max",
              "at_omega_max", "published_violations", "fallback_ticks", "stop_ticks", "dv_rms", "dv_p95", "dv_max",
              "dv_reversals_per_s", "dv_at_upper_slew", "dv_at_lower_slew", "domega_rms", "domega_p95", "domega_max",
              "domega_reversals_per_s", "domega_at_upper_slew", "domega_at_lower_slew")


def csv_row(summary: dict[str, Any]) -> dict[str, Any]:
    row = {k: summary.get(k) for k in CSV_FIELDS}
    for ch in ("dv", "domega"):
        for k, v in summary.get("smoothness", {}).get(ch, {}).items():
            if f"{ch}_{k}" in CSV_FIELDS:
                row[f"{ch}_{k}"] = v
    return row


def _one(cfg: LoopConfig) -> dict[str, Any]:
    return summarize(run_loop(cfg))


def grid(args: argparse.Namespace) -> list[LoopConfig]:
    return [LoopConfig(path=p, size=args.size, speed=args.speed, pso=args.pso, nu=args.nu, gain_mode=g,
                       r_omega=r, seed=s, plant_speed_gain=args.plant_speed_gain, plant_yaw_gain=yg, plant_delay=d)
            for p in args.paths for g in args.gains for yg in args.plant_yaw_gain for d in args.plant_delay
            for r in args.r_omega for s in args.seeds]


def _git_rev() -> str:
    try:
        return subprocess.run(["git", "rev-parse", "HEAD"], cwd=_HUSKY, capture_output=True, text=True,
                              check=True).stdout.strip()
    except (OSError, subprocess.CalledProcessError):
        return "unknown"


def main(argv: list[str] | None = None) -> int:
    p = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    p.add_argument("--paths", nargs="+", default=["circle", "figure8"], choices=("line", "circle", "figure8"))
    p.add_argument("--size", type=float, default=2.0)
    p.add_argument("--speed", type=float, default=0.2)
    p.add_argument("--pso", default="7x30")
    p.add_argument("--nu", type=int, default=25)
    p.add_argument("--gains", nargs="+", default=["plain"], choices=pso_mod.GAIN_MODES)
    p.add_argument("--r-omega", nargs="+", type=float, default=[5e-5, 5e-6, 0.0])
    p.add_argument("--seeds", nargs="+", type=int, default=[1, 2, 3])
    p.add_argument("--plant-yaw-gain", nargs="+", type=float, default=[1.0, 0.92])
    p.add_argument("--plant-speed-gain", type=float, default=1.0)
    p.add_argument("--plant-delay", nargs="+", type=float, default=[0.0], help="s, see the module docstring")
    p.add_argument("--jobs", type=int, default=6)
    p.add_argument("--out", required=True)
    args = p.parse_args(argv)
    out = Path(args.out).expanduser()
    out.mkdir(parents=True, exist_ok=False)
    configs = grid(args)
    started = datetime.datetime.now().isoformat(timespec="seconds")
    with Pool(args.jobs) as pool:
        summaries = pool.map(_one, configs)
    meta = {"tool": "sil.offline_loop", "git_rev": _git_rev(), "started": started,
            "finished": datetime.datetime.now().isoformat(timespec="seconds"), "argv": sys.argv[1:] if argv is None else argv,
            "plant": PLANT_LABEL, "substeps": SUBSTEPS, "settle": SETTLE,
            "deadbands": {"dv": metrics.DEADBAND_V, "domega": metrics.DEADBAND_W},
            "deadline": "not modelled (C_sim = 0); C_wall measured for information only",
            "core_config_example": json.loads(json.dumps(asdict(configs[0].core_config()), default=str)),
            "numpy": np.__version__}
    (out / "meta.json").write_text(json.dumps(meta, indent=2, default=str))
    with open(out / "results.jsonl", "w") as handle:
        for s in summaries:
            handle.write(json.dumps(s, default=str) + "\n")
    with open(out / "results.csv", "w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=CSV_FIELDS)
        writer.writeheader()
        for s in summaries:
            writer.writerow(csv_row(s))
    for s in summaries:
        sm = s.get("smoothness", {})
        print(f"{s['run']}: {s['ended']}; xt_rmse={s.get('cross_track_rmse', float('nan')):.4f} "
              f"max={s.get('cross_track_max', float('nan')):.4f} viol={s.get('published_violations')} "
              f"fallback={s.get('fallback_ticks')} dw_rms={sm.get('domega', {}).get('rms', float('nan')):.4f} "
              f"dw_rev/s={sm.get('domega', {}).get('reversals_per_s', float('nan')):.2f}")
    print(f"results: {out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
