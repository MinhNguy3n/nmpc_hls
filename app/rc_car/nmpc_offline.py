"""Offline closed loop of the NMPC arm (plain Python; tests and quick checks, never a result).

Plant: the kinematic bicycle about the rear axle (exact arcs, 1 ms) driven by the emulated
actuator (actuator_model) with the applied (v, delta); pose: the mocap emulator on the true pose
(100 Hz captures); controller: nmpc_controller at 20 Hz with the Scheduler. No Gazebo: no tyre
contact, no Gazebo term. Simulation time only, so Python's wall time cannot affect the result.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from pathlib import Path
import sys
import time

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import actuator_model as act_m  # noqa: E402
import mocap_emulator as moc_m  # noqa: E402
import nmpc_controller as nc  # noqa: E402
import vehicle_params as vp  # noqa: E402
from nmpc_engine import am  # noqa: E402
from pose_input import NewestPose  # noqa: E402


@dataclass
class OfflineResult:
    t: np.ndarray
    x: np.ndarray
    y: np.ndarray
    yaw: np.ndarray
    commands: list
    stats: nc.Stats
    overruns: int
    done: bool


def run(ctrl: nc.NmpcController, path, speed: float, seed: int, actuator: str = "realistic_guess",
        mocap: str = "realistic_guess", t_max: float | None = None, slow_solve_s: float = 0.0) -> OfflineResult:
    geometry = vp.load_geometry()
    raw = vp.load_raw()
    model = act_m.ActuatorModel(act_m.load_profile(actuator), geometry.delta_max, float(raw["v_max_sim"]["value"]), seed)
    model.rephase(0.0, seed)
    em = moc_m.MocapEmulator(moc_m.load_profile(mocap), seed)
    em.reset_episode(0.0, seed)
    ctrl.start_episode(path, speed, seed)
    pose_in, sched = NewestPose(), nc.Scheduler()
    state = np.array(path.start, dtype=float)
    dt, t, next_truth, next_tick = 0.001, 0.0, 0.0, ctrl.cfg.ts
    t_max = t_max if t_max is not None else path.length / max(speed, 0.1) * 1.6 + 3.0
    rows, commands, done = [], [], False
    n = 0
    while t < t_max and not done:
        if t >= next_truth - 1e-9:
            em.on_truth(round(t, 6), state[0], state[1], state[2])
            next_truth += 0.01
            rows.append((t, *state))
        for f in em.due(t):
            pose_in.update(f.capture, f.x, f.y, f.yaw)
        if t >= next_tick - 1e-9:
            plan = ctrl.prepare(round(next_tick, 6), pose_in.pose)
            if plan is not None:
                if slow_solve_s:
                    time.sleep(slow_solve_s)  # a deliberately slow "Python": wall time only
                cmd = ctrl.finish(plan, ctrl.solve(plan))
                sched.submit(t, cmd)
                done = cmd.done
            next_tick += ctrl.cfg.ts
        for cmd in sched.due(t):
            model.command(t, cmd.v, cmd.delta)
            commands.append((t, cmd.v, cmd.delta, cmd.deadline_miss))
        v_app, d_app = model.advance(t + dt)
        state = am.step_exact_arc(state, (v_app, math.tan(d_app) / geometry.wheelbase), dt)
        n += 1
        t = n * dt
    a = np.array(rows)
    return OfflineResult(a[:, 0], a[:, 1], a[:, 2], a[:, 3], commands, ctrl.stats, sched.overruns, done)
