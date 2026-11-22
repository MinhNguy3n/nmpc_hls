"""Offline measuring tools of the PSO redesign (D1, D2 of the diagnosis plan). OFFLINE, KINEMATIC
MODEL, EMULATED SOLVER: never a Gazebo or vehicle result.

D2, closed loop: the kinematic bicycle about the rear axle (exact arcs, 1 ms) driven by the emulated
actuator (actuator_model, `realistic_guess`) plus a pure delay equal to the Gazebo term (11 ms, so
that the NMPC's delay compensation sees the same chain it was built for); poses from the mocap
emulator (`realistic_guess`); the same tick-anchored timing model. Two arms:
  - nmpc: nmpc_controller.NmpcController with any Redesign switches;
  - pp:   a Python pure pursuit with the frozen baseline lookahead (lookahead_time 0.2 s, min 0.2 m,
          max 2.0 m, speed from the actuator's applied speed as the VESC telemetry), 20 Hz, applied
          at the tick (no solve delay), steering clamped to delta_max.
Metrics: cross-track and heading RMSE (tracking_metrics, end margin 0.5 m as in Gazebo), RMS of the
per-tick delta-kappa of the commands, the fraction of time the applied steering sits on the slew limit.

D1, optimality gap: for sampled NMPC ticks, the same cost (double arithmetic, same horizon, bounds,
rate windows, reference and predicted initial state, same decision space) is minimised with SLSQP
from several starts (the PSO best, the warm start, zero curvature); J_PSO / J_opt is reported with
J_PSO the PSO plan's cost evaluated in double.
"""

from __future__ import annotations

from dataclasses import dataclass, field
import math
from pathlib import Path
import sys
import time

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))
import actuator_model as act_m  # noqa: E402
import mocap_emulator as moc_m  # noqa: E402
import nmpc_controller as nc  # noqa: E402
import nmpc_engine as ne  # noqa: E402
import paths as P  # noqa: E402
import tracking_metrics as tm  # noqa: E402
import vehicle_params as vp  # noqa: E402
from nmpc_engine import am  # noqa: E402
from pose_input import NewestPose  # noqa: E402

GEOM = vp.load_geometry()
RAW = vp.load_raw()
CFG = nc.load_config()
DELTA_RATE = float(act_m.load_profile("realistic_guess").delta_rate_max)


class PurePursuit:
    """The baseline's law in Python: RPP with the frozen lookahead, constant speed."""

    def __init__(self, path, speed: float, t_la: float = 0.2, la_min: float = 0.2, la_max: float = 2.0) -> None:
        self.path, self.speed, self.t_la, self.la_min, self.la_max = path, speed, t_la, la_min, la_max
        self.follow = nc.PathFollower(path)
        self.end_s = path.length - nc.PATH_TRIM - 0.15

    def command(self, pose, v_meas: float) -> tuple[float, float, bool]:
        p = self.path
        s0 = self.follow.project(pose.x, pose.y)
        la = min(max(abs(v_meas) * self.t_la, self.la_min), self.la_max)
        i = self.follow.idx
        c, s = math.cos(pose.yaw), math.sin(pose.yaw)
        j = i
        while j < len(p.s) - 1 and math.hypot(p.x[j] - pose.x, p.y[j] - pose.y) < la:
            j += 1
        dx, dy = p.x[j] - pose.x, p.y[j] - pose.y
        xl, yl = c * dx + s * dy, -s * dx + c * dy
        d2 = max(xl * xl + yl * yl, 1e-9)
        kappa = 2.0 * yl / d2
        delta = max(-GEOM.delta_max, min(GEOM.delta_max, math.atan(GEOM.wheelbase * kappa)))
        return self.speed, delta, s0 >= self.end_s


@dataclass
class Episode:
    t: np.ndarray
    x: np.ndarray
    y: np.ndarray
    yaw: np.ndarray
    kappa_cmd: list
    delta_applied: np.ndarray
    t_applied: np.ndarray
    done: bool
    d1: list = field(default_factory=list)
    stats: object = None


def run(arm: str, path, speed: float, seed: int, redesign: nc.Redesign | None = None, size: str = "10x40",
        arith: str = "half", d1_every: int = 0, actuator: str = "realistic_guess", mocap: str = "realistic_guess",
        gz_delay: float = CFG.gazebo_term) -> Episode:
    model = act_m.ActuatorModel(act_m.load_profile(actuator), GEOM.delta_max, float(RAW["v_max_sim"]["value"]), seed)
    model.rephase(0.0, seed)
    em = moc_m.MocapEmulator(moc_m.load_profile(mocap), seed)
    em.reset_episode(0.0, seed)
    pose_in = NewestPose()
    if arm == "nmpc":
        ctrl = nc.NmpcController(CFG, size, actuator, True, arith, redesign=redesign)
        ctrl.start_episode(path, speed, seed)
        sched = nc.Scheduler()
    else:
        pp = PurePursuit(path, speed)
    state = np.array(path.start, dtype=float)
    dt, n, next_truth, next_tick = 0.001, 0, 0.0, CFG.ts
    t_max = path.length / speed * 1.6 + 3.0
    rows, kap, dapp, tapp = [], [], [], []
    delay_q: list[tuple[float, float, float]] = []  # (t, v, delta) applied by the actuator, delayed by gz_delay
    done, d1 = False, []
    applied_now = (0.0, 0.0)
    tick = 0
    while n * dt < t_max and not done:
        t = n * dt
        if t >= next_truth - 1e-9:
            em.on_truth(round(t, 6), state[0], state[1], state[2])
            next_truth += 0.01
            rows.append((t, *state))
        for f in em.due(t):
            pose_in.update(f.capture, f.x, f.y, f.yaw)
        if t >= next_tick - 1e-9:
            tick += 1
            if arm == "nmpc":
                plan = ctrl.prepare(round(next_tick, 6), pose_in.pose)
                if plan is not None:
                    sol = ctrl.solve(plan)
                    cmd = ctrl.finish(plan, sol)
                    sched.submit(t, cmd)
                    kap.append(cmd.kappa)
                    done = cmd.done
                    if d1_every and tick % d1_every == 0:
                        d1.append((plan.problem, sol.plan, ctrl.weights))
            elif pose_in.pose is not None:
                v, delta, done = pp.command(pose_in.pose, model.speed)
                model.command(t, v, delta)
                kap.append(math.tan(delta) / GEOM.wheelbase)
            next_tick += CFG.ts
        if arm == "nmpc":
            for cmd in sched.due(t):
                model.command(t, cmd.v, cmd.delta)
        v_app, d_app = model.advance(t + dt)
        delay_q.append((t + dt + gz_delay, v_app, d_app))
        while delay_q and delay_q[0][0] <= t + dt + 1e-12:
            applied_now = delay_q.pop(0)[1:]
        dapp.append(applied_now[1])
        tapp.append(t + dt)
        state = am.step_exact_arc(state, (applied_now[0], math.tan(applied_now[1]) / GEOM.wheelbase), dt)
        n += 1
    a = np.array(rows)
    return Episode(a[:, 0], a[:, 1], a[:, 2], a[:, 3], kap, np.array(dapp), np.array(tapp), done, d1,
                   ctrl.stats if arm == "nmpc" else None)


def metrics(ep: Episode, path) -> dict:
    tr = tm.track(path, ep.t, ep.x, ep.y, ep.yaw, end_margin=0.5)
    dk = np.diff(np.asarray(ep.kappa_cmd))
    rate = np.abs(np.diff(ep.delta_applied)) / 0.001
    return {"xte_rmse_mm": 1e3 * tr["cross_track_rmse_m"], "xte_max_mm": 1e3 * tr["cross_track_max_m"],
            "head_rmse_mrad": 1e3 * tr["heading_rmse_rad"], "completion": tr["completion"],
            "dkappa_rms": float(np.sqrt(np.mean(dk ** 2))) if len(dk) else float("nan"),
            "rate_lim_pct": 100.0 * float(np.mean(rate >= 0.98 * DELTA_RATE)), "done": ep.done}


# ----------------------------------------------------------------------------------------------
# D1
# ----------------------------------------------------------------------------------------------

def j_opt(problem: ne.Problem, w, pso_plan: np.ndarray, maxiter: int = 300) -> dict:
    """Minimise the same cost in double with SLSQP over the same decision space and rate windows."""
    from scipy.optimize import minimize
    lim = problem.limits
    du = ne.rate_windows(problem, am.normalized_slew_limits(lim, problem.ts, ne.SCALE))
    dim = problem.dim
    z_prev = am.normalize_controls(problem.u_prev, lim, ne.SCALE)[0]
    duw = du if du.ndim == 2 else np.tile(du, (dim, 1))

    def f(zf):
        return float(ne.cost(problem, zf.reshape(1, dim, 2), w, "double")[0])

    def grad(zf):
        h = 1e-4
        Z = np.tile(zf, (2 * dim + 1, 1))
        Z[1:] += h * np.eye(2 * dim)
        c = ne.cost(problem, Z.reshape(-1, dim, 2), w, "double")
        return (c[1:] - c[0]) / h

    # rate constraints |z_j - z_{j-1}| <= du_j as linear inequalities A z + b >= 0
    rows, rhs = [], []
    for j in range(dim):
        for c in range(2):
            for sgn in (1.0, -1.0):
                a = np.zeros((dim, 2))
                a[j, c] = -sgn
                b = duw[j, c]
                if j > 0:
                    a[j - 1, c] = sgn
                else:
                    b += sgn * z_prev[c]
                rows.append(a.ravel())
                rhs.append(b)
    A, B = np.array(rows), np.array(rhs)
    cons = [{"type": "ineq", "fun": lambda zf: A @ zf + B, "jac": lambda zf: A}]
    starts = {"pso": ne.to_decision(pso_plan, problem)}
    if problem.warm_start is not None:
        starts["warm"] = ne.to_decision(problem.warm_start, problem)
    zero = am.project_controls(np.column_stack([np.full(problem.nh, problem.uref[0, 0]), np.zeros(problem.nh)]),
                               problem.u_prev, lim, problem.ts)
    starts["zero_kappa"] = ne.to_decision(zero, problem)
    best = (math.inf, None, None)
    for name, z0 in starts.items():
        r = minimize(f, z0.ravel(), jac=grad, method="SLSQP", bounds=[(-ne.SCALE, ne.SCALE)] * (2 * dim),
                     constraints=cons, options={"maxiter": maxiter, "ftol": 1e-10})
        feasible = np.all(A @ r.x + B >= -1e-6)
        if feasible and r.fun < best[0]:
            best = (float(r.fun), name, r.x.reshape(dim, 2))
    j_pso = f(starts["pso"].ravel())
    j_best = min(best[0], j_pso)  # J_opt is at most the PSO's own cost
    k_pso = float(pso_plan[0, 1])
    k_opt = float(am.denormalize_controls(ne.to_full(best[2][None], problem)[0], lim, ne.SCALE)[0, 1]) \
        if best[2] is not None else k_pso
    return {"j_pso": j_pso, "j_opt": j_best, "ratio": j_pso / j_best if j_best > 0 else float("nan"),
            "best_start": best[1], "dkappa0": k_pso - k_opt}
