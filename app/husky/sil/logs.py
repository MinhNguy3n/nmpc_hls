"""CSV log formats of a SIL run (one directory per run) and their writers/readers.

Files (all times in simulation seconds unless named *_wall):
    meta.json        configuration, seed, path, gain mode, clocks, RTF note
    ticks.csv        one row per control tick (decisions, published command, checks, C)
    transitions.csv  ownership transitions
    episodes.csv     path starts: tau0 and the odom-frame origin of each owned episode
    feedback.csv     platform/odom/filtered  (the controller's ONLY state input)
    wheel_odom.csv   platform/odom           (diagnostic only)
    truth.csv        Gazebo ground truth     (scoring only, never fed back)
    applied.csv      cmd_vel_out samples
Signal rows carry stamp, receive time, pose, twist and covariance diagonals where available.
"""

from __future__ import annotations

import csv
import json
from pathlib import Path
from typing import Any, Iterable

SIGNAL_FIELDS = ("stamp", "received", "x", "y", "yaw", "v", "omega",
                 "cov_x", "cov_y", "cov_yaw", "cov_vx", "cov_wz", "z", "tilt")  # z, tilt: ground truth only
TICK_FIELDS = ("t", "state", "publish_v", "publish_w", "publish_kind", "u_prev_v", "u_prev_w", "u_curr_v", "u_curr_w",
               "est_x", "est_y", "est_yaw", "odom_age", "solved", "accepted", "fitness",
               "viol_speed", "viol_omega", "viol_linear_slew", "viol_angular_slew",
               "viol_side_speed", "viol_side_accel", "fallback", "deadline_miss",
               "c_sim_prep", "c_sim_solve", "c_sim_check", "c_sim_total", "events",
               "c_wall_prep", "c_wall_solve", "c_wall_check", "c_wall_total", "note")
TRANSITION_FIELDS = ("t", "old", "new", "reason")
EPISODE_FIELDS = ("episode", "tau0", "origin_x", "origin_y", "origin_yaw")


def tick_row(record: Any) -> dict[str, Any]:
    pub = record.publish or (None, None)
    up = record.u_prev or (None, None)
    uc = record.u_curr or (None, None)
    est = record.estimate or (None, None, None)
    row = {"t": record.t, "state": record.state, "publish_v": pub[0], "publish_w": pub[1],
           "publish_kind": record.publish_kind, "u_prev_v": up[0], "u_prev_w": up[1], "u_curr_v": uc[0], "u_curr_w": uc[1],
           "est_x": est[0], "est_y": est[1], "est_yaw": est[2], "odom_age": record.odom_age,
           "solved": int(record.solved), "accepted": int(record.accepted), "fitness": record.fitness,
           "fallback": record.fallback, "deadline_miss": int(record.deadline_miss), "note": record.note}
    for key in ("speed", "omega", "linear_slew", "angular_slew", "side_speed", "side_accel"):
        row[f"viol_{key}"] = record.violations.get(key)
    for part in ("prep", "solve", "check", "total"):
        row[f"c_sim_{part}"] = record.c_sim.get(part)
        row[f"c_wall_{part}"] = record.c_wall.get(part)
    return row


class CsvLog:
    def __init__(self, path: Path, fields: Iterable[str]) -> None:
        self.fields = tuple(fields)
        self.handle = open(path, "w", newline="")
        self.writer = csv.DictWriter(self.handle, fieldnames=self.fields)
        self.writer.writeheader()

    def write(self, row: dict[str, Any]) -> None:
        self.writer.writerow({k: ("" if row.get(k) is None else row.get(k)) for k in self.fields})

    def close(self) -> None:
        self.handle.close()


def write_meta(path: Path, meta: dict[str, Any]) -> None:
    path.write_text(json.dumps(meta, indent=2, sort_keys=True, default=str))


def read_rows(path: Path) -> list[dict[str, Any]]:
    """Read a CSV; numeric fields become floats, empty fields None, text stays text."""
    rows = []
    if not Path(path).exists():
        return rows
    with open(path, newline="") as handle:
        for raw in csv.DictReader(handle):
            row: dict[str, Any] = {}
            for key, value in raw.items():
                if value == "":
                    row[key] = None
                    continue
                try:
                    row[key] = float(value)
                except ValueError:
                    row[key] = value
            rows.append(row)
    return rows
