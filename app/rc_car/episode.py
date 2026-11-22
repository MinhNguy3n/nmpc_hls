"""Episode start message of the RC car runs (paired seeds, phase 3 condition 3).

The run harness publishes `episode/start` (std_msgs/String) with JSON {"t_start": <sim s>,
"seed": <int>} shortly BEFORE t_start. The mocap emulator restarts its random streams and the
actuator places its PWM hold phases relative to t_start, both from the seed. So for a given seed,
two runs (e.g. the two controller arms) get identical mocap noise, jitter, spike and dropout
streams per frame, and identical hold phases, relative to their episode starts.
"""

from __future__ import annotations

from dataclasses import dataclass
import json
import math


@dataclass(frozen=True)
class Episode:
    t_start: float
    seed: int

    def to_json(self) -> str:
        return json.dumps({"t_start": self.t_start, "seed": self.seed})


def parse(text: str) -> Episode:
    d = json.loads(text)
    t, seed = float(d["t_start"]), int(d["seed"])
    if not math.isfinite(t) or seed < 0:
        raise ValueError(f"bad episode message {text!r}")
    return Episode(t, seed)
