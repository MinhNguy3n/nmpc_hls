"""OptiTrack emulator of the RC car simulation (plain Python, tested in test_mocap_emulator.py).

Source: the Gazebo ground-truth pose of the car (the rear-axle midpoint), whose Gazebo stamp is
taken as the CAPTURE time. Each frame:
  1. decimation to `rate_hz`;
  2. dropouts: with probability `drop_prob` a burst of 1..drop_burst_max frames is lost;
  3. noise: Gaussian on x, y (`noise_xy`) and yaw (`noise_yaw`, wrapped to [-pi, pi));
  4. delay = T_mocap (processing and streaming) + Wi-Fi: wifi_base + exponential jitter of mean
     wifi_jitter_mean + with probability spike_prob a spike, uniform in [spike_min, spike_max].
     The two parts are kept separately in each frame.
Frames are delivered in ARRIVAL order (a spike can let a later frame overtake an earlier one, as
with UDP over Wi-Fi). The frame keeps its capture stamp; the ROS node puts it in the header so
the controller can compute the pose age, and the freshness gate works on that age.

Paired seeds (phase 3 condition 3): each random component (noise, jitter, spike, dropout) has
its own stream from numpy.random.SeedSequence(seed), and every frame takes the same fixed number
of draws from each stream whether or not they are used. reset_episode(t_start, seed) restarts
the streams, so frame k of an episode (the k-th frame captured at or after t_start) gets
identical noise, jitter, spike and dropout values for a given seed, whatever ran before. Frames
captured before t_start but processed after the reset use a separate stream.

The ROS node mocap_emulator_node.py publishes the frames as /mocap/<body>/pose. The controller
uses this pose; scoring keeps using the Gazebo truth. All profile values are assumed
(params/command_chain.yaml, mocap_profiles).
"""

from __future__ import annotations

from dataclasses import dataclass
import heapq
import math
from pathlib import Path

import numpy as np
import yaml

DEFAULT_CHAIN = Path(__file__).resolve().parent / "params" / "command_chain.yaml"
FIELDS = ("rate_hz", "T_mocap", "wifi_base", "wifi_jitter_mean", "spike_prob", "spike_min", "spike_max",
          "noise_xy", "noise_yaw", "drop_prob", "drop_burst_max")


@dataclass(frozen=True)
class MocapProfile:
    name: str
    rate_hz: float
    T_mocap: float
    wifi_base: float
    wifi_jitter_mean: float
    spike_prob: float
    spike_min: float
    spike_max: float
    noise_xy: float
    noise_yaw: float
    drop_prob: float
    drop_burst_max: int
    status: str = "placeholder"

    def __post_init__(self) -> None:
        if not self.rate_hz > 0:
            raise ValueError("rate_hz must be > 0")
        for f in FIELDS[1:-1]:
            v = getattr(self, f)
            if not (math.isfinite(v) and v >= 0):
                raise ValueError(f"{self.name}.{f} must be finite and >= 0")
        if self.spike_prob > 1 or self.drop_prob > 1 or self.spike_max < self.spike_min or self.drop_burst_max < 1:
            raise ValueError(f"{self.name}: inconsistent probabilities, spike range or burst length")


def load_profiles(path: str | Path = DEFAULT_CHAIN) -> dict[str, MocapProfile]:
    data = yaml.safe_load(Path(path).read_text())["mocap_profiles"]
    out = {}
    for name, e in data.items():
        missing = [f for f in FIELDS if f not in e]
        if missing:
            raise ValueError(f"mocap profile {name} lacks {missing}")
        values = {f: float(e[f]) for f in FIELDS[:-1]}
        out[name] = MocapProfile(name=name, drop_burst_max=int(e["drop_burst_max"]),
                                 status=str(e.get("status", "placeholder")), **values)
    return out


def load_profile(name: str, path: str | Path = DEFAULT_CHAIN) -> MocapProfile:
    profiles = load_profiles(path)
    if name not in profiles:
        raise KeyError(f"unknown mocap profile {name!r}; have {sorted(profiles)}")
    return profiles[name]


@dataclass(frozen=True)
class Frame:
    capture: float   # capture time (Gazebo stamp of the truth sample), s
    arrival: float   # time the frame reaches the controller, s
    x: float
    y: float
    yaw: float
    mocap_delay: float  # T_mocap part
    wifi_delay: float   # Wi-Fi part (base + jitter + spike)
    spike: bool


STREAMS = ("noise", "jitter", "spike", "drop", "pre_episode")


class MocapEmulator:
    def __init__(self, profile: MocapProfile, seed: int = 0) -> None:
        self.p = profile
        self.queue: list[tuple[float, int, Frame]] = []  # heap on arrival
        self.seq = 0
        self.stats = {"truth": 0, "frames": 0, "dropped": 0, "spikes": 0}
        self.reset_episode(-math.inf, seed)

    def reset_episode(self, t_start: float, seed: int) -> None:
        """Restart the random streams and the frame state for an episode starting at t_start."""
        children = np.random.SeedSequence(int(seed)).spawn(len(STREAMS))
        self.rng = {name: np.random.default_rng(c) for name, c in zip(STREAMS, children)}
        self.seed, self.t_start = int(seed), t_start
        self.next_frame = -math.inf
        self.burst_left = 0
        self.episode_frames = 0
        self.draws: list[tuple] = []  # per episode frame: the raw draws (tests, pairing check)

    def _draw(self, pre_episode: bool) -> tuple:
        """The fixed per-frame draws: 3 normals, 1 exponential, 2 + 2 uniforms."""
        if pre_episode:
            g = self.rng["pre_episode"]
            return (*g.standard_normal(3), g.standard_exponential(), *g.random(2), *g.random(2))
        r = self.rng
        return (*r["noise"].standard_normal(3), r["jitter"].standard_exponential(), *r["spike"].random(2),
                *r["drop"].random(2))

    def on_truth(self, capture: float, x: float, y: float, yaw: float) -> None:
        self.stats["truth"] += 1
        period = 1.0 / self.p.rate_hz
        if capture < self.next_frame - 1e-9:
            return  # decimation
        self.next_frame = max(self.next_frame + period, capture + 0.5 * period) if math.isfinite(self.next_frame) \
            else capture + period
        pre = capture < self.t_start
        nx, ny, nyaw, jit, u_spike, u_size, u_drop, u_len = self._draw(pre)
        if not pre:
            self.draws.append((nx, ny, nyaw, jit, u_spike, u_size, u_drop, u_len))
            self.episode_frames += 1
        if self.burst_left == 0 and u_drop < self.p.drop_prob:
            self.burst_left = 1 + min(int(u_len * self.p.drop_burst_max), self.p.drop_burst_max - 1)
        if self.burst_left > 0:
            self.burst_left -= 1
            self.stats["dropped"] += 1
            return
        x += self.p.noise_xy * float(nx)
        y += self.p.noise_xy * float(ny)
        yaw += self.p.noise_yaw * float(nyaw)
        if not -math.pi <= yaw < math.pi:
            yaw = (yaw + math.pi) % (2 * math.pi) - math.pi
        wifi = self.p.wifi_base + self.p.wifi_jitter_mean * float(jit)
        spike = u_spike < self.p.spike_prob
        if spike:
            wifi += self.p.spike_min + (self.p.spike_max - self.p.spike_min) * float(u_size)
            self.stats["spikes"] += 1
        frame = Frame(capture, capture + self.p.T_mocap + wifi, x, y, yaw, self.p.T_mocap, wifi, spike)
        heapq.heappush(self.queue, (frame.arrival, self.seq, frame))
        self.seq += 1
        self.stats["frames"] += 1

    def due(self, now: float) -> list[Frame]:
        """Frames whose arrival time is <= now, in arrival order."""
        out = []
        while self.queue and self.queue[0][0] <= now + 1e-12:
            out.append(heapq.heappop(self.queue)[2])
        return out
