"""Low-latency Gazebo pose stream of one model, for the OptiTrack emulator (plain Python).

`gz topic -e -t /world/<w>/dynamic_pose/info` prints gz.msgs.Pose_V as protobuf text. The Husky
parser (app/husky/identify_effective_track.parse_pose_v_text, used by sil/gz_truth.py for
scoring) yields a message only when the NEXT message's `header {` starts, i.e. one publication
period (10 ms at 100 Hz) late. That is harmless for scoring, which uses the stamps, but it would
add 10 ms of simulation artifact to the emulated mocap delay (measured 2026-11-22: source lag
median 11 ms). This parser yields the model's pose as soon as its `pose { ... }` block closes.

Protobuf text omits zero-valued fields, so a missing coordinate is 0.
"""

from __future__ import annotations

import math
import subprocess
import threading
from typing import Callable, Iterable, Iterator, TextIO


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def stream_model_poses(lines: Iterable[str], name: str) -> Iterator[tuple[float, float, float, float]]:
    """(stamp s, x, y, yaw) of model `name`, yielded when its pose block closes."""
    stack: list[str] = []
    stamp = {"sec": 0, "nsec": 0}
    pose: dict = {}
    for raw in lines:
        line = raw.strip()
        if not line:
            continue
        if line.endswith("{"):
            block = line[:-1].strip()
            if not stack and block == "header":
                stamp = {"sec": 0, "nsec": 0}
            if not stack and block == "pose":
                pose = {}
            stack.append(block)
            continue
        if line == "}":
            closed = stack.pop() if stack else ""
            if not stack and closed == "pose" and pose.get("name") == name:
                p, q = pose.get("position", {}), pose.get("orientation", {})
                yield (stamp["sec"] + stamp["nsec"] * 1e-9, p.get("x", 0.0), p.get("y", 0.0),
                       yaw_from_quaternion(q.get("x", 0.0), q.get("y", 0.0), q.get("z", 0.0), q.get("w", 0.0)))
            continue
        key, _, value = line.partition(":")
        key, value = key.strip(), value.strip()
        if stack == ["header", "stamp"] and key in ("sec", "nsec"):
            stamp[key] = int(value)
        elif stack == ["pose"] and key == "name":
            pose["name"] = value.strip('"')
        elif len(stack) == 2 and stack[0] == "pose" and stack[1] in ("position", "orientation"):
            pose.setdefault(stack[1], {})[key] = float(value)


class GzPoseStream:
    """Runs `gz topic -e` (or reads a given stream, for tests) in a daemon thread and calls
    sink(stamp, received, x, y, yaw) for every pose of `name`; `received` = now()."""

    def __init__(self, topic: str, name: str, now: Callable[[], float], sink: Callable[..., None]) -> None:
        self.topic, self.name, self.now, self.sink = topic, name, now, sink
        self.process: subprocess.Popen[str] | None = None
        self.thread: threading.Thread | None = None

    def _pump(self, stream: Iterable[str]) -> None:
        for stamp, x, y, yaw in stream_model_poses(stream, self.name):
            self.sink(stamp, self.now(), x, y, yaw)

    def start(self, stream: TextIO | None = None) -> None:
        if stream is None:
            self.process = subprocess.Popen(["gz", "topic", "-e", "-t", self.topic], stdout=subprocess.PIPE,
                                            stderr=subprocess.DEVNULL, text=True, bufsize=1)
            stream = self.process.stdout
        self.thread = threading.Thread(target=self._pump, args=(stream,), daemon=True)
        self.thread.start()

    def stop(self) -> None:
        if self.process is not None:
            self.process.terminate()
