"""Gazebo ground truth for SCORING ONLY: reads `gz topic -e -t /world/<world>/dynamic_pose/info`
(text gz.msgs.Pose_V, parsed by identify_effective_track.parse_pose_v_text) and hands the robot's
pose to a sink. The SIL controller never receives these values.

Needs GZ_IP=127.0.0.1 in the container (as for the identification tool) and no Gazebo Python
bindings. The stamp is the Gazebo simulation time of the message; `received` is the node clock.
"""

from __future__ import annotations

import subprocess
import threading
from typing import Callable, Iterable, TextIO

from . import _HUSKY  # noqa: F401
from identify_effective_track import parse_pose_v_text


def pump(lines: Iterable[str], name: str, now: Callable[[], float],
         sink: Callable[..., None]) -> int:
    """Feed parsed poses of `name` to sink(stamp, received, x, y, yaw, z, tilt); returns how many."""
    count = 0
    for stamp, poses in parse_pose_v_text(lines, full=True):
        if name in poses:
            sink(stamp, now(), *poses[name])
            count += 1
    return count


class GzTruthReader:
    def __init__(self, topic: str, name: str, now: Callable[[], float],
                 sink: Callable[..., None]) -> None:
        self.topic, self.name, self.now, self.sink = topic, name, now, sink
        self.process: subprocess.Popen[str] | None = None
        self.thread: threading.Thread | None = None

    def start(self, stream: TextIO | None = None) -> None:
        """Start `gz topic -e` (or read a given text stream, for tests) in a daemon thread."""
        if stream is None:
            self.process = subprocess.Popen(["gz", "topic", "-e", "-t", self.topic], stdout=subprocess.PIPE,
                                            stderr=subprocess.DEVNULL, text=True, bufsize=1)
            stream = self.process.stdout
        self.thread = threading.Thread(target=pump, args=(stream, self.name, self.now, self.sink), daemon=True)
        self.thread.start()

    def stop(self) -> None:
        if self.process is not None:
            self.process.terminate()
