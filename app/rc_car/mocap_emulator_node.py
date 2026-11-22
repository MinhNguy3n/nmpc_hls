"""OptiTrack emulator node of the RC car simulation (the model is in mocap_emulator.py).

Source: the Gazebo truth of the car (`/world/<world>/dynamic_pose/info` through
gz_pose_stream.GzPoseStream, which hands over each pose as soon as it is parsed; needs
GZ_IP=127.0.0.1), stamp = capture time.
Output: `/mocap/<body>/pose` (geometry_msgs/PoseStamped, frame `mocap`). The HEADER STAMP IS THE
CAPTURE TIME, so a consumer computes the pose age as now - stamp. Due frames are published by a
timer every `period` s of sim time (default 2 ms; that much quantisation is added to the delay).
`mocap/stats` (std_msgs/String, JSON) at 1 Hz: frames, dropouts, spikes, and two SIMULATION
artifacts kept apart from the emulated delay: the source lag (Gazebo stamp -> the truth sample
parsed here, through `gz topic -e`) and the frames published later than their emulated arrival
(by more than 2 timer periods), which happens when the source lag exceeds the emulated delay.

`episode/start` (std_msgs/String, JSON {"t_start", "seed"}, episode.py): restarts the random
streams from the seed, relative to t_start (paired seeds across runs).

Parameters: `profile` (ideal | realistic_guess), `seed`, `world`, `body`, `chain_file`.
ALL PROFILE VALUES ARE ASSUMED.

    python3 app/rc_car/mocap_emulator_node.py --ros-args -p use_sim_time:=true
"""

from __future__ import annotations

import json
import math
from pathlib import Path
import sys
import threading

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import episode  # noqa: E402
import mocap_emulator as me  # noqa: E402
from gz_pose_stream import GzPoseStream  # noqa: E402


def main() -> None:
    import rclpy
    from rclpy.node import Node
    from geometry_msgs.msg import PoseStamped
    from std_msgs.msg import String

    class MocapNode(Node):
        def __init__(self) -> None:
            super().__init__("mocap_emulator")
            name = self.declare_parameter("profile", "realistic_guess").value
            seed = int(self.declare_parameter("seed", 0).value)
            world = self.declare_parameter("world", "lab_floor").value
            self.body = self.declare_parameter("body", "reely_new1").value
            chain = self.declare_parameter("chain_file", str(me.DEFAULT_CHAIN)).value
            period = float(self.declare_parameter("period", 0.002).value)
            self.profile = me.load_profile(name, chain)
            self.em = me.MocapEmulator(self.profile, seed)
            self.lock = threading.Lock()
            self.period = period
            self.source_lag: list[float] = []
            self.late = []  # (publish time, publish - emulated arrival) for frames > 2 periods late
            self.pub = self.create_publisher(PoseStamped, f"/mocap/{self.body}/pose", 20)
            self.pub_stats = self.create_publisher(String, "mocap/stats", 10)
            self.create_subscription(String, "episode/start", self.on_episode, 10)
            self.create_timer(period, self.tick)
            self.create_timer(1.0, self.stats)
            self.reader = GzPoseStream(f"/world/{world}/dynamic_pose/info", self.body, self.now, self.on_truth)
            self.reader.start()
            self.get_logger().info(f"profile {name} ({self.profile.status}, ASSUMED values): {self.profile}; seed {seed}")

        def now(self) -> float:
            return self.get_clock().now().nanoseconds * 1e-9

        def on_episode(self, msg) -> None:
            ep = episode.parse(msg.data)
            with self.lock:
                self.em.reset_episode(ep.t_start, ep.seed)
            self.get_logger().info(f"episode: t_start {ep.t_start:.3f}, seed {ep.seed}")

        def on_truth(self, stamp, received, x, y, yaw) -> None:
            with self.lock:
                self.source_lag.append(received - stamp)
                self.em.on_truth(stamp, x, y, yaw)

        def tick(self) -> None:
            now = self.now()
            with self.lock:
                frames = self.em.due(now)
                self.late += [(now, now - f.arrival) for f in frames if now - f.arrival > 2 * self.period]
            for f in frames:
                msg = PoseStamped()
                sec = int(math.floor(f.capture))
                msg.header.stamp.sec, msg.header.stamp.nanosec = sec, int(round((f.capture - sec) * 1e9))
                msg.header.frame_id = "mocap"
                msg.pose.position.x, msg.pose.position.y = f.x, f.y
                msg.pose.orientation.z, msg.pose.orientation.w = math.sin(f.yaw / 2), math.cos(f.yaw / 2)
                self.pub.publish(msg)

        def stats(self) -> None:
            with self.lock:
                lag = np.array(self.source_lag) if self.source_lag else np.zeros(1)
                stats = dict(self.em.stats, source_lag_ms={
                    "n": len(self.source_lag), "median": float(np.median(lag) * 1e3),
                    "p95": float(np.percentile(lag, 95) * 1e3), "max": float(lag.max() * 1e3),
                    "n_over_20ms": int(np.sum(lag > 0.02))},
                    late_frames={"n": len(self.late), "max_ms": max((d for _, d in self.late), default=0.0) * 1e3,
                                 "times": [round(t, 3) for t, _ in self.late[:50]]})
                self.pub_stats.publish(String(data=json.dumps(stats)))

    rclpy.init()
    node = MocapNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.reader.stop()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
