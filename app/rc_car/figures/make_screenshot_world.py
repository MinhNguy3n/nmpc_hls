"""Screenshot-only Gazebo world variants (figures; SIMULATION ONLY, never used for results).

Takes a world SDF and adds, for off-screen rendering (gz sim -s --headless-rendering):
- the Sensors system with ogre2, a white background, no grid;
- static camera models that SAVE their frames as PNG (<save enabled="true">) into OUT/<camera>/;
- optionally the reference path drawn on the floor as a thin ribbon (visual only);
- optionally line outlines of the tracked area and the geofence (replacing filled boxes).
Nothing here changes physics: the added models are static and visual-only.

IMPORTANT (found 2026-11-22): the cameras render, and so save, ONLY while something subscribes to
their image topic; <save> alone does not trigger rendering. Start one subscriber per camera, e.g.
  ros2 run ros_gz_bridge parameter_bridge \
    "/world/<w>/model/cam_<name>/link/link/sensor/camera_<name>/image@sensor_msgs/msg/Image[gz.msgs.Image"
GPU EGL does not work in the nmpc-jazzy-sim image; software rendering (no --gpus) does.

    python3 make_screenshot_world.py --world IN.sdf --out-world OUT.sdf --save-dir /out \
        --camera overview:x,y,z,roll,pitch,yaw[,fov] ... [--path figure_eight] [--outline 10,10,1.0]
"""

from __future__ import annotations

import argparse
import math
from pathlib import Path
import re
import sys

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))

WIDTH, HEIGHT = 1920, 1080


def camera_model(name: str, pose: list[float], fov: float, save_dir: str, rate: float) -> str:
    x, y, z, r, p, yw = pose
    return f"""
    <model name="cam_{name}">
      <static>true</static>
      <pose>{x} {y} {z} {r} {p} {yw}</pose>
      <link name="link">
        <sensor name="camera_{name}" type="camera">
          <always_on>1</always_on>
          <update_rate>{rate}</update_rate>
          <camera>
            <horizontal_fov>{fov}</horizontal_fov>
            <image><width>{WIDTH}</width><height>{HEIGHT}</height><format>R8G8B8</format></image>
            <clip><near>0.03</near><far>200</far></clip>
            <save enabled="true"><path>{save_dir}/{name}</path></save>
          </camera>
        </sensor>
      </link>
    </model>"""


def strip(name: str, pts: list[tuple[float, float]], width: float, z: float, rgb: str, step: int = 1) -> str:
    """A polyline drawn as thin boxes (visual only)."""
    vis = []
    for i in range(0, len(pts) - step, step):
        (x0, y0), (x1, y1) = pts[i], pts[i + step]
        length = math.hypot(x1 - x0, y1 - y0)
        if length < 1e-9:
            continue
        cx, cy, yaw = 0.5 * (x0 + x1), 0.5 * (y0 + y1), math.atan2(y1 - y0, x1 - x0)
        vis.append(f"""        <visual name="{name}_{i}"><pose>{cx:.4f} {cy:.4f} {z} 0 0 {yaw:.5f}</pose>
          <geometry><box><size>{length + width:.4f} {width} 0.001</size></box></geometry>
          <material><ambient>{rgb} 1</ambient><diffuse>{rgb} 1</diffuse></material></visual>""")
    return f"""
    <model name="{name}">
      <static>true</static>
      <link name="link">
{chr(10).join(vis)}
      </link>
    </model>"""


def rect(w: float, h: float) -> list[tuple[float, float]]:
    return [(-w / 2, -h / 2), (w / 2, -h / 2), (w / 2, h / 2), (-w / 2, h / 2), (-w / 2, -h / 2)]


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--world", required=True)
    ap.add_argument("--out-world", required=True)
    ap.add_argument("--save-dir", required=True)
    ap.add_argument("--camera", action="append", default=[], help="name:x,y,z,roll,pitch,yaw[,fov]")
    ap.add_argument("--rate", type=float, default=2.0, help="camera frames per simulated second")
    ap.add_argument("--path", default=None, help="reference path to draw (app/rc_car/paths.py name)")
    ap.add_argument("--outline", default=None, help="length,width,margin: line outlines of the area and geofence")
    ap.add_argument("--remove-model", action="append", default=[], help="model names to drop (e.g. filled boxes)")
    ap.add_argument("--floor-visual", type=float, default=None,
                    help="replace the model 'floor' by a 100 m collision plane with a SIZE x SIZE visual (white background around)")
    ap.add_argument("--keep-scene", action="store_true", help="keep the world's own <scene> (indoor worlds)")
    ap.add_argument("--no-sensors-plugin", action="store_true", help="the world already loads the Sensors system")
    ap.add_argument("--husky-path", default=None,
                    help="kind,size,x0,y0,yaw0: an app/husky/sil/reference.py PathSpec composed with the start pose")
    args = ap.parse_args()

    sdf = Path(args.world).read_text()
    for name in args.remove_model:
        sdf = re.sub(rf'\s*<model name="{name}">.*?</model>', "", sdf, count=1, flags=re.S)
    extra = []
    if not args.no_sensors_plugin:
        extra.append('\n    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">'
                     "<render_engine>ogre2</render_engine></plugin>")
    if not args.keep_scene:
        sdf = re.sub(r"\s*<scene>.*?</scene>", "", sdf, flags=re.S)
        extra.append("\n    <scene><ambient>0.45 0.45 0.45 1</ambient><background>1 1 1 1</background>"
                     "<grid>false</grid><shadows>true</shadows></scene>")
    if args.floor_visual:
        sdf = re.sub(r'\s*<model name="floor">.*?</model>', "", sdf, count=1, flags=re.S)
        f = args.floor_visual
        extra.append(f"""
    <model name="floor">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry><plane><normal>0 0 1</normal><size>100 100</size></plane></geometry>
          <surface><friction><ode><mu>1.0</mu><mu2>1.0</mu2></ode></friction></surface>
        </collision>
        <visual name="visual">
          <geometry><plane><normal>0 0 1</normal><size>{f} {f}</size></plane></geometry>
          <material><ambient>0.82 0.82 0.80 1</ambient><diffuse>0.82 0.82 0.80 1</diffuse></material>
        </visual>
      </link>
    </model>""")
    for spec in args.camera:
        name, vals = spec.split(":")
        v = [float(x) for x in vals.split(",")]
        extra.append(camera_model(name, v[:6], v[6] if len(v) > 6 else 1.0, args.save_dir, args.rate))
    if args.outline:
        length, width, margin = (float(x) for x in args.outline.split(","))
        extra.append(strip("area_outline", rect(length, width), 0.05, 0.002, "0.05 0.25 0.65"))
        extra.append(strip("geofence_outline", rect(length - 2 * margin, width - 2 * margin), 0.04, 0.002, "0.8 0.25 0.05"))
    if args.path:
        import paths as P
        p = P.build(args.path, P.load_area())
        pts = list(zip(p.x[::5].tolist(), p.y[::5].tolist()))
        extra.append(strip("reference_path", pts, 0.03, 0.003, "0.05 0.05 0.05"))
    if args.husky_path:
        sys.path.insert(0, str(HERE.parents[1] / "husky"))
        import numpy as np
        from sil.reference import PathSpec, compose
        kind, size, x0, y0, yaw0 = args.husky_path.split(",")
        spec = PathSpec(kind=kind, speed=0.3, size=float(size))
        rel = spec.poses(np.linspace(0.0, spec.duration, 400))
        world = compose(np.array([float(x0), float(y0), float(yaw0)]), rel)
        extra.append(strip("reference_path", list(zip(world[:, 0].tolist(), world[:, 1].tolist())), 0.05, 0.004,
                           "0.05 0.05 0.05"))
    sdf = sdf.replace("</world>", "".join(extra) + "\n  </world>", 1)
    Path(args.out_world).write_text(sdf)
    print(args.out_world)
    return 0


if __name__ == "__main__":
    sys.exit(main())
