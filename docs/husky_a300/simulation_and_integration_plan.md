# Simulation and integration plan: unicycle MPC-PSO on the Husky A300 (TurtleBot 4 deferred)

Status: plan only (2026-10-25), revised after the GPT-6 review (same day). Companion to
`unicycle_mpc_pso_model.md` (the "model note").

**Scope and order** (revised after the second GPT-6 review, 2026-10-26):

1. Finish the A300 simulation checks V1–V13 (§2.1), including the freshness and stamp quality of `cmd_vel_out` and odometry, and the effective-track identification against Gazebo ground truth. The simulated track validates the measurement pipeline only, not the real robot.
2. The command-ownership / safety state machine, tested independently of the optimizer (`app/husky/command_ownership.py`; model note §7.2).
3. The Python-backend A300 SIL screen (stage 3): quality and constraint violations recorded separately.
4. The PC C-model backend on the same scenarios, compared with the Python reference.
5. Only then the isolated RK4 HLS prototype at the screened configuration: C-sim, C/RTL co-sim, synthesis and timing, KV260 HIL (stage 4).
6. A300 physical tests at slow mode, only after gates G0 and G1 and the user's safety plan (stage 5).
7. The interpolated chord (B3) only if measured timing or tracking justifies it.
8. **The TurtleBot 4 port is a separate, later scope (§6).**

The branch `husky-unicycle-mpc` stays isolated until the SIL results and the model/command-interface
decisions are reviewed.

**Every command below is TO BE RUN BY THE USER.** The agent that wrote this plan did not:

- install packages;
- pull or run images;
- download large files;
- contact the KV260;
- command any robot.

The `sudo` steps are marked. Physical tests (stage 5, and the deferred TB4 indoor stage) are
**gated on gates G0/G1 (§4.0) and on the user's own safety plan**, and are never run by an agent.

## 0. Host facts and constraints (checked read-only on 2026-10-25)

| Item | Value | Consequence |
| --- | --- | --- |
| OS | Ubuntu 22.04.5 LTS; `/opt/ros/humble`, `/opt/ros/rolling`; `gz` and `ign` binaries present | Jazzy targets Ubuntu 24.04, so run Jazzy + Gazebo Harmonic in Docker |
| Docker | 29.8.2; overlay2 storage in `/var/lib/docker`; user in the `docker` group; runtimes `runc` only | `--gpus` needs the NVIDIA container toolkit hook (§1.3) |
| GPU | RTX 3060 Laptop, driver 580.65.06; `nvidia-ctk` installed | GPU rendering in the container is possible after the check in §1.3 |
| Display | X11 session (`DISPLAY=:1`, no Wayland) | X11 socket forwarding |
| RAM | 15 GB | One simulator at a time; Gazebo plus RViz plus the MPC fit, two robots at once may not |
| Disk | **66 GB free** (user's figure, 2026-10-25; it was 43–46 GB at the first check) | Budget **8–10 GB** (§1.1). Existing images: `xilinx/vitis-ai-pytorch-gpu` 34.1 GB, `xilinx/vitis-ai-gpu-pytorch-base` 13.2 GB, `xilinx/finn` 9.3 GB. Removing any of them is the user's decision. |
| KV260 | `10.42.0.29`, Kria Ubuntu 22.04 + PYNQ, Python 3.10, Ethernet | HIL over TCP, not DDS (§3.4) |

## 1. Jazzy + Gazebo Harmonic in Docker (to be run by the user)

### 1.1 Image choice and disk

**Base image:** `osrf/ros:jazzy-desktop-full` (Ubuntu 24.04 "noble").

- Per REP-2001, Jazzy `desktop_full` = desktop + perception + **simulation** (`ros_gz_bridge`, `ros_gz_sim`, `ros_gz_image`, `ros_gz_interfaces`) + `ros_gz_sim_demos`. This brings Gazebo Harmonic, the Jazzy pairing.
- Docker Hub lists the amd64 image at **1.42 GB compressed** (amd64, tag updated 2026-09-10). Unpacked it is several GB; the exact figure comes from `docker image ls` after the pull.
- The smaller `jazzy-simulation` tag (0.73 GB compressed) lacks RViz and the desktop tools.

The image adds the binary package the Clearpath simulator documents:

- `ros-jazzy-clearpath-simulator` ([Clearpath simulator install](https://docs.clearpathrobotics.com/docs/ros/tutorials/simulator/install)).
- *(deferred: TB4 scope)* `ros-jazzy-turtlebot4-simulator` and `ros-jazzy-irobot-create-nodes` ([TB4 simulator](https://turtlebot.github.io/turtlebot4-user-manual/software/turtlebot4_simulator.html)) are a commented-out line in the Dockerfile, added only when the TB4 scope starts.

**Disk budget (an estimate, to be checked):**

- base image: 1.42 GB compressed, several GB unpacked;
- simulator packages and dependencies about 1–2 GB;
- temporary build layers during `docker build` (apt lists and archives before cleanup);
- Gazebo Fuel model cache (downloaded at first launch of the worlds) up to a few hundred MB;
- logs and bags.

Plan for **8–10 GB in total**, including the Gazebo model cache and the temporary build layers.
With 66 GB free this fits; stop if `df -h /` falls below 25 GB free.

```bash
# On the host, before anything else (read-only checks)
df -h / ; docker system df
```

### 1.2 Dockerfile

Save it as `~/nmpc_sim/docker/Dockerfile` on the host. It is not part of the repository.

```dockerfile
# Jazzy + Gazebo Harmonic + Clearpath simulator, for an Ubuntu 22.04 host.
FROM osrf/ros:jazzy-desktop-full
ARG DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-jazzy-clearpath-simulator \
        ros-jazzy-teleop-twist-keyboard ros-jazzy-ros2controlcli \
        python3-numpy python3-pytest \
    && rm -rf /var/lib/apt/lists/*
# Deferred TB4 scope: uncomment when that scope starts (adds the TurtleBot 4 simulator).
# RUN apt-get update && apt-get install -y --no-install-recommends \
#         ros-jazzy-turtlebot4-simulator ros-jazzy-irobot-create-nodes && rm -rf /var/lib/apt/lists/*
# Ubuntu 24.04 images ship user 'ubuntu' (UID 1000) = the host user's UID here; create it if absent.
RUN id -u ubuntu >/dev/null 2>&1 || useradd -m -u 1000 -s /bin/bash ubuntu
USER ubuntu
WORKDIR /home/ubuntu
RUN echo 'source /opt/ros/jazzy/setup.bash' >> ~/.bashrc
```

### 1.3 Build, GPU check and run

```bash
# 1. Pull and build (downloads ~1.4 GB compressed + the simulator packages)
mkdir -p ~/nmpc_sim/docker ~/nmpc_sim/clearpath ~/nmpc_sim/logs
docker pull osrf/ros:jazzy-desktop-full
docker build -t nmpc-jazzy-sim:2026-10 ~/nmpc_sim/docker
docker image ls nmpc-jazzy-sim ; df -h /

# 2. GPU in containers. `docker info` lists only runc, so first check whether --gpus works:
docker run --rm --gpus all nmpc-jazzy-sim:2026-10 nvidia-smi
#    If that fails with "could not select device driver", register the NVIDIA runtime (sudo, user):
#    sudo nvidia-ctk runtime configure --runtime=docker && sudo systemctl restart docker

# 3. Allow the container's local user on the X server (narrower than "xhost +local:")
xhost +SI:localuser:$(id -un)

# 4. Run (host network: DDS discovery inside the container and direct TCP to the KV260)
docker run -it --rm --name nmpc-jazzy \
  --gpus all -e NVIDIA_DRIVER_CAPABILITIES=all \
  -e __NV_PRIME_RENDER_OFFLOAD=1 -e __GLX_VENDOR_LIBRARY_NAME=nvidia \
  --network host --ipc host \
  -e DISPLAY=$DISPLAY -e QT_X11_NO_MITSHM=1 -v /tmp/.X11-unix:/tmp/.X11-unix:ro \
  -e ROS_DOMAIN_ID=42 -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
  -e GZ_IP=127.0.0.1 -e XDG_RUNTIME_DIR=/tmp/runtime-ubuntu \
  -v ~/nmpc_sim/clearpath:/home/ubuntu/clearpath \
  -v ~/nmpc_sim/logs:/home/ubuntu/logs \
  -v ~/Documents/UTU-Doctorate/nmpc_hls:/home/ubuntu/nmpc_hls:ro \
  nmpc-jazzy-sim:2026-10 bash

# 5. Inside the container: a quick rendering check
mkdir -p $XDG_RUNTIME_DIR && chmod 700 $XDG_RUNTIME_DIR
glxinfo -B 2>/dev/null | grep -i "renderer" || echo "install mesa-utils to check, or just start gz"
gz sim -v 3 shapes.sdf   # close it after it renders
```

**Notes:**

- **`GZ_IP=127.0.0.1` is required** (verified by the user on 2026-10-26). Gazebo Harmonic runs the server and the GUI as separate processes that find each other through gz-transport multicast discovery. With `--network host` on this PC (several interfaces, including the KV260 link on 10.42.0.x), discovery did not work: `gz sim -v 3 shapes.sdf` printed "Waited for 10s for a subscriber to [/gazebo/starting_world] and got none" and no world appeared. Pinning gz-transport to loopback fixed it. Every shell that runs a Gazebo process needs it (`docker exec` shells inherit it from `docker run -e`). `XDG_RUNTIME_DIR` only silences a Qt warning.

- **ROS domain.** `ROS_DOMAIN_ID=42` keeps the Jazzy graph apart from any Humble or Rolling nodes on the host. Mixing distributions in one DDS graph is not supported, so do not run host ROS nodes on domain 42. Clearpath's simulate page: every terminal must share `ROS_DOMAIN_ID` and `RMW_IMPLEMENTATION`.
  - Open further terminals with `docker exec -it nmpc-jazzy bash`.
- **Hybrid graphics.** The `__NV_PRIME_*` variables request NVIDIA render offload on a hybrid-graphics laptop.
  - If Gazebo falls back to software rendering, check `nvidia-smi` inside the container first.
  - As a last resort, use `LIBGL_ALWAYS_SOFTWARE=1`. It is slow, but enough for headless SIL with `gz sim -s`.
- **Headless runs** (no GUI, lower load, for automated SIL): the launch files below accept a headless/GUI argument only where documented. Otherwise run the server alone with `gz sim -s` and RViz off.
- **Mounts.** The repository is mounted **read-only**. Logs go to `~/nmpc_sim/logs` on the host.
- **Cleanup when done:** `docker image rm nmpc-jazzy-sim:2026-10 osrf/ros:jazzy-desktop-full`, then `docker builder prune`.

## 2. Simulators

### 2.1 Husky A300 in `clearpath_simulator` (Gazebo Harmonic)

**Example `~/nmpc_sim/clearpath/robot.yaml`.** It follows the official `clearpath_config` jazzy
samples `a300/a300_default.yaml` and `a300/a300_outline.yaml`
([samples](https://github.com/clearpathrobotics/clearpath_config/tree/jazzy/clearpath_config/sample/a300)):

- the serial format is `a300-00000` ([serial docs](https://docs.clearpathrobotics.com/docs/ros/config/yaml/serial));
- no `extras.ros_parameters` overrides, so the shipped controller limits apply;
- no extra sensors besides the IMU, to keep the simulation light.

```yaml
serial_number: a300-00000
version: 0
system:
  hosts:
    - hostname: cpr-a300-00000
      ip: 192.168.131.1
  ros2:
    namespace: a300_00000
    domain_id: 42
    middleware:
      implementation: rmw_fastrtps_cpp
platform:
  controller: ps4
  battery:
    model: S_24V20_U1
    configuration: S1P2
  drivetrain:
    control: diff_4wd
    wheels:
      front: outdoor
      rear: outdoor
  attachments:
    - name: front_bumper
      type: a300.bumper
      parent: front_bumper_mount
    - name: rear_bumper
      type: a300.bumper
      parent: rear_bumper_mount
    - name: top_plate
      type: a300.top_plate
  mcu:
    protocol: proton
sensors:
  imu:
    - model: phidgets_spatial
      urdf_enabled: true
      launch_enabled: true
      parent: base_link
      xyz: [0.059, 0.0, 0.161275]
      rpy: [0.0, 0.0, 0.0]
      ros_parameters:
        phidgets_spatial:
          data_interval_ms: 20
```

`domain_id: 42` matches the container's `ROS_DOMAIN_ID`. Confirm that the generator accepts it,
because the outline sample uses `0`.

**Bring-up** (inside the container; commands from the Clearpath simulate page):

```bash
ros2 launch clearpath_gz simulation.launch.py setup_path:=$HOME/clearpath/ world:=warehouse rviz:=false
# spawn pose: ... x:=0.0 y:=0.0 yaw:=0.0 ; worlds include warehouse (default), office, construction
```

**A300 support verification.** A300 simulation is likely but not explicitly documented. Run it
before any MPC work. Each check has a pass criterion:

| # | Command (second terminal: `docker exec -it nmpc-jazzy bash`) | Pass if |
| --- | --- | --- |
| V1 | `apt-cache policy ros-jazzy-clearpath-simulator ros-jazzy-clearpath-config ros-jazzy-clearpath-platform-description` | Installed versions recorded (changelogs: clearpath_gz 2.9.4, 2026-08-07) |
| V2 | `ls ~/clearpath` after launch | The generator wrote the URDF/launch/param files for `a300-00000` without errors |
| V3 | `ros2 node list \| grep a300_00000` | Platform nodes appear, including `controller_manager` |
| V4 | `ros2 control list_controllers -c /a300_00000/controller_manager` | `platform_velocity_controller` (diff_drive_controller) is active. If not, check the `gazebo_controllers` path: the URDF default `config/a300/control.yaml` is absent on the jazzy branch. |
| V5 | `ros2 topic info -v /a300_00000/cmd_vel` | Type `geometry_msgs/msg/TwistStamped`, subscribed by `twist_mux` |
| V6 | `ros2 param get /a300_00000/platform_velocity_controller wheel_separation_multiplier` (and `wheel_separation`, `linear.x.max_acceleration`, `angular.z.max_velocity`, `cmd_vel_timeout`) | The values match model note §6.1, or the differences are recorded |
| V7 | `ros2 topic hz /a300_00000/platform/odom` and `/a300_00000/platform/odom/filtered` | Rates recorded (controller `publish_rate` 50 Hz expected for `platform/odom`) |
| V8 | `ros2 topic pub -r 10 -t 30 /a300_00000/cmd_vel geometry_msgs/msg/TwistStamped "{twist: {linear: {x: 0.3}}}"` | The robot moves +x in odom. Stops ≤ 0.5 s after the publisher ends. |
| V9 | Same with `"{twist: {angular: {z: 0.5}}}"` | Yaw increases (CCW). The robot stays in place. |
| V10 | `ros2 topic list \| grep cmd_vel`; `ros2 topic echo /a300_00000/platform/cmd_vel --field header.stamp` while V8 publishes with a known stamp | **Command-ownership prerequisites:** the `twist_mux` output passes the input's `header.stamp` and twist through unchanged; the controller's limited command topic (`publish_limited_velocity: true`, for example `.../platform_velocity_controller/cmd_vel_out`) exists and its name and type are recorded. If the stamp is not passed through, the ownership test of model note §7.2 is redesigned before the node leaves stage 2. |
| V12 | See "V12 commands" below | Find whether `twist_mux` exposes its **selected source** (diagnostics, a topic, or a parameter). If it does, record the topic, type, rate and the value for each input; the ownership state machine then monitors it directly (`use_source_signal`). If not, record that ownership relies on the stamp match and the lock topics alone. |
| V13 | See "V13 commands" below | **Freshness and stamp quality** of `platform_velocity_controller/cmd_vel_out`, `platform/odom`, `platform/odom/filtered` and `platform/cmd_vel`: rate, stamp-interval jitter, age (receive time − stamp) median/p95/max, non-monotonic stamps. Pass if the ages stay well inside the state machine's bounds (`cmd_vel_out` 0.06 s, mux 0.12 s, odometry 3 periods) with no non-monotonic stamps; otherwise the bounds are re-derived from the measurements. |
| V11 | Publish on `/a300_00000/joy_teleop/cmd_vel` (priority 10) while V8 runs on `cmd_vel` (priority 1); then `ros2 topic pub --once /a300_00000/platform/safety_stop std_msgs/msg/Bool "{data: true}"` (if the topic exists) | The mux output switches to the teleop command (the MPC must then report "foreign"); the `safety_stop` lock blocks all commands. Record the lock topic types. |

**Results in simulation (user, 2026-10-26; image built from the Dockerfile above before `ros2controlcli` was added):**

| Check | Result |
| --- | --- |
| V3 | Pass: `controller_manager`, `platform_velocity_controller`, `gz_ros_control`, `twist_mux`, `ekf_node`, `cmd_vel_bridge`, `joint_state_broadcaster`, `robot_state_publisher` and the IMU bridge are up. |
| V4 | Pass: `platform_velocity_controller` and `joint_state_broadcaster` are `active` (checked with `ros2 service call /a300_00000/controller_manager/list_controllers controller_manager_msgs/srv/ListControllers {}`, because `ros2 control` was missing from the image; now added to the Dockerfile). The URDF controller-path risk did not materialize in this image. |
| V5 | Pass: `geometry_msgs/msg/TwistStamped`; one subscriber, `twist_mux` (QoS best effort, so a reliable publisher is compatible). |
| V6 | `wheel_separation_multiplier` = 1.75 in simulation (the Jazzy default; the real robot is still to be checked under G0). |
| V7 | `platform/odom` at about 47.6 Hz on the wall clock. The `cmd_vel_out` stamps (V10) are about 21 ms apart in simulation time, so the controller itself runs at about 47.6 Hz in simulation time (50 Hz quantized to the physics step, most likely), not 50 Hz at a reduced real-time factor. |
| V8 | Pass (sign and limits). `platform/odom` `twist.linear.x` ramps 0 → 0.300 m/s in about 0.25 s and back to 0 in about 0.17 s after the publisher stops. The ramps are jerk-limited: the largest step is 0.042 m/s per 20 ms control period (about 2.1 m/s², matching `linear.x.max_acceleration: 2.0`), and braking peaks at about 3.0 m/s² (below `max_deceleration` 4.0). The log does not show when the publisher ended, so the 0.5 s `cmd_vel_timeout` itself is not confirmed. `pose.position.x` fell from 1.944 to 1.814 m because the robot's heading was about 97° after earlier turns. Confirm the odom-frame direction from the full pose (x, y, yaw), or from a reset world. |
| V9 | Pass. `twist.angular.z` ramps to +0.500 rad/s (CCW) and the robot turns on the spot (observed). The largest step is 0.084 rad/s per period (about 4.0 rad/s², equal to `angular.z.max_acceleration: 4.0`). |
| V10 | **Pass.** `twist_mux` passes `header.stamp` through unchanged: publishing `{header: {stamp: {sec: 12345, nanosec: 678}}}` on `cmd_vel` gave `sec: 12345, nanosec: 678` on `platform/cmd_vel` (the mux output). An earlier test with a zero stamp was inconclusive, and `stamp: now` cannot distinguish pass-through from re-stamping. `cmd_vel` topics present: `/a300_00000/cmd_vel`, `joy_teleop/cmd_vel`, `rc_teleop/cmd_vel`, `twist_marker_server/cmd_vel`, `platform/cmd_vel`, and `platform_velocity_controller/cmd_vel_out` (type `geometry_msgs/msg/TwistStamped`, one publisher). **`cmd_vel_out` is re-stamped by the controller** with its own (simulation) time about every 21 ms. Consequences for the node: ownership is checked by stamp on `platform/cmd_vel`; `u_prev` is the latest `cmd_vel_out` sample (the applied, limited command), aligned by its own stamp, not matched by stamp. The node runs with `use_sim_time:=true` in simulation so that both share one clock. |
| V11 | Safety lock: **pass.** `ros2 topic pub --once /a300_00000/platform/safety_stop std_msgs/msg/Bool "{data: true}"` stopped the robot and keeps it stopped (latched) until `{data: false}` is published. The MPC node must treat an engaged lock as "not in control". Teleop takeover: **pass, with one leak.** While `cmd_vel` published `sec: 12345` and `joy_teleop/cmd_vel` published `sec: 999` (both 10 Hz), the mux output switched to `sec: 999`, then back to `sec: 12345` after teleop stopped. **One `sec: 12345` message got through in the middle of the teleop period.** Cause unknown: V12 shows the `joy` timeout is 0.5 s, so 10 Hz publish jitter does not explain it (an age judgement under simulated time is a guess). Consequences: the ownership gate needs hysteresis (one matching stamp during an override must not re-arm the MPC; require an explicit re-arm or N consecutive owned cycles), and the MPC publishes nothing while it does not own the channel, so a mux leak cannot apply an MPC command. |
| V12 | **No selected-source signal.** `twist_mux` (`ros2 node info`, `ros2 param dump`) publishes only `/a300_00000/platform/cmd_vel` and `/a300_00000/diagnostics` (namespaced; `diagnostic_updater.period: 1.0`, i.e. 1 Hz, so at most a slow cross-check, not a 50 ms ownership signal; its content is still to be read with `ros2 topic echo /a300_00000/diagnostics`). Inputs: `cmd_vel` (external, priority 1), `twist_marker_server/cmd_vel` (8), `joy_teleop/cmd_vel` (10), `rc_teleop/cmd_vel` (12), all with timeout 0.5 s. Locks, all latched (timeout 0.0): `platform/emergency_stop` (255), `platform/safety_stop` (254), `joy_teleop/bt_quality_stop` (253). `use_stamped: true`, `use_sim_time: true`. **Ownership therefore relies on the stamp match plus all three lock topics.** |
| Notes | The odometry twist has periodic 1–3 % blips (for example 0.31/0.2925 m/s, 0.517/0.4875 rad/s); `ros2 topic echo` reports "A message was lost" (best-effort odometry). Odometry yaw rate equals the command because it uses the same 1.75 multiplier; the simulated body's actual yaw rate needs Gazebo ground truth (the identification step below). |

**If V3–V5 fail:** build `clearpath_simulator` from source on the `jazzy` branch, as the install
page documents (vcs import of `dependencies.repos`, then `rosdep` and `colcon build`). Report the
issue to Clearpath. Do **not** substitute an A200: its geometry and controller differ.

**V12 commands** (container, simulation running):

```bash
ros2 node info /a300_00000/twist_mux                      # publishers: any status/selected topic?
ros2 topic list | grep -i -E "twist_mux|diagnostic|select|active"
ros2 param list /a300_00000/twist_mux
ros2 param dump /a300_00000/twist_mux
ros2 topic echo --once /diagnostics                        # look for a twist_mux status entry
ros2 topic echo /diagnostics | grep -i -B2 -A25 "twist"    # while V11 switches cmd_vel <-> joy_teleop
ros2 topic list | grep -i diagnostic | xargs -r -n1 ros2 topic info -v   # namespaced diagnostics?
```

Record whatever reports the selected input (or the masked/locked state) and how fast it updates
when V11 switches sources.

**V13 commands** (passive, the tool publishes nothing):

```bash
cd ~/nmpc_hls/app/husky
python3 identify_effective_track.py freshness --duration 30 --out ~/logs/freshness.csv     --ros-args -p use_sim_time:=true          # run once idle, once while V8 publishes
python3 identify_effective_track.py freshness-report ~/logs/freshness.csv
ros2 topic hz /a300_00000/platform_velocity_controller/cmd_vel_out --window 500
ros2 topic hz /a300_00000/platform/odom --window 500
ros2 topic hz /a300_00000/platform/odom/filtered --window 500
```

**Effective-track identification in simulation** (model note §4.4; `app/husky/identify_effective_track.py`).
**Simulation only:** the `record` command refuses to run without `--sim` and `use_sim_time:=true`.
It drives a short script of in-place turns (±0.3, ±0.6 rad/s) and arcs (0.3 m/s, ±0.3 rad/s,
radius 1 m), capped at 0.3 m/s and 0.6 rad/s; start in an open area of the world.

```bash
# terminal A: the simulation (§2.1 bring-up)
# terminal B: Gazebo ground truth -> ROS (check the world name first)
gz topic -l | grep dynamic_pose
ros2 run ros_gz_bridge parameter_bridge     "/world/warehouse/dynamic_pose/info@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V"     --ros-args -p use_sim_time:=true -r /world/warehouse/dynamic_pose/info:=/gt/dynamic_poses
# terminal C:
ros2 topic list | grep joint_states        # default --joint-topic /a300_00000/platform/joint_states
ros2 param get /a300_00000/platform_velocity_controller wheel_radius   # pass as --wheel-radius if not 0.1625
cd ~/nmpc_hls/app/husky
python3 identify_effective_track.py list-gt-frames --ros-args -p use_sim_time:=true
python3 identify_effective_track.py record --sim --gt-frame <robot frame from the list>     --out ~/logs/track_id.csv --ros-args -p use_sim_time:=true
python3 identify_effective_track.py analyze ~/logs/track_id.csv
```

The analysis reports, per segment:

- `ω_gt` (line fit of the ground-truth yaw) with its standard error;
- `ω_odom` and `ω_cmd`;
- the mean wheel-speed difference `(v_r − v_l)` from `joint_states`;
- `W_eff = (v_r − v_l)/ω_gt`.

It then gives a pooled `W_eff` with its uncertainty, the implied multiplier on 0.562 m, and
`ω_gt/ω_cmd`. Simulated friction is not the real surface: these numbers validate the pipeline, not
the robot.

### 2.2 What simulation does and does not validate

- **It validates:** signs, units, topic types, timing, watchdogs, the fallback logic, backend I/O, and closed-loop tracking under the simulator's physics.
- **It does not validate:** real slip, payload, motor limits, the real watchdog values, or the E-stop.

## 3. ROS 2 MPC node architecture

A new package (proposed name `nmpc_unicycle_ros`, `rclpy`, inside the container) in its own new
directory. It is not in `app/kria-pynq/`. **It is not written yet; this is its design.**

```text
 /<ns>/platform/odom/filtered (or odom) --+
 /reference (nav_msgs/Path)  -------------+      +----------------------------+
 /<ns>/platform/emergency_stop (Bool) ----+      | mpc_node (20 Hz timer)     |--> /<ns>/cmd_vel (TwistStamped, mux priority 1)
 /<ns>/platform/cmd_vel  (mux output) ----+----> |  ownership gate            |--> /mpc/diagnostics (terms, violations,
 .../cmd_vel_out (controller limited) ----+      |  state + delay comp.       |        timing, ownership, fallback state)
                                                 |  local frame, warm start   |
                                                 |  Backend.solve(deadline)   |
                                                 |  check_controls + rollout  |
                                                 |  idle (stop publishing)    |
                                                 +-------------+--------------+
                                         +---------------------+--------------------+
                                  (a) Python ref.     (b) C PSO subprocess    (c) KV260 over TCP

 Independent of the MPC (not part of it):
   obstacle monitor (lidar/bumpers) --> /<ns>/platform/safety_stop lock or the E-stop chain
   hardware E-stop buttons + wireless E-stop --> drivetrain power (PLc)
```

**Separation of concerns.** The MPC is a trajectory tracker: it does not perceive or avoid
obstacles, and its fitness is not a safety signal. Obstacle monitoring runs as a separate process
with its own authority path (the `twist_mux` `safety_stop` lock, priority 254, or the E-stop
chain), and the E-stop is hardware. Neither depends on the MPC node being alive or correct.

### 3.1 Control loop (every `Ts = 50 ms`, wall-clock timer, `use_sim_time` in simulation)

**0. Freeze inputs at the tick `t_tick`.** Latest odometry, latest `twist_mux` output
(`platform/cmd_vel`), latest controller limited command, E-stop and lock status.

**1. Safety-data gate (no solve, no cached plan).** If the odometry is older than `T_odom_stale`
(PLACEHOLDER_ policy: 3 odometry periods, 60 ms at 50 Hz), the applied command (`cmd_vel_out`) is
stale, or the E-stop/lock status is missing: **no solve and no publication**. The node goes to
`disarmed` and needs an explicit re-arm. A cached plan is never replayed in this case.

**What that does and does not do.** Stopping publication is not a stop command: the mux forwards
nothing after its 0.5 s timeout and the controller brakes after its own 0.5 s `cmd_vel_timeout`,
so the last command can persist for up to about 1 s. A low-priority `(0, 0)` `cmd_vel` would only
be best effort, because it is not selected during a takeover or under a lock. **Actual safety
stops are the job of the independent, verified path:** the E-stop chain, the `safety_stop` lock,
and an obstacle monitor with its own authority.

**2. Ownership state machine** (`app/husky/command_ownership.py`; model note §7.2; tested offline).
Feed it every event:

- own publications (stamp, twist, send time);
- `platform/cmd_vel` samples;
- the mux-selected source, if V12 finds one;
- lock status (`platform/emergency_stop`, `platform/safety_stop`, `joy_teleop/bt_quality_stop`: all three `twist_mux` locks, latched, priorities 255/254/253, from V12);
- the tick.

Then act on its state:

- **`owned`:** solve and publish.
- **`arming`:** publish only the hold probe (the applied command); no solve.
- **`unknown`, `foreign`, `disarmed`:** no solve, no publication. The warm start is reset when the machine flags it. Re-arm is only by the operator service, and only accepted with no lock, no foreign selection and a fresh `cmd_vel_out` inside the MPC envelope.
- **A single MPC-stamped sample leaking through the mux during a takeover (V11) can never re-arm the node.**

**3. `u_prev` = the applied command:** the latest `cmd_vel_out` sample within 0.06 s by its own
stamp (`AppliedCommandTracker`). It is re-stamped by the controller (V10), so it is never matched
to mux stamps. Never the node's own last publication.

**4. Delay compensation, one-period scheme.**

- Estimate the state at `t_tick` by integrating the **measured** odometry twist from the odometry stamp, then predict to `t_tick + Ts` with the **applied** command (`unicycle_model.estimate_state_at_actuation`; assumptions and limits under slip and lag in model note §7.7).
- Solve for the plan that starts at `t_tick + Ts`.
- Publish its first element **at** that tick, so actuation timing is deterministic. The odometry age is absorbed here and does not consume compute budget (§3.5).

**5. Local frame and warm start** (`to_local_frame`).

- Rotate the reference window (`Nh` poses resampled at `Ts` along the path) into the predicted robot frame, with headings unwrapped from 0.
- The warm start is the shifted plan (feasible, no projection).
- A projected candidate is on the critical path only when needed: feed-forward at arming, in-place turn when the heading error exceeds PLACEHOLDER_ π/2.

**6. Solve.** Call `Backend.solve(x0, xref, u_prev, warm_start, deadline)`.

- The node prepares the stop candidate while the board solves.
- The reply deadline is `t_tick + 40 ms` (§3.5).

**7. Check.**

- `check_controls(plan, u_prev, limits)` gives the violations, each reported separately.
- `rollout` checks finiteness and an optional geofence in odom.
- On pass, the plan is accepted.
- On fail or deadline miss, **while `owned` with fresh odometry and `u_prev`**: apply the next element of the last accepted plan, re-checked from the current `u_prev`, at most 3 times (PLACEHOLDER_), then the stop candidate (a best-effort deceleration, not a safety stop).

**8. Publish at `t_tick + Ts`.**

- `TwistStamped` with `header.stamp` = that tick time and `frame_id = base_link`.
- The stamp (strictly increasing) and twist go to the state machine's own-command history (`on_published`).
- Publish every tick while `owned`, so the robot's 0.5 s timeouts do not trigger by accident.

**9. Re-arm.** Only by an explicit operator service call (`request_arm`), and only when the
fresh applied command is inside the MPC envelope (in practice: the robot is stopped), so the
projection starts from a feasible `u_prev`. The A300 acts on the latest command after an E-stop release, so the node must
be idle (not publishing) during any E-stop reset.

**Lifecycle.** The node is a lifecycle node; `deactivate` immediately stops publishing.

**10. Logging.** Log per tick to a rosbag and a CSV:

- the state, the reference and the plan;
- each cost term and each violation in its own column;
- the ownership status and the `u_prev` source;
- the backend, each `C` component of §3.5, deadline misses and fallback state;
- the odometry age.

### 3.2 Backend (a): Python reference

- A Python PSO with the same update rules as `hls_pso.cpp`:
  - inertia schedule `w0 → wf`;
  - `c1`, `c2`;
  - velocity clamp `_max_v`;
  - particle 0 = warm start, particle 1 = stable zero;
  - per-dimension `du` box projection;
  - normalized ±100.
- It uses `unicycle_model.rollout` (`method="chord"` or `"rk4"`) and `mpc_cost(..., hls_residual=True)`.
- **Uses:** first closed loop in Gazebo, and the software screen of `Nu`, `n_S`×`maxiter`, weights and `_max_v`.
- **Not real time** in pure Python at 10×100. That is acceptable in simulation with `use_sim_time` and a slowed real-time factor (`gz` world `real_time_factor`), or with fewer evaluations.

### 3.3 Backend (b): the PC C model of the solver (software in the loop)

**Mechanism.**

- `main_hls_validate serve` reads records `seed x_curr[Nx] xref[Nx·Nh] u_curr[n_U] last_best[Nu·n_U]` on stdin.
- It prints `config n_S maxiter Nh Nu` once, then `u …`, `Jrecomputed J` and `end` per solve.
- The backend keeps one subprocess open and writes and reads one record per tick. Seed 0 continues the random stream, as the hardware's free-running generator does.

**Prerequisite (a separate, reviewed HLS change; not part of this plan):**

- a `UNICYCLE_CONFIG` block in `config.hpp`;
- `models/hls_unicycle.hpp`;
- the `hls_model.hpp` switch.

**Build.**

- A **new** builder script mirrors `validate_hw_run.build()`, which hard-codes `-DSNIFFBOT_CONFIG`. It passes `-DUNICYCLE_CONFIG` and the `_n_S`/`_maxiter` overrides.
- It supports the three arithmetic modes `float` / `hw` / `half` (the last is the hardware's own arithmetic).

**Uses:**

- bit-faithful SIL in Gazebo;
- the per-variant accuracy confirmation of model note §5.4 in the real `half` and `hls::` operators;
- the input to the HLS C-sim tests.

### 3.4 Backend (c): the KV260 over Ethernet (hardware in the loop)

**Board side.** A new server script on the KV260 (not in `app/kria-pynq/`).

- It wraps an `NmpcOverlay` generalized to the unicycle ABI: `NX = 3`, `NU = 2`, `(Nu, NU)` buffers, and manifest checks (model note §8).
- It runs one solve per request.
- **It never actuates anything** and has no safety role.
- The user starts it, via the staged `sudo bash …` step for loading the overlay (per the existing board procedure). Agents do not SSH to the board.

**Transport.**

- TCP on one persistent connection, `TCP_NODELAY`, little-endian, fixed-size frames, a CRC32 per frame.
- One outstanding request at a time. A late reply (sequence number older than the newest request) is discarded.

**Messages:**

| Message | Header (32 B) | Payload |
| --- | --- | --- |
| `HELLO` (host → board) | magic `NMPC`, version u16, type u16 = 0, seq u32, flags u32, t_host_ns u64, len u32, crc32 u32 | – |
| `HELLO_ACK` | same header, type 1 | `design_revision`, `model_id`, `Nx`, `n_U`, `Nh`, `Nu`, `n_S`, `maxiter` (i32 each), bitstream SHA-256 prefix (16 B), `Ts` and the normalization constants (f32). **The host refuses to run on any mismatch.** |
| `SOLVE` | type 2 | int32 Q15 (the hardware format; the host rounds to nearest): `u_curr[2]`, `x_curr[3]`, `xref[25·3]`, `last_best[Nu·2]`. That is 130 words = **520 B** at `Nu = 25`. |
| `RESULT` | type 3 | status i32, iterations i32, solve_us u32 (`ap_start`→`ap_done`), ps_us u32, t_board_rx_ns and t_board_tx_ns u64 (board monotonic clock, for intervals only), best fitness i32, `new_best[Nu·2]` int32 Q15. That is **236 B** at `Nu = 25` (36 B of fields + 200 B of plan). |
| `PING` / `PONG` | types 4 / 5 | – (RTT measurement at start-up and every 1 s while idle) |

**Network.** Run the container with `--network host`, so it reaches `10.42.0.29` exactly as the
host does. DDS never crosses to the board. For a later on-robot KV260 (A300 12 V user power and
network switch), the same protocol applies over the robot's LAN.

### 3.5 Deadline: one end-to-end metric

**Metric: cycle latency `C = t_ready − t_tick`.** It runs from the tick that freezes the inputs to
the moment the checked command is ready. The command is published at the next tick.

- **Why this one:** the odometry-stamp-to-publish interval is `odometry age + Ts` by construction (≤ 70 ms on the A300). The forward prediction absorbs it, and the staleness gate bounds it; it is logged, not budgeted.
- **Requirement:** `C ≤ 40 ms`, keeping the 20 % margin on the 50 ms period (5 ms publish guard + 5 ms unallocated).
- **Accounting:** each component is counted once. The solver figure excludes the PS overhead, which has its own line.

| Component (KV260 backend) | Budget (ms) | Covered by |
| --- | --- | --- |
| Host preparation (prediction, local frame, warm start, encode) | 4.0 | Measured on this PC (Python/numpy): local frame 0.21, shifted plan 0.003, one projected candidate 1.1–3.4. rclpy overhead to be measured in the node. |
| Network request + reply | 1.0 | Estimate; replaced by the measured `PING` p99 |
| KV260 PS overhead | 2.35 | Measured: max of 500 solves (3×24 run; 1.9 ms in the 100-solve run) |
| Solver `ap_start → ap_done` | ≤ 31.65 | Estimate (cycle model, upper value; model note §9) |
| Host decode, check, rollout | 1.0 | Measured on this PC: 0.11 + 0.53 |
| **`C`** | **≤ 40.0** | Reply deadline `t_tick + 40 ms`; a miss triggers the fallback (§3.1 step 7) |

**Reconciliation.** The first draft summed 38 + 2.35 + 2 + 1 + 2 = 45.35 ms against a 45 ms reply
target, and its solver estimates also contained the PS overhead. **What must shrink is the PSO size:**

- **Screening candidates** (upper estimate; model note §9.5), not a selected configuration; the screen in stage 3 selects one:
  - variant A (RK4): 4×50 (C 31.5–36.7 ms), 7×30 (32.5–38.0), 9×24 (33.2–38.8);
  - B3 (only if justified): 7×100 (32.4–37.1), 15×50 (33.9–38.9).
- **No longer fitting:** 10×100 for any variant, A at 5×50 and 9×30.
- **Measurement rule:** every line is measured in stages 2–4 and the solver budget is re-derived from the measurements before a configuration is frozen.

## 4. Staged test plan

### 4.0 Gates before any physical test

**G0: identify the robot's actual configuration** (user, read-only on the robot):

- the robot's `robot.yaml` (on Clearpath ROS 2 robots it lives under `/etc/clearpath/`; confirm the location on the robot);
- the generated controller and `twist_mux` parameters:
  - `ros2 param dump /<ns>/platform_velocity_controller`;
  - `ros2 param dump /<ns>/twist_mux`;
- the controller yaml in the generated setup directory.

Record `wheel_separation`, `wheel_separation_multiplier` (1.75 is only the Jazzy default), the
velocity and acceleration limits, `cmd_vel_timeout`, `publish_limited_velocity` and the
limited-command topic, the mux priorities and locks, and the software versions. Re-run the
model-note parameter table against them.

**G1: agree a conservative operating envelope** (user and supervisor, written):

- speed and yaw-rate caps (≤ 0.3 m/s slow mode to start), forward-only or reverse, and accelerations below the identified controller limits;
- because the side-speed and side-acceleration limits are unknown, the envelope keeps `abs(v) + abs(omega)·W·m/2` well inside the documented 2 m/s top speed, with `W·m` from G0;
- the surface, payload and slope (none), and the geofence;
- the independent obstacle monitor and its authority path.

**The placeholders stay placeholders** until identified (`PLACEHOLDER_` in the code and in model
note §6.1).

### 4.1 Stages

| Stage | Content | Entry criteria | Exit / stop criteria |
| --- | --- | --- | --- |
| **1. Reference model** | `/usr/bin/python3 -m pytest app/husky -q -p no:cacheprovider` (62 tests); `python3 -B app/husky/hw_estimates.py` | – | All pass (done, model note §11) |
| **2. A300 in Gazebo: model, controller and interface validation** | V1–V13 (§2.1): controller limits and ramps, `preserve_turning_radius`, timeout, stamp pass-through, the mux-selected source (V12), freshness and stamp quality of `cmd_vel_out` and odometry (V13), and the effective-track identification against Gazebo ground truth. The ownership state machine is complete and passes its offline tests (done: `test_command_ownership.py`). | Stage 1; container verified | All checks recorded; the state machine's age bounds re-derived from V13; `W_eff` with its uncertainty (simulation pipeline only). |
| **3. SIL screen: Python backend, then the PC C-model backend** | The node (state machine + Python backend (a)) in the A300 closed loop on straight lines, arcs, in-place turns, a figure-eight and a ±π heading crossing, plus teleop takeovers and lock tests. Screen `Nu`, `n_S`×`maxiter` (screening candidates, model note §9.5), weights, `_max_v`, and the increment-penalty comparison (model note §7.3). Then backend (b), the PC C model with a unicycle configuration, on the same scenarios and seeds, compared with Python. | Stage 2 passing | Tracking RMSE and max error, **violations (must be 0)**, deadline misses, ownership and fallback events, each reported separately. Commands never trigger the controller's own limiter. A screened configuration is selected; the C model agrees with Python within the agreed tolerance. **Review of the SIL results before stage 4.** |
| **4. RK4 HLS prototype, then HIL** | A separate, reviewed HLS change: the isolated RK4 model swap at the screened configuration; C-sim against `unicycle_model.py`, C/RTL co-sim, synthesis and timing at 100 MHz. Then backend (c), the KV260, against the A300 simulation (board steps by the user). | Stage 3 reviewed | C-sim within the §5.4 emulation bounds; timing closed; board results vs backend (b) in `half` mode for the same seeds; every `C` component measured; 0 violations. Stop on manifest mismatch, CRC error or repeated deadline misses. **B3 only if A's measured timing or tracking falls short.** |
| **5. A300 physical, slow mode** | See §4.2 | **G0 and G1 done**; the user's written safety plan approved; stages 2–4 passed; the independent obstacle monitor in place | §4.2 |

### 4.2 Stage 5: Husky A300 at slow mode (physical, gated)

Entry: gates G0 and G1 (§4.0), the user's safety plan, stages 2–4 passed and reviewed.

**Setup:**

- Follow the manual's safe work procedures:
  - keep at least 10 m away when possible;
  - never assume the robot can be stopped reliably;
  - test the stop systems before and after each use;
  - assert a stop before approaching;
  - chock the wheels when stationary;
  - no slopes; never stand downhill of the robot.
- Operating conditions per the manual: flat ground, grades ≤ 15°, payload within the signage.
- **Wireless E-stop kit fitted and tested.** It asserts automatically on loss of link. Never use the bypass key.
- Envelope as agreed at G1: at most the manual's slow mode **0.3 m/s**, with a yaw-rate cap chosen by the user (PLACEHOLDER_).
- The independent obstacle monitor runs, with its authority path tested (it must stop the robot with the MPC node killed).
- A wheels-off-ground run first if the user's plan allows lifting the robot safely (78.5–105 kg; the manual's Lifting and Transport section). Otherwise start at the lowest cap in an open area.

**E-stop procedure (rehearse before the first run):**

1. **Primary:** the wireless E-stop, held by a dedicated person.
2. **Secondary:** the front and rear E-stop buttons, which cut drivetrain power.
3. **Software:** deactivate the MPC node, and publish `true` on `/<ns>/platform/safety_stop` (a `twist_mux` lock, if the topic exists on the robot's configuration; verify first).
4. **Reset:** twist all depressed buttons out and press Safety Restart. The manual warns that the robot may move immediately and acts on the latest commands. **The MPC node must be deactivated, not publishing, before any reset.**

**Stop conditions:**

- any constraint violation reported by the host check;
- tracking error above a user threshold (PLACEHOLDER_ 0.3 m);
- odometry or E-stop status stale (the node then stops publishing and disarms, §3.1 step 1; the stop itself comes from the independent path);
- an ownership change the operator did not intend;
- more than N consecutive deadline misses (PLACEHOLDER_ N = 3);
- leaving the geofence;
- any operator call;
- `platform/emergency_stop` asserted unexpectedly;
- `/diagnostics` or `/status` warnings on motor current or temperature;
- an in-place-turn current concern on high-friction ground;
- any loss of the wireless E-stop link;
- anyone entering the test area.

**Gating.** Stage 5 (and the deferred TB4 indoor stage) starts only after G0 and G1 and when the
user has a written, robot-specific safety and E-stop plan. Agents never command a robot or run these
stages.

## 5. Order of work (proposal)

1. User: §1 (Docker), then V1–V13 and the effective-track identification in §2.1. Record the results.
2. Ownership/safety state machine: done offline (`command_ownership.py`); wire it into the node first, before any optimizer.
3. Stage 3a: Python-backend SIL screen in the A300 simulation.
4. Stage 3b: PC C-model backend on the same scenarios, compared with Python. Review of the SIL results; the branch stays isolated until then.
5. Stage 4: the isolated RK4 HLS prototype at the screened configuration, then KV260 HIL (board steps by the user).
6. Gates G0 and G1, the user's safety plan, then stage 5.
7. Only if justified by measurements: a B3 (interpolated chord) prototype.
8. Later, separate scope: the TurtleBot 4 port (§6).

## 6. Deferred scope: TurtleBot 4 / Create 3 port

Kept for later; nothing in stages 1–5 depends on it. When it starts:

- uncomment the TB4 line in the Dockerfile;
- re-verify the Create 3 facts in model note §6.2 (firmware, `wheel_base`, `cmd_vel` type and timeout);
- write a Create 3 variant of the ownership gate. The Create 3 has no `twist_mux`, but reflexes, hazards and actions override or block `cmd_vel`. `u_prev` would come from `/wheel_vels` or the odometry twist; to be designed.

### 6.1 TurtleBot 4 in `turtlebot4_simulator` (Gazebo Harmonic)

```bash
ros2 launch turtlebot4_gz_bringup turtlebot4_gz.launch.py            # model standard, world warehouse
# options documented: model (standard|lite), world (depot|maze|warehouse), slam, nav2, localization, rviz
```

| # | Check | Pass if |
| --- | --- | --- |
| T1 | `ros2 topic info -v /cmd_vel` and `ros2 topic list \| grep cmd_vel` | The type is recorded: the Create 3 docs say `Twist` on `/cmd_vel`, I.0.0 adds `TwistStamped` on `cmd_vel_stamped`, and the TB4 Jazzy tutorial publishes `TwistStamped` on `/cmd_vel`. The MPC node's publisher type follows what is found. |
| T2 | `ros2 topic hz /odom` | About 20 Hz |
| T3 | `ros2 param get /motion_control safety_override`, `wheel_accel_limit`, `reflexes_enabled` (if the simulated `motion_control` exposes them) | `none`, 900, true |
| T4 | Drive tests as V8/V9 at `linear.x` 0.2 and `angular.z` 1.0 | Sign conventions as REP-103; stops after the publisher ends |
| T5 | `ros2 topic echo /hazard_detection --once` while driving into a wall | Bump hazards appear; the reflex overrides the command |

### 6.2 TurtleBot 4 indoors (physical, gated by its own G0/G1 and the user's safety plan)

**Setup:**

- A closed, flat area free of drops and stairs.
- Default safety: `safety_override: none`, reflexes enabled, `wheel_accel_limit` default.
- MPC envelope forward-only (`v_min = 0`).
- Speed cap below the documented 0.306 m/s, chosen by the user (PLACEHOLDER_, for example 0.1 m/s first).
- Backend (a) or (b) first, then (c); after TB4 SIL with checks T1–T5 (§6.1) passing.

The Create 3 detects wheel drop and kidnap (`kidnap_status`, wheel-drop reflex). Lifting it is
therefore not a valid "wheels-off-ground" test: start on the floor at the lowest cap.

**E-stop procedure (rehearse before the first run):**

1. **Primary:** the `e_stop` service, prepared in an open terminal: `ros2 service call /e_stop irobot_create_msgs/srv/EStop "{e_stop_on: true}"`. The robot then ignores velocity commands ([Create 3 safety]).
2. **Secondary:** deactivate the MPC lifecycle node (publishing stops; the robot's command timeout stops it).
3. **Physical:** the operator stays within reach and picks the robot up if needed.
4. **Recovery:** release the E-stop with `"{e_stop_on: false}"` only after the MPC node is deactivated.

The physical stop means must be written into the user's safety plan.

**Stop conditions** (any one stops the run, through the E-stop service and node deactivation):

- any constraint violation reported by the host check;
- tracking error above a user threshold (PLACEHOLDER_ 0.3 m);
- odometry or E-stop status stale;
- more than N consecutive deadline misses (PLACEHOLDER_ N = 3);
- any `hazard_detection` event, `slip_status` true for more than 1 s, or `stop_status` or `kidnap_status` events;
- leaving the geofence;
- low battery;
- any operator call.

[Create 3 safety]: https://iroboteducation.github.io/create3_docs/api/safety/
