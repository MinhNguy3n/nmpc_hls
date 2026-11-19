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

# Headless (SSH sessions, automated runs; used for the 2026-11-19 checks): the same bring-up with
# the Gazebo server only. clearpath_gz always starts the GUI, which needs a display.
ros2 launch $H/launch/a300_sim_headless.launch.py setup_path:=$HOME/clearpath/ world:=warehouse
# A view-only GUI can be attached and closed at any time without stopping the server: from a
# desktop session on the PC (remotely: RDP through xrdp), run ~/nmpc_sim/docker/gz_viewer.sh. It
# starts a second container with the X socket, software rendering and the Teleop panel removed
# (that panel publishes unstamped cmd_vel through Clearpath's cmd_vel_bridge).

# Required workaround (2026-10-28): the generator writes sensors/config/imu_0.yaml as an empty
# bridge list ([]), so the IMU never reaches ROS and the EKF fuses wheel odometry only.
# Second shell, after the robot has spawned (the generator has then written its files), for as long
# as the simulation runs: the version-controlled bridge launch (app/husky/launch/). It runs the
# generator check first and starts the bridge only while the generated list is empty.
H=~/nmpc_hls/.claude/worktrees/agent-a7f53c7be12d59b73/app/husky
python3 $H/imu_checks.py check-generated --setup-path ~/clearpath   # exit 1 + "EMPTY" = defect present (expected)
ros2 launch $H/launch/a300_sim_imu_bridge.launch.py               # setup_path:=~/clearpath check:=true by default
```

The check fails loudly in both directions: exit 1 while the generated list is empty (the bridge is
needed), exit 3 once the generator bridges the IMU itself (then the launch file does **not** start
the extra bridge, to avoid duplicate publishers, and this plan must be updated). The upstream report
is drafted in `docs/husky_a300/upstream_issue_empty_imu_bridge.md` (not filed; the user decides).

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
| V12 | **No selected-source signal.** `twist_mux` (`ros2 node info`, `ros2 param dump`) publishes only `/a300_00000/platform/cmd_vel` and `/a300_00000/diagnostics` (namespaced; `diagnostic_updater.period: 1.0`, i.e. 1 Hz, so at most a slow cross-check, not a 50 ms ownership signal). The 'twist_mux: Twist mux status' entry reports each input as `masked`/active, each lock as `free`/engaged, and **`current priority`**, plus `data age`. **Correction (2026-11-19):** `current priority` is the highest engaged **lock** priority (0 = no lock, 254 `safety_stop`, 253 `bt_quality_stop`), not the selected input; it stayed 0 while `cmd_vel` was forwarded (twist_mux 4.5.0, `twist_mux_diagnostics_status.hpp`). The selected source is the highest-priority input reported `unmasked` (fresh within its timeout and not below the lock priority, `topic_handle.hpp` `isMasked`). Cross-check (about 2 Hz in the capture): an unmasked input above `external` while the node believes it is owned means foreign. Inputs: `cmd_vel` (external, priority 1), `twist_marker_server/cmd_vel` (8), `joy_teleop/cmd_vel` (10), `rc_teleop/cmd_vel` (12), all with timeout 0.5 s. Locks, all latched (timeout 0.0): `platform/emergency_stop` (255), `platform/safety_stop` (254), `joy_teleop/bt_quality_stop` (253). `use_stamped: true`, `use_sim_time: true`. **Ownership therefore relies on the stamp match plus all three lock topics.** |
| V13 (idle) | 30 s capture with `identify_effective_track.py freshness`, robot idle. `cmd_vel_out`, `joint_states` and `platform/odom`: 47.62 Hz, gap std 0.6 ms, max gap 24 ms, age ≤ 3 ms, no non-monotonic stamps. The state machine's placeholder bounds (`cmd_vel_out` age 0.06 s, mux stale 0.12 s) have ample margin. Mux output: 15 samples, all one stamp, age about 920 s in simulated time, i.e. a zero-stamped publisher (a leftover `ros2 topic pub` or a teleop node publishing unstamped). Correctly classified as foreign by the stamp match; other sources can publish zero stamps. The capture while publishing is V13 (active) below. |
| Effective track (simulation) | `identify_effective_track.py record --sim --gt-frame a300_00000/robot` with ground truth from `gz topic -e` (2026-10-26): in-place turns at ±0.3 and ±0.6 rad/s give ω_gt/ω_cmd = **0.952**, W_eff = 1.034 m (multiplier 1.84 on 0.562 m); 1 m arcs at 0.3 m/s, ±0.3 rad/s give ω_gt/ω_cmd = **0.825**, W_eff = 1.19 m (multiplier 2.12). Left/right symmetric; the wheel-speed difference matches the controller's 0.9835 m mapping exactly. The pooled 1.054 ± 0.024 m mixes the two regimes; use per-regime values. Wheel odometry reports ω = ω_cmd (same multiplier), so it **overstates yaw rate by 5–17 %** in simulation. Consequences: (1) check `platform/odom/filtered` (EKF with IMU) against ground truth before choosing the state-estimate source; (2) treat ω_body ≠ ω_cmd as a model mismatch to screen in SIL (closed-loop feedback vs a speed-dependent yaw-rate gain). These are simulator physics, not the real robot's slip (model note §4.4). |
| State estimate (simulation) | **The IMU bridge is generated empty.** `sensors/config/imu_0.yaml` contains `[]`, so `imu_0_gz_bridge` bridges nothing: Gazebo publishes `/a300_00000/sensors/imu_0/data` (frame `imu_0_link`), but the ROS topic has 0 publishers. The EKF (`platform/config/localization.yaml`: 2D, 50 Hz; odom0 fuses x, y, yaw, vx, vy, yaw rate; imu0 fuses yaw rate and ax from `sensors/imu_0/data`) then sees odometry only, and `platform/odom/filtered` reported ω = 0.300 at ω_cmd = 0.3 (ground truth 0.2855). With the IMU bridged by hand (launch step above), `odom/filtered` settles at **0.2855 rad/s**, matching ground truth, in about 0.15 s. **Use `platform/odom/filtered` with the IMU bridge as the MPC's state source in simulation**; plain `platform/odom` overstates yaw rate by 5–17 %. The generator files are rewritten on every launch, so the fix is the extra bridge (or a corrected generator), not a hand edit. |
| V13 (active), 2026-11-19 | Headless simulation, RTF 1.0, `mux_timing.py` scenario `active` (`cmd_vel` at 20 Hz, stamp = simulation now) and capture, 30 s. 599 sent, 599 forwarded: **no loss, no reordering, no duplicates**. Echo delay (input receive → mux output receive) median **0.14 ms**, p99 0.51 ms, max 0.65 ms wall clock (0 in simulation time, whose step is 3 ms). Mux output age median 0, `cmd_vel_out` 47.6 Hz with max gap 24 ms and age ≤ 6 ms. One start-up transient: the first four messages arrived together 0.36 s late while the publisher was being discovered. The bounds `max_mux_age` 0.12 s and `max_applied_age` 0.06 s keep a wide margin; the provisional values stay. |
| V11 (scripted), 2026-11-19 | **Takeover** (`joy_teleop` during 10–20 s and 25–30 s): every joy message forwarded (101/101), **takeover in the same tick** (0 ms), `cmd_vel` suppressed while joy was fresh (317), **hand-back 0.549 s** after the last joy message (0.5 s timeout + one tick), **no leak**. The report first showed one "leak" per takeover: the `cmd_vel` message sent 77–107 µs before the first joy message in the same tick (same simulation stamp), forwarded in send order before the mux had any joy input, i.e. a race; the report now breaks simulation-time ties by wall clock and attributes outputs by source tag first. **Locks** (`safety_stop` 10–15 s, `bt_quality_stop` 20–25 s): **0 outputs while engaged**, first output 48–51 ms after release (next tick); diagnostics report lock priority 254/253 with every input masked. **Discovery delay:** in the lock run twist_mux forwarded nothing for the first **1.9 s** of a newly started publisher (the capture node saw it at once), longer than the 0.5 s arming timeout; the SIL node therefore arms only after twist_mux has matched its publisher. The original V11 leak (one `sec: 12345` message during teleop, wall-clock stamps from `ros2 topic pub`) was not reproduced with simulation-time stamps; the likeliest explanation is the same same-instant race, unconfirmed. Clearpath also runs `cmd_vel_bridge` (Gazebo → ROS on `/a300_00000/cmd_vel`, for the GUI Teleop panel); its messages carry no stamp, which explains the zero-stamped mux samples of V13 (idle, GUI open). |
| V14, 2026-11-19 | IMU (bridged): **about 100 Hz** (simulation time), frame `imu_0_link`, stamped in simulation time (age 0), **all covariances 0** (the EKF then trusts the gyro almost completely, which is why `odom/filtered` matches ground truth so closely; the real Phidgets IMU reports non-zero values). TF `base_link → imu_0_link` exists (0.059, 0, 0.161 m; Clearpath's TF topics are namespaced: `-r /tf:=/a300_00000/tf`). EKF: `imu0 = sensors/imu_0/data`, `imu0_config` fuses yaw rate and ax, `two_d_mode`, 50 Hz. **IMU loss (bridge stopped for 14.6 s during ±0.3 rad/s turns): silent fallback.** `odom/filtered` kept publishing at full rate and reported the wheel-odometry yaw rate (±0.300 rad/s vs ground truth ±0.2855), i.e. 5 % high, with no diagnostic; it returned to ground truth within 0.5 s after the IMU resumed. Filtered yaw-rate latency against ground truth: best-fit lag **45 ms**, 50 % step delay up to 0.13 s. Consequence: the SIL controller now treats a stale or missing IMU (`max_imu_age` 0.05 s, provisional) as missing safety data: it refuses to arm and disarms when owned. |
| V15, 2026-11-19 | **`set_physics` applies the whole `gz.msgs.Physics` message, gravity included.** The request with only `real_time_factor: 0.25` returned `data: true` and the stats reported RTF 0.2500 with the 3 ms step, but it also set **gravity to zero**: the robot lifted to z = 0.6 m did not fall, and under the MPC's commands it pitched up slowly with its wheels spinning (z 0.14 → 0.28 m, tilt 0.5 rad in 15 s) while wheel odometry and `odom/filtered` reported a normal circle. The first SIL runs were therefore invalid. Correct request (the world's values: `<gravity>0 0 -9.8</gravity>`, `max_step_size` 0.003): `--req "max_step_size: 0.003, real_time_factor: 0.25, gravity: {z: -9.8}"`; the robot then falls and rests at z = 0.135 m. `sil/run_batch.sh` sends this request itself and drop-tests the robot before the first run, and the SIL metrics now mark a run invalid when the ground-truth tilt exceeds 0.35 rad. The EKF runs at 12.5 Hz wall clock = 50 Hz simulation time. |
| First valid SIL run, 2026-11-19 | `run_batch.sh`, circle, 7×30, plain, seed 1, RTF 0.25, gravity on: owned for the whole lap, robot upright, 480 plans, 0 deadline misses (C_sim median 9.7 ms, C_wall median 38.7 ms), 0 constraint violations, 0 fallbacks. **Tracking against ground truth: position RMSE 0.81 m, cross-track RMSE 0.12 m, heading error 1.09 rad at the end of the lap.** Cause: the estimator, not the MPC. The EKF fuses the wheel odometry's **absolute** x, y and yaw (`odom0_config` x, y, yaw, vx, vy, vyaw; `odom0_differential: false`; pose covariance 0.001) and only the yaw **rate** from the IMU, so the `odom/filtered` pose follows wheel odometry (feedback-vs-truth and wheel-vs-truth drift are identical: 1.126 m, 1.088 rad). On the arc the body turns at about 0.83 × the wheel-odometry yaw rate, so the controller closes a circle that exists only in odometry. The 2026-10-28 choice of `odom/filtered` was based on its yaw **rate**, which does match ground truth. **Open decision before the batch:** (a) keep `odom/filtered` and treat the drift as part of the result; (b) a simulation-only EKF override that fuses wheel odometry as velocities only (vx, vyaw, or `odom0_differential: true`) plus the IMU yaw rate, so the heading follows the gyro; (c) wait for the real robot's EKF configuration (G0). |
| SIL estimator validation, 2026-11-19 (GPT-6 option b) | `launch/a300_sil_estimator.launch.py`: simulation-only EKF instances (`sil_ekf_<variant>`, output `sil/<variant>/odom/filtered`, `publish_tf: false`; the generated EKF and its TF untouched) fusing wheel **v_x only** and the IMU **yaw rate only**, the IMU passed through `sil/imu_cov_relay.py`, which sets the gyro variance. robot_localization raises a measurement variance below 1e-9 to 1e-9 (`ekf.cpp`, `Ekf::correct`), so Gazebo's 0 enters as 1e-9; the phidgets_spatial driver default is 0.095 °/s ("as per manual") = **2.75e-6 (rad/s)²**, a datasheet value, not a calibration. Lateral velocity is not fused (the diff-drive odometry reports 0, not a measurement); the IMU a_x of the generated EKF is left out (it only feeds the predicted acceleration, v_x is measured at 50 Hz, it was not validated). Scenario `mux_timing.py scenario estimator` (still 3 s; in place ±0.3 rad/s 6 s each; 1 m arcs left and right at 0.3 m/s, 8 s each; still 3 s), `estimator_checks.py` aligns each source to the truth once at the start. The first attempt fused no IMU at all (yaw rate 0): the EKF did not get `base_link → imu_0_link` because Clearpath namespaces TF; fixed by remapping `/tf`, `/tf_static`. Result (`est_val2_report.json`): **heading error max 0.002 rad** for the SIL EKF (driver variance), against **0.42 rad** for the generated EKF and wheel odometry after the left arc; yaw rate equal to truth (ratio 1.000, lag 0). Covariance sweep: native (1e-9), 2.75e-6 and 1e-4 identical; 1e-2 adds 55 ms of yaw-rate lag. **Remaining error: position 0.48 m after about 4.8 m of arcs** (generated EKF 0.78 m): on the arcs the body moves at 0.87–0.89 × the wheel speed (longitudinal slip, sideslip ≈ 0), which no velocity fusion removes without an absolute position source. The SIL screen uses variant `driver`; the generated EKF is kept as the drift reference. |
| SIL with the SIL estimator, 2026-11-19 | `run_batch.sh` (feedback `sil/driver/odom/filtered`), circle, 7×30, plain, seed 1, RTF 0.25, gravity on. Control: owned for the whole lap, 480 plans, 0 published violations, 0 fallbacks, 0 late ticks; about 22 callbacks drained per tick (the normal per-tick volume, no backlog growth). **C_sim** (RTF-scaled) median 9.9 ms, max 10.9 ms, 0 misses; **C_wall** (unscaled Python) **p50 39.5 / p95 41.0 / max 43.5 ms: 139 of 481 above 40 ms, 0 above 50 ms** (Python 7×30 does not meet the 40 ms budget in wall time; a property of this Python code only). Heading: the feedback's final heading error is now **0.0003 rad** (generated EKF: 1.09 rad). Tracking against ground truth: position RMSE 0.84 m, **cross-track RMSE 0.47 m**, max 0.94 m: rejected by the provisional thresholds. Split: against its own feedback the controller has cross-track RMSE **0.15 m** (max 0.25 m) and heading RMSE 0.39 rad; the feedback's position drifts up to **1.17 m** from truth (the 0.87–0.89 longitudinal slip). Two effects therefore remain: (1) the estimator's position scale error on arcs, (2) the predictor's yaw-rate mismatch, now visible because the feedback shows the true yaw rate (about 0.83 × ω_cmd on arcs) while the plain predictor assumes ω_cmd; the sim-derived gain variant addresses (2). The report now has `tracking_vs_feedback` beside `tracking_vs_ground_truth`. |
| IMU dropout with the SIL estimator, 2026-11-19 | IMU bridge stopped 10.4 s into an owned SIL run (feedback `sil/driver/odom/filtered`, gate on the relayed `sil/driver/imu`): `owned → disarmed` "IMU stale or missing", the batch run ended (no self re-arm); bridge restarted, relay resumed at 100 Hz (simulation time). |
| V11 exact repeat, 2026-11-19 | The original configuration: `ros2 topic pub -r 10` on `cmd_vel` with the fixed stamp 12345.000000678 (running throughout) and, in 5 trials, `ros2 topic pub -r 10` on `joy_teleop/cmd_vel` with stamp 999 for 10 s (a new publisher process each trial), RTF 1.0 with gravity; capture with header stamps and monotonic receive times (`v11_repeat.csv`, `v11_analyse.py`). **Reproduced in 1 of 5 trials:** in trial 2 the mux forwarded **5 MPC messages during the first 500 ms** of the takeover; the capture node had received the first 5 joy messages (each about 21 ms before the MPC message that was forwarded), but **twist_mux forwarded none of them**; from the 6th joy message on it forwarded joy and suppressed `cmd_vel` normally. Trials 1, 3, 4, 5: takeover within 0.3 ms, no MPC output while joy was fresh; hand-back 0.52–0.59 s. In trial 3 the mux forwarded joy before the capture node had matched the joy publisher (the reverse case). Interpretation: **a newly started publisher is not yet matched by twist_mux** (DDS discovery, as the 1.9 s in V11-locks), so its first messages are not seen by the mux, which keeps forwarding the lower-priority input; not a mux priority/timeout fault. Status: "reproduced; consistent with publisher-discovery delay; twist_mux's match time not directly observed". Consequences: a teleop *process started* during an MPC run takes over only after discovery (up to about 0.5–1.9 s here); a long-running teleop node (the joystick driver) is matched in advance. The ownership state machine stays fail-closed: it sees its own commands forwarded, which is what the mux actually does. |
| Paired predictor batch (first attempt), 2026-11-19 | `run_batch.sh`, 7×30, SIL estimator, circle and figure-eight (1 m, 0.3 m/s), plain and sim-derived, seeds 1–3: **not usable for the comparison.** (1) Every sim-derived run had 7–17 "deadline misses" and as many **stop** commands in its first second: C_sim read 0.11–0.17 s for 37 ms of Python because the real-time factor, estimated from consecutive ticks, jumped to about 4.5 while the timer fired in bursts after node start (start-up transient, ownership began 0.1–0.9 s after node start). (2) Figure-eight seed 2 ended 0.2 s after arming with "filtered odometry stale or missing" (age 66 ms in the same transient; steady-state odometry age ≤ 21 ms). Fixed in the node: RTF over ≥ 1 s of wall clock (`rtf_from_history`), and `--auto-arm` only after `--warmup` 3 s of simulation time. (3) Figure-eight seed 1 lost the track even against its own feedback (cross-track 0.53 m): the commanded yaw rate saturated at 0.75 rad/s, see the yaw-rate map. The generated-EKF drift reference scored cross-track 0.12 m against truth: its two errors (wheel yaw rate and wheel speed both about 0.85–0.9 × the body's at κ = 1) give a correctly shaped, slightly larger circle, which hides both. |
| Yaw-rate map, 2026-11-19 | `mux_timing.py scenario steps` at v = 0, 0.1, 0.2, 0.3 m/s × ω = 0.1–0.75 rad/s (5 s steps), `estimator_checks.py yawmap` from ground truth (steady 40 % of each step). **The simulated A300 responds as a function of the commanded curvature κ = ω/v, independent of speed:** κ ≤ 0.67 → body speed 0.97–1.0 × and yaw rate 0.90–0.94 × the command; **κ = 1 → 0.89 / 0.825** (every speed); κ = 1.5 → 0.655 / 0.605; κ = 2–2.5 → 0.38–0.46 / 0.43–0.57; κ = 3 → 0.41 / 0.70; in place 0.952. Consequences: the 1 m paths (κ = 1) are already in the slipping regime, and at 0.3 m/s (= v_max) the controller cannot recover lag by speeding up, so it raises ω, pushing κ to 2–2.5 where the robot slows to about 40 % (the figure-eight runaway). A constant yaw-rate gain cannot represent this; the unicycle predictor is close only for κ ≲ 0.67. These are simulator physics (Gazebo contact friction), not robot values. |
| Primary path geometry and clearance, 2026-11-19 (GPT-6) | Primary (low curvature): radius **2 m**, reference speed **0.2 m/s** (≤ 2/3 of the unchanged 0.3 m/s envelope), κ = 0.1/0.2 = 0.5; stress case: radius 1 m at 0.3 m/s (κ = 1), labelled. **Clearance** (`sil/clearance.py`): obstacles at robot height from `warehouse.sdf` poses and the Fuel models' collision geometry (shelf box 3.6 × 0.6 m, shelf_big 2.1 × 18 m, the standing person's collision mesh, the warehouse mesh's pillars at x = ±7.2…7.7 m and outer walls; far objects as 1.5 m squares), cross-checked against the live `/world/warehouse/pose/info`; the robot as a 0.6 m circle (A300 collision: 0.74 × 0.42 m plate, wheels r 0.165 m on a 0.68 m outer track). At the spawn the 2 m figure-eight crosses `shelf_7` (−0.60 m) and the 2 m circle passes 0.18 m from the standing person; **the 1 m circle used until now passed 0.24 m from that person**. **Chosen start (0, 3.9) m, yaw π/2**: circle x −4…0, y 1.9…5.9; figure-eight x −4…4, y 1.9…5.9; **min clearance 1.95 m** (to `shelf_big_3`) for both 2 m paths, 2.95 m for the 1 m stress paths. `run_batch.sh` re-checks every path before a batch (≥ 1.0 m beyond the robot circle) and puts the robot at this start before every run. Predictor variant for the primary comparison: `sim-lowk`, constant gain **0.92**, fixed before the runs from the map value 0.923 at κ = 0.5 (band 0.90–0.94); `sim-derived` (0.825, κ = 1) stays a diagnostic. |
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
cd ~/nmpc_hls/.claude/worktrees/agent-a7f53c7be12d59b73/app/husky   # the branch worktree, read-only
python3 identify_effective_track.py freshness --duration 30 --out ~/logs/freshness.csv     --ros-args -p use_sim_time:=true          # run once idle, once while V8 publishes
python3 identify_effective_track.py freshness-report ~/logs/freshness.csv
ros2 topic hz /a300_00000/platform_velocity_controller/cmd_vel_out --window 500
ros2 topic hz /a300_00000/platform/odom --window 500
ros2 topic hz /a300_00000/platform/odom/filtered --window 500
```

**V13-active and the V11 leak investigation** (`app/husky/mux_timing.py`; simulation only).
Four shells in the container, all with `H=~/nmpc_hls/.claude/worktrees/agent-a7f53c7be12d59b73/app/husky`: A the simulation, B the IMU bridge launch, C the
passive capture, D the scripted publisher. Start C first, then D. The scenarios publish zero twists
(the robot does not move), tag their source in `twist.linear.y` (ignored by the diff-drive
controller), stamp with simulation time, and never touch `platform/emergency_stop`.

```bash
cd $H
# Run 1: active publishing (V13-active): cmd_vel at 20 Hz for 30 s
python3 mux_timing.py capture --duration 40 --out ~/logs/v13_active.csv --ros-args -p use_sim_time:=true            # C
python3 mux_timing.py scenario active --sim --out ~/logs/v13_active_sent.csv --ros-args -p use_sim_time:=true      # D
python3 mux_timing.py report ~/logs/v13_active.csv --out ~/logs/v13_active_report.json
# Run 2: teleop takeover (V11): cmd_vel at 20 Hz; joy_teleop at 10 Hz from 10-20 s and 25-30 s
python3 mux_timing.py capture --duration 50 --out ~/logs/v11_takeover.csv --ros-args -p use_sim_time:=true          # C
python3 mux_timing.py scenario takeover --sim --out ~/logs/v11_takeover_sent.csv --ros-args -p use_sim_time:=true  # D
python3 mux_timing.py report ~/logs/v11_takeover.csv --out ~/logs/v11_takeover_report.json
# Run 3: locks: safety_stop engaged 10-15 s, bt_quality_stop 20-25 s (released at the end in any case)
python3 mux_timing.py capture --duration 40 --out ~/logs/v11_lock.csv --ros-args -p use_sim_time:=true              # C
python3 mux_timing.py scenario lock --sim --out ~/logs/v11_lock_sent.csv --ros-args -p use_sim_time:=true          # D
python3 mux_timing.py report ~/logs/v11_lock.csv --out ~/logs/v11_lock_report.json
# Run 4 (reproduces the original V11 conditions): the scenario `active` on cmd_vel in D, plus in a
# fifth shell for about 10 s, then Ctrl-C (wall-clock stamps, as in the original V11 run):
ros2 topic pub -r 10 /a300_00000/joy_teleop/cmd_vel geometry_msgs/msg/TwistStamped \
    "{header: {stamp: {sec: 999}}, twist: {linear: {y: 0.002}}}"
```

The report gives:

- echo delay (input receive → mux-output receive, matched by exact stamp): median, p95, max, std;
- lost cmd_vel samples, split into unexpected and suppressed (a higher-priority source within its 0.5 s timeout, or a lock);
- reordered and duplicated mux outputs;
- per takeover: takeover latency and hand-back delay after the last joy sample (expected about the 0.5 s timeout);
- **leak events:** cmd_vel samples forwarded although joy was active within 0.5 s, or a lock was engaged, when the input arrived. Each comes with the last joy age, the time to the next joy sample, the input age at forwarding, and the neighbouring outputs;
- per lock: mux outputs while engaged (expected 0) and the first output after release;
- per-topic freshness, and the 1 Hz `current priority` samples as a slow cross-check.

**These measurements set the ownership thresholds.** `max_mux_age`, `max_match_age`, `unknown_grace`,
`confirmations`, `max_applied_age`, `max_odom_age` and the deadline are all **provisional** until
then (§3.1). Re-derive each from the measured p99.9 gaps and delays with margin, and record them.

**IMU stream and EKF checks (V14)** (`app/husky/imu_checks.py`; shells as above):

```bash
ros2 topic info -v /a300_00000/sensors/imu_0/data            # 1 publisher (a300_imu_0_bridge_workaround); ekf_node subscribed
ros2 topic hz /a300_00000/sensors/imu_0/data                 # rate
ros2 topic echo --once /a300_00000/sensors/imu_0/data --field header                      # frame_id (Gazebo: imu_0_link), stamp
ros2 topic echo --once /clock                                                              # compare: the stamp must be simulation time
ros2 topic echo --once /a300_00000/sensors/imu_0/data --field orientation_covariance      # -1 in [0] = no orientation
ros2 topic echo --once /a300_00000/sensors/imu_0/data --field angular_velocity_covariance
ros2 topic echo --once /a300_00000/sensors/imu_0/data --field linear_acceleration_covariance
ros2 run tf2_ros tf2_echo base_link imu_0_link               # the frame exists in TF (Ctrl-C after one print)
ros2 param get /a300_00000/ekf_node imu0                     # the topic the EKF reads
# IMU loss and filtered-state latency: capture (C) during the turn scenario (D); in shell B stop the
# bridge launch (Ctrl-C) at about 6 s into the scenario, and start it again at about 19 s.
python3 imu_checks.py capture --duration 32 --out ~/logs/imu_check.csv --ros-args -p use_sim_time:=true             # C
python3 mux_timing.py scenario turn --sim --out ~/logs/imu_turn_sent.csv --ros-args -p use_sim_time:=true          # D (the robot turns in place)
python3 imu_checks.py report ~/logs/imu_check.csv --out ~/logs/imu_check_report.json
```

**Record:**

- the IMU frame id, stamp clock, covariance fields and rate;
- for the IMU gap: whether `odom/filtered` kept publishing and moved to the wheel-odometry yaw rate (`silent_fallback_to_wheel_odometry`), and its error against ground truth before and during the gap;
- the filtered yaw-rate latency against ground truth (best-fit lag and 50 % step delays; about 0.15 s settling was seen).

**The EKF does fall back silently** (V14, 2026-11-19). The SIL controller therefore treats a stale
or missing IMU stream as missing safety data (`CoreConfig.max_imu_age`, 0.05 s, provisional): it
refuses to arm and disarms when owned.

**Real-time factor for Python SIL** (V15; verify the service on this Gazebo version first):

```bash
gz service -l | grep -E "set_physics|/control"
grep -n -E "max_step_size|real_time_factor|real_time_update_rate" \
    $(ros2 pkg prefix clearpath_gz)/share/clearpath_gz/worlds/warehouse.sdf
gz service -s /world/warehouse/set_physics --reqtype gz.msgs.Physics --reptype gz.msgs.Boolean --timeout 3000 \
    --req "max_step_size: 0.003, real_time_factor: 0.25, gravity: {z: -9.8}"   # ALL of them: omitted fields become 0
gz topic -e -t /world/warehouse/stats -n 3 | grep real_time_factor     # confirm about 0.25
# sil/run_batch.sh does this itself and checks that the robot rests on the floor (gravity on)
```

**Effective-track identification in simulation** (model note §4.4; `app/husky/identify_effective_track.py`).
**Simulation only:** the `record` command refuses to run without `--sim` and `use_sim_time:=true`.
It drives a short script of in-place turns (±0.3, ±0.6 rad/s) and arcs (0.3 m/s, ±0.3 rad/s,
radius 1 m), capped at 0.3 m/s and 0.6 rad/s; start in an open area of the world.

Ground truth is read from Gazebo directly (`--gt-source gz`, the default): the tool runs
`gz topic -e -t /world/<world>/dynamic_pose/info` in the background and selects the robot's pose by
its Gazebo name. The `ros_gz_bridge` route (`Pose_V` → `tf2_msgs/TFMessage`) is not usable here:
on 2026-10-26 it delivered every transform with empty `frame_id` and `child_frame_id`, so the robot
could not be selected. The Gazebo Python bindings (`gz.transport13`, `gz.msgs10`) are not in the
image, and `GZ_IP=127.0.0.1` must be set for `gz topic` too (it is, through `docker run -e`).

```bash
# terminal A: the simulation (§2.1 bring-up)
# terminal B (in the container):
gz topic -l | grep dynamic_pose            # world name; here /world/warehouse/dynamic_pose/info
gz model --list                            # the A300 model is "a300_00000/robot"
ros2 topic list | grep joint_states        # default --joint-topic /a300_00000/platform/joint_states
ros2 param get /a300_00000/platform_velocity_controller wheel_radius   # pass as --wheel-radius if not 0.1625
cd ~/nmpc_hls/.claude/worktrees/agent-a7f53c7be12d59b73/app/husky      # or the branch checkout once merged
python3 identify_effective_track.py list-gt-frames --ros-args -p use_sim_time:=true   # prints the pose names
python3 identify_effective_track.py record --sim --gt-frame a300_00000/robot \
    --out ~/logs/track_id.csv --ros-args -p use_sim_time:=true
python3 identify_effective_track.py analyze ~/logs/track_id.csv
```

For another world, add `--gz-topic /world/<world>/dynamic_pose/info`.

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
to mux stamps. Never the node's own last publication. With the one-period scheme, the command published AT this tick is used as the slew base of the plan computed now (which starts
one period later), but only if it is reachable in one step from the applied `cmd_vel_out`;
otherwise (limiter active, lag, mismatch) the applied command is the base and the tick is noted
(`MpcCore.finish_tick`, tested in `sil/test_sil_core.py`).

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

**Thresholds are provisional.** Every age bound and timeout in this loop is a placeholder until
V13-active and the V11 investigation are measured (§2.1). That covers the ownership bounds, the
odometry and applied-command ages, and C ≤ 40 ms. No active-publisher SIL result is reviewed before
then.

**Signals are kept distinct:**

| Role | Signal | Use |
| --- | --- | --- |
| Feedback | `platform/odom/filtered` (EKF with the IMU bridge), with its stamp, age and covariance | The only state input of the controller. Its message stamp and the delay compensation are used; it is not treated as instantaneous truth (it settles in about 0.15 s). |
| Applied command | `platform_velocity_controller/cmd_vel_out` | `u_prev`, the re-arm check and the slew base |
| Ownership | `platform/cmd_vel` stamp match and the three locks; the 1 Hz `current priority` as a cross-check only | The state machine |
| Scoring | Gazebo ground truth (`gz topic -e`, `a300_00000/robot`) | Logged and scored offline only; **never fed back** |
| Diagnostic | Wheel odometry `platform/odom` | Logged and compared with feedback and truth; never used for control |

This is a simulation choice. It is not evidence about the real robot's EKF configuration (gate G0).

### 3.2 Backend (a): Python reference

- A Python PSO with the same update rules as `hls_pso.cpp`:
  - inertia schedule `w0 → wf`;
  - `c1`, `c2`;
  - velocity clamp `_max_v`;
  - particle 0 = warm start, particle 1 = stable zero;
  - per-dimension `du` box projection;
  - normalized ±100.
- **Implemented** in `app/husky/sil/` (2026-10-29):
  - `pso.py`: the hardware structure, vectorized over particles. The cost is the hardware's (`Q`/`Qf` on the states with the `8c61f1b` angle residual, `R (z − uss)²`). Prediction is RK4 (variant A), with the predictor-gain option.
  - `controller.py`: ownership, applied command, state estimate, fallbacks, checks and C, without ROS.
  - `sil_node.py`: the thin rclpy node.
  - `metrics.py`: offline scoring.
  - `gz_truth.py`: the ground-truth reader.
  - Tests: `app/husky/sil/test_sil_core.py`.
- **Python solve time** on this PC (Python 3.10, numpy 2.2.6; the container runs Python 3.12, so measure there) at `Nu = 25`: 4×50 51 ms, 7×30 31 ms, 9×24 25 ms median wall clock; at `Nu = 10`: 45, 28 and 22 ms. 4×50 exceeds the 40 ms budget in wall time.
- **Real-time factor:** the SIL deadline is therefore checked in **simulation time**, with Gazebo slowed to a real-time factor of 0.25 for every configuration (`rtf_for_deadline`: the largest real-time factor with 20 % margin is about 0.58 for 4×50). Wall-clock C is reported separately. **The Python timing is not a hardware claim.**

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
| **1. Reference model** | `/usr/bin/python3 -m pytest app/husky -q -p no:cacheprovider` (84 tests on 2026-10-29); `python3 -B app/husky/hw_estimates.py` | – | All pass (done, model note §11) |
| **2. A300 in Gazebo: model, controller and interface validation** | V1–V13 (§2.1): controller limits and ramps, `preserve_turning_radius`, timeout, stamp pass-through, the mux-selected source (V12), freshness and stamp quality of `cmd_vel_out` and odometry (V13), and the effective-track identification against Gazebo ground truth. The ownership state machine is complete and passes its offline tests (done: `test_command_ownership.py`). | Stage 1; container verified | All checks recorded; the state machine's age bounds re-derived from V13; `W_eff` with its uncertainty (simulation pipeline only). |
| **3. SIL screen: Python backend, then the PC C-model backend** | §4.3: the node (`app/husky/sil/`) in the A300 closed loop, with feedback from `odom/filtered` and scoring from Gazebo ground truth. Configurations 4×50, 7×30, 9×24 (RK4); predictor plain vs `sim-derived` yaw gains; the same paths and seeds; plus teleop takeover and lock runs during SIL. Then backend (b), the PC C model with a unicycle configuration, on the same scenarios and seeds, compared with Python. | Stage 2 passing, **including V13-active, the V11 investigation and V14** | Tracking RMSE and max error, **violations (must be 0)**, deadline misses, ownership and fallback events, each reported separately. Commands never trigger the controller's own limiter. A screened configuration is selected; the C model agrees with Python within the agreed tolerance. **Review of the SIL results before stage 4.** |
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

### 4.3 Stage 3a: Python SIL in the A300 simulation (definition, runs, pass criteria)

**Setup:**

- shells A (simulation, headless or with GUI) and B (IMU bridge launch) as in §2.1;
- shell C: the simulation-only SIL estimator (GPT-6 2026-11-19, option b): `ros2 launch $H/launch/a300_sil_estimator.launch.py` (variant `driver`; feedback `sil/driver/odom/filtered`, IMU gate on `sil/driver/imu`). `run_batch.sh` uses it by default; `FEEDBACK=platform/odom/filtered IMU=sensors/imu_0/data` gives the generated-EKF drift reference;
- the real-time factor 0.25 **with gravity and the step size in the same request** (V15); `sil/run_batch.sh` sets it and drop-tests the robot;
- the robot at rest in an open area of the warehouse. The paths are relative to the pose where the MPC takes ownership: a line of 3 m ahead, a 1 m-radius circle to the left, and a figure-eight of two 1 m-radius lobes (2 m × 4 m footprint centred on the start). From the spawn pose (origin, heading +x) both stay clear of `shelf_7` at (0.4, −2) m. `run_batch.sh` puts the robot back at the origin before every run; run the line alone.

**Node behaviour found in the first runs (2026-11-19), now in the code:**

- the node arms only after twist_mux has matched its publisher (discovery took up to 1.9 s, V11) and only with fresh filtered odometry and IMU; a batch run (`--auto-arm`) ends when it leaves ownership, as it cannot re-arm itself;
- the executor is single-threaded: with the multi-threaded executor the callbacks competing for the GIL made a 7×30 solve take 130–480 ms instead of 35 ms. C_sim is the wall-clock time since the tick start times the real-time factor measured between ticks, because the node clock does not advance during a tick;
- the ground truth also logs height and tilt; a run with tilt > 0.35 rad is invalid (`validity` in the report).

**Runs** (same paths and seeds for every configuration and gain):

```bash
H=~/nmpc_hls/.claude/worktrees/agent-a7f53c7be12d59b73/app/husky
cd $H
bash sil/run_batch.sh      # 2 paths x 3 sizes x 2 gains x 3 seeds = 36 runs, about 1.8 h; scores each run
                           # and stops at the first run without ownership or with a tilted robot
# one run by hand, with a takeover and a lock while owned (shell D):
python3 -m sil.sil_node --sim --path figure8 --pso 7x30 --gain plain --seed 1 --auto-arm --rtf 0.25 \
    --ros-args -p use_sim_time:=true
ros2 topic pub -r 10 -t 20 /a300_00000/joy_teleop/cmd_vel geometry_msgs/msg/TwistStamped "{}"   # takeover -> foreign, idle
ros2 service call /sil_mpc/arm std_srvs/srv/Trigger "{}"                                         # explicit re-arm afterwards
ros2 topic pub --once /a300_00000/platform/safety_stop std_msgs/msg/Bool "{data: true}"          # lock -> disarmed
ros2 topic pub --once /a300_00000/platform/safety_stop std_msgs/msg/Bool "{data: false}"
```

Each run writes `<out>/<path>_<cfg>_nu25_<gain>_seed<k>_<time>/` (`run_batch.sh`: `~/logs/sil_batch_<date>/`):

- `meta.json`, `ticks.csv`, `transitions.csv`, `episodes.csv`;
- `feedback.csv`, `wheel_odom.csv`, `truth.csv`, `applied.csv`, `diagnostics.csv`;
- `report.json` after scoring.

**`--gain sim-derived`** applies the simulator-identified yaw-rate gains in the **predictor** only, by
motion class: 0.952 for in-place motion (`abs(v)` < 0.05 m/s) and 0.825 otherwise. It is labelled
`sim-derived` in the logs. It is a diagnostic sensitivity run against the plain model, which stays
the baseline. It is never a robot parameter, and no pooled gain is selected.

**Reported separately for every run** (`sil.metrics`):

1. tracking against ground truth: position RMSE and max, heading RMSE and max, cross-track RMSE and max, per episode;
2. constraint violations: of the solutions (rejected ones counted) and of the published commands;
3. deadline C: simulation time (the SIL deadline) and wall clock (Python only), with the misses;
4. fallback events by kind;
5. ownership transitions;
6. diagnostics: feedback vs truth and wheel odometry vs truth;
7. validity: maximum ground-truth tilt (the run is not scored when the robot tipped).

**Pass criteria (provisional; to be agreed at the SIL review):**

- published-command violations = 0 in every run;
- no MPC publication outside `arming`/`owned` (by construction; cross-checked with a `mux_timing.py capture` during the takeover run);
- ownership: one arming → owned per run, `foreign` and `disarmed` exactly when a takeover or lock is applied, and no automatic re-arm;
- deadline: C_sim ≤ 40 ms with misses ≤ 1 % of solved ticks at the chosen real-time factor;
- tracking: PLACEHOLDER_ cross-track RMSE ≤ 0.10 m and max ≤ 0.25 m on circle and figure-eight at 0.3 m/s;
- no `stop` fallbacks in nominal runs;
- ground truth never in the feedback path (code review: `controller.py` has no truth input).

The screened configuration (size, `Nu`, weights) is chosen from these results at the review, before
stage 3b (C model) and the RK4 HLS prototype.

## 5. Order of work (proposal)

1. User: §1 (Docker), V1–V13 and the effective-track identification (done for idle V13), then V13-active, the V11 investigation, V14 (IMU and EKF) and V15 (real-time factor) in §2.1, with the IMU bridge launch in every run.
2. Ownership/safety state machine: done offline (`command_ownership.py`); wire it into the node first, before any optimizer.
3. Stage 3a: Python-backend SIL screen in the A300 simulation (§4.3); thresholds re-derived from V13-active first.
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
