# DRAFT upstream issue: empty IMU bridge for the A300 in `clearpath_simulator` (Jazzy)

Status: **draft, not filed.** The user decides whether and where to file it. The likely target is
https://github.com/clearpathrobotics/clearpath_simulator/issues (the generator that writes the
Gazebo bridge files is part of that repository's `clearpath_generator_gz`). Fill in every
`<FILL: …>` before filing, and re-run the reproduction on the versions you report.

---

**Title:** A300 simulation: generated `sensors/config/imu_0.yaml` is an empty bridge list, so the IMU never reaches ROS and the EKF runs on wheel odometry only

**Environment**

| Item | Value |
| --- | --- |
| OS (container) | Ubuntu 24.04, `osrf/ros:jazzy-desktop-full` base image |
| ROS 2 | Jazzy |
| Gazebo | Harmonic 8.11 (`gz sim --versions`: `<FILL: output>`) |
| Packages | `<FILL: output of the commands below>` |

```bash
apt-cache policy ros-jazzy-clearpath-simulator ros-jazzy-clearpath-gz ros-jazzy-clearpath-generator-gz \
    ros-jazzy-clearpath-generator-common ros-jazzy-clearpath-config ros-jazzy-clearpath-common \
    ros-jazzy-clearpath-sensors-description ros-jazzy-clearpath-platform-description ros-jazzy-ros-gz-bridge
dpkg -l | grep -E "ros-jazzy-clearpath|ros-jazzy-ros-gz" | awk '{print $2, $3}'
gz sim --versions
```

**robot.yaml** (minimal; follows the `clearpath_config` jazzy samples `a300_default.yaml` and
`a300_outline.yaml`):

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

**Steps to reproduce**

```bash
mkdir -p ~/clearpath && cp robot.yaml ~/clearpath/
export GZ_IP=127.0.0.1
ros2 launch clearpath_gz simulation.launch.py setup_path:=$HOME/clearpath/ world:=warehouse rviz:=false
# second shell, once the robot has spawned:
cat ~/clearpath/sensors/config/imu_0.yaml
gz topic -l | grep imu
gz topic -i -t /a300_00000/sensors/imu_0/data
ros2 topic info -v /a300_00000/sensors/imu_0/data
ros2 node list | grep -i imu
grep -n -A12 "imu0" ~/clearpath/platform/config/localization.yaml
```

**Observed** (2026-10-28):

- `~/clearpath/sensors/config/imu_0.yaml` contains `[]`.
- Gazebo publishes `/a300_00000/sensors/imu_0/data` (`gz.msgs.IMU`, frame `imu_0_link`).
- The ROS topic `/a300_00000/sensors/imu_0/data` has **0 publishers**. The bridge node for the IMU (`imu_0_gz_bridge`) runs with the empty list, so it bridges nothing.
- `ekf_node` subscribes to the IMU topic: `localization.yaml` has `imu0` fusing yaw rate and x acceleration. It therefore fuses wheel odometry only. At a 0.3 rad/s in-place turn, `platform/odom/filtered` reported 0.300 rad/s, while Gazebo ground truth was 0.2855 rad/s.
- With a manual bridge, `platform/odom/filtered` settles at 0.2855 rad/s (matching ground truth) in about 0.15 s:

  ```bash
  ros2 run ros_gz_bridge parameter_bridge "/a300_00000/sensors/imu_0/data@sensor_msgs/msg/Imu[gz.msgs.IMU" \
      --ros-args -p use_sim_time:=true
  ```

**Expected:** the generated `imu_0.yaml` contains a `GZ_TO_ROS` entry for the IMU topic, as for
the other sensors, so that the EKF receives the IMU in simulation.

**Notes**

- `<FILL: whether other sensors' generated bridge files are populated in the same run (ls ~/clearpath/sensors/config/; cat each)>`
- `<FILL: whether the same robot.yaml on a Humble/Fortress setup generates a non-empty IMU bridge, if available>`
- The generator rewrites the setup directory on every launch, so editing the file by hand does not persist.

---

Local workaround in this project (not part of the issue):
`app/husky/launch/a300_sim_imu_bridge.launch.py` with `app/husky/launch/a300_imu_bridge.yaml`.
`app/husky/imu_checks.py check-generated` fails loudly while the generated list is empty, and
signals when the generator starts bridging the IMU, so the extra bridge can be dropped.
