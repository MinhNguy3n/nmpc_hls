# Gate G0 for the lab Husky A200: read-only checklist

**For:** the user. §1–§4 are run on the robot's onboard computer (or a machine on its ROS network). §4b is read on the OptiTrack PC and on the machine that runs any existing bridge. The plan's §4.0 G0 is written for the A300 on ROS 2 Jazzy. This version works whether the A200 runs ROS 1 or ROS 2.

**Status:** not run. Every value below is `PLACEHOLDER_` until it is read off the robot.

**Rules:**
- **Read-only.** Every command lists, prints or measures; none publishes, calls a service, changes a parameter or moves the robot.
- **E-stop pressed** for the whole session, so the motors cannot be driven even by mistake. Reading parameters and topics does not need the motors.
- **No values by analogy.** A200 values come only from this robot or from Clearpath's A200 documentation, never from the A300 model note. If a command fails, record the failure and the error text: it is information too.
- Save every output to a file. Paste the record table (§5) back to the agent with the files' content or location.

**Where to run it (user-reported, 2026-11-21):** the robot's onboard computer is an NVIDIA Thor, which drives the MCU over the Prolific PL2303 USB-serial cable. Run §1–§3 in a shell **on the Thor** (SSH or keyboard and monitor), with the PL2303 cable plugged into the Thor as the lab left it. Never run them through that cable from another machine: it reaches the MCU, not a computer.

```bash
mkdir -p ~/g0_a200_$(date +%Y%m%d) && cd ~/g0_a200_$(date +%Y%m%d)   # all outputs go here
cat /etc/nv_tegra_release 2>&1 | tee jetson.txt                         # Jetson Linux (L4T) release
dpkg -l | grep -iE 'nvidia-l4t-core|nvidia-jetpack' 2>&1 | tee -a jetson.txt
ls -l /dev/prolific /dev/serial/by-id/ 2>&1 | tee mcu_port.txt          # which port the stack uses for the MCU
```

---

## 1. Identify the stack (both cases)

```bash
lsb_release -a                                   2>&1 | tee os.txt
uname -a                                         2>&1 | tee -a os.txt
printenv | grep -iE 'ros|rmw|domain|cyclone|fastrtps' | sort  2>&1 | tee ros_env.txt
ls /opt/ros                                      2>&1 | tee ros_distros.txt
dpkg -l | grep -iE 'husky|clearpath|twist-mux|robot-localization|vrpn|mocap|natnet|optitrack' 2>&1 | tee packages.txt
systemctl list-units --all | grep -iE 'ros|husky|clearpath'   2>&1 | tee services.txt
ls /etc/ros /etc/clearpath 2>&1 | tee etc_dirs.txt   # either may not exist
```

Then decide:
- **ROS 1** (Melodic on 18.04 or Noetic on 20.04): `rosversion -d` prints a distro, and `ROS_MASTER_URI` is set → §2.
- **ROS 2** (for example Humble or Jazzy): `printenv ROS_DISTRO` names a ROS 2 distro, and `ros2` exists → §3.
- **Both installed:** record which one the robot's startup service actually launches (`services.txt`, then `systemctl cat <unit>`).

```bash
systemctl cat <the ros/husky/clearpath unit found above>  2>&1 | tee startup_unit.txt
```

## 2. ROS 1 branch (Melodic / Noetic)

Source the environment the robot itself uses (for example `source /etc/ros/setup.bash`, if `etc_dirs.txt` shows it; otherwise `/opt/ros/<distro>/setup.bash`).

```bash
rosversion -d                         2>&1 | tee r1_distro.txt
echo $ROS_MASTER_URI $ROS_IP $ROS_HOSTNAME  2>&1 | tee r1_network.txt
rosnode list                          2>&1 | tee r1_nodes.txt
rostopic list -v                      2>&1 | tee r1_topics.txt
rosparam list                         2>&1 | tee r1_param_names.txt
rosparam dump r1_params_all.yaml      # read-only: writes the robot's parameters to a local file
```

**2.1 Velocity controller.** Find its name in `r1_nodes.txt` / `r1_param_names.txt` (Clearpath's ROS 1 Husky packages usually call it `husky_velocity_controller`; confirm, don't assume).

```bash
C=/husky_velocity_controller    # replace with the name found
rosparam get $C                 2>&1 | tee r1_controller.yaml
for p in type wheel_separation wheel_separation_multiplier wheel_radius wheel_radius_multiplier \
         cmd_vel_timeout publish_rate base_frame_id odom_frame_id enable_odom_tf publish_cmd \
         linear/x/has_velocity_limits linear/x/max_velocity linear/x/min_velocity \
         linear/x/has_acceleration_limits linear/x/max_acceleration linear/x/min_acceleration \
         angular/z/has_velocity_limits angular/z/max_velocity angular/z/min_velocity \
         angular/z/has_acceleration_limits angular/z/max_acceleration angular/z/min_acceleration \
         pose_covariance_diagonal twist_covariance_diagonal; do
  printf "%s: " $p; rosparam get $C/$p 2>&1; done | tee r1_controller_items.txt
rosparam get /husky_hardware_interface 2>&1 | tee r1_hardware.yaml   # name may differ; see r1_param_names.txt
```

**2.2 Command path and `twist_mux`.**

```bash
rostopic info $C/cmd_vel              2>&1 | tee r1_cmd_vel_info.txt    # type: Twist or TwistStamped; publishers/subscribers
rostopic info /cmd_vel                2>&1 | tee -a r1_cmd_vel_info.txt
rosnode list | grep -i mux            2>&1 | tee r1_mux_nodes.txt
rosparam get /twist_mux               2>&1 | tee r1_twist_mux.yaml      # topics (name, topic, timeout, priority) and locks
rosnode info /twist_mux               2>&1 | tee r1_twist_mux_info.txt  # which topics it really subscribes/publishes
```

**2.3 E-stop and status.**

```bash
rostopic list | grep -iE 'stop|status|diagnostic|lock'  2>&1 | tee r1_status_topics.txt
rostopic info /status                 2>&1 | tee r1_status_info.txt     # if present
rostopic echo -n 1 /status            2>&1 | tee r1_status_sample.txt   # with E-stop pressed: does a field show it?
rostopic echo -n 1 /diagnostics       2>&1 | tee r1_diagnostics_sample.txt
```

Record which topic and field report the E-stop, and whether `twist_mux` has a lock tied to it (§2.2 output).

**2.4 Odometry, IMU and EKF.**

```bash
rostopic list | grep -iE 'odom|imu|filtered'  2>&1 | tee r1_state_topics.txt
rostopic hz -w 200 $C/odom            2>&1 | head -20 | tee r1_odom_hz.txt     # stop after ~10 s (Ctrl-C)
rostopic echo -n 1 $C/odom            2>&1 | tee r1_odom_sample.txt
rostopic hz -w 200 /imu/data          2>&1 | head -20 | tee r1_imu_hz.txt      # topic from r1_state_topics.txt
rostopic hz -w 200 /odometry/filtered 2>&1 | head -20 | tee r1_ekf_hz.txt      # if an EKF runs
rosnode list | grep -iE 'ekf|localization'  2>&1 | tee r1_ekf_nodes.txt
rosparam get /ekf_localization        2>&1 | tee r1_ekf.yaml                   # name from r1_ekf_nodes.txt
rosrun tf tf_echo odom base_link      2>&1 | head -8 | tee r1_tf.txt          # frame names from the odometry sample
```

## 3. ROS 2 branch

Source the robot's own setup (on Clearpath ROS 2 robots, the generated workspace and `/etc/clearpath/`; confirm on the robot). The namespace `<ns>` is in the topic names (for example `/a200_XXXX/...`; read it, don't assume it).

```bash
printenv ROS_DISTRO ROS_DOMAIN_ID RMW_IMPLEMENTATION 2>&1 | tee r2_env.txt
ls -R /etc/clearpath 2>&1 | head -60  | tee r2_etc_clearpath.txt
cat /etc/clearpath/robot.yaml         2>&1 | tee r2_robot.yaml          # if present
ros2 node list                        2>&1 | tee r2_nodes.txt
ros2 topic list -t                    2>&1 | tee r2_topics.txt
```

**3.1 Velocity controller and `twist_mux`.** These are the A300 commands of plan §4.0 with A200 names: take the node names from `r2_nodes.txt`.

```bash
ros2 param dump /<ns>/platform_velocity_controller  2>&1 | tee r2_controller.yaml   # name from r2_nodes.txt
ros2 param dump /<ns>/twist_mux                     2>&1 | tee r2_twist_mux.yaml
ros2 param dump /<ns>/controller_manager            2>&1 | tee r2_controller_manager.yaml
ros2 topic info -v /<ns>/cmd_vel                    2>&1 | tee r2_cmd_vel_info.txt  # type: Twist or TwistStamped
ros2 topic info -v /<ns>/platform/cmd_vel_unstamped 2>&1 | tee -a r2_cmd_vel_info.txt  # if it exists
```

From `r2_controller.yaml`, record: `wheel_separation`, `wheel_separation_multiplier`, `wheel_radius` (+ multipliers), the linear and angular velocity and acceleration limits, `cmd_vel_timeout`, `publish_rate`, `publish_limited_velocity` and the limited-command topic, `use_stamped_vel`, and the odometry frames.

**3.2 E-stop, odometry, IMU, EKF.**

```bash
ros2 topic list -t | grep -iE 'stop|status|diagnostic|lock'  2>&1 | tee r2_status_topics.txt
ros2 topic echo --once /<ns>/platform/emergency_stop 2>&1 | tee r2_estop_sample.txt    # name from r2_status_topics.txt
ros2 topic hz /<ns>/platform/odom                   2>&1 | head -20 | tee r2_odom_hz.txt     # Ctrl-C after ~10 s
ros2 topic hz /<ns>/platform/odom/filtered          2>&1 | head -20 | tee r2_ekf_hz.txt
ros2 topic hz /<ns>/sensors/imu_0/data              2>&1 | head -20 | tee r2_imu_hz.txt
ros2 param dump /<ns>/ekf_node                      2>&1 | tee r2_ekf.yaml               # name from r2_nodes.txt
ros2 run tf2_ros tf2_echo odom base_link            2>&1 | head -8 | tee r2_tf.txt
# if r2_topics.txt shows /<ns>/tf rather than /tf (namespaced TF), echo that one instead:
ros2 run tf2_ros tf2_echo odom base_link --ros-args -r /tf:=/<ns>/tf -r /tf_static:=/<ns>/tf_static 2>&1 | head -8 | tee r2_tf.txt
```

## 4. On the robot, without software

| Item | How |
|---|---|
| Track width | Tape measure, centre of the left tyres to centre of the right tyres, at the axles; also between the outer edges |
| Wheelbase | Front axle to rear axle |
| Tyre diameter | Tape around the tyre (circumference / π), each side if they differ |
| Tyre type | Photo (stock, all-terrain, indoor) |
| Payload | What is mounted, approximate mass, and whether a top plate or rails are fitted (photo) |
| E-stop hardware | Where the buttons are; whether a wireless E-stop is fitted (photo) |
| Clock sync | `timedatectl`; `chronyc tracking` or `ntpq -p` (matters for the OptiTrack latency, Task C) |
| Battery voltage | From the status sample (§2.3 `r1_status_sample.txt` / §3.2 status topic), using the field name as printed; otherwise the robot's own display. Record the voltage and the time it was read |
| Firmware / MCU version | From the status or diagnostics sample if it is reported there; otherwise the label on the MCU board or Clearpath's tooling already installed on the robot. Write "not found" rather than guess |

## 4b. OptiTrack (lab facts)

These are read on the OptiTrack PC in Motive and on whatever machine already runs a ROS bridge. Same rules as above: **look, don't change.** Open the panes, read them and take screenshots. Don't edit the streaming settings, rigid bodies or calibration, and don't save the Motive project. Don't start a new bridge for G0. If a bridge is already running, measure it as it is.

| Item | Where / how |
|---|---|
| Motive version | Motive: Help → About (screenshot) |
| Protocol | Motive's streaming pane: is NatNet streaming on? Record the NatNet version, the local interface, multicast or unicast, and the command/data ports. Is VRPN streaming on? Record its port. Screenshot the whole pane |
| Up axis | The streaming pane's "Up Axis" setting (Y or Z), as set. ROS frames are Z-up, so the bridge or Task C must convert |
| Frame rate | The camera system frame rate in Motive (Hz). If a bridge is already running, also measure the stream rate on the ROS side: `rostopic hz <pose topic>` (ROS 1) or `ros2 topic hz <pose topic>` (ROS 2), about 10 s → `optitrack_hz.txt` |
| Rigid body for the Husky | Does one exist? Record its name, its streaming ID and its marker count. Where is the pivot point relative to the robot (centre of the base? top plate? measure the offset to the base centre with a tape)? Is its orientation aligned with the robot's forward axis? Take a photo of the markers on the robot and a screenshot of the rigid body's properties. **If none exists, write "none".** Creating one is a later step with the lab's agreement |
| Existing ROS bridge | Which package (for example `vrpn_client_ros`, `mocap_optitrack`, `mocap4r2`, `vrpn_mocap`, or a lab-specific one), which machine runs it (the OptiTrack PC, a lab PC or the robot), its pose topic, message type and frame id. From `packages.txt` and `r1_topics.txt` / `r2_topics.txt`, or ask the lab |
| Time sync of the OptiTrack PC | Is it synced to the same time source as the robot? (Windows: `w32tm /query /status`.) Is a NatNet latency or timestamp field used by the bridge? |
| Network addressing | The OptiTrack PC's IP and subnet on the streaming interface (Motive's local interface, or `ipconfig` on the PC). The robot's IP (`ip -br addr` on the robot → `robot_ip.txt`). The KV260's IP, taken from the lab's records or the KV260's own config file: don't power it up or ping it for G0. Are all three on one subnet? Is each link wired or Wi-Fi? Is multicast allowed between them? |
| Free tracked floor area | The free floor inside the tracked volume, measured with a tape (length × width, plus obstacles; a sketch with dimensions). Check tracking without the robot moving: carry a hand-held marker set or the calibration wand at the height of the Husky's markers around that area, and note where Motive loses it. Don't take the markers off the robot. The robot stays switched off with its E-stop pressed |

## 5. Record template

Paste this table back filled in. Keep `PLACEHOLDER_` wherever a value could not be read, and put the reason in "Notes".

| # | Item | Value | Unit | Command / file | Date | Replaces (model note / code) | Notes |
|---|---|---|---|---|---|---|---|
| 1 | OS and kernel; Thor JetPack / L4T release; MCU serial port | PLACEHOLDER_ | — | `os.txt`, `jetson.txt`, `mcu_port.txt` | | Task D deployment (where the bridge runs) | |
| 2 | ROS version and distro; stack launched at boot | PLACEHOLDER_ | — | `ros_env.txt`, `startup_unit.txt` | | plan §3 node architecture (ROS 1 bridge needed?) | |
| 3 | Husky / Clearpath package versions | PLACEHOLDER_ | — | `packages.txt` | | — | |
| 4 | Robot namespace / serial | PLACEHOLDER_ | — | topic names | | `--namespace` of the SIL node | |
| 5 | Velocity controller name and type | PLACEHOLDER_ | — | §2.1 / §3.1 | | plan §3.1 | |
| 6 | `wheel_separation` | PLACEHOLDER_ | m | §2.1 / §3.1 | | §6.1 "Effective track (kinematics)" | |
| 7 | `wheel_separation_multiplier` | PLACEHOLDER_ | — | §2.1 / §3.1 | | §6.1 `wheel_separation_multiplier` (W·m) | |
| 8 | `wheel_radius` (× multiplier) | PLACEHOLDER_ | m | §2.1 / §3.1 | | §6.1 "Effective tyre radius" | |
| 9 | Linear max velocity (fwd / rev) | PLACEHOLDER_ | m/s | §2.1 / §3.1 | | §6.1 "Controller `linear.x`"; SIL envelope `v_max` | |
| 10 | Linear max acceleration / deceleration | PLACEHOLDER_ | m/s² | §2.1 / §3.1 | | §6.1 "Controller `linear.x`"; `accel_max`, `decel_max`; `_du_max[0]` | |
| 11 | Angular max velocity | PLACEHOLDER_ | rad/s | §2.1 / §3.1 | | §6.1 "Controller `angular.z`"; `omega_max`; `omega_per_unit` | |
| 12 | Angular max acceleration / deceleration | PLACEHOLDER_ | rad/s² | §2.1 / §3.1 | | `alpha_accel_max`, `alpha_decel_max`; `_du_max[1]` | |
| 13 | `cmd_vel_timeout` | PLACEHOLDER_ | s | §2.1 / §3.1 | | §6.1 "`cmd_vel` timeout" | |
| 14 | Controller / odometry `publish_rate` | PLACEHOLDER_ | Hz | §2.1 / §3.1 | | §6.1 "Controller and odometry rate" | |
| 15 | Limited-command output (`publish_cmd` / `publish_limited_velocity`) and its topic | PLACEHOLDER_ | — | §2.1 / §3.1 | | the applied-command (`cmd_vel_out`) input of `command_ownership` | |
| 16 | Command topic and message type (Twist / TwistStamped) | PLACEHOLDER_ | — | §2.2 / §3.1 | | §6.1 "Command" | |
| 17 | `twist_mux` inputs: topic, priority, timeout | PLACEHOLDER_ | — | §2.2 / §3.1 | | `command_ownership` priorities | |
| 18 | `twist_mux` locks: topic, priority, timeout | PLACEHOLDER_ | — | §2.2 / §3.1 | | `command_ownership` locks | |
| 19 | E-stop: reporting topic, type, field; mux lock | PLACEHOLDER_ | — | §2.3 / §3.2 | | §6.1 "E-stop"; safety items (§6) | |
| 20 | Wheel odometry topic, rate (measured), frames | PLACEHOLDER_ | Hz | §2.4 / §3.2 | | feedback cross-check (Task C) | |
| 21 | IMU: present, topic, rate (measured), covariance | PLACEHOLDER_ | Hz | §2.4 / §3.2 | | SIL estimator inputs | |
| 22 | EKF: present, inputs (which states fused), output rate | PLACEHOLDER_ | Hz | §2.4 / §3.2 | | §6.1 "EKF `odom/filtered` rate" | |
| 23 | Network: master URI / domain id, RMW, robot IP | PLACEHOLDER_ | — | `r1_network.txt` / `r2_env.txt` | | backend (c) bridge (Task D) | |
| 24 | Clock sync status | PLACEHOLDER_ | — | §4 | | OptiTrack latency (Task C) | |
| 25 | Measured track width (tyre centres / outer edges) | PLACEHOLDER_ | m | §4 | | §6.1 "Track width (spec)" | |
| 26 | Measured wheelbase | PLACEHOLDER_ | m | §4 | | §6.1 "Wheelbase" | |
| 27 | Measured tyre diameter; tyre type | PLACEHOLDER_ | m | §4 | | §6.1 "Effective tyre radius" (cross-check) | |
| 28 | Payload and mass | PLACEHOLDER_ | kg | §4 | | the stopping distance and slip assumptions | |
| 29 | E-stop hardware: buttons, wireless E-stop fitted? | PLACEHOLDER_ | — | §4 | | §6.1 "E-stop" | |
| 30 | Battery voltage (and time read) | PLACEHOLDER_ | V | §4 | | test-session preconditions (G1) | |
| 31 | Firmware / MCU version | PLACEHOLDER_ | — | §4 | | — (record; the stop behaviour may depend on it) | |
| 32 | Motive version | PLACEHOLDER_ | — | §4b screenshot | | Task C bridge choice | |
| 33 | Streaming protocol: NatNet (version, multicast/unicast, interface, ports) / VRPN (port) | PLACEHOLDER_ | — | §4b screenshot | | Task C bridge choice | |
| 34 | Up axis (Y / Z) | PLACEHOLDER_ | — | §4b screenshot | | Task C frame transform (OptiTrack → `map`/`odom`) | |
| 35 | Camera frame rate; measured ROS-side pose rate | PLACEHOLDER_ | Hz | §4b, `optitrack_hz.txt` | | Task C freshness/dropout gate | |
| 36 | Husky rigid body: name, streaming ID, marker count | PLACEHOLDER_ | — | §4b screenshot, photo | | Task C state input | |
| 37 | Rigid-body pivot offset from the base centre; heading aligned with forward? | PLACEHOLDER_ | m, rad | §4b tape, screenshot | | Task C pivot → `base_link` transform | |
| 38 | Existing ROS bridge: package, machine, pose topic, type, frame id | PLACEHOLDER_ | — | §4b | | Task C / Task D bridge | |
| 39 | OptiTrack PC time sync; latency field used? | PLACEHOLDER_ | — | §4b | | Task C latency and freshness | |
| 40 | Network: OptiTrack PC IP/subnet, robot IP, KV260 IP; one subnet?; wired / Wi-Fi; multicast allowed? | PLACEHOLDER_ | — | §4b, `robot_ip.txt` | | Task D deployment (where the bridge and the server run) | |
| 41 | Free tracked floor area (L × W, obstacles, dropout spots) | PLACEHOLDER_ | m | §4b tape, sketch | | Task C geofence; safety question 4 (§6) | |

**Still `PLACEHOLDER_` after G0** (identify by experiment, after G1): side speed and side acceleration limits, effective track / ICR on the lab floor, command-to-motion delay.

## 6. Safety questions for the user and supervisor (to answer before G1, not here)

These are questions, not answers:

1. **E-stop:** where are the E-stop buttons on this A200, who stands at one during every test, and how is it tested before each session (pressed → the motors are disabled and the robot does not move on a command)?
2. **Wireless E-stop:** is one fitted? If so, what is its range in the lab, and does it trip on loss of link?
3. **Lab rules:** who must be present, what training is required, and are there booking, signage or area rules for a moving robot in the OptiTrack volume?
4. **Space:** the size of the free floor inside the OptiTrack volume; the walls, people and equipment nearby; the geofence margin.
5. **Takeover:** who holds the joystick or teleop with priority over the controller, and how a takeover is shown to the operator?
6. **Envelope (G1):** the first speed and yaw-rate caps (plan G1 starts at ≤ 0.3 m/s), forward-only or not, and who signs off the written envelope?
7. **Surface and payload:** the floor type and whether the payload stays fixed for all tests?
8. **Stop behaviour:** what the robot does on `cmd_vel` timeout, on loss of OptiTrack and on loss of network, and who checks it before the first closed-loop run?

## 7. After the values come back

- The agent fills model note §6 with an A200 table, each row citing this G0 record (file and date).
- The A300 values stay where they are (Gazebo SIL); none is copied to the A200.
- The `UNICYCLE_CONFIG` block and the model constants stay **PROVISIONAL (A300 SIL)** until G1 sets the A200 envelope.
- No changes to the HLS weights before 27 Nov.
