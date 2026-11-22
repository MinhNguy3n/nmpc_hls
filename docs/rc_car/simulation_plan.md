# Reely NEW1 in Gazebo: simulation plan and status

**Prompts:**
- `.copilot/chats/2026-11-21-claude-to-claude-husky-agent-rc-car-prompt.md` (base);
- `.copilot/chats/2026-11-22-claude-to-claude-rc-car-phase1-review-phase2-go.md` (STOP 1 review: the C3 diagnostic, the Pixhawk 4 additions);
- `.copilot/chats/2026-11-22-claude-to-claude-rc-car-phase2-review-phase3-go.md` (STOP 2 review: the phase 3 conditions);
- `.copilot/chats/2026-11-22-claude-to-claude-rc-car-phase3-review-phase4-go.md` (STOP 3 review: the NMPC conditions, steps 4a/4b);
- `.copilot/chats/2026-11-22-claude-to-claude-rc-car-lab-facts.md` (lab facts: 100 m² floor, KV260 on the car, track and full lock). The latest wins where they differ.

**Parameters:**
- `docs/rc_car/parameters.md`, used by the simulation through `app/rc_car/params/reely_new1.yaml`;
- the command chain (actuator, mux, mocap): `app/rc_car/params/command_chain.yaml`. **Every chain value is assumed.**

**Every result here is a simulation result**, with `PLACEHOLDER_` vehicle values and assumed chain values. None is a vehicle result.

## Status

| Phase | Status | Report |
|---|---|---|
| 0: park the Husky, import the Ackermann code, car parameters | Done | `.copilot/chats/2026-11-21-claude-rc-car-reply-phase0.md` |
| 1: the car in Gazebo, C1–C6 | Done: C1–C5 pass, C6 RTF 0.985 | `.copilot/chats/2026-11-22-claude-rc-car-reply-phase1.md` |
| 1b: C3 scrub diagnostic | **Done: the wide contact cylinders cause it** (section 5) | phase 2 reply |
| 2: actuator emulation, mux and watchdog, OptiTrack emulator | **Done** (section 6) | `.copilot/chats/2026-11-22-claude-rc-car-reply-phase2.md` |
| 3: baseline (Nav2 RPP → own `twist_to_ackermann` → mux → actuator) | **Done** (section 7) | `.copilot/chats/2026-11-22-claude-rc-car-reply-phase3.md` |
| 4a: NMPC node, smoke runs, run budget (lab facts folded in) | **Done** (section 7.8) | `.copilot/chats/2026-11-22-claude-rc-car-reply-phase4a.md` |
| 4b: the batch (B1–B4, 2026-11-22) and the comparison | **Done** (section 7.9) | `.copilot/chats/2026-11-22-claude-rc-car-reply-phase4.md` |
| R: PSO redesign, offline (ablation, tuning) | **Done offline** (section 7.11) | `.copilot/chats/2026-11-22-claude-rc-car-reply-redesign.md` |
| BR: redesign confirmation in Gazebo (1.5 kg payload) | **Done** (section 7.12) | `.copilot/chats/2026-11-22-claude-rc-car-reply-br.md` |
| BH: payload pairing, held-out seeds 6–10; offline solver-gap study | **Done** (section 7.14) | `.copilot/chats/2026-11-22-claude-rc-car-reply-bh.md` |

## 1. Files (`app/rc_car/`)

**Phase 1:**

| File | What |
|---|---|
| `params/reely_new1.yaml` | All vehicle parameters. Each has a value, unit, status (`spec_unverified`, `placeholder`, `design`) and source |
| `vehicle_params.py` | Loads and validates the file; `CarGeometry` (`kappa_max`, `r_min`, the inner-wheel joint limit); lists what is still `PLACEHOLDER_` |
| `ackermann_joints.py` | (applied speed, bicycle δ) → 2 steering angles + 4 wheel speeds: ideal Ackermann (note §6), no-slip wheel speeds about the ICR |
| `description/reely_new1.urdf.xacro` | Chassis, 2 steering knuckles, 4 wheels (4WD), `ros2_control` block; `base_link` = the rear-axle midpoint. Diagnostic arguments `drive:=rwd` and `contact_width:=<m>` (defaults give the normal model) |
| `config/controllers.yaml` | `controller_manager` at 200 Hz: `joint_state_broadcaster`, `steering_position_controller`, `wheel_velocity_controller`; steering P gain 0.5 |
| `config/controllers_rwd.yaml` | Diagnostic only: the wheel controller holds the rear wheels only |
| `worlds/lab_floor.sdf` | Flat floor (μ = 1, `PLACEHOLDER_`), 1 ms physics step, truth at 100 Hz; a `PLACEHOLDER_` 6 × 4 m OptiTrack outline with a 0.5 m geofence (visual only) |
| `launch/rc_car_sim.launch.py` | Gazebo (headless, or `gui:=true`), `/clock` bridge, `robot_state_publisher`, spawn, controller spawners, plant interface |
| `plant_interface_node.py` | `plant/ackermann_cmd` (*applied* speed and δ) → the joint commands, at once on every input and at 100 Hz. Sim-only guard: wheels stop after 0.5 s without input |
| `checks/bringup_analysis.py`, `checks/bringup_checks.py` | C1–C6 analysis (tested offline) and the runner; `--only-full-lock` for the diagnostic |

**Phase 2:**

| File | What |
|---|---|
| `params/command_chain.yaml` | Actuator profiles (`ideal`, `realistic_guess`, `fast_servo`), mux, freshness gate, mocap profiles (`ideal`, `realistic_guess`). All assumed; the PX4 counterpart of each item is in a comment |
| `actuator_model.py` | The command chain: link delay → output hold (ZOH, random phase) → VESC (lag, ramp limits) and servo (dead time, slew, saturation). Exact event timing |
| `actuator_node.py` | `actuator/ackermann_cmd` → the model → `plant/ackermann_cmd` every 2 ms (sim); `actuator/speed` (emulated VESC speed telemetry from the rear wheel joints) at 50 Hz |
| `command_mux.py` | Mux, setpoint watchdog, kill lock, mocap freshness gate (plain Python) |
| `command_mux_node.py` | Topics `teleop/ackermann_cmd`, `controller/ackermann_cmd`, `mux/lock`, `/mocap/<body>/pose`; services `mux/arm`, `mux/disarm`; output `actuator/ackermann_cmd`; `mux/state` (JSON) |
| `mocap_emulator.py` | OptiTrack emulator: decimation, dropout bursts, noise, delay = mocap processing + Wi-Fi (base + jitter + rare spikes); keeps the capture stamp |
| `mocap_emulator_node.py` | Gazebo truth → `/mocap/<body>/pose` (PoseStamped, **header stamp = capture time**); `mocap/stats` (JSON, including the simulation artifacts) |
| `gz_pose_stream.py` | Low-latency parser of `gz topic -e` for one model (section 6.5) |
| `launch/rc_car_stack.launch.py` | `rc_car_sim.launch.py` + mux + actuator + mocap emulator; arguments `profile`, `mocap_profile`, `seed` |
| `checks/chain_analysis.py`, `checks/chain_checks.py` | The delay distributions (D1–D3) and the live mux checks (S1–S5) in Gazebo; RTF during D1 |

**Phase 3:**

| File | What |
|---|---|
| `params/lab_area.yaml` | Tracked area `PLACEHOLDER_` 6 × 4 m, margin 0.5 m: every path scales from it |
| `paths.py` | Evaluation paths `line`, `circle` (2 laps), `figure_eight`, `lane_change`; tuning path `s_curve` |
| `twist_to_ackermann.py`, `twist_to_ackermann_node.py` | Own converter: δ = atan(L·ω/v), clamp, δ = 0 below `v_min` (`PLACEHOLDER_` 0.05 m/s); TwistStamped in, the stamp passed through |
| `pose_input.py` | The controllers' pose input: the newest capture stamp wins (shared by both arms) |
| `mocap_bridge_node.py` | Mocap pose → TF `map → base_link` (capture stamp) and `odom` (with the VESC speed) for Nav2 |
| `episode.py` | `episode/start` {t_start, seed}: paired seeds (mocap streams and PWM hold phases restart from the seed) |
| `config/baseline_rpp.yaml`, `launch/baseline_rpp.launch.py` | Nav2 RPP in a minimal `controller_server` (20 Hz, empty costmap, constant speed); **frozen lookahead** |
| `tracking_metrics.py` | Cross-track and heading error with a forward-only local match (no branch jumps at the crossing or on lap 2) |
| `checks/track_run.py` | The run harness: episodes, goal segments, safety-event counts, raw series |
| `checks/tuning_select.py`, `checks/plot_tracks.py` | The tuning rule, applied to the grid (either arm's axes); path vs reference plots |

**Phase 4a:**

| File | What |
|---|---|
| `params/nmpc.yaml` | Horizon (Nh = Nu = 25, Ts 0.05 s), sizes with the timing model's solve times (board; 10×60 extrapolated from the board's 10×40 measurement, no 10×60 board run), PSO parameters, weights (§7.7), τ terms |
| `nmpc_engine.py` | The emulated engine: HLS-structured PSO, §9.3 cost, exact-arc chord rollout in half precision with the table sine and `ap_fixed<32,17>` states (bit-identical to `hw_estimates.step_chord_hw`), plus float and double variants |
| `nmpc_controller.py` | Host side: limits from the YAML files, timing model, delay compensation, path-following reference, warm start, fallback, Scheduler |
| `nmpc_offline.py` | Offline closed loop (kinematic plant + actuator + mocap emulators) for tests and checks |
| `nmpc_node.py`, `launch/nmpc.launch.py` | The ROS node (PSO in a worker process; publishes at t_apply in sim time; overrun and busy counters) |
| `params/payload` entries in `reely_new1.yaml` | KV260 payload (`PLACEHOLDER_` 0.5 kg on top of the chassis), in the xacro's inertia |
| `checks/run_phase4_batch.sh` | The phase 4b batch, in four blocks, run by the user |
| `figures/make_screenshot_world.py` | Screenshot-only world variants (figures; never results) |

**Tests (offline, `/usr/bin/python3 -m pytest app/rc_car app/ackermann -q -p no:cacheprovider`): 206 passed**:
- 126 in `app/rc_car`:
  - phase 1: `test_ackermann_joints.py` 24, `checks/test_bringup_analysis.py` 15;
  - phase 2: `test_actuator_model.py` 21, `test_command_mux.py` 13, `test_mocap_emulator.py` 8, `test_gz_pose_stream.py` 3, `checks/test_chain_analysis.py` 3;
  - phase 3: `test_twist_to_ackermann.py` 11, `test_paths.py` 11, `test_tracking_metrics.py` 9, `test_seed_pairing.py` 3, `test_pose_input.py` 3, `checks/test_tuning_select.py` 2;
- 80 in `app/ackermann`.

`app/husky` (110) still passes: 316 in all. **After step 4a: 341** (151 car, 80 Ackermann, 110 Husky).

## 2. Docker

**No change is needed.** The existing image `nmpc-jazzy-sim:2026-10` already contains everything phases 1 and 2 use. These packages were checked in the image with `dpkg -l` on 2026-11-22:

| Package | Version |
|---|---|
| `ros-jazzy-gz-ros2-control` | 1.2.20 |
| `ros-jazzy-ros2-controllers` | 4.42.1 (includes `position_controllers` and `velocity_controllers`) |
| `ros-jazzy-controller-manager`, `ros-jazzy-joint-state-broadcaster` | (installed) |
| `ros-jazzy-ackermann-msgs` | 2.0.2 |
| `ros-jazzy-ackermann-steering-controller` | (installed; not used) |
| `ros-jazzy-nav2-regulated-pure-pursuit-controller` | 1.3.13 (for phase 3) |
| `ros-jazzy-xacro`, `ros-jazzy-robot-state-publisher`, `ros-jazzy-ros-gz-*`, `std_srvs` | (installed) |

**`twist_to_ackermann`** is not in the Jazzy apt repository. Phase 3 wrote its own converter (`twist_to_ackermann.py`) from the formula, not from the upstream code:
- the same interface (`Twist` in, `AckermannDriveStamped` out);
- δ = atan(L·ω/v), clamped to ±δ_max;
- δ = 0 below a speed floor `v_min` (`PLACEHOLDER_`), with a test for that case.

The ROS package is cited only as the interface it replaces.

## 3. Plant interface decision (spike, 2026-11-22)

**Chosen: ros2_control via `gz_ros2_control`** (the prompt's default).
- **It worked on the first headless launch.** All three controllers configured and activated; `/joint_states` runs at 200 Hz.
- **The vehicle geometry stays in tested Python** (`ackermann_joints.py`), not in a Gazebo plugin.
- **The steering reaches its commanded angles exactly:** the steady-state error is below 1e-4 rad.
- **The fallback (Gazebo's `AckermannSteering` system) was not needed.**

**Two details found:**
1. **The steering gain key.** `GazeboSimSystem` turns a position command into a joint velocity proportional to the position error. The gain lives on the node `gz_ros_control`, not `gz_ros2_control`; under the wrong key it silently stayed at 0.1.
2. **The steering joint's velocity limit must stay above the servo rate.** It is 30 rad/s (design), so all servo lag comes from the phase 2 model.

**The wheels are ideal velocity servos** (torque cap 2 N·m, `PLACEHOLDER_`). All speed and steering dynamics come from the phase 2 emulation.

## 4. Bring-up checks C1–C6 (simulation, 2026-11-22)

**Reports:** `~/nmpc_sim/logs/rc_car_bringup_20261122/`:
- `run2_steer30/` (the STOP 1 numbers);
- `run3_phase2_regression/`: after the phase 2 changes to the plant interface and the xacro. Identical C1–C4 results; adds the rear-axle speed to C3.

**Setup:** headless, throwaway container, `--memory 3g`. Commands go straight to the plant interface: no actuator emulation, no controller. The car is reset with `set_pose` before each scenario.

| Check | Result (run 3) | Threshold | Verdict |
|---|---|---|---|
| C1 rest, 9.5 s | max xy 0.000 mm, max yaw 0.000 mrad | ≤ 1 mm, ≤ 1 mrad | PASS |
| C2 v = 0.5 / 1.0 / 1.5 m/s | 0.5000 / 1.0000 / 1.5000 m/s; heading change ≤ 0.005 mrad | speed ±2 %, heading ≤ 10 mrad | PASS |
| C3 δ = ±0.270 rad, R_kin 0.9936 m | R +0.92 / −0.51 / −0.70 %; **rear-axle speed +2.76 / +0.36 / −0.15 %**; lateral speed 4.3 / 20.3 / 30.7 mm/s, at 0.5 / 1.0 / 1.5 m/s. Left and right identical; turn sign correct | radius ±3 %, sign | PASS |
| C3 δ = ±0.450 rad (full lock), 0.5 m/s, R_kin 0.5693 m | R +2.34 %; **rear-axle speed +4.71 %**; lateral speed 4.0 mm/s | ±3 % | PASS (close; diagnosed in section 5) |
| C4 steering at standstill | Steady state equals the Ackermann angles to < 1e-4 rad; 10–90 % rise 15–30 ms | ≤ 2 mrad | PASS |
| C5 rates and stamps | `/joint_states` 200.0 Hz; truth 95.1 Hz; plant commands 150 Hz (now also published on every input); stamp lag ≤ 31 ms; `use_sim_time` True | ≥ 90 % of nominal; lag ≤ 50 ms | PASS |
| C6 real-time factor | 0.988 | reported | — |

**From now on, C3 reports the rear-axle speed error and the lateral speed** next to the radius (review, section 2).

## 5. The C3 error: the wide contact cylinders (diagnostic, 2026-11-22)

**Question:** at low speed the rear axle runs faster than commanded and the radius is too large. Is that the over-constrained 4WD, or scrub of the 70 mm rigid contact cylinders?

**Setup:**
- the full-lock circle (δ = ±0.45 rad) at 0.5 m/s;
- four model variants, one throwaway container each;
- the variants are diagnostic arguments only, and their defaults give the normal model.

**Reports:** `~/nmpc_sim/logs/rc_car_c3_diag_20261122/<variant>/report.json`.

| Variant | Radius error | Rear-axle speed vs command | Lateral speed |
|---|---|---|---|
| Baseline (4WD, 70 mm contact) | +2.34 % | +4.71 % | 4.0 mm/s |
| (a) rear drive only, front wheels free (`drive:=rwd`) | **+3.81 %** | −3.30 % | 6.2 mm/s |
| (b) 4WD, 10 mm contact (`contact_width:=0.01`) | **−0.01 %** | +0.15 % | 2.3 mm/s |
| (a) + (b) | +0.29 % | −0.35 % | 1.9 mm/s |

Left and right turns gave identical numbers in every variant.

**Finding: the wide rigid contact cylinders cause the error; the over-constrained 4WD does not.**
- **Narrowing the contact (b) removes it.** The radius and the rear-axle speed then match the kinematics to 0.15 %.
- **Rear drive alone (a) makes it worse.** The free front wheels scrub with their wide contacts and drag; the rear wheels then slip (−3.3 % speed) and the radius grows further.
- **With narrow contacts, rear drive leaves a small +0.3 %.** That is the drag of the free-rolling front wheels.

**Model unchanged** (review: "don't change the model just to pass 3 %"). The real car has its own version of this effect: compliant tyres with a contact patch, differentials that split torque, not speed. Phase 4 must show the controller absorbing it.

`contact_width` stays available. Whether to narrow the contact for the comparison runs is a decision for the ARC session and the user. My suggestion: keep the 70 mm default, because it is the harder case for the controller.

## 6. Phase 2: command chain, mux and OptiTrack emulator (simulation, assumed parameters)

**The chain in simulation** (`launch/rc_car_stack.launch.py`):

```
teleop/ackermann_cmd ───┐
controller/ackermann_cmd ─> command_mux ─> actuator/ackermann_cmd ─> ackermann_actuator ─> plant/ackermann_cmd ─> plant interface ─> Gazebo
mux/lock, mux/arm ──────┘        ^
Gazebo truth ─> mocap_emulator ─> /mocap/reely_new1/pose (freshness gate; the controller's pose input)
```

**The planned vehicle chain** (review, section 4):
`OptiTrack → Wi-Fi → KV260 (NMPC) → serial link → Pixhawk 4 (PX4) → PWM → VESC + servo`.
The code uses generic names. The PX4 counterparts are below and in `params/command_chain.yaml`; **each must be checked against the PX4 version on the car.**

### 6.1 Actuator emulation (`actuator_model.py`)

The stages:
1. **Command link:** a command arrives `T_link` after it is sent. A serial link does not reorder.
2. **Output hold:** ZOH updates at `f_esc_out` and `f_servo_out`, each with a random phase per run. An update takes the latest arrived command.
3. **VESC:** target clipped to ±v_max, first-order lag `tau_v`, ramp limits `accel_max` (speeding up) and `decel_max` (slowing down).
4. **Servo:** target clipped to ±δ_max, dead time `T_d` after each output update, slew `delta_rate_max`.

Events are handled at their exact times; the lag and slew are integrated between events in ≤ 0.25 ms steps.

**Profiles (all assumed; `ideal` is a design reference):**

| Parameter | `ideal` | `realistic_guess` | `fast_servo` | PX4 counterpart |
|---|---|---|---|---|
| `T_link` | 0 | 4 ms | 4 ms | uXRCE-DDS serial link |
| `f_esc_out` | none | 50 Hz | 50 Hz | `PWM_MAIN_RATE` (per timer group) |
| `f_servo_out` | none | 50 Hz | **333 Hz** | `PWM_MAIN_RATE` (per timer group) |
| `tau_v` | 0 | 0.10 s | 0.10 s | VESC speed loop |
| `accel_max` / `decel_max` | none | 3 / 4 m/s² | 3 / 4 m/s² | VESC firmware ramps |
| `T_d` | 0 | 10 ms | 10 ms | servo |
| `delta_rate_max` | none | 5 rad/s | 5 rad/s | servo slew through the linkage |

`fast_servo` differs from `realistic_guess` only in `f_servo_out`, so it isolates the update-rate effect.

**Step responses (offline tests, `test_actuator_model.py`, 20 tests):**
- the ideal pass-through, with saturation;
- the link delay;
- the hold, which samples the latest arrived command at the first update at or after its arrival, for 8 phases;
- only the last of several commands in one PWM period is applied;
- the first-order speed lag against the analytic curve;
- the accel/decel limits, including reversing through zero;
- the servo dead time, slew and saturation;
- whole-profile steps (start delay within [T_link + T_d, T_link + T_d + one hold period], final value reached);
- the Monte Carlo delay statistics.

### 6.2 Mux, watchdog, lock and freshness gate (`command_mux.py`)

**Modes:**

| Mode | Output | Leaves by |
|---|---|---|
| `hold` (start) | speed 0, steering held | explicit `mux/arm` only |
| `controller` | the controller's setpoints, forwarded at once with their stamp | timeout, gate, teleop, lock, `mux/disarm` → `hold` |
| `teleop` | the teleop command | teleop silence `T_teleop` → `hold`; lock |
| `locked` | speed 0, steering held | lock release → `hold` |

**Rules and their PX4 counterparts:**

| Rule (our name) | Value (assumed) | PX4 counterpart |
|---|---|---|
| Teleop takes over from any mode except `locked`; the controller is disarmed | — | RC override / the transmitter's mode switch; leaving offboard returns control to the transmitter |
| Setpoint timeout `T_wd` → stop and hold, no resume | 0.3 s | `COM_OF_LOSS_T`, then the action `COM_OBL_RC_ACT` (here Hold) |
| Teleop silence `T_teleop` → hold, never back to the controller | 0.3 s | `COM_RC_LOSS_T` / leaving the manual mode |
| Arming needs a controller setpoint fresher than `T_arm_fresh`, no lock, no teleop, a fresh pose | 0.1 s | PX4 accepts offboard only while setpoints already stream (> ~2 Hz) |
| A setpoint older than `T_wd` at arrival, or not newer than the last one, is dropped | — | — |
| Mocap pose age > `T_fresh` disarms (latched; re-arm only explicitly) | 0.1 s | EKF2 external-vision timeout, later (`EKF2_EV_CTRL`) |
| Lock (kill) over everything; release gives `hold` | — | kill switch |

**Tests (offline, `test_command_mux.py`, 13 tests):**
- starts disarmed;
- arming preconditions;
- forwarding with the stamp;
- stale setpoint → hold, with no automatic resume and an explicit re-arm;
- old and reordered setpoints dropped;
- teleop takeover and release, with no hand-back;
- arming refused during teleop;
- the lock latches over teleop too, and release needs a re-arm;
- a non-finite setpoint disarms;
- the gate's age and out-of-order handling;
- a stale pose disarms with no self-rearm;
- operator disarm.

**`app/husky/command_ownership.py` is not reused.**
- **Why:** on the Husky it infers ownership from the `twist_mux` output, because the mux is not ours. On the car, this mux is the single authority, so no inference is needed.
- **Kept from it:** the fail-closed rules (explicit re-arm, no resume after a takeover, stale → stop).

### 6.3 OptiTrack emulator (`mocap_emulator.py`)

The steps, per frame:
1. decimation to `rate_hz` (≤ 100 Hz: the truth rate);
2. dropout bursts (probability per frame, 1..`drop_burst_max` frames);
3. Gaussian noise on x, y and yaw;
4. delay = `T_mocap` + Wi-Fi (`wifi_base` + exponential jitter + spikes with probability `spike_prob`, uniform in [`spike_min`, `spike_max`]).

The two delay parts are kept apart in each frame. Frames are delivered in arrival order: a spike lets a later frame overtake, as with UDP.

**The header stamp is the capture time.** The controller computes the pose age as now − stamp, and the freshness gate works on that age.

`realistic_guess` (all `PLACEHOLDER_`):
- 100 Hz;
- `T_mocap` 5 ms;
- Wi-Fi base 3 ms, jitter mean 2 ms;
- spikes 1 % at 20–80 ms;
- noise 0.5 mm and 2 mrad;
- dropout bursts starting with probability 0.5 % per frame, ≤ 3 frames long.

**Tests:** `test_mocap_emulator.py`, 8 tests.

### 6.4 End-to-end delay distributions (Gazebo, 2026-11-22; SIMULATION, ASSUMED PARAMETERS)

**Runner:** `checks/chain_checks.py`, as a stand-in controller and operator.

**Reports:** `~/nmpc_sim/logs/rc_car_chain_20261122/<profile>_mocap-<profile>_seed<N>/report.json`.

**Command path (D1):**
- **What it measures:** from the controller command's header stamp to the front-left steering joint moving 2 mrad from its previous angle, over 30 steering steps of ±0.25 rad sent at random times (not on the 50 Hz grid), with the car at rest.
- **Chain only:** the offline model's own delay (`actuator_model.servo_start_delays`, 2000 random phases).

| Actuator profile | Gazebo: median (p5–p95), min–max | Chain only, offline: median (p5–p95), min–max |
|---|---|---|
| `ideal` | 10.5 (8.0–13.6), 7–30 ms | 0 ms |
| `realistic_guess` | 33.0 (24.0–43.5), 23–44 ms | 23.7 (15.2–33.2), 14.3–34.2 ms |
| `fast_servo` | 27.0 (23.0–30.5), 22–45 ms | 15.7 (14.4–17.1), 14.3–17.3 ms |

**Reading:**
- **The chain-only delay is T_link + the hold wait + T_d.**
  - At 50 Hz, the hold wait is uniform over 0–20 ms; at 333 Hz, over 0–3 ms.
  - So the 333 Hz servo signal cuts the median by 8 ms and the spread from about 18 ms to about 3 ms.
- **Gazebo adds about 10 ms on top: a simulation floor, not part of the emulated vehicle.** The `ideal` run measures it: median 10.5 ms. Its parts:
  - the actuator's 2 ms tick;
  - the 200 Hz controller manager (≤ 5 ms);
  - the 200 Hz `/joint_states` sampling (≤ 5 ms);
  - the P-loop ramp of the steering joint.
- **The few outliers** (30 ms in `ideal`, 45 ms in `fast_servo`; one sample each) are probably simulation scheduling.

**Pose path (D2):**
- **What it measures:** the age of every `/mocap/reely_new1/pose` at the stand-in controller's input (receive time − capture stamp), from arming on.
- **Offline:** the emulator's own delay over 60 000 frames.

| Mocap profile | Gazebo: median, mean, p95, max | Offline emulator: median, mean, p95, p99, max |
|---|---|---|
| `ideal` | 2.0, 1.7, 3.0, 13 ms (n = 2528) | 0 |
| `realistic_guess` (2 runs) | 11.0, 12.6 / 12.1, 20.0 / 18.0, 89 / 100 ms | 9.4, 10.5, 14.5, 33.1, 92.9 ms |

Gazebo adds 2 ms at the median and 4–5 ms at p95: the 2 ms publish tick and the 1 ms resolution of the sim clock. Frames dropped: 1.0–1.3 % (51–73), as configured. Spikes: 0.9–1.0 % of frames.

**Speed step (D3)**, 0 → 0.5 m/s, start (truth speed > 0.01 m/s) / 10–90 % rise:
- `ideal`: 2 / 42 ms;
- `realistic_guess`: 18 / 232 ms;
- `fast_servo`: 30 / 235 ms.

The rise is the VESC lag `tau_v` = 0.1 s plus the ramp. The start varies with the ESC hold phase (0–20 ms) plus the truth's 10 ms sampling.

**Live mux checks (S1–S5), all PASS in all three runs:**
- **S1:** the setpoint stream stops; `hold` with reason `setpoint_timeout` 0.30–0.32 s later. That is T_wd plus the 50 Hz mux tick plus the ≤ 20 ms since the last setpoint. The car stops: truth speed ≤ 0.019 m/s at 0.3 s after the trip, the tail of `tau_v`.
- **S2:** new setpoints for 0.6 s; the mode stays `hold`.
- **S3:** an explicit `mux/arm` succeeds.
- **S4:** teleop takes over at its first message and releases to `hold` T_teleop after its last message, not back to `controller`.
- **S5:**
  - `locked` at once;
  - arming refused while locked ("mode is locked; lock engaged");
  - release gives `hold`.

### 6.5 Simulation artifact found and removed: the Gazebo truth parser's 10 ms lag

**What:** the first chain runs showed the pose age pinned at about 12 ms even with the `ideal` mocap profile. The emulator's own source lag (Gazebo stamp → parsed) had a median of 11 ms.

**Cause:** the Husky parser (`identify_effective_track.parse_pose_v_text`, used by `sil/gz_truth.py`) yields a message only when the next message's `header {` begins, one 100 Hz period later.
- **Scoring is unaffected:** it uses the stamps.
- **The mocap path was affected:** the emulated delay was buried under 10 ms of simulation lag.

**Fix (car only; the Husky code is unchanged):** `gz_pose_stream.py` yields the car's pose as soon as its block closes. The source lag is now 0 ms at the median and ≤ 6 ms at most.

**Also seen:** the first smoke run had a ramp of ages from 300 ms down to 10 ms before arming (a start-up backlog of the truth pipe). D2 now counts only from arming on; no age above 100 ms was seen after arming in the final runs.

## 7. Phase 3: the baseline arm (simulation, assumed parameters)

**The chain:**

```
FollowPath goal segments (checks/track_run.py)
  -> controller_server + Nav2 RPP (20 Hz) --/baseline/cmd_vel (TwistStamped)--> twist_to_ackermann
  -> controller/ackermann_cmd -> command_mux -> actuator -> plant -> Gazebo
/mocap/reely_new1/pose -> mocap_bridge (newest capture stamp wins) -> TF map->base_link + odom -> controller_server
```

The NMPC arm in phase 4 uses the same mocap topic and pose input (`pose_input.py`), the same mux, actuator, paths, episode protocol and harness.

### 7.1 Baseline setup: Nav2 Regulated Pure Pursuit (no fallback needed)

**RPP ran on the first launch**, well inside the 2 h box. The in-repo fallback was not written.

**Hosting:**
- a minimal `controller_server` (Nav2 1.3.13) with a lifecycle manager;
- an empty rolling local costmap (one inflation layer, no sources: ROS 2 parameters cannot hold an empty plugin list);
- no map, planner or BT navigator.

**Settings** (`config/baseline_rpp.yaml`):

| Setting | Value | Why |
|---|---|---|
| `controller_frequency` | 20 Hz | the NMPC's Ts = 0.05 s |
| `enable_stamped_cmd_vel` | true | the command's stamp is its compute time |
| `desired_linear_vel` | the reference speed (set per run) | the same constant speed reference as the NMPC |
| `use_regulated_linear_velocity_scaling`, `use_cost_regulated_…` | false | no curvature slow-down |
| `approach_velocity_scaling_dist` | 0 | no proximity slow-down |
| `use_collision_detection` | false | no collision checking |
| `use_rotate_to_heading`, `allow_reversing` | false | Ackermann |
| `use_velocity_scaled_lookahead_dist` | true | lookahead = clamp(lookahead_time · v, min, max), v from the VESC telemetry |
| `max_robot_pose_search_dist` | 1.0 m | local closest-point search (figure-eight crossing) |
| `transform_tolerance` | 0.3 s | the TF carries the capture stamp, up to ~0.1 s old |

**Two harness findings:**
1. **Nav2's `SimpleGoalChecker` tests only the distance to the path's last pose.** On the two-lap circle it ended the run after one lap (47 % completion; that run is kept in `eval_single_goal_superseded/`). The harness now sends overlapping 2 m goal segments, preempting 1 m before each segment's end, switched on the mocap progress. The re-run of the tuning point on the S-curve gave the same result (1.1 / 1.2 mm against 1.2 / 1.3 mm in the grid).
2. **Once, the lifecycle manager's activate call hit a DDS response timeout,** and `controller_server` stayed inactive (the run is kept in `eval_failed_lifecycle_timeout/`, with no episodes). The harness now checks that the server is active before the first episode, activates it through the lifecycle services if it is not, and prints any error.

### 7.2 Same conditions for both arms

**The pose input:** the same `/mocap/reely_new1/pose`, through `pose_input.NewestPose`. A frame overtaken by a later one is ignored and counted: 7–14 per run.

**The command path:** the same mux and actuator.

**Paired seeds** (`episode.py`; `test_seed_pairing.py`, 3 tests):
- the harness publishes `episode/start` {t_start, seed} 0.3 s ahead;
- the mocap emulator restarts its four random streams (noise, jitter, spike, dropout; a fixed number of draws per frame) from the seed;
- the actuator places its PWM hold phases, drawn from the seed, relative to t_start.

**What the test checks:**
- **Mocap:** two emulators with the same seed, different launch seeds and different histories (0 vs 777 frames before, other absolute times and capture offsets) produce identical per-frame draws, identical dropout patterns, and identical noise, delay and spike values. A different seed gives different streams.
- **Actuator:** identical servo update times relative to the episode start.

### 7.3 Tuning (S-curve, 1.0 m/s, `realistic_guess`, seeds 100 and 101; not an evaluation path)

**The grid** (36 + 24 episodes; reports in `~/nmpc_sim/logs/rc_car_baseline_20261122/tuning_*`, selection in `tuning_selection.txt`):
- **First grid:** `lookahead_time` ∈ {0.3, 0.45, 0.6, 0.8, 1.0, 1.2} s × `min_lookahead_dist` ∈ {0.2, 0.35, 0.5} m.
- **Its minimum lay on the grid edge** (0.3 s / 0.2 m), so the grid was extended downward: `lookahead_time` ∈ {0.1, 0.15, 0.2, 0.25} × `min_lookahead_dist` ∈ {0.1, 0.15, 0.2}.

**Mean cross-track RMSE over the two seeds (mm):**

| lookahead_time \ min_lookahead_dist | 0.1 | 0.15 | 0.2 | 0.35 | 0.5 |
|---|---|---|---|---|---|
| 0.1 | 308 (osc.) | 42.7 (osc.) | 1.2 | | |
| 0.15 | 51.7 (osc.) | 68.2 (osc.) | 1.2 | | |
| 0.2 | 1.2 | 1.4 | **1.2 (chosen)** | | |
| 0.25 | 1.6 | 1.6 | 1.5 | | |
| 0.3 | | | 1.9 | 3.1 | 10.0 |
| 0.45 | | | 6.8 | 6.8 | 9.6 |
| 0.6 | | | 16.3 | 16.7 | 16.8 |
| 0.8 | | | 34.8 | 34.9 | 35.1 |
| 1.0 | | | 57.6 | 57.5 | 57.7 |
| 1.2 | | | 84.2 | 84.4 | 84.0 |

**What the grid shows:**
- **The error falls with the lookahead** until about 0.2 m effective lookahead at 1 m/s.
- **Below that it collapses into oscillation** ("osc."): whenever the lookahead can fall to 0.1–0.15 m at low speed. Two such episodes timed out.

**The selection rule** (`checks/tuning_select.py`, tested):
- **Written down after the extension had shown the cliff,** and stated as such.
- **The rule:** the lowest mean RMSE among points that are acceptable (completed, no safety event, ≤ 3× the best mean) and whose neighbours along both grid axes are acceptable too. That keeps one grid step of margin from the oscillation.
- **Result: `lookahead_time` = 0.2 s, `min_lookahead_dist` = 0.2 m (mean 1.24 mm). Frozen.** The unconstrained best (0.15 s / 0.2 m, 1.15 mm) sits next to an oscillating point.
- **Effective lookahead:** 0.2 m at 1.0 m/s and 0.3 m at 1.5 m/s.

### 7.4 One run per evaluation path (1.0 m/s, `realistic_guess`, seed 1; frozen values)

**Reports:** `~/nmpc_sim/logs/rc_car_baseline_20261122/eval_v1_realistic_seed1/`; plots of path vs reference and cross-track error in `plots/`.

| Path | Cross-track RMSE / max | Heading RMSE / max | Completion | Safety events | Rear-axle lateral speed RMS / max |
|---|---|---|---|---|---|
| `line` (4.5 m) | 0.4 / 1.1 mm | 3.9 / 12.9 mrad | 1.00 | 0 | 0.0 / 0.2 mm/s |
| `circle` (R 1.2 m, 2 laps) | 2.9 / 6.8 mm | 19.3 / 48.8 mrad | 1.00 | 0 | 16.4 / 36.8 mm/s |
| `figure_eight` (R 1.0 m) | 3.4 / 8.7 mm | 22.8 / 66.6 mrad | 1.00 | 0 | 16.5 / 42.4 mm/s |
| `lane_change` (0.75 m) | 0.9 / 2.0 mm | 6.2 / 17.0 mrad | 1.00 | 0 | 4.4 / 13.0 mm/s |

**Safety events** (gate disarms, setpoint timeouts, holds, takeovers): **0** in every run. During the runs:
- the mocap emulator dropped 3–15 frames;
- it produced 5–12 latency spikes;
- the pose input ignored 7–14 overtaken frames.

**Commands:**
- requested at 20.1 Hz;
- no converter clamps;
- the applied steering reached the servo slew limit (5 rad/s) in every run, and 0.30–0.40 rad at most (at the start, and at the figure-eight's curvature steps).

**What it says, and its limits:**
- **Tracking is millimetre-level** because the paths are smooth relative to the 0.2 m lookahead, and the model has no tyre lag at 1 m/s.
- **On the circle,** the error oscillates by about ±3 mm with a period of about 1 s around −2.5 mm. That is a small limit cycle of the short lookahead with the chain delay.
- **These are single runs on one seed.** Phase 4 runs 5 seeds per arm, plus the speed sweep.

### 7.5 The Gazebo term of the command delay (simulation only)

**Measured with n = 100 steering steps per profile** (`~/nmpc_sim/logs/rc_car_chain_20261122/phase3_gazebo_term/`):

| Profile | Gazebo: median (p5–p95), min–max | Chain only, offline: median | Gazebo term (median difference) | RTF |
|---|---|---|---|---|
| `ideal`, 2 ms actuator tick | 11.0 (8.0–13.0), 7–13 ms | 0 | 11.0 ms | 0.998 |
| `ideal`, 1 ms actuator tick | 11.0 (8.0–13.0), 7–14 ms | 0 | 11.0 ms | 0.992 |
| `realistic_guess` | 36.0 (26.0–44.0), 26–50 ms | 24.2 ms | 11.8 ms | 0.993 |
| `fast_servo` | 27.0 (23.0–29.0), 22–30 ms | 15.8 ms | 11.2 ms | 0.998 |

**A 1 ms actuator tick does not lower the Gazebo term** (the condition's test), so the tick stays at 2 ms. The term comes from:
- the 200 Hz controller manager (≤ 5 ms);
- the 200 Hz `/joint_states` sampling of the measurement (≤ 5 ms);
- the steering joint's P loop.

**For phase 4's delay compensation, τ is the sum of these terms:**
- **the assumed car chain:** T_link 4 ms + the hold wait (0–20 ms at 50 Hz, uniform) + T_d 10 ms, plus the servo slew; chain-only median 24 ms for `realistic_guess`;
- **the Gazebo term:** about 11 ms, simulation only;
- **the solve time:** board-measured, about 9.3 ms for 3×24 and about 26.4 ms for 10×40. It is a parameter, not a car result;
- **the pose age at the controller input:** median 11 ms for the `realistic_guess` mocap.

### 7.6 Phase 4 additions recorded here

- **Contact sensitivity point:**
  - **Runs:** 10×40 NMPC and the baseline, figure-eight at 1.0 m/s, `realistic_guess`, 5 seeds, with `contact_width:=0.01`.
  - **What to report:** whether the ranking changes.
- **Safety events are a metric:** counted per arm, never a reason to drop or repeat a run.
- **NMPC weight tuning:** fixed in §7.7 before any NMPC run.

### 7.7 NMPC tuning rule (written 2026-11-22, BEFORE any NMPC run; not to be changed)

Set by the STOP 3 review (§2.1 of `.copilot/chats/2026-11-22-claude-to-claude-rc-car-phase3-review-phase4-go.md`) and fixed here before the first NMPC tuning run. The baseline's rule (§7.3) was written after its grid had been seen; that is stated there and stays stated.

**Setting (the same as the baseline's):**
- **Path:** the tuning S-curve, at 1.0 m/s. Its geometry stays exactly the one the baseline was tuned on (`paths.py` builds it with fixed dimensions, independent of the area).
- **Profiles:** actuator `realistic_guess`, mocap `realistic_guess`.
- **Seeds:** tuning seeds 100 and 101, never used for evaluation.
- **Delay compensation on** (the design). The "off" variant reuses the same weights.
- **Arithmetic:** the half-precision emulation, as in the comparison.

**Swarm size:** tune with 10×40 only. 3×24 and 10×60 reuse those weights, so swarm size is the only variable between the three.

**The cost and the fixed weights:**
- **The cost:** the October Ackermann study's §9.3 cost (`docs/ackermann/ackermann_mpc_pso_model.md` §9.3; `ackermann_model.mpc_cost`).
- **Fixed at the study's values** (`app/ackermann/test_ackermann_model.py`, `PLACEHOLDER_WEIGHTS`, line 41):
  - q_v = 1 s²/m²;
  - r_kappa = 0.1 m²;
  - the control-rate weights s_dv = 1 s²/m² and s_dkappa = 1 m²;
  - the terminal-to-stage ratio qf/q = 10 for both position and heading (as in the study: 1000/100 and 100/10).

**The two axes (log grid, half-decade steps, centred on the study's values):**
- **Position weight:** q_pos ∈ {10, 31.6, 100, 316, 1000} 1/m², with qf_pos = 10·q_pos.
- **Heading weight:** q_psi ∈ {1, 3.16, 10, 31.6, 100} 1/rad², with qf_psi = 10·q_psi.
- **Grid size:** 25 points × 2 seeds = 50 episodes. The budget is at most 30 points × 2 seeds.

**Selection (the same rule as the baseline; `checks/tuning_select.py` with the axes renamed):**
1. **Acceptable:** both seeds completed without a timeout or a safety event, and the cross-track RMSE is ≤ 3× the best mean RMSE of the grid.
2. **Eligible:** the point and all its existing neighbours along both axes are acceptable.
3. **Chosen:** the eligible point with the lowest mean cross-track RMSE over the two seeds.

**Extension (at most once, within the 30-point budget):**
- **When:** the best mean-RMSE point lies on the grid edge.
- **What:** add one row or column of 5 points one half-decade beyond that edge, then apply the rule to all 30 points.
- **At a corner:** extend the position axis.
- **Never twice.**

Then the weights are frozen in `params/nmpc.yaml`, and the grid, the rule's table and the chosen point are reported.

### 7.8 Phase 4a: the NMPC arm, smoke runs and the run budget (simulation, assumed parameters)

#### Lab facts folded in (2026-11-22)

**Area:** `PLACEHOLDER_` 10 × 10 m (about 100 m²), margin 1.0 m, so 8 × 8 m usable (`params/lab_area.yaml`; the world outline is redrawn).

**Paths** (`paths.py`): every path's peak curvature is ≤ 70 % of κ_max (R ≥ 1.06 m at κ_max = 1.352 m⁻¹), checked in the tests.

| Path | Set | Size | Length | Peak κ (% of κ_max) |
|---|---|---|---|---|
| `line` | core | across the usable length | 7.50 m | 0 |
| `circle` | core | R 1.5 m, 2 laps | 18.84 m | 0.667 (49 %) |
| `figure_eight` | core | **smooth** lemniscate of Bernoulli, half-width 3.5 m | 18.35 m | 0.857 (63 %) |
| `lane_change` | core | 1.0 m quintic shift over 3 m | 7.72 m | 0.587 (43 %) |
| `arcs_and_lines` | stress (speed sweep) | **two arcs joined by lines**, R 1.2 m; curvature jumps | 17.02 m | 0.833 (62 %) |
| `s_curve` | tuning | **fixed**: the geometry the baseline was tuned on | 4.79 m | 0.914 (68 %) |

**Vehicle values** (`params/reely_new1.yaml`, one file for the plant, the converter and the NMPC):
- **Track:** `front_track` = `rear_track` = 0.30 m, `measured_approx` (user, sheet item 2).
- **Full lock:** `delta_max` = 0.356 rad, `measured_approx`. That is "about 25° full lock" read conservatively as the **inner** wheel, giving bicycle 20.4°, κ_max 1.352 m⁻¹ and rear-axle R_min 0.740 m; inner and outer wheels 25.0° and 17.2°. **To be confirmed with the turning-circle test.**
- **Payload:** `payload_mass` 0.5 kg `PLACEHOLDER_` (KV260, power supply, plate), at 0.1375 m ahead of the rear axle and 0.09 m above it, a 0.12 × 0.12 × 0.04 m box in the inertia. **The slip in the speed sweep depends on it.**

**Chain comments** (`params/command_chain.yaml`):
- the KV260 rides on the car: `T_link` is now a serial cable on the car (still `PLACEHOLDER_` 4 ms);
- the only wireless hop is the pose path.

**C1–C6 with the new geometry** (`~/nmpc_sim/logs/rc_car_bringup_20261122/run4_lab_facts_geometry/`): all PASS.
- **C3 at 0.6·δ_max (±0.214 rad), R_kin 1.268 m:** R error +0.29 / −0.54 / −0.59 % at 0.5 / 1.0 / 1.5 m/s; rear-axle speed +2.20 / +0.31 / −0.11 %; lateral speed 3.4 / 16.3 / 24.7 mm/s.
- **C3 at full lock (±0.356 rad), 0.5 m/s, R_kin 0.740 m:** +1.47 %, rear-axle speed +3.65 %.
- **C4:** the Ackermann angles to < 1e-4 rad (full lock: L/R 0.4364/0.2998 rad); rise 13–36 ms.
- **C5:** pass. **C6:** RTF 0.988.

#### Baseline under the new limit

**Did earlier runs command more than 0.356 rad?** Checked over the 66 earlier baseline runs.
- **The ones that did:** the 8 oscillating tuning points (rejected by the rule) and the old κ = 1.0 figure-eight run (that path is replaced).
- **The ones that did not:** the chosen point, its acceptable neighbours and the other evaluation runs (≤ 0.282 rad).
- **Consequence:** strictly "some did", and the track and payload change the plant too. **So the neighbourhood of the chosen point (3 × 3 points, 2 seeds) is re-run under the new limit as batch item B1,** with the same rule, and both results are reported.

**The four core evaluation runs, plus the stress path, re-run on the new paths** (1.0 m/s, `realistic_guess`, seed 1, frozen tuning; `~/nmpc_sim/logs/rc_car_phase4a_20261122/baseline_eval_newpaths/`):

| Path | Cross-track RMSE / max | Heading RMSE | Safety events | Max commanded steering | Steering rate on its limit |
|---|---|---|---|---|---|
| `line` | 0.4 / 1.3 mm | 4.2 mrad | 0 | 0.042 rad | 2.6 % |
| `circle` | 2.4 / 6.3 mm | 14.6 mrad | 0 | 0.246 rad | 2.4 % |
| `figure_eight` (smooth) | 2.0 / 5.3 mm | 12.6 mrad | 0 | 0.286 rad | 2.8 % |
| `lane_change` | 0.8 / 2.4 mm | 5.4 mrad | 0 | 0.187 rad | 1.7 % |
| `arcs_and_lines` | 2.6 / 7.7 mm | 17.5 mrad | 0 | 0.327 rad | 5.7 % |

No run commanded more than 0.356 rad.

#### The NMPC arm

**What is emulated:** the proposed Ackermann engine of the October note (§9–10), described in `nmpc_engine.py` and `params/nmpc.yaml`:
- the HLS PSO structure;
- the §9.3 cost;
- the exact-arc chord rollout in **half precision** with the 1024-entry table sine (shared index) and `ap_fixed<32,17>` states;
- each stage's cost term rounded to the `ap_fixed` grid, with a saturating sum.

**The HLS changes of §10** (reference controls and increments in the cost) **are not implemented in HLS.** The vectorized half step is **bit-identical** to the October study's scalar emulation (`hw_estimates.step_chord_hw`, 3000 random cases); the vectorized horizon rollout is bit-identical to the step-by-step one.

**Host side** (`nmpc_controller.py`):
- **Timing model (board, not Python; tick-anchored since the STOP 4a review §2.1):**
  - at each 50 ms tick t_k, take the newest pose (age a_k) and **start the solve at t_k**; apply the result at **t_k + solve time**, in simulation time;
  - **one solver:** a solve never starts before the previous one has ended (tested);
  - solve time: 9.25 ms (3×24, board), 26.35 ms (10×40, board), 38.9 ms (10×60, **extrapolated from the board's 10×40 measurement; no 10×60 board run**: 60 × 25.01/40 ms + 1.34 ms PS);
  - **a deadline miss** is an apply time beyond the next tick (solve time > 50 ms): never with the board times; the counter stays;
  - **a stale pose at tick** (older than one period) is counted separately; the freshness gate (T_fresh 100 ms) and the prediction handle it.
  - **Replaced:** the earlier capture-anchored model (apply at capture + 30 ms allowance + solve time) added ~20 ms in the common case and let solves overlap.
- **Delay compensation (on):**
  - the state is predicted, through the buffer of commands already sent, from the capture time to t_eff = t_apply + chain term + Gazebo term (11 ms): a span of a_k + solve time + chain + Gazebo, known exactly every step;
  - chain terms: 24.2 ms for `realistic_guess`, 15.8 ms for `fast_servo`, 0 for `ideal`;
  - **"off"** keeps the timing and skips the prediction.
- **Path-following reference:** project the predicted pose (forward-only search), then take Nh points at the commanded speed; v_ref = the commanded speed, κ_ref = the path curvature.
- **Warm start:** the cheaper, in double, of the shifted plan and the projected feed-forward.
- **Plan check:** `check_controls` in double; a failing plan is replaced by the warm start (a counted fallback).
- **Limits** from `reely_new1.yaml` (δ_max, L, v_max_sim) and `command_chain.yaml` (`vehicle_limits_profile: realistic_guess`):
  - κ_max = tan(0.356)/0.275 = 1.352 m⁻¹;
  - κ rate = 5/0.275 = 18.2 m⁻¹s⁻¹;
  - accel / decel 3 / 4 m/s²;
  - v ∈ [0, 2] m/s.
- **PSO RNG:** seeded per episode from the episode seed.

**Tests** (`test_nmpc_engine.py`, 12; `test_nmpc_controller.py`, 9) cover:
- the bit-level step;
- the double cost = `mpc_cost`;
- feasible plans;
- the saturation;
- **limits from the YAML files**: a different δ_max in `reely_new1.yaml` changes the bound;
- the timing model;
- the prediction;
- the reference;
- **a repeated seed reproduces the run exactly**;
- **a deliberately slow Python solve (60 ms of wall time) changes neither the applied times nor the trajectory**;
- Scheduler overruns.

**Python speed:**
- **Half-precision solves on the host:** 13.4 / 24.7 / 36.6 ms (3×24 / 10×40 / 10×60), after removing numpy 1.21's `np.clip` overhead (80 % of the time).
- **The Python solve must finish before t_apply in simulation time,** so the batch runs Gazebo at a **reduced real-time factor:** 0.35 (3×24), 0.5 (10×40), 0.4 (10×60).
- **Overruns** (a result after its t_apply) and **busy ticks** are counted. They must be 0.

**Finding: the real-time factor must be set in the world file.**
- **What happened:** setting it at run time through `gz service …/set_physics` broke the plant. The car barely moved while turning on the spot, and the NMPC (3 episodes) and the baseline (1 diagnostic episode) both timed out.
- **The fix:** `rc_car_sim.launch.py rtf:=` writes a copy of the world with the factor in it. `track_run.py --rtf` now only records the factor; `--set-physics` remains, hidden, for diagnosis.

**Smoke runs, UNDER THE OLD (capture-anchored) TIMING** (lane change, 1.0 m/s, `realistic_guess`, seed 1, compensation on, half; `~/nmpc_sim/logs/rc_car_phase4a_20261122/nmpc_smoke2/`). B1 repeats one per size under the tick-anchored timing:

| Size | RTF (measured) | Cross-track RMSE / max | Deadline misses | Overruns / busy | Fallbacks | Python solve, median / max (wall) |
|---|---|---|---|---|---|---|
| 10×40 | 0.500 | 2.1 / 5.4 mm | 0 / 154 | 0 / 0 | 0 | 35.0 / 40.3 ms (allowed 52.7 at RTF 0.5) |
| 3×24 | 0.350 | 2.0 / 5.2 mm | 0 / 153 | 0 / 0 | 0 | 18.6 / 21.8 ms (allowed 26.4) |
| 10×60 | 0.499 | 2.0 / 5.2 mm | 1 / 153 | 0 / 0 | 0 | 55.1 / 74.3 ms (allowed 77.8: too tight, so the batch uses 0.4) |

Also on the lane change, the baseline gave 0.8 / 2.4 mm. These are single smoke runs, not the comparison.

**The 10×60 smoke run's 1/153 miss (old model):**
- **It was an old pose:** a pose older than the 30 ms allowance at the tick. That is the only way the old model counted a miss; a reused pose implies the same.
- **Overlaps went uncounted:** the old model did not detect solve overlaps. From the run's apply times, **3 solve pairs overlapped** (apply-time gaps of 38–39 ms against a 38.9 ms solve). 10×40 and 3×24 had none.

**The first smoke attempt** (`nmpc_smoke/`, kept as a record) found:
- the `set_physics` problem;
- a stale NMPC node left running when the launch was killed (now `pkill -f nmpc_node.py`).

**Comparison runs use the `realistic_guess` mocap in every run;** the two plant profiles differ only in the actuator.

#### Run budget and schedule (phase 4b; `checks/run_phase4_batch.sh`, run by the user, one block per assigned window)

**The estimates come from the smoke runs:**
- the episode's run time is (L − 0.45 m)/v + 0.3 s;
- +3.4 s simulated overhead per episode;
- wall = simulated / RTF, +47 s per container.

| Block | Content | Episodes | Wall |
|---|---|---|---|
| **B1** | **First:** the RTF-invariance check (review §3: baseline figure-eight 1.0 m/s, seed 1, at RTF 1.0 and 0.4; NMPC 10×40 at 0.5 and 0.4; pass = RMSE pairs within 0.2 mm and equal event counts, `checks/rtf_check.py`; on failure every arm runs at 0.35) and one tick-timing smoke episode per size. **Then:** B0, the baseline neighbourhood under the new δ_max (S-curve, 9 points × seeds 100, 101) + NMPC tuning (§7.7, 25 points × 2 seeds, 10×40), at the decided factors | 4 + 3 + 18 + 50 | **~0.45 h** |
| (stop) | The agent applies both selection rules, freezes the baseline lookahead and the NMPC weights | — | — |
| **B2** | Core, `realistic_guess`: the baseline in both profiles (40) + NMPC 10×40, 3×24, 10×60 (60); 4 core paths × seeds 1–5 at 1.0 m/s | 100 | **~0.9 h** |
| **B3** | Core, `ideal`: NMPC 10×40, 3×24, 10×60 (60) + the float spot check (10×40, figure-eight, 5 seeds paired with the half runs) | 65 | **~0.8 h** |
| **B4** | Compensation off (10×40 and 3×24, figure-eight at 1.0 and 1.5 m/s) and on at 1.5 m/s; 10 mm contact (baseline + 10×40, figure-eight 1.0 m/s); speed sweep (`arcs_and_lines`, 0.5 / 1.0 / 1.5 / 2.0 m/s, baseline + 10×40) | 80 | **~0.9 h** |
| **Total** | | **320** | **~3.1 h** (+15 % contingency, about 3.5 h) |

**Real-time factors:** per arm (baseline 1.0; NMPC 3×24 0.35, 10×40 0.5, 10×60 0.4) if B1's RTF check passes; otherwise **one common factor, 0.35, for every arm** (baseline included), read by B2–B4 from `B1/rtf_decision.json` and reported.

**Order:** the core comparison first (B2, B3), the sweeps last (B4). B1 must come first, because its results freeze both arms.

**Cut list**, if time runs out (in this order; never the core):
1. The 0.5 m/s point of the speed sweep (−9 min).
2. The whole speed sweep (−23 min).
3. Compensation "on at 1.5 m/s" (keep "off" at 1.0 and 1.5) (−8 min).
4. The contact point down to 3 seeds (−3 min).
5. The float check down to 1 seed (−3 min).

### 7.9 Phase 4b results (2026-11-22; SIMULATION ONLY)

**Where the results are:**
- the STOP 4 report: `.copilot/chats/2026-11-22-claude-rc-car-reply-phase4.md`;
- all tables: `docs/rc_car/phase4_comparison.md` and `.json` (from `checks/compare_phase4.py`);
- raw logs: `~/nmpc_sim/logs/rc_car_phase4_batch/`.

**B1, the frozen controllers:**
- **NMPC weights:** q_pos = 3160, q_psi = 10, by the §7.7 rule with its one extension. The surface is flat: 3.1–4.7 mm.
- **Baseline:** lookahead 0.2 s / 0.2 m, re-confirmed at RTF 1.0.

**B1, the RTF check:** it failed for the baseline only, because Nav2's `controller_server` paces on wall time (`rclcpp::WallRate`: 50 Hz in simulation time at RTF 0.4). With the user's decision, the baseline runs at RTF 1.0 and the NMPC at its per-size factors (`B1/rtf_policy.json`).

**B2–B4:** 245 episodes, all completed, 0 safety events. NMPC: 0 deadline misses, 0 busy ticks, 0 fallbacks, 2 overruns (0.7 and 6.7 ms), 19 stale poses.

**Findings:**
- **Cross-track error:** the baseline is lower than every NMPC configuration, by +0.4 to +7.1 mm (paired medians, `realistic_guess`, 1.0 m/s).
- **Other metrics:** the NMPC has slightly better heading and about 4× more steering-rate saturation.
- **Delay compensation** cuts the NMPC's error 6–10×.
- **Swarm size:** 3×24 is worse than 10×40 ≈ 10×60.
- **Contact model:** the ranking is unchanged with the 10 mm contact.
- **Speed:** on the stress path the gap narrows with speed (4.2× at 0.5 m/s, 1.4× at 2.0 m/s).
- **Float vs half:** no difference.

The hypotheses for the gap (model mismatch with the scrubbing contact, the time-indexed cost vs the cross-track score, steering chatter) are untested; see the report, §9.

### 7.10 PSO redesign: the ablation settings and the tuning rule (written 2026-11-22, BEFORE any ablation or tuning run; not to be changed)

**Brief:** `.copilot/chats/2026-11-22-claude-to-claude-rc-car-pso-redesign.md`.
**Label for everything here:** offline, kinematic model, emulated solver (`checks/redesign_eval.py`).

**Switches** (`nmpc_controller.Redesign`, `nmpc_engine.Problem`; all off = bit-identical to the frozen phase 4 controller, tested against a recorded fixture):
- **R1, warm start:** particle 0 is always the previous best shifted by one step. The feed-forward is kept as a further seed, plus max(1, S // 5) perturbed copies of particle 0 (S ≥ 4; std 0.25 × the rate window, projected).
- **R2, move blocking:** M = 5 knots per input at stages (0, 6, 12, 18, 24), linear interpolation. Each knot's rate window is du × knot spacing.
- **R3, path-frame cost:** lateral and along-track position weights; terminal = 10 × stage, as before.
- **R4:** the Δκ weight s_dkappa (frozen: 1).

**Ablation, fixed values, stated now:**
- **R3:** q_lat = 3160 (= the frozen q_pos), q_lon = 316 (a tenth of it).
- **R4:** s_dkappa = 10 (10 × the frozen value).
- All other weights as frozen (q_psi = 10 and the October values).
- **Variants:** frozen, +R1, +R1+R2, +R1+R2+R3, +R1+R2+R3+R4.
- **Run settings:** 10×40, half, `realistic_guess` actuator and mocap, figure-eight and circle at 1.0 m/s, seeds 1–5.
- **Reference arm:** the Python pure pursuit (frozen lookahead 0.2 s / 0.2 m, 20 Hz).
- **Metrics:** cross-track and heading RMSE, J_PSO / J_opt (D1: SLSQP on the same cost, every 40th tick), RMS Δκ per tick, time on the steering-rate limit.

**First check (D2):** the offline loop must reproduce the phase 4 ordering (the baseline better than the frozen NMPC on both paths, seeds 1–5). If it doesn't: stop and report.

**Choosing the variant:**
- **The rule:** the variant with the lowest median cross-track RMSE over the 10 (path, seed) runs, provided every run completes.
- **A switch is kept** only if adding it lowers that median. A switch that does not help is dropped, even if a later one depends on it.

**Tuning rule for the chosen variant** (the §7.7 rule; S-curve at 1.0 m/s, tuning seeds 100 and 101, the offline loop, at most 30 points):
1. **If R3 is in the variant:** a 5 × 5 log grid.
   - q_lat ∈ {316, 1000, 3160, 10000, 31600};
   - q_lon ∈ {10, 31.6, 100, 316, 1000};
   - s_dkappa at its ablation value.
   - Selection by the §7.7 rule (acceptable = completed and ≤ 3× the best mean; eligible = it and its grid neighbours acceptable; lowest mean cross-track RMSE).
2. **If R4 is in the variant:** a 5-point axis s_dkappa ∈ {1, 3.16, 10, 31.6, 100} at the step-1 choice (or at the frozen weights without R3), the same rule in one dimension.
3. **The budget:** at most 30 points in all; **no extension**. If the best point lies on an edge, that is reported.
4. **Then** the tuned variant is re-run on the ablation settings (figure-eight and circle, seeds 1–5) and frozen in `params/nmpc.yaml` (a new section; the phase 4 values stay recorded).

**Gazebo confirmation, after STOP R, in a window the user assigns:**
- B2's core set at `realistic_guess`, 10×40 and 3×24, against the baseline;
- the baseline core re-run too, because of the 1.5 kg payload;
- **C1–C6 re-run first** with the new mass;
- about 60 episodes;
- the control rate of each arm stated in simulation time (both 20 Hz).

### 7.11 PSO redesign: results (OFFLINE, kinematic model, emulated solver; 2026-11-22)

**Where the results are:**
- the report: `.copilot/chats/2026-11-22-claude-rc-car-reply-redesign.md`;
- the table: `~/nmpc_sim/logs/rc_car_redesign_20261122/ablation_table.md` (`checks/redesign_table.py`).

**First check:** passed. Offline, the frozen NMPC is worse than the Python pure pursuit in 10 of 10 runs (median 0.83 against 0.33 mm).

**D1:** the frozen PSO is a median 3.6× above the deterministic optimum of its own cost.

**The rule's choice: R1 + R3 + R4** (R2 dropped: it raised the median cross-track RMSE, though it made the steering by far the smoothest).

**Tuned** (30 points, no extension): q_lat = 1000, q_lon = 31.6, s_dkappa = 10 (interior). It is frozen in `params/nmpc.yaml` `redesign`, and the node takes `variant:=redesign`.

**Offline result** (the tuned variant against the frozen NMPC against pure pursuit, figure-eight and circle, seeds 1–5):

| Metric | Tuned redesign | Frozen NMPC | Pure pursuit |
|---|---|---|---|
| Cross-track RMSE (median) | 0.48 mm | 0.83 mm | 0.33 mm |
| Heading RMSE | 1.9 mrad | 2.5 mrad | 3.4 mrad |
| RMS Δκ per tick | 0.041 | 0.117 | 0.048 |
| Time on the rate limit | 1.9 % | 7.4 % | 2.9 % |
| J_PSO / J_opt | 2.9 | 3.6 | — |

**D1 on the same ticks** (STOP R review, item 1; `checks/d1_same_ticks.py`): 90 ticks from the frozen controller's offline runs, each solved by every variant (10×40, half, the same 3 PSO seeds), each against the optimum of its own cost.

| Variant | J_PSO / J_opt, median (p10–p90) | Ticks within 1.5× |
|---|---|---|
| Frozen | 3.74 (2.50–9.65) | 0 % |
| Chosen R1+R3+R4 | 2.79 (1.66–8.28) | 7 % |
| Smoother R1+R2+R3+R4 (not selected) | 2.73 (1.63–8.02) | 8 % |

**Most of the solver gap remains.** The chosen changes act mainly on the cost; a remaining PSO gap of about 2.8× is the main lever left.

**Scale (STOP R review, item 3):** the offline errors are below 1 mm on a scrub-free model. The 0.48 vs 0.33 mm gap to pure pursuit is physically negligible; the heading error and steering smoothness are the more useful differences.

**Gazebo confirmation:** block BR of `checks/run_phase4_batch.sh`, waiting for the user's PC window (about 0.85 h).
1. C1–C6 with the 1.5 kg payload.
2. The baseline core re-run (RTF 1.0, 20 Hz in simulation time).
3. The chosen redesign core at 10×40 (0.5×) and 3×24 (0.35×).
4. **The smoother variant** (`redesign_smooth`; "not selected by the rule; run for steering smoothness"; the 10×40 core only), reported next to the chosen variant, never instead of it.

That is 80 episodes, plus C1–C6.

**HLS change list:** see the report, §7. Nothing in `hw/hls` has changed.

### 7.12 Block BR: the redesign in Gazebo (2026-11-22; SIMULATION, emulated solver, assumed and approximate parameters, 1.5 kg payload)

**Where the results are:**
- the report: `.copilot/chats/2026-11-22-claude-rc-car-reply-br.md`;
- the tables: `docs/rc_car/phaseR_br_comparison.md`.

**C1–C6 with 1.5 kg:** all PASS (full-lock C3 +1.65 %; RTF 0.982).

**Integrity:** 80 episodes completed, 0 safety events; NMPC with 0 misses, overruns, busy ticks or fallbacks. Every arm ran at 20 Hz in simulation time.

**Cross-track RMSE, mm, median over seeds 1–5:**

| Path | Baseline | Chosen R1+R3+R4, 10×40 | Smoother R1+R2+R3+R4, 10×40 (not selected) | Chosen R1+R3+R4, 3×24 |
|---|---|---|---|---|
| line | 0.3 | 0.7 | 0.7 | 0.5 |
| circle | 2.4 | 7.5 | 6.4 | 8.5 |
| figure-eight | 2.1 | 6.4 | 5.2 | 7.0 |
| lane change | 0.8 | 1.8 | 1.4 | 2.0 |

**Findings:**
- **Steering:** time on the rate limit is 0.4–1.0 % for both 10×40 redesign arms, against 8–10 % for the frozen phase 4 NMPC and about 2 % for the baseline.
- **The offline selection did not carry over:** in Gazebo the not-selected smoother variant has the lower cross-track error.
- **The comparison with the frozen NMPC is unpaired** (B2 used the 0.5 kg payload).

### 7.13 Carried-forward variant, held-out check and the solver-gap study (rules written 2026-11-22, BEFORE the runs)

**Brief:** `.copilot/chats/2026-11-22-claude-to-claude-rc-car-br-review.md`.

**The variant decision:**
- **Carried forward as the candidate:** the smoother R1+R2+R3+R4 (`redesign_smooth`, the ablation's values, not tuned).
- **This choice was made after seeing the BR results on the evaluation seeds 1–5** (post hoc). The reason: steering smoothness for the real servo, plus the lower Gazebo error.
- **The rule's choice** R1+R3+R4 (`redesign`) stays in the record as "selected by the rule, not carried forward".

**Gazebo block BH** (`checks/run_phase4_batch.sh BH`; SIMULATION, emulated solver, assumed and approximate parameters, 1.5 kg payload):
1. **Payload pairing:** the frozen phase 4 10×40 NMPC (`variant:=phase4`) at 1.5 kg, core set, seeds 1–5 (20 episodes), paired with BR's baseline and redesign arms.
2. **Held-out check:** the smoother variant (10×40) and the baseline, core set, **new seeds 6–10** (40 episodes). Reported separately from seeds 1–5.

**Offline solver-gap study** (OFFLINE, kinematic model, emulated solver; `nice -n 19`; alongside BH):
- **(a) Re-tune the swarm velocity clamp `max_v` for the 5-knot parameterisation** (smoother variant, 10×40, half).
  - **Grid:** max_v ∈ {10, 15, 20, 30, 45, 60, 90, 120, 160, 200} normalized units (10 points; 30 is the current value).
  - **Setting:** S-curve at 1.0 m/s, tuning seeds 100 and 101, the offline loop.
  - **Selection:** the §7.7 rule in one dimension: acceptable = completed and ≤ 3× the best mean; eligible = it and its neighbours on the axis acceptable; the lowest mean cross-track RMSE. **No extension.**
  - **Then:** D1 (J_PSO / J_opt) on the same 90 ticks as in §7.11, at 30 and at the chosen max_v.
- **(b) The larger budget:** the smoother variant at 10×60 (max_v 30, and the chosen max_v if it differs), on D1's same 90 ticks; plus the offline closed loop on the figure-eight and circle, seeds 1–5, against the 10×40 smoother variant. **The question:** does a larger budget close the 2.7× gap?
- **Not now:** a deterministic refinement step after the swarm (not a fit for the engine; it would need its own hardware design). Listed as future work.
- **Not changed by the results:** the carried-forward variant's `max_v` stays 30 in `params/nmpc.yaml`. The results of (a) and (b) are reported; any change needs a decision.

### 7.14 Block BH and the solver-gap study: results (2026-11-22)

**Where the results are:**
- the report: `.copilot/chats/2026-11-22-claude-rc-car-reply-bh.md`;
- the tables: `docs/rc_car/phaseR_bh_comparison.md`.

**Payload pairing** (Gazebo; phase 4 10×40 at 1.5 kg, seeds 1–5):
- **The heavier payload barely changes it:** 0.94 / 6.30 / 5.76 / 2.03 mm, against 0.87 / 6.56 / 5.82 / 2.06 mm at 0.5 kg.
- **Paired, the smoother variant against phase 4:** −0.6 to +0.1 mm cross-track, 0.4–1.0 % against 7–10 % on the rate limit, +1.6–1.8 mrad heading on the curves.
- **Two overruns** (15.6 and 6.6 ms late) coincided with the offline item 3 runs, which ran alongside at `nice -n 19`.

**Held-out seeds 6–10** (Gazebo, the carried-forward smoother variant): paired gap to the baseline +0.36 / +4.09 / +3.14 / +0.98 mm, consistent with seeds 1–5 (+0.40 / +3.92 / +3.12 / +0.62).

**Solver gap** (offline):
- **max_v:** the S-curve rule picked 60 on a non-monotone, noisy surface. It is worse on D1 (3.40×) and in the closed loop, so `max_v` stays 30.
- **10×60:** J_PSO / J_opt 2.73 → 2.52×; closed loop 0.66 → 0.53 mm. It does not close the gap.
- **Future work:** a deterministic refinement step.

### 7.15 Decisions after BH (ARC session, `2026-11-22-claude-to-claude-rc-car-bh-review-next.md`)

1. **Accepted as the candidate controller:** the smoother R1+R2+R3+R4 10×40 variant (`params/nmpc.yaml` `redesign_smooth`). It was chosen after seeing the BR results on seeds 1–5 (post hoc) and is consistent on the held-out seeds 6–10 (block BH). The rule's choice R1+R3+R4 (`redesign`) stays in the record as "selected by the rule, not carried forward".
2. **No 10×60** for the candidate: the offline gain (about 0.1 mm) does not justify a board run. 10×40 stays (board: 25.0 ms solver median).
3. **The Gazebo study is paused** until the car's measurements arrive (turning circle, payload position, floor size). Everything stays reproducible from `params/` and `checks/run_phase4_batch.sh`. Lesson kept: offline jobs run only between Gazebo blocks.
4. **Next (done at C level, 2026-11-22):** the bicycle model and the redesign switches in the HLS sources. See `docs/rc_car/hls_bicycle_redesign.md` and the STOP H reply. The HLS engine integrates the bicycle model with RK4 (Stage A); the Gazebo candidate predicted with the chord step (Stage B).

## 8. How to run (in the container, as for the Husky)

The repository is mounted read-only at `/home/ubuntu/nmpc_hls`. The car files are in the worktree.

**Before any run,** check that `pgrep -af 'vivado|vitis|xsim|csim'` prints nothing.

```bash
R=/home/ubuntu/nmpc_hls/.claude/worktrees/agent-a7f53c7be12d59b73/app/rc_car
# phase 1: the car and the plant interface only
ros2 launch $R/launch/rc_car_sim.launch.py                # headless (gui:=true for the GUI)
python3 $R/checks/bringup_checks.py                       # C1-C6, about 2.5 min
# phase 2: the full command chain
ros2 launch $R/launch/rc_car_stack.launch.py profile:=realistic_guess mocap_profile:=realistic_guess seed:=1
python3 $R/checks/chain_checks.py --profile realistic_guess --seed 1   # D1-D3, S1-S5, about 40 s
# drive by hand through the mux (teleop has priority; it ends 0.3 s after the last message)
ros2 topic pub -r 20 /teleop/ackermann_cmd ackermann_msgs/msg/AckermannDriveStamped "{drive: {speed: 0.5, steering_angle: 0.3}}"
ros2 topic echo /mux/state                                # mode, lock, pose age, counters
ros2 service call /mux/arm std_srvs/srv/Trigger           # explicit arm (needs a streaming controller)
ros2 topic pub --once /mux/lock std_msgs/msg/Bool "{data: true}"   # kill lock (false releases to hold)
```

**Phase 3: the baseline arm** (a second terminal for each launch):

```bash
ros2 launch $R/launch/rc_car_stack.launch.py profile:=realistic_guess
ros2 launch $R/launch/baseline_rpp.launch.py
python3 $R/checks/track_run.py --paths line circle figure_eight lane_change --seeds 1 --profile realistic_guess
# tuning only: --paths s_curve --grid lookahead_time=0.2,0.25 min_lookahead_dist=0.15,0.2
python3 $R/checks/plot_tracks.py <run dir> --out <run dir>/plots    # needs the container's matplotlib
```

**C3 diagnostic variants:**

```bash
ros2 launch $R/launch/rc_car_sim.launch.py drive:=rwd controllers_file:=$R/config/controllers_rwd.yaml contact_width:=0.01
python3 $R/checks/bringup_checks.py --only-full-lock --speeds 0.5 --label rwd_narrow
```

## 9. After 27 Nov (not now)

**HLS and KV260:**
- **HLS bicycle model:** `BICYCLE_CONFIG`, or a variant of `hls_unicycle.hpp` with ω = v·κ, in the same pragma-on-own-line style. C checks against `ackermann_model.py` and `hw_estimates.py`.
- **PC C-model backend** in the SIL; then the KV260 overlay and the TCP server, shared with the parked Husky ABI work.
- **The KV260 side:** a uXRCE-DDS agent plus a ROS 2 node, or a MAVLink client, in the KV260 host loop. Choose after checking what runs on the board's Ubuntu image.

**PX4 and the Pixhawk 4** (check every item against the PX4 version on the car):
- **Interface:**
  - uXRCE-DDS client on a telemetry port, Micro XRCE-DDS Agent on the companion, `/fmu/in/...` and `/fmu/out/...` (`px4_msgs`); MAVLink is the fallback;
  - **the `px4_msgs` version must match the firmware.**
- **First version:**
  - `OffboardControlMode` with `direct_actuator`;
  - `ActuatorServos` (steering) and `ActuatorMotors` (VESC speed command);
  - this is exactly the applied [v, δ] the simulation models.
- **Offboard keep-alive and loss:**
  - setpoints streamed continuously, above about 2 Hz;
  - `COM_OF_LOSS_T` and `COM_OBL_RC_ACT`;
  - the RC mode switch always returns control to the transmitter.
- **Firmware in the loop: PX4 SITL with Gazebo Harmonic** (PX4's gz Ackermann rover target in recent versions; check the name and version against the car's firmware), with the uXRCE-DDS agent. That tests the offboard path, keep-alive and failsafe before the car moves. **It needs the PX4 toolchain or image: ask the user before any pull, install or build.** Disk space is tight.
- **Estimation:** EKF2 fusing the OptiTrack pose (external vision, `/fmu/in/vehicle_visual_odometry`, `EKF2_EV_CTRL`) with the IMU; yaw rate and velocity fed to the NMPC.
- **The real end-to-end delay,** from a command step against the gyro response in the Pixhawk log. It replaces τ and the assumed chain values.
- **An optional turn-rate inner loop** on the Pixhawk (the NMPC sends [v, ω = v·κ]). It needs identification of the inner loop.

**The KV260 on the car (lab facts 2026-11-22; list only, not simulation tasks):**
- how it is powered: from the drive battery through a DC-DC converter, or a separate pack;
- vibration and cooling;
- weighing the payload assembly (`payload_mass` is `PLACEHOLDER_` 0.5 kg).

**The real car:**
- parameter measurements (`parameters.md` §C);
- VESC/servo calibration, and step-response identification of `tau_v`, `T_d` and `delta_rate_max`;
- a car-specific safety plan: speed and current caps in the VESC firmware, a radio kill switch, a deadman, an enclosed area, and a person on the kill switch;
- only then supervised low-speed runs. **The user operates the car; agents never command it.**

**Bill of materials (verification items only, no ordering):**
- the VESC and its 2S/3S range;
- 3S compatibility of the drivetrain and battery tray;
- the 550 → 540 motor mount, shaft and pinion;
- a sensored motor's kV;
- a faster servo;
- the companion computer and its power (BEC);
- an electronics deck.

**Optional:** `ros-jazzy-ackermann-nlmpc` exists in apt. It could serve as a second reference controller; it is not part of the agreed comparison.
