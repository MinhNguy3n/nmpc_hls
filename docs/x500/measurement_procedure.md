# Holybro X500: measurement procedure for the KV260 PSO-NMPC

**Purpose.** The values measured here replace the generic quadrotor constants that the solver currently uses (`hw/hls/source/include/models/hls_sniffbot.hpp`: m 2 kg, Ixx = Iyy 1.2 kg·m², Izz 2.3 kg·m², arm 0.25 m, kf 1, kM 0.2, a "+" motor layout). The NMPC-PSO overlay is then rebuilt for the X500. The same values set up the PX4 + Gazebo simulation.

**Control interface (decided 2026-11-23).** The NMPC runs on the KV260. From its predicted trajectory it sends **position and velocity setpoints** to PX4 in Offboard mode. PX4's own position, velocity, attitude and rate loops fly the drone. So two things matter:
- the rigid body and propulsion (sections 2–5), which go into the NMPC's prediction model;
- PX4's closed-loop response to setpoints (section 8), which is what the NMPC actually commands.

**Where the results go.** Record every value in `app/x500/params/x500_measured.yaml` (template in section 12). Give each value a `source` tag: `measured`, `spec` (from a datasheet) or `placeholder`. Keep the raw data (photos, videos, logs, spreadsheets) in a dated folder, for example `data/x500/2026-11-xx/`, and name the files after the section numbers below.

---

## 0. Safety rules and the "as-flown" configuration

- **Props off** for every test except section 5, which uses a thrust stand, and the flights in sections 7–9.
- **Flights** (sections 7–9): only in the netted arena, by a pilot with the RC kill switch in hand, and with a second person present. Agents never command the drone, the Pixhawk or the KV260.
- **LiPo handling:** charge in a LiPo bag, never below the storage voltage on the bench, and stop flights at the landing voltage you set in section 7.
- **The as-flown configuration.** Do sections 1–4 only after the final mounting of the KV260, its DC-DC regulator, the cables, the OptiTrack markers and the flight battery in its flight position. Photograph the drone from the top, front and side, with a ruler in the picture.
  - **If anything moves or is added later** (battery position, a different battery, the mount, the markers), repeat sections 1–4.
- **Body frame used in this document:** PX4's FRD (x forward, y right, z down), with the origin at the geometric centre of the frame (the centre of the four motor shafts, at the height of the motor mounting plane). Mark "forward" on the frame with tape. Motor numbering follows PX4's quad-X airframe (check it in QGroundControl → Actuators).

**Equipment**

| Item | Used in |
|---|---|
| Kitchen or postal scale, 1 g resolution, ≥ 5 kg | 1, 3 |
| Steel ruler, calipers, tape measure (1 mm) | 2, 3, 4 |
| Thin, non-stretch string (braided fishing line, 0.3–0.5 mm), two hooks, a rigid beam or door frame ≥ 1.5 m high | 4 |
| Phone that records slow-motion video (120 or 240 fps), or a stopwatch | 4 |
| A reference object with a known inertia (e.g. a uniform aluminium or wooden plate of about 0.4 × 0.3 m and 1–2 kg) | 4 |
| Thrust stand with thrust, torque and rpm (e.g. Tyto/RCbenchmark Series 1580/1585), a spare motor + prop + ESC of the same types, and a bench supply or the flight battery | 5 |
| Multimeter or a USB-C/DC power meter | 6 |
| QGroundControl, a microSD card in the Pixhawk with logging enabled | 7–9 |

---

## 1. Mass

1. Weigh the complete drone, as flown, with the battery: **`mass_takeoff`**. Weigh three times, lifting it off the scale in between, and record the mean.
2. Weigh each part separately: the flight battery, the KV260 with its heatsink and fan, the DC-DC regulator, the mount, and the marker set. Record each, for the payload budget.
3. Repeat for each battery you will fly with, and record the difference.

**Check:** the sum of the parts plus the bare drone is within 10 g of `mass_takeoff`.

## 2. Geometry

1. **Motor positions.** Measure each motor shaft's position from the frame centre in x and y. Measure the diagonals between opposite motor shafts, and the distance between neighbouring shafts. Record **`rotor_positions`** (x, y; m, FRD) and the spin direction of each motor (CW or CCW, from PX4's motor diagram).
   - **Check:** both diagonals agree within 3 mm; the nominal value is 500 mm, which puts each motor about 0.177 m from the frame centre along x and along y.
2. **Propeller plane height:** the vertical distance from the motor mounting plane to the propeller hub (z, m).
3. **Positions of the main payload items:** the KV260, the battery and the regulator, each as the centre of its box (x, y, z from the frame centre). These are used to check the CoM and inertia (sections 3–4) by estimate.
4. **Record the propeller type and size** (e.g. 1045), the motor type and kV (e.g. 2216 KV880), the ESC type and protocol (PWM, OneShot or DShot), and the battery's cell count and capacity.

## 3. Centre of mass (CoM)

**Horizontal position, x and y (balance method):**
1. Lay a round rod or the edge of a ruler on the table, parallel to the y axis.
2. Place the drone on it, landing gear removed or hanging free, and move it until it balances. Mark the balance line on the frame.
3. Measure the line's distance from the frame centre: **`com_x`**.
4. Repeat with the rod parallel to the x axis: **`com_y`**.
5. Do each three times.

**Vertical position, z (suspension method):**
1. Hang the drone from a string tied to one motor arm, near the motor.
2. Take a photo straight from the side, with a plumb line from the same hook in the picture. The CoM lies on the plumb line.
3. Repeat, hanging it from the opposite side or from the battery strap.
4. The two plumb lines cross at the CoM. Read its height from the photos, using a ruler in the picture as the scale: **`com_z`** (FRD, so a CoM below the motor plane is positive).

**Check:** the horizontal offset should be under 1 cm. If it is larger, move the battery to rebalance the drone and repeat sections 1–3. A large offset costs thrust margin and shows up as a steady tilt in hover.

## 4. Moments of inertia: bifilar pendulum

The drone hangs from two parallel strings of length L, a distance d apart, placed symmetrically about the CoM. Twist it by a small angle (under 10°) about the vertical axis through the CoM, release it, and time the oscillation period T. Then

  **I = m · g · d² · T² / (16 · π² · L)**

where m is the total suspended mass, g = 9.81 m/s², d is the distance between the strings, and L is the string length.

**Set-up:**
- **String length:** L ≈ 1.0–1.5 m. Measure it from the hook to the attachment point on the drone.
- **String spacing:** d ≈ 0.3–0.4 m. Place the strings symmetrically about the CoM, so the rotation axis passes through it (this is why section 3 comes first).
- Both strings exactly vertical and the same length. The drone should hang level; check it with a phone level app.
- **Timing:** start the oscillation gently, with no swinging, then time 20 full periods and divide by 20. Do three trials per axis. Slow-motion video, counting from a mark on the frame, is more accurate than a stopwatch.

**Validate the method first.** Hang the reference plate and compare its measured inertia with the computed value, I = m(a² + b²)/12 about the axis normal to the plate.
- **Pass:** within 5 %. If it isn't, check that the strings are vertical, L, and the timing before measuring the drone.

**The three axes:**
- **Izz (yaw):** hang the drone level, with the strings attached at two points on a line through the CoM (e.g. on the arms, or on a light bar fixed under the frame).
- **Ixx (roll) and Iyy (pitch):** fix the drone in a light rigid jig (e.g. a wooden or aluminium frame) so that its x axis (and then its y axis) is vertical, then hang the jig the same way.
  - **Jig correction:** measure the jig alone in the same way and subtract it: I_drone = I_total − I_jig. The m in the formula is the mass of whatever hangs: drone + jig for I_total, jig alone for I_jig.
  - **CoM:** the drone's CoM must lie on the rotation axis. Adjust the jig until the drone hangs level in the jig.

**Record:**
- **`inertia_diag`** [Ixx, Iyy, Izz] in kg·m², about the CoM;
- the raw table: L, d, m, the 20-period times of each trial, and T.

**Plausibility:** a drone of this size and mass should give values of the order of 10⁻² kg·m², with Izz the largest. The current solver uses 1.2 kg·m², a generic value that is far too large for this frame. If your result is within a factor of 3 of the generic value, recheck the set-up.

**Optional cross-check:** estimate the inertia from the component masses and positions (section 2.3), treating motors and the battery as point masses and the KV260 as a box. The estimate and the measurement should agree within about 20 %.

## 5. Propulsion: thrust stand

Use a spare motor, prop and ESC of exactly the types on the drone. Run the ESC protocol the drone uses, with the prop in its flying orientation. Supply 16.8 V (a full 4S battery) and repeat at about 14.8 V (a half-empty 4S battery).

1. **Steady-state curve.** Step the throttle command from 0 to 100 % in 5 % steps, using 2.5 % steps between 30 and 60 %, where hover will be. Hold each step for 3 s and record the means over the last second of:
   - thrust (N);
   - torque (N·m);
   - rotor speed (rpm);
   - current (A);
   - voltage (V).
2. **Fits** (raw table in a spreadsheet):
   - **`kf`:** thrust = kf · ω², with ω in rad/s (rad/s = rpm · 2π/60);
   - **`km_over_kf`** (m): yaw torque = km_over_kf · thrust;
   - **`thrust_max_per_motor`** at 16.8 V and at 14.8 V;
   - **`omega_max`**;
   - the command → thrust curve, for PX4's `THR_MDL_FAC`, with the thrust normalized to its maximum.
3. **Motor time constant.** Command throttle steps 30 → 60 % and 60 → 30 %, five each, with the stand logging at its highest rate (≥ 100 Hz; rpm is best). Fit a first-order response to each, giving **`motor_tau_up`** and **`motor_tau_down`** (s).
4. **The hover operating point.** Hover thrust per motor = mass_takeoff · g / 4. Read the throttle and current needed for it from the curve. Record the thrust-to-weight ratio, 4 · thrust_max_per_motor / (mass_takeoff · g), at 14.8 V.
   - **Pass:** the ratio is at least 2.0. If it's lower, the KV260 payload is too heavy for safe control tests. Lighten the payload, or move to larger props or a 5S setup, before flying.

**If no thrust stand is available:** take the hover throttle from the flight log (section 7) and the motor data from the manufacturer's published thrust table for the same motor, prop and voltage. Tag all of these values `spec`. The NMPC model then has larger uncertainty in kf and km_over_kf, so state that in any result.

## 6. KV260 power (bench, before flight)

Power the KV260 through the same DC-DC regulator from the flight battery (or a bench supply at 16.8 V). Measure the input current and voltage on the battery side:
1. booted, idle;
2. while the solver runs continuously (the board notebook's closed loop);
3. at peak, during boot.

**Record:** **`kv260_power_idle_W`**, **`kv260_power_solve_W`** and the peak, plus the regulator's own loss (compare its input and output). This also gives the energy figure that is missing from the paper.

## 7. Hover flight (Position mode, flown by the pilot)

**Before the flight, in QGroundControl:**
- Set `SDLOG_PROFILE` to include high-rate logging (the "high rate" and "estimator replay" bits).
- Set `SDLOG_MODE` to log from boot.
- Set the battery parameters (`BAT1_N_CELLS`, voltage empty and full) and choose the landing voltage.

**The flight:**
1. Take off.
2. Hover at 1.5 m for 60 s, hands off the sticks as far as possible.
3. Make one slow lap of the arena at about 0.5 m/s.
4. Land.

**Record from the log** (PX4 Flight Review, or `pyulog`):
- **hover thrust:** the `hover_thrust_estimate` topic, or the mean collective thrust in steady hover. This checks section 5.4.
- **vibration:** the accelerometer vibration metric and the raw accelerometer spectrum. Compare them with PX4's vibration-isolation guidance; the KV260 fan and mount are the new suspects. If the vibration is high, add damping before any NMPC flight.
- **voltage sag and current at hover**, and the hover time to the landing voltage. Plan runs to use at most 60 % of that time.

## 8. PX4's response to setpoints (Offboard mode)

These are the dynamics the NMPC actually drives, so they matter most. You need the companion link (section 9) and an offboard setpoint script. The test is run by you, never by an agent.
- The offboard script holds the drone at a point and steps the setpoint. It is written and tested in the PX4 SITL simulation first.
- Use the same PX4 firmware and parameters as for the real flights.

1. **Position step:** 0.5 m in x, then y, then z. Three of each, from hover at 1.5 m, with 5 s between steps.
2. **Velocity step:** 0 → 0.5 m/s in x for 2 s, then back to 0 (position setpoint NaN, velocity only).
3. **Combined:** a 2 m circle at 0.5 m/s, with position and velocity setpoints from the analytic circle.
4. **Stopping distance:** fly straight at a steady 0.5 m/s and then 1.0 m/s, through the middle of the arena and away from the nets. Command a hold at a marked point and record the overshoot past it and the time to stop. This sets the fence and speed limits in section 11.

**Record:**
- rise time, overshoot and settling time for each step;
- the steady tracking error on the circle;
- a full parameter dump (QGroundControl → Parameters → Tools → Save to file). Keep the `MPC_*`, `MC_*` and `EKF2_*` values in particular.

The circle run is also the baseline the NMPC will be compared against: PX4 tracking the reference directly.

## 9. Companion link: KV260 ↔ Pixhawk 4

The Pixhawk 4 (FMUv5) has no Ethernet, so the link is serial (TELEM2), using the uXRCE-DDS client in PX4 1.14 or newer.

1. **Firmware.** Record the PX4 version on the Pixhawk (QGroundControl → Vehicle Setup → Summary). Use the same version as the SITL simulation; the `px4_msgs` definitions must match. Upgrade if it is older than 1.14 (the link only exists in 1.14 and newer). PX4 1.12 used a different link (microRTPS), which the simulation will not use.
2. **Link settings.** Record:
   - the port and `UXRCE_DDS_CFG`;
   - the baud rate (`SER_TEL2_BAUD`, e.g. 921600);
   - the cable;
   - which UART on the KV260.
3. **Link quality.** Measure the message rates received on the KV260 (`ros2 topic hz` on `/fmu/out/vehicle_odometry`) and the time-sync offset and round-trip time the agent reports. Run 5 min on the bench with the props off.

## 10. OptiTrack

**The arena:** 7 m × 8 m, with protective nets on the sides and a ceiling net at 5 m.

1. **Calibration.** Record the Motive version, the wanding result (mean 3D error) and the date. Set the ground plane with the calibration square at the arena centre, which becomes the origin.
2. **Tracked volume at flying heights.** Carry the drone by hand around the planned path at 0.5, 1.5 and 2.5 m. Record lost frames per height, and mark any zones with dropouts.
3. **Rigid body:**
   - **marker set:** at least 4 markers in an asymmetric layout, mounted where the props, the KV260 and the battery can't hide them, and rigid with the frame;
   - **reference point:** set it to the CoM if Motive allows; otherwise measure its offset to the CoM (section 3);
   - **yaw zero:** place the drone with its forward axis along the arena's x axis (a taped line on the floor) and record the yaw Motive reports.
4. **Axes and frames.** Record Motive's up axis (Y-up by default, Z-up selectable) and the streaming frame. Write down the mapping to PX4's local NED frame (x north/forward, y east/right, z down) and to the NMPC's frame.
   - **Check:** move the drone by hand 1 m along each arena axis, and rotate it +90° in yaw, while watching PX4's `vehicle_visual_odometry` and `vehicle_local_position`. The signs and directions must match. A wrong sign here is the most common cause of crashes in a first mocap flight.
5. **Rate and noise.** Record the frame rate (e.g. 120 Hz). Leave the drone still on the floor for 60 s and record the standard deviation of x, y, z and yaw.
6. **Latency and dropouts.**
   - Motive's reported system latency.
   - The end-to-end latency to the Pixhawk: tune PX4's `EKF2_EV_DELAY` from a log with fast hand-held motion (the delay that minimises the vision innovations; PX4's EV documentation describes how).
   - Over 5 min, the dropout rate and the longest gap.
7. **Fusion settings.** Record `EKF2_EV_CTRL`, `EKF2_HGT_REF` (vision), `EKF2_GPS_CTRL` (off indoors), `EKF2_EV_NOISE_MD` and the EV noise values. Record the ROS 2 bridge used, for example mocap4r2, VRPN or NatNet → `/fmu/in/vehicle_visual_odometry`.

## 11. Safety envelope (agree it with your supervisor)

Proposals; your decision:
- **Geofence:** 1.0 m inside the side nets, giving a usable 5 × 6 m. Height 0.5–3.0 m; the ceiling net is at 5 m. In PX4, set the geofence parameters with the "Land" action, and set the maximum altitude.
- **Kill switch** on the RC, tested before each session. The RC can override Offboard at any time.
- **Loss of the offboard link** (`COM_OBL_RC_ACT`) and **loss of mocap** (the EKF falls back): hold position, then land.
- **Ground-station link** (found in SITL on 2026-11-23): the stock X500 airframe sets `NAV_DLL_ACT` = 2. With that setting the drone refuses to arm without a ground-station link, and on link loss it switches to Return mode.
  - Indoors, Return mode climbs to `RTL_RETURN_ALT` (tens of metres by default) above the 5 m net.
  - **Proposal:** keep QGroundControl connected during flights, and set `NAV_DLL_ACT` = 3 (Land).
  - If Return mode can be triggered any other way, also set `RTL_RETURN_ALT` and `RTL_DESCEND_ALT` below the net. Check every failsafe action parameter for Return mode.
- **Settings from the simulation are not lab settings.** On the real Pixhawk, check each of these failsafe actions for the PX4 version actually flashed: Offboard loss, RC loss, data-link loss, estimator or mocap loss, and battery. Every action must be safe below the ceiling net, and none may depend on a working KV260.
- **Braking distance against the nets** (learned in SITL on 2026-11-24, where geofence trips at speed ended in the nets):
  - **The rule:** the distance from the geofence to the nets, minus the drone's radius, must be larger than the drone's stopping distance at the highest outward speed it can reach near the fence.
  - **Measure the stopping distance** in section 8: from steady 0.5 m/s and 1.0 m/s, command a hold and record how far the drone overshoots.
  - **Then either** cap the speed (`MPC_XY_VEL_MAX`, and the offboard setpoints) so the stopping distance fits, **or** move the fence inward.
  - **Also check** whether the PX4 version on the drone can trigger the geofence predictively (a `GF_PREDICT`-type parameter), and how it predicts.
- **Stale or lost mocap:** the drone lands or disarms and does **not** re-arm by itself. Test this props-off, by stopping the OptiTrack stream.
- **People and procedures** (from the GPT-6 review, 2026-11-23):
  - a dedicated safety pilot on the RC, and a separate spotter;
  - an independent way to disarm the motors, tested before each session (RC kill switch, plus a second method you agree on);
  - a written checklist for arming, take-off and landing;
  - props-off checks before the first offboard flight of each session: motor order and direction, then setpoint direction (+x, +y, +yaw move the way expected);
  - a written safety plan for this drone, agreed with your supervisor, before any offboard flight.
- **The first NMPC flights:** at 0.3 m/s, with a 1.5 m circle at 1.5 m height, before the full reference.

## 12. Results file: `app/x500/params/x500_measured.yaml`

```yaml
# Holybro X500 + KV260, as flown. Each entry: value, unit, source (measured|spec|placeholder), date, note.
meta:
  configuration_photos: data/x500/2026-11-xx/0_config/
  px4_version: {value: null, source: placeholder}          # section 9
  px4_params_file: {value: null}                            # section 8
airframe:
  mass_takeoff: {value: null, unit: kg, source: placeholder}            # 1
  battery: {cells: 4, capacity_mAh: null, mass: null}                   # 1, 2
  payload_items: []      # [{name, mass, position_xyz}]                 # 1, 2
  com_offset: {value: [null, null, null], unit: m, frame: FRD_from_frame_centre}   # 3
  inertia_diag: {value: [null, null, null], unit: kg m^2, about: CoM}   # 4
  rotor_positions: {value: [[null, null, null], [null, null, null], [null, null, null], [null, null, null]], unit: m, frame: FRD_from_CoM, order: PX4_quad_x}  # 2
  rotor_directions: {value: [null, null, null, null], note: "+1 = CCW from above"}  # 2
propulsion:
  motor: null      # type, kV
  prop: null       # size, pitch
  esc_protocol: null
  kf: {value: null, unit: N/(rad/s)^2}                  # 5.2
  km_over_kf: {value: null, unit: m}                    # 5.2
  omega_max: {value: null, unit: rad/s}                 # 5.2
  thrust_max_per_motor: {value_16v8: null, value_14v8: null, unit: N}   # 5.2
  motor_tau_up: {value: null, unit: s}                  # 5.3
  motor_tau_down: {value: null, unit: s}                # 5.3
  thrust_curve_file: null                               # 5.1 raw table
  thrust_to_weight_14v8: {value: null}                  # 5.4
flight:
  hover_thrust_normalized: {value: null}                # 7
  hover_time_to_landing_voltage: {value: null, unit: s} # 7
  vibration_ok: {value: null, note: ""}                 # 7
  setpoint_response:                                     # 8
    pos_step_rise_time: {x: null, y: null, z: null, unit: s}
    pos_step_overshoot: {x: null, y: null, z: null, unit: m}
    vel_step_rise_time: {value: null, unit: s}
    circle_rmse: {value: null, unit: m, radius: 2.0, speed: 0.5}
link:
  port: TELEM2
  baud: null
  odometry_rate_hz: null
  rtt_ms: null
kv260_power: {idle_W: null, solve_W: null, peak_W: null, regulator_efficiency: null}   # 6
mocap:
  motive_version: null
  calibration_error_mm: null
  rate_hz: null
  noise_std: {x: null, y: null, z: null, yaw: null, unit: [m, m, m, rad]}
  latency_motive_ms: null
  ekf2_ev_delay_ms: null
  dropout_rate: null
  longest_gap_ms: null
  rigid_body_ref_to_com: {value: [null, null, null], unit: m}
  yaw_offset: {value: null, unit: rad}
  axis_mapping: null      # e.g. "Motive Y-up -> ENU: x=X, y=-Z, z=Y"
arena:
  size: {value: [7.0, 8.0], unit: m, source: measured}
  net_height: {value: 5.0, unit: m, source: measured}
  geofence_margin: {value: 1.0, unit: m, source: placeholder}
  max_altitude: {value: 3.0, unit: m, source: placeholder}
```

## Order of work

1. **Bench, no flight:** sections 1–4 and 6, then 5 (thrust stand). These alone let the NMPC model be rebuilt and the simulation switched from the generic constants to the X500.
2. **Mocap set-up:** section 10, items 1–5 (hand-held, props off).
3. **Flights:** sections 7 → 10.6–10.7 → 9 → 8, in that order. Section 8 needs the offboard script that has been tested in SITL.
