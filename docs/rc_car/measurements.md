The measurements fall into three groups, in the order you need them. The car's measurement sheet is in [parameters.md §C](docs/rc_car/parameters.md); its item numbers are given in brackets below. Every value the simulation uses now is either a manufacturer figure or a placeholder; the only exceptions are your rough track (30 cm), full-lock (~25°) and payload (1.5 kg) values.

### 1. To restart the Gazebo study (car on a stand, motor unplugged)

| What | How | Why it matters |
|---|---|---|
| **Wheelbase L** [1] | Axle centre to axle centre, both sides | Sets the steering geometry and the maximum curvature (the manual says 275 mm, not checked) |
| **Full-lock angle of each front wheel, at both left and right lock** [4] | Protractor, or a photo from above over a grid | The 25° can be read three ways (inner wheel, outer wheel or a single average). Four angles settle it |
| **Turning circle, left and right** [5] | Push the car by hand at full lock and mark the rear-axle midpoint's path; `fit_turning_circle.py` is ready | Checks item 4 directly. The paths must stay within 70 % of the tightest curvature |
| **Loaded wheel radius** [3] | Push the car through 5 wheel turns: r = d/(10π) | Converts wheel speed to car speed |
| **Mass, centre of mass and payload position** [8] | Scale: whole car, front axle only, rear axle only; where the KV260 plate sits (ahead of the rear axle, and its height) | The payload position is still a guess |
| **Steering speed** [7] | Slow-motion phone video, lock to lock | The steering-rate limit; it's 5 rad/s now, an assumed value |
| **Floor size** | Length × width of the area where tracking is reliable **at car height** (see 2) | Sets the paths and the safety margin (10 × 10 m with a 1 m margin assumed) |

### 2. OptiTrack, before the first real drive

| What | How |
|---|---|
| **Usable tracking area at floor level** | Push the car (markers at ~10–15 cm) around the planned paths and count frames where tracking is lost. The ceiling cameras may cover less near the floor than the nominal 100 m² |
| **Rigid body** | An asymmetric marker set. Record where its reference point sits relative to the rear-axle midpoint, and the heading offset to the car's forward axis (place the car on a straight line and read the yaw) |
| **Frame rate** | As set in Motive (100 Hz assumed) |
| **Noise** | Car standing still for 60 s: spread of x, y and heading (0.5 mm and 2 mrad assumed) |
| **Latency, jitter and dropouts on the path to the car** | Motive's reported latency, plus the Wi-Fi delay to the KV260. Log Motive's frame timestamp and the arrival time on the KV260 for a few minutes |
| **Streaming and frames** | How the pose is streamed to ROS 2 (NatNet, VRPN or mocap4r2); Motive's up axis and ground-plane alignment |
| **Wi-Fi on the car** | Which network the KV260 joins, and its signal strength across the floor |

### 3. Car hardware facts (no driving needed)

- **Servo pulse → wheel angle map, with deadband** [6]: from 1000 to 2000 µs, approaching the centre from both sides.
- **Radio fail-safe** [11]: what the servo does when the transmitter is switched off.
- **The servo's brand and model** [12], and **the gear ratio** [10].
- **PX4 version** on the Pixhawk 4: the ROS 2 message definitions must match it.
- **Whether the car's motor and controller are being changed**: the plan assumes a VESC with a sensored brushless motor (and its 3S battery caveat), while the sheet describes the brushed stock setup.

### Later, once the KV260, Pixhawk and VESC are wired

These are step tests on the stand, run by you:
- the full command-to-wheel delay;
- the motor controller's speed response, acceleration and braking;
- the lowest speed the car can hold steadily.

They replace the remaining placeholder settings for the command chain.

**What unblocks the agent:** group 1 plus the floor size. Group 2 is needed before any real run, and it's also what a real-experiment section of the paper would report as setup facts.


## Holybro X500

Before the list, three things in the current quadrotor setup decide which measurements matter:

- **The solver's model uses made-up drone constants, and they are built into the KV260 hardware.** [hls_sniffbot.hpp:41-48](hw/hls/source/include/models/hls_sniffbot.hpp#L41-L48) sets mass 2 kg, inertia 1.2 / 1.2 / 2.3 kg·m², arm 0.25 m, kf = 1, kM = 0.2. Those inertias are tens of times larger than a 500 mm frame's. The motor mixing assumes a "+" layout, but the X500 is an "X" frame. Flying needs the X500's measured values and a rebuilt overlay.
- **The solver outputs collective thrust and three body torques at 20 Hz** (Ts = 50 ms). A real drone can't be stabilised by torque commands at that rate from a companion computer; PX4 runs its attitude loops at 250 Hz or more. You need to decide what the NMPC sends to PX4:
  - **Position and velocity setpoints from the predicted trajectory:** keeps the 12-state model and is the safest first flight. I'd start here.
  - **Attitude plus thrust setpoints:** the usual level for a real drone NMPC, but it needs a new model in hardware.
- **The test trajectory doesn't fit the arena.** It's a circle of 2.5 m radius centred at (7, 12.5) m whose height ramps towards 15 m ([quadrotor.py:16-23](app/kria-pynq/quadrotor.py#L16-L23)). In 7 × 8 × 4 m it has to be re-centred and its height capped, for example radius ≤ 2 m and 1.0–2.5 m high. A 2.5 m radius would leave only 1 m to the walls across the 7 m side.

## What to measure

### A. The drone as flown (KV260, its power supply and the battery all mounted)
1. **Take-off mass.**
2. **Centre of mass.** Balance the drone on an edge along both axes. Note how far it is from the frame centre, and how high.
3. **Inertia Ixx, Iyy, Izz.** Hang the drone as a two-string (bifilar) pendulum about each axis and time 20 swings.
4. **Motor positions.** Hub-to-hub distances; the nominal 500 mm diagonal gives 0.177 m per axis for an X frame.
5. **Mounting check.** Props clear the KV260 and the markers, and the KV260 fan's vibration doesn't reach the Pixhawk IMU (see C2).

### B. Motors and propellers (props on, drone secured on a stand)
1. **Thrust against throttle command**, using a load cell or a kitchen scale under one motor arm, on a charged 4S battery. This gives kf and the maximum thrust.
2. **Yaw torque coefficient kM.** Use a torque stand, or estimate it from a yaw step in flight (C3).
3. **Motor time constant.** From a throttle step on the stand, or from the ESC rpm in the flight log.
4. **Thrust loss as the battery voltage drops.** Thrust at full charge and at the voltage you land at.
5. **Thrust-to-weight ratio with the payload.** It should stay at about 2 or more; if it's lower, the KV260 payload is too heavy for safe tests.

### C. PX4 and the flight log (you fly in Position mode; this is setup work, not an NMPC flight)
1. **PX4 firmware version.** It decides the ROS 2 link: PX4 1.12 uses microRTPS, 1.14 and later use uXRCE-DDS. It also decides which offboard setpoint types exist.
2. **A hover log with the payload:** hover throttle (a check on B1) and vibration levels.
3. **Step responses to roll, pitch and yaw setpoints.** These give the inner-loop response time if the NMPC works at attitude level.
4. **The KV260 ↔ Pixhawk link:** which port (TELEM2 serial, since the Pixhawk 4 has no Ethernet), its baud rate, and the round-trip time and message rate.

### D. Power
1. **KV260 current draw from its regulator, at idle and while solving.** This also gives the energy figure the paper doesn't have yet (the board power measurement you'd made optional).
2. **Hover time with the payload**, which sets the length of a test run.

### E. OptiTrack (the arena is 7 × 8 × 4 m)
1. **Where tracking is reliable** at flying heights of 0.5–3 m. Walk the drone around the planned circle by hand, at each height, and count frames where tracking is lost.
2. **Rigid body.** An asymmetric marker set that the props don't hide. Record the offset from its reference point to the centre of mass (A2) and the yaw offset to the drone's forward axis.
3. **Axis and frame conventions.** Motive's up axis (Y-up by default) and how its frame maps onto PX4's ENU/NED. A sign error here is the most common first-flight crash.
4. **Frame rate, and noise at rest** (60 s, drone still): spread of position and yaw.
5. **Latency and dropouts** from Motive → ROS 2 → the Pixhawk's estimator (fed as external vision), and → the KV260. Log the time stamps on both ends.
6. **Where the NMPC gets its velocities and angular rates.** Its 12 states include them. I'd take them from PX4's fused odometry rather than differentiating the OptiTrack data; then that odometry's latency is the number to measure.

### F. Safety envelope (you and your supervisor)
- A geofence inside the arena, for example 1 m from the walls and at most 3 m high.
- A kill switch on the RC transmitter.
- What happens if the offboard link or the OptiTrack feed drops: land or hold position.
- I don't know whether the arena has a net; if it does, decide whether it's needed for these tests.

## Order and timing
- **Needed first:** A1–A3, B1, C1 and the PX4 version. With those, an X500 simulation can run and the overlay can be rebuilt with real constants.
- **Simulation while you measure:** PX4's own simulator has a built-in X500 model, so the simulation agent can start there. It needs PX4 and Gazebo installed, which I'll ask you about before pulling anything.
- **The paper:** a flight by 27 Nov isn't realistic, because it needs the measurements, a rebuilt overlay, the setpoint decision and safety approval. The paper stays with the board-in-the-loop results, and the flight comes after submission.

I've saved the vehicle change, the arena size and these open points to memory.