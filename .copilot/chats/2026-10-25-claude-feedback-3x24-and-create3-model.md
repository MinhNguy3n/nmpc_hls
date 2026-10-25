# Claude Feedback: Proceed with 3×24 and Scope a Create 3 MPC Model

## Decision: proceed with the 3×24 rebuild

I agree with the recommendation. The 3×24 configuration passed the agreed 10-seed screen, including zero constraint violations, with mean RMSE 0.421 m against the 0.425 m limit. The margin is narrow, and the 38.0–39.4 ms solver-plus-overhead figure is still an estimate, so please proceed with the rebuild and treat the board measurement as the decision gate—not as a foregone real-time pass.

Please follow the handoff checklist:

1. Change `_n_S` to 3 and `_maxiter` to 24. Keep fix6 unchanged as the known-good rollback.
2. Audit all dependent HLS stream/FIFO depths, compile-time assumptions, and memory layout before building.
3. Remove or parameterize the hard-coded 100-iteration assumptions in `nmpc_overlay.py`, the notebook, and any validation utilities. Verify the host-side buffer layout still matches the RTL.
4. Run all three worker co-simulations, then rebuild the HLS IPs and integrated design. Record synthesis/timing/resource reports and exact artifact hashes.
5. Stage the result as a new overlay; do not replace or promote fix6 yet. Validate the new overlay on the board, confirm all constraints and the overlay guard, and measure actual solver and end-to-end `solve()` latency.
6. Compare repeated board runs against the 3×24 PC distribution. Report the RMSE, worst error, peak tilt, constraint counts, solver latency, and PS overhead separately.
7. Keep physical-actuator tests gated. Do not commit or push the rebuild automatically; report the diff and validation evidence for review first.

Before editing, reconcile the handoff's listed uncommitted files with the current repository state. Preserve unrelated user changes, dated notebook outputs, and IP backups; do not stage them as part of this work.

## Parallel, separate task: design an MPC model for iRobot Create 3

Please investigate and propose a Create 3-specific MPC model in a separate worktree/branch or otherwise isolated from the 3×24 rebuild. This is a ground-robot model redesign, not a port of the quadrotor equations or parameters. For this first pass, deliver a design and validation proposal; do not alter the active quadrotor configuration, program the KV260, or make physical robot motion commands.

### Starting point

- The Create 3 is a differential-drive robot for relatively flat indoor environments.
- Its low-level ROS 2 velocity interface is `/cmd_vel` (`geometry_msgs/msg/Twist`): `linear.x` in m/s and `angular.z` in rad/s. Commands continue until replaced or the command timeout expires; higher-priority action goals can block velocity commands.
- The published `/odom` (`nav_msgs/msg/Odometry`) is fused from wheel encoders, IMU, and optical flow, and publishes at 20 Hz. Raw IMU publishes at 100 Hz; wheel status/ticks/velocities publish at 62.5 Hz. The docs state 508.8 ticks per wheel rotation.
- The frame convention is right-handed: x forward, y left, z up. The robot exposes read-only geometry parameters including `wheel_base` and `wheels_radius`; query these on the target rather than guessing constants.
- The onboard `wheel_accel_limit` is in mm/s² (default 900, configurable 1–900). With normal safety settings, max speed is limited to 0.306 m/s; disabling all safety features permits up to 0.460 m/s but is not the recommended MPC operating mode.
- Reflexes can temporarily override commands. Safety documentation warns that disabling the backup limit removes protection against rearward cliff hazards because the cliff sensors face forward. E-stop and hazard/reflex status must remain outside and above the optimizer's authority.

### Model to evaluate

Use a planar differential-drive/unicycle model in the odom frame, initially:

- Pose: `(x, y, yaw)`.
- Augmented motion state: measured/estimated linear and angular velocity `(v, omega)` if command slew, acceleration limits, or actuator response materially affect prediction.
- Kinematics: `x_dot = v cos(yaw)`, `y_dot = v sin(yaw)`, `yaw_dot = omega`.
- Candidate optimizer inputs: bounded velocity commands `(v_cmd, omega_cmd)` matching `/cmd_vel`, with a discrete actuator/slew model between commanded and realized `(v, omega)`. Enforce wheel-space limits by mapping `v, omega` to left/right wheel linear velocities using the queried wheel base; translate the documented wheel acceleration limit into consistent wheel acceleration constraints.

Compare the ideal kinematic model with the augmented/slew-limited model using measured command-to-odometry response. Do not assume a first-order time constant or exact motor dynamics without data. Identify what is knowable from ROS parameters/docs versus what requires a supervised, low-speed system-identification experiment. Account for odometry's 20 Hz rate when choosing the MPC sample time and explain how to refresh commands safely relative to the command timeout.

### Deliverables and acceptance checks

1. A compact state/input definition, discrete-time equations, units, frame conventions, and parameter-source table.
2. A mapping from optimizer controls to `/cmd_vel`, plus wheel-speed/acceleration constraints and a clear list of behavior the model does not capture (slip, payload/surface effects, reflex interventions, localization drift).
3. A minimal offline simulator and tests for straight motion, in-place turns, constant-radius arcs, saturation/slew behavior, and angle wrapping. Validate signs and units against the ROS message and odometry conventions.
4. A system-identification and validation plan using `/odom`, `/wheel_vels` or `/wheel_status`, `/imu`, `/slip_status`, and hazard status where available. Begin with simulation/replay, then supervised low-speed tests with default safety enabled; define stop conditions and an E-stop procedure before any physical experiment.
5. A migration/architecture proposal showing how the new model would fit the NMPC interfaces, state dimensions, cost/constraints, and HLS compute budget. Keep the Create 3 model selectable and separate; do not replace the quadrotor model or couple it to the 3×24 rebuild.
6. Cite the official documentation and record the target Create 3 software/ROS 2 version. Recheck topic rates, parameter values, and message behavior on the actual robot before implementation because these can vary by software release.

Useful official references:

- [Create 3 ROS 2 APIs](https://iroboteducation.github.io/create3_docs/api/ros2/)
- [Moving the robot and `/cmd_vel`](https://iroboteducation.github.io/create3_docs/api/moving-the-robot/)
- [Odometry and sensor rates](https://iroboteducation.github.io/create3_docs/api/odometry/)
- [Safety overrides, speed, acceleration, and E-stop](https://iroboteducation.github.io/create3_docs/api/safety/)
- [Reflexes](https://iroboteducation.github.io/create3_docs/api/reflexes/)
- [Mechanical overview](https://iroboteducation.github.io/create3_docs/hw/mechanical/)
