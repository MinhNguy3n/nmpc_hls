# Unicycle / skid-steer model for the MPC-PSO solver: Husky A300 (TurtleBot 4 deferred)

Status: first-pass design proposal and software reference (2026-10-25), revised after the GPT-6
review (same day). Branch `husky-unicycle-mpc`, based on `3145854` (`origin/build`). Nothing here changes the HLS sources, HDL, generated IP,
overlays, `app/kria-pynq/` or any robot. **Cycle and timing numbers are screening estimates, not
synthesis or board results.**

| File | Purpose |
| --- | --- |
| `docs/husky_a300/unicycle_mpc_pso_model.md` | This note |
| `docs/husky_a300/simulation_and_integration_plan.md` | Docker / Gazebo / ROS 2 / HIL plan and staged tests |
| `app/husky/unicycle_model.py` | Independent double-precision reference model with sourced parameter sets |
| `app/husky/test_unicycle_model.py` | pytest tests (44, all passing; output in §11) |
| `app/husky/hw_estimates.py` | Bit-level emulation of the hardware arithmetic and the cycle-budget model behind §§5.4 and 9 |

**Reuse.** These parts are adapted, with attribution in the code, from the Ackermann proposal
(`docs/ackermann/ackermann_mpc_pso_model.md` and `app/ackermann/*.py` in worktree
`agent-a3984fccbf29de483`):

- the chord step (exact reference) and its division-free hardware approximation;
- the table-index and interpolation accuracy variants;
- the warm-start and feasible-fallback scheme;
- per-dimension normalization;
- host-side constraint checking;
- the HLS ABI audit;
- the cycle model.

The reviews' requirements are applied throughout (`.copilot/chats/2026-10-25-ackermann-mpc-pso-model-review.md`,
`2026-10-25-claude-feedback-ackermann-review.md`):

- accuracy and timing are reported per variant;
- cycle numbers are estimates;
- violations are reported separately from the cost;
- slip and dynamics are a separately validated extension.

**Scope and order (after review).**

1. **Husky A300 in simulation first:** validate the reference model and the controller behaviour (plan §2, §4).
2. **First HLS prototype: the RK4 model swap (variant A)**, with closed-loop quality screened in software.
3. **The interpolated chord (B3) only if** measured timing or tracking justifies it.
4. **The TurtleBot 4 / Create 3 port is a separate, later scope.** Its material is kept below, marked *(deferred: TB4 scope)*.

## 1. Decisions at a glance

| Topic | Proposal | Reason / source |
| --- | --- | --- |
| Reference point | Robot centre, `base_link` | A300 diff_4wd URDF centres the chassis on `base_link`. Create 3: `base_link` is "at the center of rotation" ([Create 3 ROS 2 API]). |
| State, control | `[x, y, psi]`, `[v, omega]` | `v` = `twist.linear.x`, `omega` = `twist.angular.z` of `geometry_msgs/TwistStamped` ([CP Platform API], [CP Driving]) |
| Reference discretization | Exact chord step `x+ = x + v Ts sinc(omega Ts/2) [cos, sin](psi + omega Ts/2)`, `psi+ = psi + omega Ts`, with the exact `sinc(a) = sin(a)/a` | Exact under zero-order hold; no branch; continuous at `omega = 0`; in-place turns exact. **The exact `sinc` is a division.** |
| Hardware approximation (Stage B only) | `sinc(a) ≈ 1 − a²/6`: division-free | Error `0 ≤ sinc − (1 − a²/6) ≤ a⁴/120`: ≤ 5.3e-8 relative, ≤ 5.3e-9 m per step over `abs(omega Ts) <= 0.1` (§3.2) |
| **First HLS prototype: Stage A** (no `System` change) | Swap the model functor, keep RK4 (variant A) | Least HLS risk; the most accurate of the plain variants (§5.4). 9×24, 7×30, 4×50 fit the solver budget (estimate, §9) |
| Stage B (conditional) | Discrete-step hook with the chord step; only B3 (interpolated) is worth prototyping, and only if timing or tracking measured with A justifies it | B3: 7×100, 15×50 fit the solver budget (estimate) |
| Skid-steer limits | Body boxes (controller limits) **plus** wheel-side diamonds `abs(v) + abs(omega) W/2 <= v_side_max`, and per-side slew | The diff_drive_controller limits only the body (`linear.x`, `angular.z`). Per-side limits are motor or Create 3 limits. |
| Effective track | Two numbers are kept apart. **Prediction:** `psidot = omega` needs no track. **Wheel mapping:** the Jazzy default controller uses `W·m = 0.562 × 1.75 = 0.9835 m`. | [A300 control yaml] (`wheel_separation_multiplier: 1.75`). **This is a Jazzy default, not the confirmed target configuration: gate G0 (plan §4.0).** The manual's W = 0.562 m is "a starting point". |
| Slip / ICR model | Deferred: identified separately (§4.4) | Surface- and load-dependent; no data |
| Warm start | Shifted plan, plus host-selected candidates: stop, in-place turn, feed-forward, hold | Feasible by construction (tested) |
| Heading cost | `wrap(psi - psi_ref)` via the `8c61f1b` residual (one conditional ±2π, exact for `abs(x̂ - ref) < 3π`), solving in a local frame | Keeps the contract with margin and keeps `abs(psi)` small for half precision |
| Normalization | ±100 per dimension (existing ABI). `du_max` is derived per dimension. | Existing `_max_v`, `value_map` and particle code |
| Command ownership | Solve only while the `twist_mux` output is the MPC's own command. `u_prev` = the command actually applied (controller's limited output, else the mux output), never the MPC's last publication (§7.2) | External `cmd_vel` has the lowest mux priority; teleop/RC can override it ([CP twist_mux]) |
| Safety | Host re-checks every plan in double precision and reports each violation separately. Stale odometry or missing safety data → immediate safe stop and idle, never a cached plan. Obstacle monitoring and the E-stop stay **outside** the MPC. The robot's own watchdogs stay in place. | PSO fitness is not a safety signal; the MPC tracks a trajectory and does not avoid obstacles |
| Smoothness | Hard slew limits only; the hardware increment-cost term is deferred (§7.3) | Add it only if a software closed-loop comparison requires it |

## 2. Frames, sign conventions and the ROS interface

- **odom frame:** right-handed, `x`, `y` in m, `z` up.
  - The A300 manual says Clearpath ground robots use a frame "based on ISO 8855" (vehicle axes x forward, y left, z up). Positive translational velocity moves the wheels toward +X ([A300 manual], Orientation References).
  - The Create 3 frame is "right-handed, with x forward, y left, and z up" ([Create 3 ROS 2 API]).
  - Both match REP-103.
- **Heading `psi`:** the angle from `+x_odom` to the body x axis, counter-clockwise positive. Wrapped values lie in `(-pi, pi]`. This is the range of the HLS cost residual (`8c61f1b`) and of ROS `angles::normalize_angle`. The yaw is extracted from the odometry quaternion (`yaw_from_quaternion`).
- **Controls:** `v` (m/s) along body x, `omega` (rad/s) about +z.
  - **Command:** `TwistStamped.twist.linear.x = v` and `twist.angular.z = omega`, `header.stamp` = publish time, `header.frame_id = base_link`.
  - **Sign:** `omega > 0` turns left (CCW). The right side then runs faster (`v_r = v + omega W/2`). Clearpath's driving tutorial: positive `angular.z` "will command the robot to rotate anticlockwise" ([CP Driving]). Tested in `test_in_place_turn_and_yaw_sign`.
- **Topics:**
  - **A300** (namespace `a300_<serial>`, for example `a300_00000`):
    - publish `/<ns>/cmd_vel` (`TwistStamped`). It is the lowest-priority `twist_mux` input: priority 1, timeout 0.5 s. The joystick (priority 10) and RC (12) override it, and the locks `platform/emergency_stop` and `platform/safety_stop` block it ([CP twist_mux], [CP Platform API]);
    - read `/<ns>/platform/odom/filtered` (EKF wheel + IMU) or `/<ns>/platform/odom` (wheel odometry, controller `publish_rate: 50` Hz).
    - observe `/<ns>/platform/cmd_vel` (the `twist_mux` output, `TwistStamped`, [CP Platform API]) for command ownership, and the controller's limited command (`publish_limited_velocity: true` in [A300 control yaml]; topic name to be confirmed in simulation, plan check V10) for `u_prev`.
  - **TurtleBot 4** *(deferred: TB4 scope)*: publish `/cmd_vel` and read `/odom` (20 Hz, [Create 3 odometry]). The message type differs between sources and must be checked on the robot (§13).

## 3. Model and discretization

### 3.1 Continuous model

```text
xdot = v cos(psi),   ydot = v sin(psi),   psidot = omega
```

This is the unicycle (differential-drive) kinematic model. For a skid-steer robot it is the
no-slip idealization: §4.4 treats the effect of slip on the wheel-to-body map. Note that the
**predictor needs no track width**, because it uses the body twist directly. The track enters only
the wheel-side limits (§4) and the odometry inside the robot.

### 3.2 Chord step: exact reference and hardware approximation

**Exact reference** (`step_chord`). With `a = omega Ts / 2` and `s = v Ts`:

```text
x+   = x + s sinc(a) cos(psi + a)
y+   = y + s sinc(a) sin(psi + a)
psi+ = psi + 2a
sinc(a) = sin(a)/a = 1 - a²/6 + a⁴/120 - ...
```

- This is the exact solution under a zero-order hold. It follows from `sin(psi + 2a) - sin psi = 2 sin a cos(psi + a)` (the Ackermann identity with `dpsi = omega Ts`).
- **`omega → 0`:** `sinc(a) → 1` and the step becomes the straight line, with no branch in the step.
  - The exact `sinc(a) = sin(a)/a` **is a division**. In double precision, `sinc` switches to its series below `abs(a) = 1e-4` (truncation error under 2e-28), so it never divides by a tiny number.
- **Hardware approximation** (`sinc_poly2`, `step_chord_poly`; used by variants B1–B3): `sinc(a) ≈ 1 − a²/6`, which is division-free.
  - Error: the Taylor series alternates with decreasing terms for `abs(a) < √20`, so `0 ≤ sinc(a) − (1 − a²/6) ≤ a⁴/120`. The polynomial slightly *under*-estimates the chord length; the heading is unaffected.
  - Intended range: `abs(omega Ts) <= 0.1 rad` (A300 controller limit 2 rad/s at `Ts = 0.05 s`; the Create 3's 1.9 rad/s gives 0.095), so `abs(a) <= 0.05`. The relative chord error is then at most **5.3e-8**, that is at most **5.3e-9 m per step** at `abs(v Ts) <= 0.1 m`. This is far below the half-precision spacing near 1 (4.9e-4), which it stays below for `abs(a) < 0.49`.
  - Tested: `test_hardware_sinc_approximation_is_separate_and_bounded` (the bound over `abs(a) <= 0.49`, and 2,000 random steps at A300 limits against the exact step).
- **`v = 0`:** the step is an exact in-place turn. The position is unchanged bit for bit (`test_in_place_turn_and_yaw_sign`).
- **Literal arc form** `x+ = x + (v/omega)(sin(psi + dpsi) - sin psi)`: kept in the reference only, with the Ackermann near-zero threshold `abs(dpsi) <= 2e-8`. It is unusable in half precision (§5.4, row X).

### 3.3 Comparison with RK4, midpoint and Euler (double)

`f` does not depend on `x` or `y`, and `psi` is linear in time, so:

- RK4 is exact in `psi` and is Simpson's rule in position, with local error `<= abs(s) dpsi⁴/2880`;
- midpoint errs by `abs(s)(1 - sinc(dpsi/2)) <= abs(s) dpsi²/24`;
- Euler errs by about `abs(s) abs(dpsi)/2`.

These bounds are asserted on 3,000 random cases with `abs(v), abs(omega) <= 2`, `Ts ∈ {0.02, 0.05, 0.1}`
(`test_chord_vs_rk4_and_other_integrators_within_stated_tolerance`).

**Stated tolerance, RK4 vs chord:** at most 3.5e-9 m per step at `Ts = 0.05 s`, and at most
1.2e-7 m at `Ts = 0.1 s`. Over 25 steps the bound is `25 × 1e-7 m`
(`test_horizon_rk4_vs_chord_over_nh_25`).

Worst position error against the exact step, from `hw_estimates.py` (300 constant-input trials,
`Ts = 0.05`, `Nh = 25`):

| Method (double) | A300 limits `abs(v), abs(omega) <= 2`: 1 step | 25 steps | Create 3 `v <= 0.306`, `abs(omega) <= 1.9`: 1 step | 25 steps |
| --- | --- | --- | --- | --- |
| Euler | 4.8e-3 m | 9.3e-2 m | 7.0e-4 m | 1.4e-2 m |
| Midpoint | 3.8e-5 m | 7.4e-4 m | 5.3e-6 m | 1.1e-4 m |
| RK4 | 3.0e-9 m | 5.7e-8 m | 3.7e-10 m | 7.3e-9 m |
| Chord with the hardware polynomial `1 − a²/6` (`step_chord_poly`) | 4.4e-9 m | 8.6e-8 m | 5.5e-10 m | 1.1e-8 m |
| Chord / exact (reference) | 0 | 0 | 0 | 0 |

## 4. Skid-steer specifics

### 4.1 Wheel-side mapping

From the A300 manual ("Robot Equations", "as a starting point"):

```text
v = (v_r + v_l)/2,   omega = (v_r - v_l)/W   ⇔   v_l = v - omega W/2,   v_r = v + omega W/2
```

The manual gives W = 0.562 m. The same formulas hold for the Create 3, with its wheel base
(`body_to_sides` / `sides_to_body`; round trip tested).

**Which W.** The Jazzy default controller configuration for the A300 (clearpath_common, not yet
confirmed on the target robot) sets
`wheel_separation: 0.562` **and** `wheel_separation_multiplier: 1.75` ([A300 control yaml]). The
`diff_drive_controller` applies the multiplier both when it converts a commanded `omega` into wheel
speeds and in its odometry. So for a commanded `omega` the motors receive
`v ± omega × 0.9835/2`, which is 1.75× the side-speed difference that W = 0.562 m implies. The
multiplier is how Clearpath compensates the skid (§4.4).

**Consequence for the MPC:**

1. Use `track = 0.562 × multiplier` when the side limits are meant as commanded motor speeds. `limits_from_parameter_set("husky_a300")` uses the Jazzy default 1.75 only as a labelled default; pass `track=` once the robot's value is known.
2. **Gate G0 (plan §4.0), before any physical test:** read the target robot's actual `robot.yaml`, generated controller parameters and `ros2 param dump` of `platform_velocity_controller` and `twist_mux`. Until then 1.75 is a default, not a fact about the target.
3. The predictor uses the twist and is unaffected, provided the robot really yaws at the commanded `omega`. That is exactly what §4.4 must identify.

### 4.2 Side speed and slew limits

- **Side speed:** `max(abs(v_l), abs(v_r)) <= v_side_max`, that is `abs(v) + abs(omega) W/2 <= v_side_max`, a diamond in `(v, omega)`. It is enforced only when `v_side_max` is known.
  - **A300:** not documented, so PLACEHOLDER_ (identify). Only a lower bound follows from the docs: the 2 m/s top speed needs at least 2 m/s per side.
  - **A300 gate:** with the side limit unknown, the physical operating envelope must be chosen so that `abs(v) + abs(omega) W·m/2` stays well inside the documented 2 m/s top speed, and agreed before stage 5 (plan §4.0). It stays a PLACEHOLDER_ until identified or stated by Clearpath.
  - *(deferred: TB4 scope)* **Create 3:** the 0.306 m/s limit is documented as the robot's speed limit with safety on ([Create 3 safety]). Whether it bounds each wheel is not stated, so it is PLACEHOLDER_ as a side limit.
  - *(deferred: TB4 scope)* The documented 1.90 rad/s TurtleBot 4 maximum angular velocity ([TB4 features]) corresponds to 0.221 m/s per side with the 0.233 m simulation track. This matches neither 0.306 nor 0.46, which is another reason not to infer a per-wheel limit.
- **Side slew:** `abs(Δv_side) <= a_side Ts`, that is `abs(Δv) + abs(Δomega) W/2 <= a_side Ts`, a diamond in the increments.
  - *(deferred: TB4 scope)* **Create 3:** `wheel_accel_limit` = 900 mm/s² by default, and "wheel velocity commands are ramped using this value" ([Create 3 safety], [Create 3 ROS 2 API]). So `a_side = 0.9 m/s²` is documented. The iRobot simulation config derives the body angular limit `2 × 0.9/0.233 = 7.725 rad/s²` from it ([Create 3 sim control]).
  - **A300:** not documented, so PLACEHOLDER_ (an explicit gate, plan §4.0).
- **Body boxes** (what the controller itself enforces):
  - `abs(v) <= v_max` and `abs(omega) <= omega_max`;
  - acceleration limits that are asymmetric in magnitude: A300 `linear.x` accel 2.0 / decel 4.0 m/s², `angular.z` 4.0 / 4.0 rad/s², velocity 2.0 m/s and 2.0 rad/s ([A300 control yaml]);
  - `preserve_turning_radius: true`, so the controller scales `v` and `omega` together when it limits.
  - Commands that stay inside these limits are passed through unmodified, so the prediction remains valid. This is why the MPC must stay inside them.
- **Slew semantics** (`slew_window`):
  - `abs(v)` may grow by at most `accel·Ts` and shrink by at most `decel·Ts` per sample;
  - a sign change counts the part past zero as growth;
  - the same holds for `omega`.
  - This is conservative with respect to the controller's `max_acceleration` / `max_deceleration` / `*_reverse` parameters.

### 4.3 Saturation that preserves turn direction

- If a raw command violates the side-speed limit, `saturate_sides(..., "scale")` scales `v` and `omega` by one common factor.
  - It keeps the curvature `omega/v`, the sign of each, and therefore the turn direction.
  - It mirrors the controller's `preserve_turning_radius`.
- Clipping each side separately changes the turn radius. It is implemented only as a comparison (`test_saturation_preserves_turn_direction_and_curvature`).
- Inside the MPC, the projection of §7.5 handles saturation.

### 4.4 Effective track and ICR slip: deferred, separately identified

A skid-steer robot must slip laterally to turn. The ICR kinematics of Martínez et al. and Mandow
et al. model this by:

- left/right ICR positions `y_ICR,l` and `y_ICR,r`; or
- an "expansion factor" `chi = W_eff/W`, plus a longitudinal ICR offset `x_ICR`.

These factors depend on surface, load and speed. Clearpath's multiplier of 1.75 is one such
effective-track correction in the controller. It is not a validated model for every surface.

**Proposal:**

- Keep the predictor in body-twist form.
- Treat slip as a separate, identified extension:
  1. In simulation and then on the robot, log commanded twist, wheel joint speeds (`/joint_states`), `platform/odom`, the EKF output and an external ground truth (Gazebo pose, or motion capture / RTK on the real robot).
  2. Fit `omega_actual = k_omega(surface, v, omega) omega_cmd` and `v_actual = k_v v_cmd`, or the ICR parameters `(y_ICR,l, y_ICR,r, x_ICR)`.
  3. Report one-step and `Nh`-step prediction error with and without the fit.
- Only if the fitted factors are stable within a surface class does the model gain them, as constant gains on `v` and `omega`: 2 multiplies, no structural change.
- A dynamic or force model (tyre forces, motor current limits) is out of scope. It would be a separately validated step.

## 5. Hardware arithmetic accuracy (per variant)

### 5.1 Emulated arithmetic

`hw_estimates.py` emulates (not C/RTL simulation):

- `ap_fixed<32,17>` top-level values;
- the half-precision model;
- the 1024-entry table sine reached through `local_sin` (index `round(angle × 163)` in half).

The Ackermann emulation is reused and its step functions are rewritten for `psidot = omega`.

### 5.2 Variants

| Id | Variant | What changes in hardware |
| --- | --- | --- |
| A | RK4 as in `System::one_step_prediction`, cos via `local_sin(x + π/2)` | Model functor only (Stage A) |
| B1 | Chord with `sinc ≈ 1 − a²/6`, half angle, one shared table index (`cos = table[(i+256)&1023]`) | Discrete-step hook (Stage B) |
| B2 | Chord with `sinc ≈ 1 − a²/6`, table position from the `ap_fixed` angle, nearest entry | Stage B + fixed-point index |
| B3 | Chord with `sinc ≈ 1 − a²/6`, `ap_fixed` angle, linear interpolation between entries | Stage B + interpolated table |

All B variants use the division-free polynomial of §3.2; none evaluates `sin(a)/a`. Their error
against the exact reference is dominated by the table and half precision: the polynomial alone
contributes at most 8.6e-8 m over 25 steps (§3.3).
| X | Literal arc `(v/omega)(sin(psi+dpsi) - sin psi)` | Not proposed: shown to quantify the cancellation |

### 5.3 Local frame vs world frame

The host transforms the state and reference so that `x0 = y0 = psi0 = 0` (§7.3). This keeps
`abs(psi)` small, where half precision is finest. The world-frame column is the worst case (random
pose within 20 m, random heading).

### 5.4 Worst position error vs the double exact step (m), `Ts = 0.05`, `Nh = 25`, 300 trials

| Variant | A300 limits, local, 1 / 25 steps | Create 3 limits, local, 1 / 25 | A300, world, 1 / 25 | Create 3, world, 1 / 25 |
| --- | --- | --- | --- | --- |
| A: RK4 hw | 2.1e-4 / **2.9e-3** | 3.7e-5 / 4.8e-4 | 3.2e-4 / 4.9e-3 | 6.0e-5 / 6.7e-4 |
| B1: chord, shared index | 3.0e-4 / **7.7e-3** | 5.4e-5 / 8.2e-4 | 3.8e-4 / 5.9e-3 | 6.8e-5 / 6.8e-4 |
| B1′: chord, cos via +π/2 | 3.0e-4 / 7.1e-3 | 5.4e-5 / 6.8e-4 | 3.9e-4 / 5.4e-3 | 8.0e-5 / 6.9e-4 |
| B2: chord, fixed angle, nearest | 3.0e-4 / **6.0e-3** | 6.0e-5 / 7.7e-4 | 3.1e-4 / 5.9e-3 | 6.0e-5 / 7.0e-4 |
| B3: chord, fixed angle, interpolated | 9.7e-5 / **1.9e-3** | 3.7e-5 / 4.5e-4 | 1.1e-4 / 1.8e-3 | 3.4e-5 / 5.0e-4 |
| X: literal arc (division) | 7.8e-2 / 6.1e-2 | 1.4e-2 / 1.0e-2 | 8.6e-2 / 3.8e-1 | 1.3e-2 / 4.5e-2 |

### 5.5 Reading the table

The GPT-6 review asked that accuracy and timing not be conflated, so the variants are kept apart
here as well.

- **The hardware error floor is the trig table and half precision (a few mrad of heading), not the integrator.**
- **For the unicycle at A300 limits in the local frame, hardware RK4 (A) is more accurate than the plain chord (B1, B2)**: 2.9 mm against 6–7.7 mm after 25 steps. Only the interpolated chord (B3, 1.9 mm) beats it. **So A is the first prototype; B1 and B2 are not worth building, and B3 only if A's measured timing or tracking falls short.**
- *(deferred: TB4 scope)* At Create 3 speeds all variants except X are below 1 mm over the horizon.
- The literal division form (X) must not be used in hardware.
- Two known small biases carry over from the existing code:
  - `Ts` and `Ts/6` in `ap_fixed<32,17>`: a relative bias of −2.4e-4 in predicted distance;
  - `encode_fixed` rounds with `ceil`: a +1 LSB (3.05e-5) bias.
- These are emulation results. They must be confirmed by HLS C-simulation of the eventual model against `unicycle_model.py`.

### 5.6 `local_cos`

`local_cos` must not be used as written:

- **Bug:** with `USE_FAST_SIN_COS`, `local_cos` scales the angle by 512/π and `fastcos` scales it again and truncates (`hls_arith_aux_functions.hpp`, `fast_sin_cos.hpp`; re-verified at `3145854`).
- **Workaround:** the model must use `local_sin(psi + π/2)` (variant A, B1′) or a shared/fixed index (B1–B3).
- **Proper fix:** a separately reviewed change. Make `local_cos` call a `fastcos` that takes the already-scaled index and rounds like `fastsin`, with a unit test over ±4π against `cos`. It is not part of this proposal.

## 6. Parameter sets and sources

All values were fetched on 2026-10-25. The configuration files are the `jazzy`-branch defaults
(clearpath_common 2.9.x, create3_sim); the robot's generated configuration may override them, so
"verify on the robot" applies to every row. The code carries the same table (`PARAMETER_SETS`,
each entry with its source). Unknowns are `None` and labelled PLACEHOLDER_.

### 6.1 Husky A300

| Parameter | Value | Source |
| --- | --- | --- |
| Drive | 4 BLDC motors, skid steer | [A300 manual] ("four wheels, each with its own motor", "1080 W total, from 4 BLDC motors"); [A300 spec] |
| Wheelbase | 0.512 m | [A300 manual] spec table; [A300 URDF] `wheel_base` |
| Track width (spec) | 0.566 m | [A300 manual] spec table |
| Effective track (kinematics) | 0.562 m, "as a starting point" | [A300 manual] Robot Equations; [A300 URDF] `wheel_track`; [A300 control yaml] `wheel_separation` |
| `wheel_separation_multiplier` | 1.75 (so W·m = 0.9835 m): **Jazzy default, unconfirmed on the target (gate G0)** | [A300 control yaml] |
| Effective tyre radius | 0.1625 m | [A300 manual]; [A300 control yaml] `wheel_radius` |
| Max speed | 2.0 m/s | [A300 manual]; [A300 spec] |
| Off-road speed | 1.5 m/s with no payload; 1.0 m/s with ≤ 50 kg | [A300 manual] Important System Limits |
| Teleop (manual) | slow 0.3 m/s, fast 2.0 m/s | [A300 manual] Connecting and Using the Controller |
| Teleop (Jazzy config) | normal 0.4 m/s, 0.75 rad/s; turbo 1.5 m/s, 1.5 rad/s | [A300 teleop yaml]; clearpath_control changelog #206 |
| Controller `linear.x` | max 2.0 m/s, accel 2.0, decel 4.0 m/s² (reverse: −2.0 / 4.0) | [A300 control yaml] |
| Controller `angular.z` | max 2.0 rad/s, accel 4.0, decel 4.0 rad/s² | [A300 control yaml] |
| Controller and odometry rate | 50 Hz | [A300 control yaml] `update_rate`, `publish_rate` |
| `cmd_vel` timeout | 0.5 s (controller and `twist_mux`) | [A300 control yaml]; [CP twist_mux]; [CP Driving] ("stops if it receives no new command for 0.5 seconds") |
| Frame | ISO 8855 based | [A300 manual] Orientation References |
| Command | `/<ns>/cmd_vel`, `geometry_msgs/TwistStamped`; ROS 2 Jazzy | [CP Platform API]; [CP Driving]; [A300 manual] Using ROS |
| E-stop | Front and rear buttons, PLc (ISO 13849). Optional wireless E-stop "asserted automatically if the robot is not able to communicate wirelessly". Electrical braking PLa. | [A300 manual] |
| User power, network | 12 V/10 A, 24 V/5 A, VBAT 5 A (Large VBAT 40 A); network switch for payloads | [A300 manual] |
| Stopping distance (default decel, high friction) | 0.5 m (0 kg, 0°), 0.7 m (100 kg, 0°); initial speed not stated | [A300 manual] |
| Side speed limit `v_side_max` | PLACEHOLDER_ (gate G0/G1) | identify by experiment / ask Clearpath |
| Side acceleration limit | PLACEHOLDER_ (gate G0/G1) | identify by experiment |
| Effective track / ICR on the test surface | PLACEHOLDER_ | identify by experiment (§4.4) |
| Command-to-motion delay | PLACEHOLDER_ | identify by experiment |
| EKF `odom/filtered` rate | PLACEHOLDER_ | query on the robot (`ros2 topic hz`) |

### 6.2 TurtleBot 4 / Create 3 *(deferred: TB4 scope)*

Kept for the later TB4 port; nothing in the A300 first scope depends on it.

| Parameter | Value | Source |
| --- | --- | --- |
| Wheel base | 0.233 m (simulation model); real robot: query | [Create 3 sim control] `wheel_separation`; [Create 3 sim URDF] `distance_between_wheels`; real: `/static_transform` `wheel_base` (read-only, [Create 3 ROS 2 API]) |
| Wheel radius | 0.03575 m (simulation); TB4 manual: wheel diameter 72 mm; real robot: query | [Create 3 sim control]; [TB4 features]; `/static_transform` `wheels_radius` |
| Max linear speed | 0.306 m/s with safety on (`safety_override` none/backup_only); 0.46 m/s with `full` (not for MPC). TB4 manual: 0.31 / 0.46 m/s. | [Create 3 safety]; [TB4 features] |
| Max angular velocity | 1.90 rad/s | [TB4 features] |
| Wheel acceleration | `wheel_accel_limit` 900 mm/s² default (range 1–900); wheel commands ramped at this rate | [Create 3 safety]; [Create 3 ROS 2 API] |
| Body angular acceleration (sim) | 7.725 rad/s² (= 2·0.9/0.233) | [Create 3 sim control] |
| Odometry | `/odom` 20 Hz; wheel topics 62.5 Hz; IMU 100 Hz (G.4/H.1 and later); `slip_status` 20 Hz | [Create 3 odometry] |
| `cmd_vel` timeout | simulation 0.5 s; robot: "a fixed timeout period", value not stated, so PLACEHOLDER_ | [Create 3 sim control]; [Create 3 moving] |
| Command priority | Actions and autonomous behaviours block `cmd_vel`; reflexes "temporarily override any user-provided command" | [Create 3 moving]; [Create 3 reflexes] |
| Safety | Rear cliffs are undetectable (cliff sensors at the front); backup limit active with `safety_override: none`; `e_stop` service turns the wheels off | [Create 3 safety] |
| Payload | 9 kg max | [TB4 features]; [Create 3 mechanical] |
| Side speed limit | PLACEHOLDER_ | identify by experiment |
| Command-to-motion delay | PLACEHOLDER_ | identify by experiment |

## 7. MPC-PSO formulation

### 7.1 Sample time, horizons and decision size

- **Ts = 0.05 s** (20 Hz). It equals the existing `_Ts` and the 50 ms solver period, and matches the Create 3 `/odom` rate. For the A300, the controller and wheel odometry run at 50 Hz, so a fresh state is at most 20 ms old.
- **`Nh = 25`** (1.25 s look-ahead).
  - The A300 stops from 2 m/s in 0.5 s (10 stages) at the 4 m/s² controller deceleration (`test_stop_from_documented_a300_maximum_takes_decel_limited_time`).
  - *(deferred: TB4 scope)* The Create 3 stops from 0.306 m/s in 0.34 s.
- **`Nu ≤ Nh`:**
  - the decision vector is `Nu × 2` (50 at `Nu = 25`, against 100 for the quadrotor);
  - stages from `Nu` on hold `u_{Nu-1}`, as `uu_loop` / `get_next_uu` do (`expand_horizon`, `rollout`);
  - **`Nu = 10`** is the screening candidate: it shrinks the search space 2.5× and the particle memory per particle.

### 7.2 Warm start and known-feasible fallbacks

| Candidate | Definition | Feasibility |
| --- | --- | --- |
| Shifted plan (particle 0) | `[u1 … u_{Nu-1}, u_{Nu-1}]` | Feasible without projection when `u0` was applied (tested) |
| Stop | Nearest-feasible projection toward `(0, 0)`, stage by stage (`stop_candidate`) | Feasible by construction; magnitudes non-increasing (tested). A stop, not a collision-free guarantee. |
| In-place turn | Stage targets `(0, clip(remaining heading error / Ts, ±omega_turn))`, projected (`in_place_turn_candidate`) | Feasible; translation decays to 0. For large heading errors (skid steer and differential drive can turn on the spot). |
| Feed-forward | Exact inverse of the chord step between consecutive reference poses: `omega = wrap(Δpsi)/Ts`, `v = ±abs(Δp)/(Ts sinc(omega Ts/2))` (`feedforward_from_reference`), then projected | Exact for arc-consistent references (tested to 1e-9) |
| Hold | Repeat `u_prev` | Feasible when `u_prev` is |

**Hardware mapping.**

- `initializeParticles_set` copies `last_best` into particle 0, **unprojected** in iteration 0. The hardware "stable zero" particle uses `_uss`; with `uss = (z_v(v = 0), 0)` it is a box-projected stop.
- Until a second candidate buffer exists (an ABI change), the host sends a **feasible** warm start as `last_best`. To keep the critical path short (§9.1), the default warm start is the shifted plan (no projection). A projected candidate (feed-forward at arming, in-place turn when the heading error is large) is on the critical path only when it is needed; the stop candidate is prepared while the board solves.

**Command ownership and `u_prev` (A300).** Feasibility "from `u_prev`" is only meaningful if
`u_prev` is the command the robot is actually executing:

- **`u_prev` = the command actually applied:** the controller's limited command (`diff_drive_controller`, `publish_limited_velocity: true`), or, if that is not observable, the `twist_mux` output `platform/cmd_vel`. **Never the MPC's own last publication.** It is what the controller's slew limiter continues from.
- **Ownership gate:** the MPC solves only while the mux output carries the stamp and twist of one of its own recent commands (`assess_command_ownership`, tested). External `cmd_vel` has priority 1, the lowest: joystick (10), RC (12) and interactive markers (8) override it, and the `e_stop` / `safety_stop` locks block it ([CP twist_mux]).
  - **Foreign command selected** (teleop/RC active): idle, stop publishing, reset the warm start, require an explicit re-arm.
  - **Mux output missing or stale:** safe stop (below), then idle.
  - Whether `twist_mux` passes the stamp through unchanged must be verified in simulation (plan check V10); if not, the ownership test is redesigned before the node leaves stage 2 (simulation).
- **Arming:** only when the applied command is inside the MPC envelope (in practice: robot stopped), so that `project_controls` starts from a feasible `u_prev`.

**Fallbacks, by cause:**

| Cause | Action |
| --- | --- |
| Solve misses its deadline or fails the check (§7.6), with **fresh** odometry, safety data and ownership | Apply the next element of the last accepted plan, at most `k` times (`k` = 3, a PLACEHOLDER_ policy value), re-checked from the current applied `u_prev`; then the stop candidate |
| **Stale odometry, missing E-stop/safety status, or unknown ownership** | **Immediate safe stop, never a cached plan:** publish `(0, 0)` every tick for the controller's stop time (≤ 0.5 s from 2 m/s at 4 m/s², plus margin), then stop publishing and idle until re-armed. The controller's deceleration limit shapes the stop. |
| E-stop / `safety_stop` asserted, or foreign command selected | Idle at once (publish nothing), reset the warm start, require re-arm |

- The robot-side watchdogs (0.5 s on the A300 and in simulation) remain as the last layer.
- **The MPC is a trajectory tracker. It does not avoid obstacles.** Obstacle monitoring (for example a lidar-based monitor with authority through the `platform/safety_stop` lock or the hardware E-stop) and the E-stop chain stay outside the MPC, independent of it (plan §3.1).

### 7.3 Cost and units

Each weight is `1/(acceptable error)²`, so `J` is dimensionless (`mpc_cost`, terms returned separately):

```text
J =  Σ_{k=1}^{Nh-1} [ q_pos ‖p_k − p_ref,k‖²              q_pos    [1/m²]
                    + q_psi e_psi(psi_k, psi_ref,k)² ]     q_psi    [1/rad²]
   + qf_pos ‖p_Nh − p_ref,Nh‖² + qf_psi e_psi(…)²          qf_*     [1/m²], [1/rad²]
   + Σ_{k=0}^{Nh-1} [ q_v (v_k − v_ref,k)²                 q_v      [s²/m²]
                    + r_omega (omega_k − omega_ref,k)²     r_omega  [s²/rad²]
                    + s_dv Δv_k² + s_domega Δomega_k² ]    s_*      [s²/m²], [s²/rad²]   (Δ from u_prev at k = 0)
```

**Heading residual (the `8c61f1b` contract).**

- `e = x̂ − ref`, then one conditional ∓2π into `(−π, π]` (`hls_angle_residual`). It equals the true wrap while `abs(x̂ − ref) < 3π` (tested over 5,000 random cases, and shown to fail outside the contract).
- The hardware RK4 does not wrap the predicted `psi`, so the host must keep the contract.
- **Solve in a local frame** (`to_local_frame`): `x0 = y0 = psi0 = 0`, with references rotated into the robot frame and their headings unwrapped continuously from 0.
  - Then `abs(x̂) <= Nh omega_max Ts = 2.5 rad` (A300 limits).
  - The residual stays far inside `3π`, and `abs(psi)` stays where half precision is finest.
  - `[v, omega]` are frame-invariant, so the plan is applied unchanged (tested: `test_local_frame_keeps_residual_contract`).
- A world-frame solve is also correct while `abs(psi_ref) <= π` and `Nh omega_max Ts < 2π`.

**Mapping to the existing hardware cost.**

- `one_step_error` sums `Q_j e_j²` for the predicted states `x_1 … x_Nh`, with `Qf` at the last one:
  - `_Q = {q_pos, q_pos, q_psi}`, `_Qf = {qf_pos, qf_pos, qf_psi}`;
  - `_controlled_state = {1, 1, 2}`;
  - state limits ±1e3 (no `psi` limit, which would trigger the 1e4 penalty).
- `one_step_u_error` sums `R_j (z_j − uss_j)²` in **normalized** units, so `R_omega,hw = r_omega (omega_max/100)²`.
- **Gaps:**
  - the hardware has no increment term `Δu` and no time-varying `u_ref`;
  - set `R_v = 0`, because a constant-`uss` speed penalty fights tracking.

**Increment cost: deferred.** The first versions use hard slew limits only (`s_dv = s_domega = 0`
in the reference, matching the hardware). A software closed-loop comparison then decides whether an
increment penalty is needed:

- runs: `s_dv = s_domega = 0` against a small sweep of non-zero values, same seeds and references (A300 simulation and the software plant);
- metrics, reported separately: tracking RMSE and max error, `Δv` and `Δomega` statistics (RMS, sign changes per second), time on the slew limits, violations (must stay 0);
- only if the zero-increment runs show unacceptable chatter is a generic increment term added to `hls_system.hpp`, as its own reviewed change.

### 7.4 Normalization and per-dimension `du_max`

The existing ABI is kept: decisions in `[−100, 100]` per dimension, mapped in the model by a
multiply:

```text
v = v_c + v_h z_v/100,  v_c = (v_max + v_min)/2,  v_h = (v_max − v_min)/2;   omega = omega_max z_omega/100
du_max = [100 · min(accel, decel) · Ts / v_h,   100 · min(alpha_acc, alpha_dec) · Ts / omega_max]
```

`calculate_du_min` is symmetric, so each dimension uses its smaller rate (conservative).

| Envelope (limits from §6; envelope values marked PLACEHOLDER_ are not robot properties) | `du_max` (normalized) |
| --- | --- |
| A300 controller limits, `v ∈ [−2, 2]`, `abs(omega) <= 2` | [5.0, 10.0] |
| A300, PLACEHOLDER_ envelope `v ∈ [0, 1]`, `abs(omega) <= 1` | [20.0, 20.0] |
| A300, PLACEHOLDER_ slow envelope `v ∈ [0, 0.3]`, `abs(omega) <= 0.75` | [66.7, 26.7]: the PSO velocity clamp `_max_v = 30` then binds in `v`; retune `_max_v` |
| *(deferred: TB4 scope)* Create 3, `v ∈ [0, 0.306]`, `abs(omega) <= 1.9`, body rates 0.9 m/s² and 7.725 rad/s² | [29.4, 20.3] |
| *(deferred: TB4 scope)* Create 3, side-slew diamond as an inscribed box (half each): `abs(Δv) <= 0.0225 m/s`, `abs(Δomega) <= 0.193 rad/s` | [14.7, 10.2] |

**Why ±100 rather than a new scale.** The PSO update, `_max_v`, the random draws and the projection
are scale-generic. ±100 keeps the overlay, testbenches and screening scripts unchanged. What
matters is that both dimensions use the full range and that `du_max` is derived per dimension,
not copied (the quadrotor has 20 everywhere).

**Alternative parameterization for the Create 3** *(deferred: TB4 scope)*: decide on wheel-side speeds `[v_l, v_r]`.

- **Advantages:** the documented per-wheel ramp and any side-speed limit become exact boxes, and the model is a linear map (2 adds, 1 multiply).
- **Costs:** `abs(omega) <= 1.9` and forward-only `v ≥ 0` become coupled constraints, needing a penalty plus the host check.
- **Recommendation:** evaluate it in the software screen. Keep `[v, omega]` for the first HLS version, because it serves both robots with one model.

### 7.5 Hard limits: projection

- **Software (`project_controls`):** each stage is the exact nearest-point projection (normalized metric) onto a convex polygon: body boxes ∩ slew windows ∩ side-speed diamond ∩ side-slew diamond.
  - The polygon contains `u_prev`, so the projection always succeeds.
  - Fast path: clipping to the axis-aligned boxes is already the answer when it satisfies the diamonds; otherwise all edge projections and vertices are evaluated at once. Measured on this PC: 1.1 ms per 25-stage candidate with box limits, 3.4 ms with both diamonds (§9.1).
  - It is tested on 900 random 25-stage sequences, including NaN and 1e9 entries, for the documented A300 limits, A300 with PLACEHOLDER_ side limits, and TB4 with a PLACEHOLDER_ side-speed limit.
- **Hardware (`updateParticlesWithDuConstrains`):** per-dimension boxes only.
  - Body boxes and symmetric slew are exact.
  - Coupled side limits need an inscribed box (`inscribed_box`), a penalty, or a later projection extension.
  - **The host check is mandatory either way.**

### 7.6 Constraint violations are reported separately

The hardware returns only a fitness (1e4 penalties; `ap_fixed` saturates NaN to ±65535.99997). The
host therefore, in double precision:

1. runs `check_controls(plan, u_prev, limits)`, which returns separately: `speed`, `omega`, `linear_slew`, `angular_slew`, `side_speed`, `side_accel`, `finite`;
2. rolls the plan out with `rollout` for any state constraints (geofence, later obstacles) and the predicted tracking error;
3. applies only a passing plan. Otherwise the fallback of §7.2 applies.

Violations, cost terms, solve times and fallback events are logged as separate columns.

## 8. HLS / ABI impact audit (nothing implemented)

| Area | Item | Change for the unicycle |
| --- | --- | --- |
| `config.hpp` | Model block | New `UNICYCLE_CONFIG`. The build scripts hard-code `-DSNIFFBOT_CONFIG`. |
| | Shape | `_Nx 3`, `_n_U 2`, `_Nh 25`, `_Nu` 25 or 10, `_Ts 0.05` |
| | Controls | `_u_min/_u_max` ±100; `_du_max` per §7.4; `_uss = {z_v(0), 0}` |
| | Cost | `_controlled_state {1, 1, 2}`; state limits ±1e3; `_Q/_Qf/_R` per §7.3 |
| | PSO | `_n_S`, `_maxiter` per §9; retune `_max_v`, `_w0/_wf` in normalized units |
| `hls_model.hpp` | Include | `models/hls_unicycle.hpp` under `UNICYCLE_CONFIG` |
| New `models/hls_unicycle.hpp` | `__model` | `ap_fifo` depths 3/2 (not 12/4). Normalized → physical by multiply. Outputs `[v cos psi, v sin psi, omega]` with `local_sin(psi)` and `local_sin(psi + π/2)` (**not** `local_cos`). |
| `hls_system.hpp` | Stage A | None: `System` is shape-generic over `Nx`, `n_U`, `Nh`, `Nu`, and its FIFO depths follow the template |
| | Stage B, generic changes (each separately reviewed) | (1) a model-provided discrete step hook in place of the four-stage RK4 (returns `(dx, dy, dpsi)`, added in `ap_fixed`); (2) increment cost `(u_k − u_{k−1})²`, **only if** the software comparison of §7.3 requires it; (3) optional `u_ref` buffer; (4) optional violation flag / per-term outputs. The wrapped residual already exists (`8c61f1b`). |
| `hls_pso.cpp` | Sizes | `part_S = Nu × n_U` (50 or 20). Particle memories `n_S × part_S`. Random numbers per update `2 × n_S × part_S`. |
| | Projection | Box per dimension (unchanged); asymmetric `calculate_du_min` optional |
| | IP cores | All five (`initializeParticles_set`, evaluate, detect, update, `pso_fsm`) are regenerated together |
| Block design | BRAM | `x/y/v_mem` 8 KiB = 2048 words each: **`n_S <= 40` at `Nu = 25`, `<= 102` at `Nu = 10`**. `pso_mem` fitness history ≤ 128 iterations (before `find_local` at 0x200). `current_mem` holds `n_U + Nx + Nx Nh + Nu n_U` = 130 words at `Nu = 25` (416 for the quadrotor). The `mem_bank` address constants must be re-laid out. |
| Host ABI | `nmpc_overlay.py` | `NX = 3`, `NU = 2`, `HORIZON = 25`. `last_best` / `new_best` shape must become `(Nu, NU)`: the current code uses `HORIZON` for both, which is valid only while `Nu = Nh`. `MAX_PARTICLES = 8192 // (Nu·NU·4)` (40 at `Nu = 25`); `MAX_ITERATIONS` 128 unchanged; `DESIGN_REVISION` bump. The manifest must carry the model id, `Nx`, `n_U`, `Nh`, `Nu`, `Ts` and the normalization constants. |
| | `encode_fixed` | `ceil`, not round: +1 LSB (3.05e-5 m or rad) bias. Negligible here; a new revision could use `np.round`. |
| | `kv260_nmpc_demo.py` | `memory_regions` byte sizes are hard-coded for 12/4 |
| | `validate_hw_run.py`, `screen_pso_config.py` | `-DSNIFFBOT_CONFIG` and the quadrotor plant are hard-coded. A unicycle SIL builder must be a new script (§ SIL in the plan). |

**These files are not modified here.** `app/kria-pynq/**` and `hw/**` stay untouched until the
user approves an HLS prototype.

## 9. Deadline, cycle estimate and PSO-size candidates (ESTIMATES)

### 9.1 One end-to-end metric and its budget

**Metric: cycle latency `C = t_ready − t_tick`.**

- `t_tick` is the 50 ms timer tick at which the node freezes its inputs: the latest odometry, the observed applied command, the safety status.
- `t_ready` is the moment the checked command for the next period is ready.
- The command is **published at the next tick** (`t_tick + Ts`; the one-period scheme of plan §3.1). So actuation timing is deterministic and `C` is the only quantity that must fit.

**Why not "odometry stamp to publish".** That interval is `odometry age + Ts` by construction:
up to 20 ms + 50 ms on the A300. It is not a compute budget. The odometry age is absorbed by
predicting the state forward from the odometry stamp to `t_tick + Ts` with the applied commands, and
it is bounded separately by the staleness gate (stale → safe stop, §7.2). It is logged as a
monitored quantity.

**Requirement:** `C ≤ 40 ms`. This keeps the existing design rule of 20 % margin on the 50 ms
period: 5 ms publish/timer guard plus 5 ms unallocated. Each component is counted **once**:

| Component of `C` (KV260 backend) | Budget (ms) | Covered by |
| --- | --- | --- |
| Host preparation: forward prediction, local frame, warm start, encode | 4.0 | **Measured on this PC** in Python/numpy: local frame 0.21 ms, shifted plan 0.003 ms, one projected candidate 1.1 ms (box limits) to 3.4 ms (with side diamonds). At most one projected candidate is on the critical path; the stop candidate is prepared while the board solves. rclpy overhead not yet measured. |
| Network request + reply (1 GbE) | 1.0 | **Estimate**; to be replaced by the measured `PING` p99 (plan §3.4) |
| KV260 PS overhead (encode, cache flush and invalidate, start, poll, decode) | 2.35 | **Measured**: max over 500 solves of the 3×24 board run (the 100-solve run: 1.9 ms). Counted here only, **not** inside the solver figure. |
| Solver `ap_start → ap_done` | **≤ 31.65** | **Estimate**: cycle model below, upper value |
| Host decode, `check_controls`, rollout and cost | 1.0 | **Measured on this PC**: 0.11 + 0.53 ms |
| **Total `C`** | **≤ 40.0** | 50 − 5 (guard) − 5 (margin) |

**Reconciliation with the first draft.** The first draft budgeted 38 ms solver + 2.35 ms PS + 2 + 1
+ 2 ms = 45.35 ms against a 45 ms reply target. It also counted the PS overhead both inside its
"solver + overhead" estimates and as a separate line. Here the solver figure excludes the PS
overhead, every other line appears once, and **the solver must shrink to ≤ 31.65 ms** (upper
estimate). The PSO sizes below are therefore smaller than in the first draft.

For the software backends the network and PS lines are replaced by their measured subprocess or
Python solve times; the same `C ≤ 40 ms` applies only where real-time operation is claimed.

### 9.2 Cycle model and calibration

From the quadrotor 3×24 HLS reports and the board, 2026-10-25, at 100 MHz:

- evaluation 147,914 cycles for 3 particles (49,305 per particle; 25 × 1,955 + 430);
- update `391 + 1,050 n_S` at `D = 100`;
- detect 330;
- one iteration 151,785 cycles, so 36.43 ms for 24 iterations, against **38.05 ms median measured** solve time (ap_start → ap_done). The gap is **+4.45 %** (fix6 showed +3.7 %).

The solver model is `iterations × (n_S × eval + update(n_S, D) + detect(D))` cycles, and the
**upper** value adds the 4.45 % gap. No PS overhead is included.

For comparison: the quadrotor 3×24 solver (36.4–38.1 ms) would **not** fit this 31.65 ms solver
budget. Its own validation used solver + PS overhead ≤ 40 ms with no host loop.

### 9.3 Unicycle per-step estimates

None of these models has been synthesized:

| Variant | Cycles / step | Eval / particle (`Nh = 25`) | Basis |
| --- | --- | --- | --- |
| A: RK4, model swap | 435–515 | 10.9k–12.9k (3.8–4.5× less than quadrotor) | The Ackermann RK4 estimate. Unicycle `psidot = omega` removes the `v·kappa` multiply, but it is off the critical path, so no saving is claimed. |
| B1: chord (`1 − a²/6`), half angle, shared index | 97–112 | 2.5k–2.9k | Ackermann chord 105–120, minus two 4-cycle half multiplies on the critical path (`v·kappa`, `·Ts`): `a = omega·(Ts/2)` is one multiply |
| B2: chord (`1 − a²/6`), fixed angle, nearest | 97–117 | 2.5k–3.0k | An `ap_fixed` multiply and bit slicing replace the half add, multiply and int conversion; the range is widened by +5 for the extra fixed multiply |
| B3: chord (`1 − a²/6`), fixed angle, interpolated | 112–132 | 2.9k–3.4k | B2 + 15 (two extra reads, a subtract, a multiply and an add per output, as in the Ackermann estimate) |

### 9.4 Largest PSO size whose upper solver estimate fits 31.65 ms

High end of each step range; BRAM caps `n_S ≤ 40` at `Nu = 25` and `≤ 102` at `Nu = 10`
(`python3 -B app/husky/hw_estimates.py`):

| Variant | `Nu` | ×24 | ×30 | ×50 | ×100 | ×128 |
| --- | --- | --- | --- | --- | --- | --- |
| Quadrotor (reference) | 25 | 2 | – | – | – | – |
| **A: RK4** | 25 | **9** | **7** | **4** | 2 | – |
| A: RK4 | 10 | 9 | 7 | 4 | 2 | – |
| B1 | 25 | 37 | 29 | 17 | 8 | 6 |
| B2 | 25 | 35 | 28 | 17 | 8 | 6 |
| **B3** | 25 | 32 | 25 | **15** | **7** | 6 |
| B3 | 10 | 35 | 28 | 16 | 8 | 6 |

### 9.5 Candidate configurations

Each candidate is listed with its solver estimate and the resulting `C` (lower–upper):

| Candidate | Solver (ms) | `C` (ms) | 25-step emulated error (A300 limits, local) | Role |
| --- | --- | --- | --- | --- |
| **A: RK4, 4×50, `Nu = 25`** | 23.1–28.3 | 31.5–36.7 | 2.9 mm | **First HLS prototype** (model swap only) |
| A: RK4, 7×30, `Nu = 25` | 24.2–29.6 | 32.5–38.0 | 2.9 mm | Alternative; screen 4×50 vs 7×30 vs 9×24 in software |
| A: RK4, 9×24, `Nu = 25` | 24.8–30.5 | 33.2–38.8 | 2.9 mm | Alternative |
| A: RK4, 4×50, `Nu = 10` | 22.4–27.6 | 30.7–35.9 | 2.9 mm | If the screen favours `Nu = 10` |
| B3: 7×100, `Nu = 25` | 24.0–28.8 | 32.4–37.1 | 1.9 mm | **Only if** A's measured timing or tracking justifies a Stage B prototype |
| B3: 15×50, `Nu = 25` | 25.5–30.6 | 33.9–38.9 | 1.9 mm | Same condition |
| B1 / B2: 8×100, `Nu = 25` | 24.4–28.6 / 24.4–29.7 | 32.8–37.0 / 32.8–38.0 | 7.7 / 6.0 mm | Not proposed: less accurate than A |

**What no longer fits.** 10×100 does not fit for any variant (B1 30.4–35.7 ms solver; B3
34.2–40.9 ms). Variant A at 5×50 (solver up to 35.3 ms) and 9×30 (up to 38.1 ms) do not fit either.

**Before choosing:**

1. screen `Nu`, `n_S`×`maxiter`, weights and `_max_v` in software (closed loop, separate violation counts), within the sizes above;
2. build the variant-A HLS prototype;
3. run C-sim against `unicycle_model.py` and C/RTL co-sim;
4. synthesize and close timing at 100 MHz;
5. measure every line of the §9.1 table on the board and the PC, and re-derive the solver budget from the measurements.

**Risks:**

- the model cores are not synthesized;
- the 3×24 evaluator path had negative estimated HLS slack (closed in Vivado at WNS +0.330 ns);
- the D-scaling of update and detect is assumed;
- memory-port contention is assumed unchanged;
- the host lines are measured on this PC, not inside the ROS node.

## 10. Validity limits

The kinematic model assumes:

- planar rigid-body motion;
- the commanded twist is realized: no longitudinal or lateral slip beyond what the controller's effective track absorbs;
- zero-order hold;
- negligible actuator lag at `Ts = 0.05 s`.

It degrades with the following:

- **Slip and terrain.** Skid steering always slips when turning, and the effective track depends on the surface.
  - Grass, gravel, wet or loose ground and high-friction floors change `omega_actual/omega_cmd` (§4.4).
  - In-place turns on high-friction surfaces need large motor current: the manual notes spikes "several times higher" than the 17 A continuous rating during turns on the spot.
- **Payload.** The manual's off-road limits are 1.5 m/s with no payload, 1.0 m/s with up to 50 kg, and "payloads above 50 kg are not supported" off-road.
  - The allowable payload depends on the battery: 101.5 / 86.5 / 75 kg, and 51.5 / 36.5 / 25 kg for all terrain.
  - The stability analysis assumes accelerations up to 4 m/s².
  - Payload also shifts the centre of mass, and with it the ICR (`x_ICR`).
- **Speed.** The model has no tyre dynamics. Keep to the envelope agreed at gate G0/G1 (plan §4.0); the first physical tests use at most the manual's 0.3 m/s slow mode on the A300.
- **Slopes:** the planar assumption fails. The manual limits operation to grades ≤ 15° and warns about regenerative-current limits on hills.
- *(deferred: TB4 scope)* **Create 3:**
  - reflexes, hazards (bump, cliff, wheel drop), the backup limit, docking, and actions that block `cmd_vel` all change the realized command;
  - `slip_status` flags wheel slip;
  - the odometry is a dead-reckoning estimate.
- **Latency:** odometry stamp to actuation is odometry age + one period (≤ 20 + 50 ms on the A300). It is compensated by predicting the state forward with the applied command (§9.1); the actuator delay itself is a PLACEHOLDER_ to identify.

## 11. Tests (`/usr/bin/python3 -m pytest app/husky -q -p no:cacheprovider`)

| Requirement | Test(s) |
| --- | --- |
| Straight motion; zero motion | `test_straight_motion`, `test_zero_motion_leaves_pose_unchanged` (all 6 methods, including the hardware polynomial) |
| In-place turn (`v = 0`, `omega ≠ 0`) and yaw sign | `test_in_place_turn_and_yaw_sign` |
| Constant-arc radius `v/omega` | `test_constant_arc_radius` (chord and arc 1e-12, RK4 1e-8, midpoint 1e-3 relative over 200 steps; full revolution closes) |
| `omega → 0` continuity and finiteness | `test_omega_to_zero_continuity_and_finiteness` (0, −0, subnormal, 1e-300 … 1e-6, both signs; no jump) |
| Chord vs RK4 within a stated tolerance | `test_chord_vs_rk4_and_other_integrators_within_stated_tolerance`, `test_horizon_rk4_vs_chord_over_nh_25` |
| Exact reference vs the hardware approximation `1 − a²/6` | `test_hardware_sinc_approximation_is_separate_and_bounded` |
| Command ownership; `u_prev` from the applied command | `test_command_ownership_uses_the_applied_command` |
| Wheel-side mapping round trip; saturation preserving turn direction | `test_wheel_side_mapping_round_trip_and_manual_equations`, `test_saturation_preserves_turn_direction_and_curvature` |
| Acceleration and slew bounds never exceeded by generated feasible controls | `test_projected_controls_never_exceed_bounds` (3 limit sets), `test_candidates_are_feasible`, `test_shifted_warm_start_is_feasible`, `test_stop_from_documented_a300_maximum_takes_decel_limited_time`, `test_in_place_turn_reaches_heading` |
| Violations reported separately | `test_check_controls_reports_each_violation_separately` |
| Wrapping across ±π | `test_wrap_across_pi` (also the `8c61f1b` residual contract), `test_step_across_pi_and_cost_invariance`, `test_local_frame_keeps_residual_contract`, `test_yaw_from_quaternion` |
| Sources | `test_parameter_sets_are_sourced_or_placeholder`, `test_unknown_parameters_must_be_supplied` |
| Other | `test_feedforward_inverts_the_chord_step`, `test_normalization_round_trip_and_slew` |

Output on this PC (pytest 6.2.5, Python 3.10.12, numpy 2.2.6):

```text
............................................                             [100%]
44 passed in 6.48s
```

## 12. Open questions

1. **A300 side limits:** the per-side speed and acceleration limits of the motor drivers. Can Clearpath state them, or should they be identified?
2. **A300 controller (gate G0):** the robot's actual `robot.yaml`, controller parameters (`wheel_separation_multiplier`, limits, `publish_limited_velocity` topic) and `twist_mux` configuration. Is 1.75 still there? What does it imply for commanded in-place turns on the intended surface (§4.4 experiment)?
3. **A300 operating envelope (gate G1):** conservative speed and yaw-rate caps for the first tests (≤ 0.3 m/s slow mode?), the surface, the payload and the battery variant.
4. *(deferred: TB4 scope)* **TB4:** the Create 3 firmware version, the real `wheel_base` and `wheels_radius` (`/static_transform`), the robot `cmd_vel` timeout, and the actual `cmd_vel` type on the TB4 Jazzy image (§13).
5. **Reverse:** allow reverse on the A300 (useful for skid steer)? The TB4 stays forward-only (rear cliffs).
6. **Localization source:** `platform/odom/filtered` and its rate; any external ground truth for identification (motion capture, RTK).
7. **KV260 placement:** off-board on the LAN (HIL first), or later on the robot (12 V user power and the network switch)?
8. **HLS path:** decided after review: Stage A (model swap, RK4) first; B3 only if measured timing or tracking justifies it.
9. **Cost increments:** deferred; decided by the software comparison of §7.3.
10. **Obstacle monitor:** which independent monitor (sensor, software, authority path through `platform/safety_stop` or the E-stop) will run beside the MPC for stage 5?

## 13. References, and contradictions found

**Official sources** (fetched 2026-10-25):

- [A300 manual] https://docs.clearpathrobotics.com/docs_robots/outdoor_robots/husky/a300/user_manual_husky/
- [A300 spec] https://clearpathrobotics.com/husky-spec-comparison/
- [A300 control yaml] https://github.com/clearpathrobotics/clearpath_common/blob/jazzy/clearpath_control/config/a300/control/diff_4wd.yaml
- [A300 teleop yaml] https://github.com/clearpathrobotics/clearpath_common/blob/jazzy/clearpath_control/config/a300/teleop_joy.yaml (and `clearpath_control/CHANGELOG.rst`, #206)
- [A300 URDF] https://github.com/clearpathrobotics/clearpath_common/blob/jazzy/clearpath_platform_description/urdf/a300/a300.urdf.xacro
- [CP twist_mux] https://github.com/clearpathrobotics/clearpath_common/blob/jazzy/clearpath_control/config/twist_mux.yaml
- [CP Driving] https://docs.clearpathrobotics.com/docs/ros/tutorials/driving
- [CP Platform API] https://docs.clearpathrobotics.com/docs/ros/api/platform_api
- [TB4 features] https://turtlebot.github.io/turtlebot4-user-manual/overview/features.html
- [Create 3 ROS 2 API] https://iroboteducation.github.io/create3_docs/api/ros2/
- [Create 3 safety] https://iroboteducation.github.io/create3_docs/api/safety/
- [Create 3 odometry] https://iroboteducation.github.io/create3_docs/api/odometry/
- [Create 3 moving] https://iroboteducation.github.io/create3_docs/api/moving-the-robot/
- [Create 3 reflexes] https://iroboteducation.github.io/create3_docs/api/reflexes/
- [Create 3 mechanical] https://iroboteducation.github.io/create3_docs/hw/mechanical/
- [Create 3 I.0.0] https://iroboteducation.github.io/create3_docs/releases/i_0_0/
- [Create 3 sim control] https://github.com/iRobotEducation/create3_sim/blob/jazzy/irobot_create_common/irobot_create_control/config/control.yaml
- [Create 3 sim URDF] https://github.com/iRobotEducation/create3_sim/blob/jazzy/irobot_create_common/irobot_create_description/urdf/create3.urdf.xacro
- TB4 driving tutorial: https://turtlebot.github.io/turtlebot4-user-manual/tutorials/driving.html
- TB4 Jazzy announcement: https://clearpathrobotics.com/blog/2024/10/turtlebot-4-now-supports-ros-2-jazzy/

**Literature** (not re-fetched):

- J. L. Martínez, A. Mandow, J. Morales, S. Pedraza, A. García-Cerezo, "Approximating kinematics for tracked mobile robots", IJRR 24(10), 2005.
- A. Mandow, J. L. Martínez, J. Morales, J. L. Blanco, A. García-Cerezo, J. González, "Experimental kinematics for wheeled skid-steer mobile robots", IROS 2007.
- PSO: Kennedy and Eberhart 1995; Shi and Eberhart 1998.

**Contradictions or refinements relative to the facts given in the task:**

1. **Effective track.** The manual's W = 0.562 m is the controller's `wheel_separation`, but the controller also applies `wheel_separation_multiplier: 1.75`. Commanded `omega` is therefore mapped to wheel speeds with 0.9835 m.
2. **"Max acceleration and angular velocity not stated."** True for the manual (which assumes "accelerations/decelerations up to 4 m/s²" in its stability example). The Jazzy controller configuration does state them: `linear.x` accel 2.0 / decel 4.0 m/s², `angular.z` 2.0 rad/s, 4.0 / 4.0 rad/s².
3. **Teleop modes.**
   - The manual says slow 0.3 m/s and fast 2.0 m/s.
   - The Jazzy `teleop_joy.yaml` says normal 0.4 m/s and 0.75 rad/s, turbo 1.5 m/s and 1.5 rad/s (changelog #206: "Increase A300 teleop normal speed to 0.4, reduce turbo speed to 1.5").
   - Both exist; the robot's configuration decides.
4. **Payload, internal to the manual.** "Payloads above 50 kg are not supported" (off-road, System Limits) against the footnote "maximum speed should be limited to 1 m/s in off-road conditions when the payload exceeds 50 kg".
5. **Grade.** Spec "Max. Climb Grade 30°" against the manual's operating conditions "No grades over 15°".
6. **User power.** The spec-comparison page lists A300 user power as "8 Breakouts" and shows the 12 V/10 A, 24 V/5 A, VBAT figures in the A200 column. The A300 manual gives 12 V/10 A, 24 V/5 A, VBAT 5 A, Large VBAT 40 A.
7. **Create 3 `cmd_vel` type.**
   - The Create 3 ROS 2 API page documents `/cmd_vel` as `geometry_msgs/msg/Twist`.
   - The I.0.0 firmware (required for Jazzy, per the TB4 Jazzy announcement) adds `TwistStamped` on **`cmd_vel_stamped`** and says not to use both.
   - The TB4 Jazzy driving tutorial publishes `TwistStamped` on `/cmd_vel`.
   - So the type, and whether something remaps or relays, must be checked with `ros2 topic info -v /cmd_vel` on the robot and in simulation.
8. **Clearpath simulator support for the A300.**
   - The simulator docs do not list platforms ("supports the Clearpath Config").
   - Evidence of support: `clearpath_config` (jazzy) has `sample/a300/` (9 files); the serial page lists `a300-00000`; the A300 URDF switches to `gz_ros2_control/GazeboSimSystem` when `is_sim`.
   - The URDF's default `gazebo_controllers` path (`config/a300/control.yaml`) does not exist on the jazzy branch (the configs moved to `control/diff_4wd.yaml` etc.). This is harmless if the generator passes the path; the plan's verification step checks it.
9. **PS overhead and board timing.** 1.9 ms is the max of a 100-solve run. The 500-solve 3×24 run had max 2.35 ms (median 1.30 ms). The 3×24 solver-plus-overhead median is 39.34 ms before and 39.40 ms after the angle-wrap fix.
