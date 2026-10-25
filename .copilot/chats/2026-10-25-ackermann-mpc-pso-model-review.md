# Claude Feedback: Ackermann Kinematic Model for MPC-PSO

## Change of direction

Please use this note instead of the Create 3 differential-drive proposal in `2026-10-25-claude-feedback-3x24-and-create3-model.md`. The new task is to investigate an Ackermann-steered, car-like vehicle model for the MPC-PSO design. Keep it isolated from the approved 3×24 KV260 rebuild: do not change the active quadrotor model/configuration, generated IP, or board overlay as part of this investigation.

For this first pass, produce a model/design proposal and a small software validation plan. Do not assume the Create 3 itself has Ackermann steering; this is a separate car-like vehicle target unless the user later specifies otherwise.

## Suggested baseline model

Use the rear-axle midpoint as the vehicle reference point and a planar, no-lateral-slip bicycle model. Define:

- Pose: `x`, `y` in metres and `psi` in radians, in a documented world frame.
- Longitudinal speed: `v` in m/s.
- Virtual front-wheel steering angle: `delta` in radians.
- Wheelbase: `L` in metres, measured from rear axle to front axle.
- Track widths: `W_f` and `W_r` in metres if modeling individual wheel geometry.

For a kinematic model with speed and steering held constant during one sample:

```text
xdot   = v cos(psi)
ydot   = v sin(psi)
psidot = (v / L) tan(delta)
```

The model is nonholonomic: the rear-axle reference point has no lateral velocity in the vehicle frame. Its path curvature and turn radius are:

```text
kappa = tan(delta) / L
R     = 1 / kappa
```

For an HLS-friendly formulation, consider making curvature the optimizer's steering-like variable instead of `delta`:

```text
u = [v, kappa]
psidot = v kappa
delta_command = atan(L kappa)
```

This removes `tan()` from every prediction step; the inverse mapping can be done once at the command boundary if the target interface accepts steering angle. Compare its numerical behavior and total implementation cost against optimizing `delta` directly. Do not use curvature if the actual vehicle interface or constraints make steering angle the more faithful decision variable.

With `v` and `kappa` constant over `Ts`, an exact planar arc update is available:

```text
dpsi = v kappa Ts
x+   = x + (sin(psi + dpsi) - sin(psi)) / kappa
y+   = y + (cos(psi) - cos(psi + dpsi)) / kappa
psi+ = wrap(psi + dpsi)
```

Use the straight-line limit when `kappa` is near zero:

```text
x+ = x + v Ts cos(psi)
y+ = y + v Ts sin(psi)
```

This branch avoids division by a near-zero curvature. Alternatively, use midpoint integration or RK4 and compare its error and HLS cost against the exact-arc update. State the singular/near-zero threshold and test continuity across it.

## Ackermann wheel geometry and constraints

The virtual bicycle steering angle is not the same as the two physical front-wheel angles. For a left turn with signed centerline radius `R > 0`, ideal Ackermann steering gives:

```text
tan(delta_inner) = L / (R - W_f / 2)
tan(delta_outer) = L / (R + W_f / 2)
```

Swap inner/outer wheels for a right turn. Derive/check the signed form for negative curvature and document the convention. Do not infer `L`, track width, steering travel, or speed/acceleration limits from the existing quadrotor configuration; obtain them from the target vehicle's specs or measurements.

At minimum, enforce or explicitly account for:

- Speed bounds and reverse policy.
- Steering angle bounds, equivalently `|kappa| <= tan(delta_max) / L`.
- Longitudinal acceleration and steering-rate limits (or curvature-rate limits).
- Optional lateral-acceleration limit `v^2 |kappa| <= a_y,max`, if a defensible limit is available.
- Any vehicle-specific steering geometry, actuator rate, brake, and command-watchdog constraints.

The bicycle model assumes negligible lateral tire slip and is generally the right starting point for low-speed maneuvering. If operation includes significant speed, aggressive steering, low-friction surfaces, or observed sideslip, identify where the kinematic assumption fails and propose a dynamic bicycle model only as a separately validated extension.

## MPC-PSO formulation to evaluate

1. Start with state `[x, y, psi]` and per-stage controls `[v, kappa]` (or `[v, delta]`). Apply actuator bounds and per-step slew bounds. This keeps the initial prediction model small and directly captures feasible command sequences.
2. If measurements show actuator lag is material, augment the state with actual speed and steering/curvature, and use acceleration plus steering-rate as the optimizer inputs. Do not invent a first-order lag constant; estimate it from the target vehicle or retain bounded-rate dynamics until data supports a richer model.
3. Define the control horizon explicitly. Allow `Nu < Nh` with a documented held-last-input tail if that matches the current optimizer. Use a shifted previous solution as the warm start and retain at least one known-feasible candidate. A zero-control candidate alone is not a useful fallback when the reference requires motion.
4. Use a cost such as weighted position error, wrapped heading error, optional speed/path-progress error, control effort, control increments, and terminal pose error. Use `wrap(psi - psi_ref)` for angular residuals. Keep weights and units explicit.
5. Normalize each PSO decision dimension to comparable ranges before applying PSO velocity/inertia updates; physical speed, curvature/steering, acceleration, and steering-rate ranges have different scales.
6. Prefer bounded parameterization or projection for hard control limits. If state constraints require penalties, report actual constraint violations separately from objective values and avoid treating a large penalty score as the only safety check.
7. Compare the full decision-vector size and per-particle cost with the current HLS architecture. The vehicle model changes state dimension, control dimension, math operators, buffers, and host/RTL ABI; audit these together rather than editing `_Nx`/`_n_U` alone. The current generic NMPC implementation has a fixed compile-time state/control shape and a four-stage RK integrator in [hls_system.hpp](../../hw/hls/source/include/hls_system.hpp); document any required generalization and estimate trigonometric/division cost before proposing an HLS implementation.

## Validation proposal

Build an independent software reference model first and test:

- Zero steering gives straight-line motion; zero speed leaves pose unchanged.
- Positive and negative steering produce the expected yaw sign under the chosen frame convention.
- Constant speed/steering traces the predicted circular arc.
- Left/right Ackermann inner and outer wheel angles are ordered correctly and converge to the bicycle angle as track width tends to zero.
- The exact-arc and integrated versions agree within a stated tolerance over the intended `Ts`, speed, and steering ranges.
- Near-zero curvature is continuous and finite.
- Heading error/wrapping works across `-pi`/`+pi`.
- Speed, steering, acceleration, rate, and optional lateral-acceleration bounds are never exceeded by generated feasible controls.
- Any measured-data validation uses time-aligned pose, speed, steering, and reference data; report prediction error over one step and over the full MPC horizon.

The proposal should include a parameter-source table (manufacturer/specification, vehicle API, or identified value), model-validity assumptions, and a staged plan: software tests/replay first, then supervised low-speed tests only after a vehicle-specific safety and emergency-stop plan exists.

## Reference notes

- [MathWorks `ackermannKinematics` reference](https://se.mathworks.com/help/robotics/ref/ackermannkinematics.html): the requested reference. It returned HTTP 403 from this environment, so I could not inspect its page contents directly. Please verify the exact API conventions and wheel-geometry definitions from the page or an accessible local copy before relying on them.
- [MPL Lab: Vision for Ackermann Vehicles](https://mpl.sist.shanghaitech.edu.cn/resVisionForAckermannVehicles.html): the page describes Ackermann motion as a nonholonomic ground-vehicle model and discusses constant-velocity motion along a circular arc in its visual-motion-estimation context. It is useful motivation for the arc assumption, but it is not by itself an MPC design or a full vehicle-dynamics specification.

Please return the proposed equations, coordinate/sign conventions, discrete implementation choice, constraints and parameter sources, model-validity limits, tests, and estimated HLS/PSO impact before implementing or integrating this model.
