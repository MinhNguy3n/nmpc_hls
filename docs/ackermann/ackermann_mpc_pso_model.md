# Ackermann-steered vehicle model for the MPC-PSO solver: first-pass design proposal

Status: proposal and software reference only (2026-10-25). Specification:
`.copilot/chats/2026-10-25-ackermann-mpc-pso-model-review.md`. This note replaces the Create 3
differential-drive proposal.

Nothing here changes the active quadrotor configuration, the HLS sources, the HDL, the generated
IP, the overlay or the board. The target vehicle is unspecified. **Every number below that looks
like a vehicle property is a labelled placeholder.** Real values go in the parameter-source table
(§13).

Deliverables in this change (all new files):

| File | Purpose |
| --- | --- |
| `docs/ackermann/ackermann_mpc_pso_model.md` | This note |
| `app/ackermann/ackermann_model.py` | Independent double-precision reference model |
| `app/ackermann/test_ackermann_model.py` | pytest tests covering the review's validation proposal |
| `app/ackermann/hw_estimates.py` | Bit-level emulation of the hardware arithmetic, and the cycle-budget model behind §§5.5 and 11 |

## 1. Decisions at a glance

| Topic | Proposal | Main reason |
| --- | --- | --- |
| Reference point | Rear-axle midpoint | No lateral velocity there (nonholonomic), no sideslip angle term |
| State | `[x, y, psi]` | Smallest model that captures pose |
| Decision variable | `[v, kappa]` per stage | `psidot = v*kappa` is bilinear. The model needs no `L` and no `tan`. The steering and steering-rate limits map to constant boxes in `kappa` (§7.2). `delta = atan(L*kappa)` is applied once, at the command boundary. |
| Discretization (reference) | Exact arc under zero-order hold | Exact for the model |
| Discretization (hardware) | Exact arc in **chord form** `s*sinc(dpsi/2)*[cos, sin](psi + dpsi/2)` | One sin/cos pair and no division or branch. About 4× cheaper than RK4 at the same hardware accuracy. The review's division form is unusable in half precision (§5.5). |
| Near-zero branch (literal arc form) | Straight line for `abs(dpsi) <= 2e-8 rad`, with `dpsi = v*kappa*Ts` | Balances truncation against cancellation. The jump is at most 1.1e-8·s (measured). |
| Warm start | Shifted previous plan (feasible by construction), plus a reference feed-forward candidate and a brake candidate (§9.2) | A zero-control candidate is useless when motion is required |
| PSO scaling | Normalize each dimension to ±100, as the quadrotor ABI already does | One `_max_v` and comparable PSO steps in every dimension |
| Hard limits | Projection: speed, curvature/steering, acceleration and curvature/steering-rate by construction. Lateral acceleration by the coupled projection of §7.4 (software) or a conservative static bound (first HLS version). | Feasibility does not rest on a penalty |
| Safety check | The host re-simulates the returned plan in double and checks each constraint separately before applying it | The PSO fitness is not a safety signal |
| HLS path | Stage A: swap the model functor and keep RK4 (11–13k cycles per particle). Stage B: a discrete-step hook in `System` with the chord step (2.7–3.4k cycles per particle). | Stage B fits 10 particles × 100 iterations within 40 ms, against 3×24 for the quadrotor (§11) |

## 2. Frames and sign conventions

- **World frame W:** right-handed and planar, `x`, `y` in metres, `z` up. Heading `psi` (rad) is the angle from `+x_W` to the body forward axis, **counter-clockwise positive**. It is stored wrapped to `[-pi, pi)`.
- **Body frame B:** origin at the **rear-axle midpoint**, `x_B` forward, `y_B` left.
- **Speed `v` (m/s):** the signed speed of the rear-axle midpoint along `x_B`. `v < 0` is reverse.
- **Curvature `kappa` (1/m)** and **virtual (bicycle) steering angle `delta` (rad):** positive means **steering left**. The instantaneous centre of rotation (ICR) is then at `(0, R)` in B, with `R = 1/kappa > 0`. `kappa = tan(delta)/L` keeps the sign of `delta`.
- **Consequence:** `psidot = v*kappa`. Forward with left steering turns CCW. **Reverse with left steering turns clockwise**, along the same circle (test `test_yaw_sign_follows_steering_sign`).
- **Wheel angles:** each physical front-wheel angle is signed like `delta` (left = positive).
- **Heading residuals** are always `wrap(psi - psi_ref)` in `[-pi, pi)`.
- **Vehicle interface:** if the localization source or the vehicle API uses another reference point (for example the CG or the front axle) or another sign, the host converts at the boundary. The pose offset is `p_rear = p_other - d*[cos psi, sin psi]`, with `d` the measured offset along `x_B`.

MathWorks names heading `theta` and steering angle `psi`, which clashes with this note (§16). This note follows the review: `psi` is heading, `delta` is steering.

## 3. Kinematic bicycle model about the rear-axle midpoint

With `L` the wheelbase (rear axle to front axle):

```text
xdot   = v cos(psi)
ydot   = v sin(psi)
psidot = (v / L) tan(delta)  =  v kappa,     kappa = tan(delta) / L,   R = 1 / kappa
```

The rear-axle midpoint has no lateral velocity in B (nonholonomic constraint
`-xdot sin psi + ydot cos psi = 0`). This is the standard kinematic bicycle model referenced at
the rear axle [Rajamani §2.2; Lynch & Park ch. 13; LaValle ch. 13 "simple car"]. Kong et al.
reference it at the CG, which adds a sideslip angle `beta = atan(l_r tan(delta) / L)`; the
rear-axle choice removes that term.

The model functions are `continuous_dynamics` and `continuous_dynamics_delta` in
`ackermann_model.py`.

## 4. Decision variable: `[v, kappa]` vs `[v, delta]`

| Aspect | `[v, kappa]` | `[v, delta]` |
| --- | --- | --- |
| Yaw rate in the model | `v*kappa`: 1 multiply | `(v/L)*tan(delta)`: a tan (table sin/cos and 1 division, or `hls::tan`) per model call. That is 4 per RK4 step unless `System` is changed to hoist it per stage. |
| `L` inside the predictor | Not needed: an error in `L` cannot corrupt the prediction | Needed |
| Steering bound | `abs(kappa) <= tan(delta_max)/L`, a constant box (asymmetric travel: two bounds) | `abs(delta) <= delta_max`, a constant box |
| Steering-rate bound | `abs(dkappa) <= delta_rate_max*Ts/L` **implies** `abs(ddelta) <= delta_rate_max*Ts` (§7.2): a constant box, conservative by `1 + tan²(delta)` | A constant box, exact |
| Yaw-rate resolution for PSO | Uniform: equal steps in `kappa` give equal steps in yaw rate | Non-uniform: `tan` stretches near `delta_max` |
| Command | Host computes `delta = atan(L*kappa)` once per period in double, then any servo map | Direct |

**Proposal: `[v, kappa]`.** The `tan` cost argument in the review is weaker than it looks: `delta`
is constant over a step, so `tan(delta)` could be hoisted to once per step. The decisive points are
the others: the predictor is independent of `L`, the yaw rate is linear in the decision variable,
and the steering-rate limit maps to a constant box.

**Choose `[v, delta]` instead when:**

- the steering limit or rate limit is strongly asymmetric, or depends on `delta` in a way that a `kappa` box over-restricts badly (the conservatism is 1.33× at 30°, 2× at 45°); or
- the vehicle's steering interface is a direct angle servo, and `L` is so uncertain that the atan mapping at the command boundary dominates the error.

Either way the predictor stays `psidot = v*kappa`. With `[v, delta]` the conversion
`kappa = tan(delta)/L` happens once per stage, not per RK4 sub-step.

## 5. Discretization (control held constant over `Ts`)

### 5.1 Exact arc, literal form, with the near-zero branch

With `dpsi = v*kappa*Ts` and `s = v*Ts` (signed arc length):

```text
abs(dpsi) >  eps:  x+ = x + (sin(psi + dpsi) - sin psi) / kappa
                   y+ = y + (cos psi - cos(psi + dpsi)) / kappa
abs(dpsi) <= eps:  x+ = x + s cos psi,   y+ = y + s sin psi          (straight-line limit)
                   psi+ = wrap(psi + dpsi)
```

**Threshold:** `eps = DPSI_EPS = 2e-8 rad`, applied to `abs(dpsi)` and not to `abs(kappa)`.
Thresholding `dpsi` also covers `v = 0` with any `kappa`, and it is dimensionless.

**Why this value:**

- The straight branch errs by about `abs(s)*abs(dpsi)/2` (lateral offset of the arc end).
- The division branch loses about `abs(s)*c*eps_mach/abs(dpsi)` to cancellation in `sin(psi + dpsi) - sin psi`.
- The two balance near 2e-8.

**Continuity:** both branches converge to the same limit as `dpsi -> 0`, because
`(sin(psi + dpsi) - sin psi)/kappa = s*(sin(psi + dpsi) - sin psi)/dpsi -> s*cos psi`. So the
discontinuity at the threshold is bounded by the larger of the two errors. A 20,000-sample random
scan gives a worst position error of **1.1e-8·abs(s)** against the chord form below. For other
thresholds: 3e-8 gives 1.5e-8·s and 5e-9 gives 4.6e-8·s. Tests
`test_near_zero_curvature_is_continuous_and_finite` check the following:

- `dpsi` from 1e-3·eps to 1e3·eps, both signs, and exactly at the threshold;
- `kappa` = ±0, ±5e-324 (subnormal) and 1e-300;
- finiteness, `abs(arc - chord) <= 2e-8*s`, and smooth adjacent samples.

### 5.2 Exact arc, chord form (proposed implementation)

`sin(psi + dpsi) - sin psi = 2 sin(dpsi/2) cos(psi + dpsi/2)`, so the same exact update is:

```text
a     = dpsi / 2
chord = s * sinc(a),             sinc(a) = sin(a)/a  (= 1 - a²/6 + a⁴/120 - ...)
x+    = x + chord * cos(psi + a)
y+    = y + chord * sin(psi + a)
psi+  = wrap(psi + dpsi)
```

This form has no division by `kappa`, no cancellation and no curvature branch. In double, `sinc`
switches to its series below `abs(a) = 1e-4`, where the truncation error is under 2e-28.

In hardware, `abs(a)` is bounded by `v_max*kappa_max*Ts/2`, so a fixed polynomial `1 - a²/6`
suffices: 2 multiplies and 1 subtraction. The truncation `a⁴/120` stays below the half-precision
spacing near 1 (4.9e-4) as long as `abs(a) < 0.4`, that is `abs(dpsi) < 0.8 rad` per step. The update is geometrically
a step of length `chord` along the chord direction `psi + dpsi/2`. Functions: `step_exact_arc`
(literal form) and `step_exact_arc_chord`.

### 5.3 RK4 and midpoint alternatives

`f` does not depend on `x` or `y`, and `psi` is linear in time under ZOH. So:

- **RK4** (the scheme in `System::one_step_prediction`) is exact in `psi` and reduces to Simpson's rule for `x` and `y`. Its local position error is at most `abs(s)*dpsi⁴/2880`.
- **Explicit midpoint (RK2)** gives `x+ = x + s*cos(psi + dpsi/2)`: the chord direction with length `s` instead of `s*sinc(dpsi/2)`. Its error is exactly `abs(s)*(1 - sinc(dpsi/2)) <= abs(s)*dpsi²/24`.
- **Euler** errs by about `abs(s)*abs(dpsi)/2`.

These bounds are asserted in `test_exact_arc_vs_rk4_and_midpoint_within_stated_tolerance` over
3,000 random cases with `Ts ∈ {0.02, 0.05, 0.1} s`, `abs(v) <= 2 m/s` and
`abs(kappa) <= 0.546 1/m` (placeholders). The stated tolerances are:

- RK4: ≤ 1e-7 m per step;
- midpoint: ≤ 1e-3 m per step;
- chord vs literal arc: ≤ 1e-12 m.

### 5.4 Error comparison in double (placeholder ranges, `Ts = 0.05 s`, `Nh = 25`)

Worst position error against the exact arc, from `python3 app/ackermann/hw_estimates.py`
(300 random constant-input trials):

| Method | 1 step, case A | 25 steps, case A | 1 step, case B | 25 steps, case B |
| --- | --- | --- | --- | --- |
| Euler | 4.7e-3 m | 1.1e-1 m | 2.6e-2 m | 2.5e-1 m |
| Midpoint | 2.5e-5 m | 5.6e-4 m | 4.6e-4 m | 3.0e-3 m |
| RK4 | 8.7e-10 m | 1.9e-8 m | 1.8e-7 m | 7.9e-7 m |
| Exact arc (literal, chord) | 0 (reference) | 0 | 0 | 0 |

- Case A: `abs(v) <= 3 m/s`, `abs(kappa) <= 0.5 1/m`, `max abs(dpsi)` = 0.075 rad per step, up to 3.75 m travelled.
- Case B: `abs(v) <= 5 m/s`, `abs(kappa) <= 1 1/m`, `max abs(dpsi)` = 0.25 rad per step, up to 6.25 m travelled.

All of these ranges are placeholders. The errors depend only on `dpsi` and `s`, so the table
transfers to any vehicle with similar `v_max*kappa_max*Ts`.

### 5.5 Error in the hardware arithmetic, and the choice for HLS

`hw_estimates.py` emulates the hardware arithmetic bit by bit:

- top-level `ap_fixed<32,17>` values rounded to 2^-15;
- a `half` model, including the half-precision copy of `psi`;
- the 1024-entry table sine reached through `local_sin` (index `round(angle*163)` in half).

| Hardware-arithmetic variant | 1 step, A | 25 steps, A | 1 step, B | 25 steps, B |
| --- | --- | --- | --- | --- |
| RK4 as in `hls_system.hpp`, cos via `local_sin(x+pi/2)` | 4.8e-4 | 7.1e-3 | 9.8e-4 | 9.5e-3 |
| Chord, shared table index | 6.6e-4 | 1.2e-2 | 9.0e-4 | 1.7e-2 |
| Chord, cos via `+pi/2` | 5.8e-4 | 1.2e-2 | 1.2e-3 | 1.6e-2 |
| Chord, table position from the `ap_fixed` angle, nearest entry | 4.5e-4 | 4.7e-3 | 6.7e-4 | 1.2e-2 |
| Chord, `ap_fixed` angle, **linear interpolation** | **1.7e-4** | **2.5e-3** | **2.0e-4** | **3.0e-3** |
| Literal arc (division) form | 3.1e-1 | 1.1 | 7.4e-1 | 7.3e-1 |

Errors are in metres. This emulation was written for this proposal and is not a C/RTL simulation.
Confirm it with a C-sim of the eventual model.

**Conclusions:**

1. **The hardware error is set by the trig table and half precision (about 3 mrad of heading), not by the integrator.** RK4's Simpson accuracy (1e-8 m) is far below that floor. The chord form matches RK4's hardware accuracy within about 2× and costs about 4× less. With a fixed-point table position and linear interpolation it is more accurate than hardware RK4.
2. **The literal division form must not be used in hardware.** The difference of two table sines carries about 6e-3 of quantization noise. Dividing it by a small `kappa` amplifies that noise without bound. No threshold fixes this in half precision: the balance point `sqrt(2*eps_half)` is 0.044 rad, close to the whole operating range of `dpsi`.
3. **Proposal:**
   - HLS Stage A: keep RK4 (no `System` change).
   - Stage B: the chord step, with `s = v*Ts`, `dpsi` and the table position in `ap_fixed`, and the interpolated table.
   - The half model returns increments `(dx, dy, dpsi)`. The addition to the pose happens in `ap_fixed`, because half spacing at 100 m is 0.06 m.
4. **Two smaller arithmetic effects of the existing code:**
   - `Ts = 0.05` becomes 0.0499878 in `ap_fixed<32,17>`, and `Ts/6` becomes 273·2^-15. Both are relative biases of −2.4e-4 in every predicted distance.
   - `psi` must stay small in magnitude before conversion to half. Half spacing is 2e-3 rad for `abs(psi)` in [2, 4) and 4e-3 rad in [4, 8).

## 6. Ackermann wheel geometry (signed)

A front wheel at body position `(L, y_w)`, rotating about the ICR `(0, 1/kappa)`, rolls along
`(1 - kappa*y_w, L*kappa)`, perpendicular to the ray from the ICR. Hence, for **either sign** of
`kappa`:

```text
delta_left  = atan2(L kappa, 1 - kappa W_f/2)        (y_w = +W_f/2)
delta_right = atan2(L kappa, 1 + kappa W_f/2)        (y_w = -W_f/2)
```

- **Left turn (`kappa > 0`):** the left wheel is the inner wheel, and `tan(delta_inner) = L/(R - W_f/2)`, `tan(delta_outer) = L/(R + W_f/2)`. These are exactly the review's formulas.
- **Right turn (`kappa < 0`):** both angles are negative and the right wheel is the inner wheel. `(delta_left, delta_right)(-kappa) = (-delta_right, -delta_left)(kappa)`.
- **Ordering:** `abs(delta_inner) > abs(delta_bicycle) > abs(delta_outer)`.
- **Limit as `W_f -> 0`:** both angles tend to `atan(L*kappa) = delta`. The difference is `O(W_f)`, about `L*kappa²*W_f/2/(1 + L²kappa²)`, and it vanishes exactly at `W_f = 0` and at `kappa = 0`.
- **Validity:** `abs(kappa)*W_f/2 < 1`, that is, the ICR lies outside the front track. Otherwise the inner wheel would need at least 90°, and `wheel_steering_angles` raises.

The formula neglects:

- the steering-pivot (kingpin) offset and scrub radius: `W_f` here is the distance between pivots;
- Ackermann linkage error: real linkages are often only approximately Ackermann, so measure both wheel angles against the commanded angle;
- tire slip.

The rear wheels' no-slip ground speeds are `v*(1 ∓ kappa*W_r/2)`, left and right
(`rear_wheel_speeds`). Use them to check replay data against rear encoders. `W_r` enters nowhere
else.

Tests: `test_inner_outer_ordering_left_and_right` (ordering, review formula, mirror symmetry and
perpendicularity to the ICR ray), `test_wheel_angles_converge_to_bicycle_as_track_shrinks` and
`test_rear_wheel_speeds_match_circle`.

## 7. Constraints

### 7.1 Speed and reverse policy

- **Default:** `v ∈ [0, v_max]`, forward only.
- Reverse (`v_min < 0`) is a separate, explicitly enabled mode, used only if the vehicle and the safety plan allow it. It also needs a policy for gear changes: many vehicles must stop and dwell before reversing.
- Near `v = 0` with reverse enabled, PSO can chatter across zero. Disallow direction changes inside one horizon unless the reference demands it.
- `AckermannLimits` requires `0 ∈ [v_min, v_max]`, so that stopping is always admissible.

### 7.2 Steering ↔ curvature, and their rates

- **Angle:** `abs(delta) <= delta_max` ⇔ `abs(kappa) <= tan(delta_max)/L`. With asymmetric travel, use `kappa_min = tan(delta_min_right)/L` and `kappa_max = tan(delta_max_left)/L`. This relies on the virtual bicycle angle. If the limit is on the **inner physical wheel**, convert it through §6: the inner wheel reaches its stop first, so `delta_max` (bicycle) is less than the inner wheel's stop.
- **Rate:** `d delta/d kappa = L/(1 + L²kappa²) <= L`, so `abs(dkappa/dt) <= delta_rate_max/L` guarantees `abs(d delta/dt) <= delta_rate_max` at every curvature. This bound is exact at `delta = 0` and conservative by `1 + tan²(delta)` elsewhere (`kappa_rate_limit_from_steering_rate`; asserted on `delta` in `test_projected_controls_never_exceed_bounds`).

### 7.3 Acceleration

`-decel_max*Ts <= v_k - v_{k-1} <= accel_max*Ts`, including the step from the command applied in
the previous period. The HLS projection is symmetric (`du_min = -du_max` in `calculate_du_min`),
so the first version uses `min(accel, decel)`. Asymmetric limits need only a different fill of
`local_du_min`, which already exists as a separate array.

### 7.4 Lateral acceleration (optional)

`v²*abs(kappa) <= a_y,max`, used only if a defensible value exists (§12): from the friction
coefficient of the surface and a margin, or from tests. It couples the two decision dimensions, so
per-dimension boxes cannot express it. Two ways to enforce it:

- **Software reference (`project_controls`): exact sequential projection.** Clamp `v` to the box, the slew window and `abs(v) <= sqrt(a_y/max(abs(kappa_prev) - dkappa, 0))`. Then clamp `kappa` to box ∩ slew window ∩ `abs(kappa) <= a_y/v²`. The three intervals pairwise intersect whenever `u_prev` is feasible, so by Helly's theorem in 1-D their intersection is non-empty: the projection always produces a feasible sequence. Tested on 800 random sequences, including NaN and 1e9 garbage, with and without reverse.
- **First HLS version:** choose `v_max²*kappa_max <= a_y,max`, which makes the constraint implied, or else treat it as a penalty and rely on the host check (§9.6). A speed-dependent `kappa` cap in `updateParticlesWithDuConstrains` is possible later. It costs one division or LUT per stage per particle in the update phase, not in the evaluation.

### 7.5 Vehicle-specific constraints and the watchdog

- Steering actuator rate, deadband and backlash.
- Brake behaviour and the maximum (emergency) deceleration.
- Minimum controllable speed (many drives cannot hold very low speeds).
- Command quantization.
- **Command watchdog:**
  - The vehicle side must stop if no fresh command arrives within `T_wd`. The value comes from the vehicle API and is verified by test.
  - On the host side, a solve that misses its deadline, or a returned plan that fails the check in §9.6, triggers the fallback. The host first applies the next element of the previous accepted plan, at most `k` times, then the brake candidate (§9.2).
  - The hardware fitness or penalty is never treated as the safety check.

## 8. Augmented-state option

If replay (§14, stage S3) shows material actuator lag:

- **State:** `[x, y, psi, v_a, kappa_a]`, the actual speed and curvature.
- **Inputs:** `[a, sigma]`, acceleration and curvature rate (or steering rate).
- **Model:** `v_a+ = v_a + a*Ts`, `kappa_a+ = kappa_a + sigma*Ts`, within boxes on `a` and `sigma` and on `v_a` and `kappa_a`.

This is "bounded-rate dynamics". It is also the structure of MathWorks `ackermannKinematics`,
whose state is `[x y theta psi_steer]` with input `[v, psi_steer_dot]`. Over one sample `kappa`
varies linearly, so the arc is a clothoid and the chord form is no longer exact. Use RK4, or a
midpoint-in-curvature chord step, which is second-order. Both are cheap compared with the
quadrotor.

Add a first-order lag (`kappa_a_dot = (kappa_cmd - kappa_a)/tau`) **only** when `tau` (and any
dead time) has been identified from step responses on the vehicle. This proposal does not invent
`tau`.

**The data that justify augmentation:** one-step and `Nh`-step prediction errors in replay that
are dominated by the lag. For example, the heading error grows after a steering step while the
3-state model with measured `v` and `kappa` stays accurate. Another trigger is a commanded-to-actual
delay comparable to `Ts`.

Cost of augmenting (§11): Nx 3→5 adds 2 trivial state equations per step. The decision dimension
stays at 2 per stage. The rate limits become boxes on the inputs, and the box on the actuator
state becomes a state constraint: a penalty plus the host check.

## 9. MPC-PSO formulation

### 9.1 Horizons

- Prediction horizon `Nh` and control horizon `Nu <= Nh`. Stages `k >= Nu` hold `u_{Nu-1}`. This is the existing `uu_loop`/`get_next_uu` behaviour, reproduced by `expand_horizon` and `rollout`.
- The decision vector has `Nu*2` entries, against `Nu*4` for the quadrotor.
- `Nu < Nh` (for example 10 of 25) shrinks the PSO search space by 2.5× at the same prediction length. Recommendation: screen `Nu` in software (§14 S2) before fixing it in hardware.
- `config.hpp` currently has `_Nu = _Nh = 25`.

### 9.2 Warm start and known-feasible candidates

- **Particle 0, shifted previous plan:** `[u_1, …, u_{Nu-1}, u_{Nu-1}]`. If the previous plan was feasible and `u_0` was applied (the new `u_prev`), the shifted plan is feasible without projection (`shift_warm_start`, tested).
  - The hardware evaluates particle 0 **unprojected** in iteration 0: `initializeParticles_set` memcpy's `last_best`. A host-supplied warm start that is inconsistent with `u_curr` could therefore become the returned best.
  - The host must send a feasible warm start, and must check the result (§9.6).
- **Reference feed-forward:** `feedforward_from_reference` inverts the chord step between consecutive reference poses, giving the `[v, kappa]` that moves exactly through them (tested to 1e-9). After projection from `u_prev`, this is the known-feasible candidate that also makes progress along the reference. **This is the useful fallback that zero control is not.**
- **Brake candidate:** ramp `v` to 0 at the deceleration limit with `kappa` held (`brake_candidate`). Lateral acceleration only decreases, so the candidate is feasible. In hardware, particle 1 ("stable zero", `_uss`) with `uss = (v = 0, kappa = 0)` is the du-projected version of this. It is a stop manoeuvre, not a collision-free guarantee.
- **Hold candidate:** repeat `u_prev`.
- **ABI:** carrying the feed-forward candidate into hardware needs a second candidate buffer (§10). Until then, the host can compare the shifted plan and the feed-forward candidate in double precision and send the cheaper one as `last_best`.

### 9.3 Cost (units explicit)

Each weight is `1/(acceptable error)²`, so every term is dimensionless. The stage terms run over
`k = 1 … Nh-1` and the terminal term at `k = Nh`:

```text
J =  Σ_{k=1}^{Nh-1} [ q_pos ‖p_k − p_ref,k‖²          q_pos    [1/m²]
                    + q_psi wrap(psi_k − psi_ref,k)² ]  q_psi    [1/rad²]
   + Σ_{k=0}^{Nh-1} [ q_v (v_k − v_ref,k)²             q_v      [s²/m²]   (speed / path progress)
                    + r_kappa (kappa_k − kappa_ref,k)²  r_kappa  [m²]
                    + s_dv (v_k − v_{k−1})²             s_dv     [s²/m²]
                    + s_dkappa (kappa_k − kappa_{k−1})² ] s_dkappa [m²]     (k = 0 uses u_prev)
   + qf_pos ‖p_Nh − p_ref,Nh‖² + qf_psi wrap(psi_Nh − psi_ref,Nh)²
```

- `v_ref` and `kappa_ref` come from the feed-forward of §9.2.
- This is `mpc_cost`. It returns each term separately.
- A contouring variant (lateral and lag error in the path frame) is a later option.

Differences from the current HLS cost, which matter for §10:

- `one_step_u_error` penalizes `(u − uss)²` with a compile-time `uss`. It has no increments and no time-varying reference control.
- The angle branch (`controlled_state == 2`) is not a general wrap. For `x̂ > π` and `ref < π` it computes `(2π − x̂) − ref`, which is only correct, after squaring, when `ref = 0`. It also ignores negative angles.
- For Stage A without changing `hls_system.hpp`:
  - Solve in a **local frame centred on the current pose**: the host transforms the state and the references so that `x0 = y0 = psi0 = 0`.
  - The host unwraps `psi_ref` continuously from 0.
  - Mark `psi` as `controlled_state = 1` (plain difference). This is correct while `Nh*max abs(dpsi) < π`.
  - The local frame also keeps `abs(psi)` small for the half conversion (§5.5).

### 9.4 Per-dimension normalization for PSO

The PSO velocity clamp `_max_v` (30) is a single scalar applied to every dimension. Physical `v`
(0–2 m/s, placeholder) and `kappa` (±0.55 1/m) would get very different relative steps. The
quadrotor already optimizes **normalized controls in [−100, 100]** and maps them to physical units
inside the model (`value_map`). The proposal keeps that ABI:

```text
v     = v_c + v_h * z_v / 100,     v_c = (v_max + v_min)/2,  v_h = (v_max − v_min)/2
kappa = kappa_max * z_kappa / 100
du_max (normalized) = [100 * min(accel, decel) * Ts / v_h,   100 * kappa_rate_max * Ts / kappa_max]
```

These are `normalize_controls`, `denormalize_controls` and `normalized_slew_limits`.

- **Placeholder example:** `v ∈ [0, 2]`, accel 1 m/s², `delta_max = 0.5`, `L = 1`, `delta_rate = 1 rad/s` and `Ts = 0.05` give `du_max = [5.0, 9.15]`.
- With `_max_v = 30`, the du clamp dominates the PSO step. `_max_v` and the inertia schedule should be retuned in the software screen.
- In the model, write the map as a multiply by a precomputed constant. `value_map` contains a division by a constant range.

### 9.5 Hard limits: projection over penalty

- The existing `updateParticlesWithDuConstrains` projects every dimension of every stage onto the slew window from the previous stage (starting at `u_curr`), then onto the box.
- With normalized `[v, kappa]` that enforces by construction:
  - the speed bounds;
  - the curvature bound, and so the steering bound;
  - the acceleration bound (symmetric);
  - the curvature-rate bound, and so the steering-rate bound.
- No penalty is needed for these.
- Only coupled limits (lateral acceleration, §7.4) and state limits remain for the penalty path.
- The reference implementation is `project_controls`. A small inconsistency in `initializeStableZero`: `x_ant_tmp` stores the pre-clamp value. This is harmless while `uss` lies inside the box.

### 9.6 Constraint violations are reported separately

- The hardware returns only the best fitness. Its 1e4 penalty multiplier, and the `ap_fixed` saturation of NaN/±inf to ±65535.99997, make a large fitness ambiguous.
- So the host, in double precision, runs:
  - `check_controls(new_best, u_prev, …)`, which reports **each** violation amount separately (speed, kappa, accel, decel, kappa rate, lateral acceleration, steering angle, finiteness);
  - a `rollout` of the plan for any state constraints and the predicted tracking error.
- Only a plan that passes is applied. Otherwise the §7.5 fallback applies.
- Violations and objective values are logged separately, like `tracking_metrics` in `quadrotor.py`.

## 10. HLS / ABI impact audit (nothing below is implemented)

Changing the vehicle changes the state dimension, the control dimension, the operators, the
buffers, the BRAM layout and the host ABI together. Editing `_Nx`/`_n_U` alone is not enough.

| Area | Item | Change for Ackermann |
| --- | --- | --- |
| `config.hpp` | Model selection | New `ACKERMANN_CONFIG` block. The build scripts hard-code `-DSNIFFBOT_CONFIG`. |
| | `_Nx`, `_n_U`, `_Nh`, `_Nu`, `_Ts` | 3, 2, `Nh`, `Nu`, `Ts` from the vehicle period |
| | `_u_min/_u_max` | ±100, normalized |
| | `_du_max` | Normalized, per dimension (§9.4) |
| | `_uss` | `(v = 0 → z_v = −100 if v_min = 0, kappa = 0)` for the brake particle |
| | `_controlled_state` | `{1, 1, 1}` with the local frame (§9.3), or 2 for `psi` once a proper wrap exists |
| | State limits | ±1e3 for all states: no `psi` limit, which would trigger the 1e4 penalty |
| | `Q/Qf/R` | §9.3 within the existing structure |
| | `_max_v`, `_w0/_wf` | Retune in normalized units |
| | `_n_S`, `_maxiter` | Per §11 |
| `hls_model.hpp` | Include | `models/hls_ackermann.hpp` under `ACKERMANN_CONFIG` |
| New `models/hls_ackermann.hpp` | `__model` | `ap_fifo` depths 3/2 instead of 12/4. Normalized→physical map by multiply. `local_sin(psi)` and `local_sin(psi + pi/2)`, **not** `local_cos`. 3 outputs. |
| `fast_sin_cos.hpp` | Reuse | `fastsin`, reached through `local_sin`, can be reused (§10.1). Proposed new helper: a shared-index `sincos`, optionally interpolated, fed from the `ap_fixed` angle. |
| `hls_system.hpp` | Stage A | None: the `System` template is shape-generic over `Nx`, `n_U`, `Nh`, `Nu`. The FIFO depths follow the template parameters. |
| | Stage B, needs generalizing | (1) A model-provided **discrete step** hook in place of the hard-wired four-stage RK4 in `one_step_prediction` (for example a `__discrete_step` functor returning `(dx, dy, dpsi)`, added in `ap_fixed`). (2) A proper heading wrap `e − 2π*round(e/2π)` in `one_step_error`, per angle state. (3) An increment cost `(u_k − u_{k−1})²`: `one_step_u_error` needs the previous stage input, which is available in `nmpc_cost_function_topflow` through `uu_fifo`. (4) Optionally, a time-varying reference control, which needs a `uref` buffer. (5) Optionally, per-term outputs or a violation flag. |
| `hls_pso.cpp` | Sizes | `part_S = Nu*n_U` (50 against 100). Particle memories `n_S*Nu*n_U`. Random numbers per update `2*n_S*Nu*n_U`, from the shared `axis_rand_fanout` stream. |
| | Velocity clamp | Per-dimension `_max_v` if normalization is not used |
| | Particles 0/1 | Semantics as in §9.2. Projection of the warm start is optional. |
| | `calculate_du_min` | Asymmetric accel/decel |
| | Extra candidate | A second candidate buffer (an ABI change) |
| | IP cores | All IP cores are compiled per configuration and must be regenerated together: `initializeParticles_set`, `evaluateFitnessAndDetectLocalBest`, `detectGlobalMinimum`, `updateParticlesWithDuConstrains` and `pso_fsm`. |
| HDL / block design | BRAM | `x_mem`, `y_mem`, `v_mem`, `current_mem`, `pso_mem` and `constraints_mem` are 8 KiB (2048 words) each. Particle memories need `n_S*Nu*n_U <= 2048`, so **`n_S <= 40` at `Nu = 25`**. |
| | `mem_bank` address constants | `u_curr_local_addr`, `x_curr_local_addr`, `xref_local_addr`, `last_best`, `bestfitness`, `find_local_addr`, `global_best_local` and `local_*_addr` must be re-laid out. `current_mem` holds `n_U + Nx + Nx*Nh + Nu*n_U` words: 130 against 416. |
| Host ABI | `nmpc_overlay.py` | `NX`, `NU`, `HORIZON`, `MAX_ITERATIONS`, buffer shapes, `DESIGN_REVISION`, manifest. The register offsets are unchanged while the top-level argument list is unchanged. A candidate buffer adds a register. |
| | `kv260_nmpc_demo.py` | `memory_regions` byte sizes are hard-coded (`4*4`, `12*4`, `25*12*4`, `25*4*4`, `10*25*4*4`) |
| | `validate_hw_run.py`, `screen_pso_config.py` | `-DSNIFFBOT_CONFIG` hard-coded; quadrotor plant |
| | `encode_fixed` | Uses `ceil`, not round: a +1 LSB (3e-5) bias. Negligible here, but note it. |
| | New plant | The `quadrotor.py` counterpart is `ackermann_model.py` (`step`, `check_controls` and `mpc_cost` as the tracking and constraint metrics) |

### 10.1 Trigonometric and division operations

The quadrotor counts below are counted from `hls_sniffbot.hpp`.

| Model | Table lookups per horizon step | Run-time divisions per step | `tan` |
| --- | --- | --- | --- |
| Quadrotor, RK4 (current) | 32 (8 `local_sin` per call × 4) | 12 (3 per call: `1/t4`, `1/t26`, `/(2 t16)`), plus constant divisions HLS may fold | – |
| Ackermann `[v, kappa]`, RK4 (Stage A) | 8 (sin, cos per call × 4) | 0 | – |
| Ackermann `[v, delta]`, RK4 | 8 + `tan` × 4 (16 lookups + 4 div) | 4 | 4 (1 if hoisted per stage) |
| Ackermann `[v, kappa]`, chord (Stage B) | 2 (one shared index; 4 reads if interpolated) | 0 | – |
| Ackermann, literal arc | 4 | 1 (`1/kappa`) and a branch | – |

**Can `fastsin`/`fastcos` be reused?** `fastsin` can, through `local_sin`, with these caveats:

1. **`local_cos`/`fastcos` must not be used as written.** With `USE_FAST_SIN_COS`, `local_cos` scales the angle by 512/π and then `fastcos` scales it again and truncates. The quadrotor avoids this by computing `cos(x)` as `local_sin(x + π/2)`, and an Ackermann model must do the same, or read `table[(i + 256) & 1023]` from a shared index.
2. **Accuracy:**
   - The table step is 6.1 mrad, so nearest-entry rounding errs by up to 3.1 mrad.
   - Half-precision rounding of the angle and of the index (`angle*163` in half, scale rounded to 163.0) adds up to about 0.4 index steps (2.5 mrad) for `abs(angle) > π/2`.
   - The emulated maximum sine error is 3.2e-3 for `abs(angle) < π/2` and 5.3e-3 near ±π.
   - Together this is about 0.3 % lateral drift of distance travelled: 5–17 mm over a 25-step horizon in the placeholder cases.
   - A fixed-point index with linear interpolation cuts this to 2.5–3 mm, for about 15 extra cycles per step.
3. **Range:** the int16 index limits `abs(angle)` to below 201 rad. Wrapping, or the local frame, makes this moot.
4. **Resources:** each `fastsin` instance costs about 1 BRAM, 2 DSP and 491 LUT (3×24 report). The current build allocates 2 instances.

## 11. Cycle budget and feasible PSO size

**Calibration (HLS reports of the 3×24 quadrotor build, 2026-10-25, 100 MHz):**

- One particle evaluation is 48,977 cycles: 25 steps × 1,955, plus about 100.
- Per step:
  - `one_step_prediction`: 1,892 cycles;
  - 4 model calls at 440 cycles each: half→fixed conversions 49 + 17 + 48, and the half core 339 (of which the `fastsin` instance is 7);
  - `update_state`: 3 × 15;
  - final RK4 update: 50 (II = 4 × 12);
  - copies: about 63.
- Update: `391 + 1,050*n_S` for `D = Nu*n_U = 100` (fit to 3,541 at 3 particles and 10,891 at 10).
- Detect: 330.

**Model:** `iterations × (n_S × eval + update(n_S, D) + detect(D)) / 100 MHz`, scaled for `D = 50`.
The upper bound adds the 3.7 % that the HLS reports do not explain, and both bounds add the 1.9 ms
of PS overhead. Target: 40 ms (a 50 ms period with 20 % margin). The quadrotor check gives 3×24
→ 38.1–39.4 ms (the screened value).

**Ackermann estimates.** The model cores have not been synthesized; these are estimates from the
measured operator latencies:

| Variant | Cycles per step | Eval per particle | Max particles × iterations within 40 ms (upper estimate) |
| --- | --- | --- | --- |
| Quadrotor RK4 (measured) | 1,955 | 48,977 | 3×24 |
| Ackermann RK4, `Nx = 3`, `n_U = 2` (Stage A) | 435–515 | 10.9k–12.9k (3.8–4.5× less) | 11–13 ×24, 9–10 ×30, 5–6 ×50, 2–3 ×100 |
| Ackermann chord (Stage B) | 105–120 | 2.7k–3.1k (16–18× less) | 42–47 ×24 (BRAM cap 40), 34–38 ×30, 20–22 ×50, **10–11 ×100** |
| Chord with interpolated trig | 120–135 | 3.1k–3.4k | 38–42 ×24, 30–34 ×30, 18–20 ×50, 9–10 ×100 |
| Chord with `Nu = 10 < Nh = 25` (D = 20) | 105–120 | 2.7k–3.1k | 46–52 ×24, 37–42 ×30, 22–25 ×50, 11–12 ×100 |

**Step breakdown behind the estimates:**

- **RK4 model call, 90–110 cycles:**
  - conversions 13 + 9 + 12 (4 cycles per element, as measured);
  - normalized→physical map: 2 elements;
  - `psi + π/2` (hadd), ×163 (hmul), `fastsin` (7), × `v` (hmul), with `v*kappa` in parallel, at the bound latency of 4 cycles per half operation;
  - FIFO writes.
- **RK4 step:** 4 model calls plus `update_state` (3 × about 6) plus the final update (3 × II 4) plus copies (about 25), giving 435–515.
- **Chord step, critical path:**
  - map → `v*kappa` → `*Ts` → `/2` → `+psi` → `*163` → table (7) → `*chord`, about 40–45 cycles;
  - plus conversions, about 25;
  - plus the `ap_fixed` adds and wrap, about 5;
  - plus copies, about 25;
  - giving 105–120.

**Reading of the table:**

- Stage A alone (RK4, model swap only) gives about 4× the PSO evaluations of the quadrotor: for example 10×24 to 9×30 instead of 3×24.
- Stage B fits the original **10 particles × 100 iterations** within the same 40 ms (38.1–39.4 ms).
- The update phase becomes about 15 % of the iteration time in Stage B, so `Nu < Nh` also pays off there.
- Earlier quadrotor screens found iterations worth more than particles at equal evaluation count. Re-screen in software before picking `n_S`×`maxiter`.

**Assumptions and risks:**

- The model cores are not yet synthesized.
- The 3×24 cost-function report flags estimated timing slack of −4.78 ns on the model path. A new model must close timing at 100 MHz.
- Memory-port contention is assumed unchanged.
- PS overhead is taken as 1.9 ms. Smaller buffers may reduce it slightly.
- Particle-memory capacity limits `n_S` to 40 at `D = 50` unless the BRAMs grow.

**Augmented state:** about 1.1–1.2× the 3-state RK4 step (2 more trivial states, Stage A). The
chord step does not apply directly (§8).

## 12. Model validity

The kinematic model assumes:

- planar, rigid-body motion on flat ground;
- **no lateral or longitudinal tire slip** (wheels roll where they point);
- ideal Ackermann geometry with non-steered rear wheels;
- `v` and `kappa` held over `Ts`, with instantaneous actuators, or rate-limited ones in §8;
- pose referenced at the rear-axle midpoint.

**It fails, or degrades, with:**

- **High lateral acceleration:** tire slip angles grow with `a_y`. Polack et al. use the kinematic bicycle model only with `a_y` below 0.5 μg, where μ is the tire–road friction coefficient. This bound needs μ for the actual surface and is a starting rule, not a guarantee. It motivates the optional `a_y,max` of §7.4.
- **Higher speed with aggressive steering:** yaw and sideslip dynamics appear, and the vehicle under- or oversteers relative to `psidot = v*kappa`.
- **Low-friction or loose surfaces** (wet, gravel, grass), and **wheel spin or locking** under strong acceleration or braking.
- **Uneven ground, slopes and bumps:** the planar assumption fails, and load transfer changes the effective steering.
- **Steering compliance, backlash, deadband or a non-Ackermann linkage:** the actual `delta` differs from the command. Identify the command→`delta` map.
- **Actuator lag and dead time comparable to `Ts`:** use §8.
- **Very low speed:** the model is fine for pose, since steering at `v = 0` does not move the vehicle. But a real steering actuator still takes time to reach the commanded angle while stopped, which is again §8.

**Dynamic bicycle model (separate, validated extension only).**

- **States:** `[x, y, psi, v_x, v_y, r]`.
- **Model:** linear or Pacejka tire forces [Rajamani ch. 2–3; Kong et al.].
- **Parameters to identify:** mass, yaw inertia, CG location `l_f/l_r` and cornering stiffnesses, none of which are available.
- **Conditioning:** the model is ill-conditioned at low speed (slip angles divide by `v_x`), so practical schemes blend it with the kinematic model at low speed.
- **Hardware cost:** it would need divisions per RK stage. Adopt it only if replay shows the kinematic model failing in the intended operating region, and validate it separately against data.

## 13. Parameter-source table

No value is filled in. Every number in the tests and examples is a placeholder.

| Parameter | Symbol | Unit | Source |
| --- | --- | --- | --- |
| Wheelbase (rear axle → front axle) | `L` | m | manufacturer spec; verify by measurement |
| Front track (between steering pivots) | `W_f` | m | manufacturer spec / measurement |
| Rear track | `W_r` | m | manufacturer spec / measurement |
| Steering travel, left and right (bicycle-equivalent, or per wheel) | `delta_max,L`, `delta_max,R` | rad | manufacturer spec; verify by measurement (wheel angles at full lock) |
| Minimum turning radius (cross-check of `delta_max`/`L`) | `R_min` | m | manufacturer spec / measurement |
| Steering-rate limit | `delta_rate_max` | rad/s | vehicle API (rate limiter) / identify by measurement |
| Command → wheel-angle map (servo/linkage), deadband, backlash | – | – | identify by measurement |
| Forward speed limit | `v_max` | m/s | vehicle API / manufacturer spec; may be lowered by the safety plan |
| Reverse allowed, reverse speed limit, direction-change dwell | `v_min` | m/s, s | vehicle API / safety plan |
| Acceleration limit | `accel_max` | m/s² | vehicle API / identify by measurement |
| Deceleration limit (normal), emergency deceleration | `decel_max` | m/s² | vehicle API / identify by measurement |
| Lateral-acceleration limit (or friction coefficient μ) | `a_y,max` | m/s² | identify by measurement (surface); safety plan margin |
| Speed and steering actuator lag / dead time | `tau_v`, `tau_delta`, `T_d` | s | identify by measurement (step responses) |
| Minimum controllable speed, command resolution | – | m/s, rad | vehicle API / measurement |
| Command interface: speed vs throttle, angle vs curvature vs PWM; units and signs | – | – | vehicle API |
| Command watchdog timeout and stop behaviour | `T_wd` | s | vehicle API; verify by test |
| Pose reference point offset of the localization source | `d` | m | measurement / localization documentation |
| Pose, speed and steering measurement rates, latencies, timestamps | – | Hz, s | vehicle API / measurement |
| Control period (sets `Ts`) | `Ts` | s | system design (solver time + overhead + actuation latency) |
| Wheel radius, encoder resolution (for replay checks) | `r_w` | m | manufacturer spec / measurement |

## 14. Validation plan (staged)

| Stage | Content | Status |
| --- | --- | --- |
| **S0** | Reference-model unit tests: every bullet of the review's validation proposal (table below) | **Done, in this change** |
| **S1** | HLS C-sim of the new model or step against `ackermann_model.py`: bit-level comparisons in the style of `main_hls_model_unit_test.cpp`. Confirm §5.5 with the real `half` and `hls::` operators. | Next, offline |
| **S2** | Software closed loop: the PSO in C-sim, or a Python PSO with the same update rules, on the reference model. Screen `Nu`, `n_S`×`maxiter`, the weights and `_max_v` for tracking RMSE and **separately reported** violations, as was done for the quadrotor. | Offline |
| **S3** | Replay of logged vehicle data with time-aligned pose, speed, steering and reference. Report **one-step and full-`Nh`** prediction error. Identify the command→`delta` map, the effective `L` and the lags, and decide the §8 augmentation. | Needs logs from the vehicle owner; no commands sent |
| **S4** | HLS synthesis, cosim, and board timing with a **software** plant (HIL like the quadrotor notebook) | Needs a separate build; board use only with the user's approval |
| **S5** | Supervised low-speed tests **only after a vehicle-specific safety and E-stop plan exists and is approved**: wheels off the ground or blocked first, then low speed in a closed area, with the watchdog verified and a person on the E-stop | Not for this proposal; never run by an agent |

**S0 coverage** (`python3 -m pytest app/ackermann -q`, 80 tests):

| Review bullet | Test(s) |
| --- | --- |
| Zero steering → straight line; zero speed → pose unchanged | `test_zero_steering_is_straight_line`, `test_zero_speed_leaves_pose_unchanged` (all 5 methods, including `kappa = 1e6`) |
| Yaw sign for ± steering | `test_yaw_sign_follows_steering_sign` (including reverse and mirror symmetry) |
| Constant inputs → circle `R = 1/kappa` | `test_constant_inputs_trace_circle`. Exact arc and chord to 1e-12 relative; RK4 1e-9; midpoint 1e-4, over 200 steps. A full revolution returns to the start pose. |
| Inner/outer ordering; convergence as `W -> 0` | `test_inner_outer_ordering_left_and_right`, `test_wheel_angles_converge_to_bicycle_as_track_shrinks` |
| Exact arc vs integrated, within a stated tolerance | `test_exact_arc_vs_rk4_and_midpoint_within_stated_tolerance` (the §5.3 bounds), `test_horizon_prediction_error_rk4_vs_exact` |
| Near-zero curvature continuous and finite | `test_near_zero_curvature_is_continuous_and_finite` |
| Wrapping across ±π | `test_wrap_angle_and_heading_error_across_pi`, `test_step_across_pi_and_cost_invariance` |
| Generated feasible controls never exceed bounds | `test_projected_controls_never_exceed_bounds` (speed, kappa, steering angle and rate in `delta`, accel/decel, kappa rate, lateral accel; with reverse; candidates), `test_shifted_warm_start_is_feasible_from_applied_input`, `test_check_controls_reports_each_violation_separately` |
| Measured-data validation (one-step and horizon error) | Deferred to S3: no data yet |

## 15. Open questions for the real vehicle

1. Which vehicle? We need its command interface: speed or throttle, and steering angle, curvature or servo PWM. We also need its units and sign conventions.
2. All §13 values, in particular `L`, the steering travel per side, and whether the limit is on the inner wheel or on a central steering angle.
3. Is reverse needed? If so, what are the direction-change rules?
4. What are the actuator lag and dead time (step-response logs)? This decides §8.
5. Which surfaces and speeds are intended? This decides whether `a_y,max` is needed and where the kinematic model is valid.
6. What is the control period and end-to-end latency (localization → solve → actuation)? This sets `Ts`, and possibly a delay-compensation step: predict the state forward by the latency before solving.
7. What is the localization source's reference point, rate and timestamping?
8. What are the watchdog behaviour and timeout, and the E-stop design?
9. Should the hardware stay a dedicated Ackermann build, or become a model-selectable `System` (Stage B generalization) shared with the quadrotor?

## 16. References and what was verified

- **[MathWorks `ackermannKinematics`]** https://se.mathworks.com/help/robotics/ref/ackermannkinematics.html, with "Mobile Robot Kinematics Equations" (https://se.mathworks.com/help/robotics/ug/mobile-robot-kinematics-equations.html) and the `derivative` reference (https://se.mathworks.com/help/robotics/ref/ackermannkinematics.derivative.html).
  - **Fetched successfully on 2026-10-25, through a summarizing web tool, so not a raw copy.** The reviewer's HTTP 403 did not recur.
  - **What the pages say:**
    - The state is `[x y theta psi]`, with the xy-position and heading "located at the middle of the rear axle". `theta` is the heading and `psi` is the **steering angle**.
    - The commands are `[v psiDot]`, so the steering **rate** is the input.
    - The equations are `xdot = v cos theta`, `ydot = v sin theta`, `thetadot = v tan(psi)/l` and `psidot = u`.
    - `WheelBase` defaults to 1. `VehicleSpeedRange` defaults to `[-Inf Inf]`.
    - `bicycleKinematics` is the 3-state model, with inputs `[v, steering angle]` or `[v, heading rate]`.
  - **What the pages do not state:** they do not explicitly state that positive steering means left. An example labels `psiDot = 1` as a "left steering rate", which suggests it, but this note does not rely on it. They also give no track-width parameter or inner/outer wheel formula, although the equations page says the object adjusts the wheels "based on the track width". The wheel-geometry formulas here therefore come from the ICR derivation in §6, not from MathWorks.
- **[MPL Lab]** https://mpl.sist.shanghaitech.edu.cn/resVisionForAckermannVehicles.html. It describes the ICR on the rear-axle axis, motion "at least instantaneously" along a circular arc, and "temporarily constant velocities" as an approximation. This is motivation for the arc and ZOH assumptions, not an MPC or dynamics specification. Papers listed there include Huang, Wang, Kneip, CVPR 2019, and Huang, Wang, Kneip, ICRA 2021.
- **[Rajamani]** R. Rajamani, *Vehicle Dynamics and Control*, 2nd ed., Springer, 2012, ch. 2 (kinematic and dynamic lateral models).
- **[Kong et al.]** J. Kong, M. Pfeiffer, G. Schildbach, F. Borrelli, "Kinematic and dynamic vehicle models for autonomous driving control design", IEEE Intelligent Vehicles Symposium, 2015.
- **[Polack et al.]** P. Polack, F. Altché, B. d'Andréa-Novel, A. de La Fortelle, "The kinematic bicycle model: a consistent model for planning feasible trajectories for autonomous vehicles?", IEEE IV 2017. Also "Guaranteeing consistency in a motion planning and control architecture using a kinematic bicycle model", ACC 2018, arXiv:1804.08290. **The 0.5 μg criterion was checked only through the 2018 abstract and secondary citations**, not the 2017 full text.
- **[Lynch & Park]** K. M. Lynch, F. C. Park, *Modern Robotics: Mechanics, Planning, and Control*, Cambridge University Press, 2017, ch. 13 (cited by the MathWorks `derivative` page).
- **[LaValle]** S. M. LaValle, *Planning Algorithms*, Cambridge University Press, 2006, §13.1.2 (simple car).
- **[PSO]** J. Kennedy, R. Eberhart, "Particle swarm optimization", Proc. ICNN 1995; Y. Shi, R. Eberhart, "A modified particle swarm optimizer", IEEE ICEC 1998 (inertia weight).

**Contradictions with the review note:** none in substance. Points to note:

- **Naming:** MathWorks uses `psi` for the steering angle, while the review and this note use it for heading.
- **Model structure:** MathWorks `ackermannKinematics` is the 4-state, steering-rate-input model, which is the review's *augmented* option. The review's 3-state baseline corresponds to MathWorks `bicycleKinematics`.
- **Division form:** the review's exact-arc formula, as written, is numerically unsuitable for half precision and table trig (§5.5). It is exact mathematically; the chord form is equivalent and should be used in hardware.
- **`tan` cost:** the review's argument for `kappa` (removing `tan` from every prediction step) holds for RK4 as implemented. `tan` could also be hoisted to once per stage, so §4 bases the choice on the other properties.
