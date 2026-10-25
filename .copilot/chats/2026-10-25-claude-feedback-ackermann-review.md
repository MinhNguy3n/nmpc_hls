# GPT-6 Review Feedback: Ackermann MPC-PSO Proposal

## Overall judgement

The Ackermann proposal is a strong, self-contained software design study. The rear-axle bicycle equations, signed curvature convention, front-wheel Ackermann geometry, and conservative curvature-rate bound are internally consistent. The exact-arc chord identity is also mathematically correct.

The branch is still a proposal/reference model, not a vehicle-specific controller or a synthesized design. The reported 80 software tests are useful evidence, but they do not establish HLS C-simulation/RTL behavior, timing, resource use, or closed-loop performance on a real vehicle. Keep the branch isolated from the pending 3×24 quadrotor build.

## Important correction: reconcile accuracy with the timing claim

Please separate the hardware variants in the conclusion and budget:

- The half-emulated, shared-index chord step reports about **12 mm** worst position error after 25 placeholder steps, versus about **7.1 mm** for RK4.
- Fixed-point-angle lookup with nearest-entry selection reports about **4.7 mm**.
- Fixed-point-angle lookup with interpolation reports about **3.0 mm**.
- The cycle model lists the non-interpolated chord at 105–120 cycles/step and the interpolated variant at 120–135 cycles/step. Using the stated upper-bound formula, 10×100 is about **39.4 ms** for the former, but about **43.3 ms** for the latter, including the 3.7% correction and 1.9 ms PS overhead.

Therefore, do not summarize the result as “the accurate chord implementation fits 10×100 in 40 ms” without specifying which trig/index implementation supplies that accuracy. The plain shared-index chord appears to fit narrowly under the estimate but is less accurate than the current RK4 emulation; the interpolated version appears more accurate but its conservative estimate misses the 40 ms target at 10×100. The fixed-angle nearest-entry option may be a useful middle ground, but needs its own cycle estimate and HLS measurement.

Treat all cycle numbers as screening estimates only. Before selecting an Ackermann configuration, implement one isolated HLS prototype, run C-sim against the independent model, run C/RTL co-sim, synthesize and close timing at the actual clock, then measure end-to-end board latency. Confirm the exact half/fixed conversions, table indexing, interpolation, and pose-add precision in that implementation; the Python arithmetic emulator is not a substitute for HLS or RTL validation.

## Angle-wrap bug: fix before Ackermann integration

The existing `controlled_state == 2` branch in `hls_system.hpp` is not a general wrapped residual. For the branch where `x_hat > pi` and `xref < pi`, it uses `(2*pi - x_hat) - xref`; the wrapped residual should be `wrap(x_hat - xref)`, equivalent there to `(x_hat - 2*pi) - xref`. The squared values agree only in degenerate cases (notably `xref == 0`; also `x_hat == 2*pi`). Negative out-of-range angles are not wrapped.

This is latent in the current quadrotor tests because the stated spiral does not enter that branch over the tested 5-second run and its prediction horizon. It will matter for general heading tracking, including an Ackermann vehicle. Please:

1. Add a focused cost regression test for nonzero references, both signs, and both sides of ±π. Include the crossing cases `x_hat > pi, xref < pi` and `x_hat < -pi, xref > -pi`.
2. Implement the smallest generic wrapped-error fix in a separate change, with matching C-sim/RTL checks for the intended representable angle range.
3. Keep this separate from both the Ackermann feature work and the active 3×24 rebuild. Do not modify the quadrotor build inputs while the rebuild is pending.

For angles known to stay in `[-pi, pi]`, one conditional add/subtract of `2*pi` after subtracting is sufficient. If the implementation permits unwrapped angles or multiple revolutions, normalize the error for that wider range and test it explicitly.

## Other findings and scope

- The `fastcos` issue is scoped to the `USE_FAST_SIN_COS` path: `local_cos` scales the angle before calling `fastcos`, which scales again and truncates. The active quadrotor model currently avoids that path by using `local_sin(angle + pi/2)`. Record and test the bug before reusing `local_cos`; do not bundle a broad trig rewrite into the current quadrotor rebuild.
- The `Ts`/`Ts/6` fixed-point bias and the host `encode_fixed` ceil bias are worth tracking, but are small cross-cutting arithmetic items. Do not mix them into the Ackermann model decision without a measured error budget.
- The software proposal includes control-increment terms, but the existing hardware cost does not. Per-step projection enforces slew limits, not smoothness preference. When implementing the model, either add the increment cost in a separately reviewed generic cost change or demonstrate that hard slew limits and the remaining objective produce acceptable control smoothness.
- The model currently has no vehicle values. Do not select `Ts`, speed/curvature bounds, steering limits, or acceleration limits from the placeholders. First identify the actual car, interface, frame/reference point, operating speed/surface, and safe-stop/watchdog behavior.
- The attached `closed-loop_quadrotor_test.png` has empty axes and no plotted trajectory. It is not validation evidence; regenerate it from a saved run or omit it from any report.

## Recommended disposition

1. Keep `ackermann-mpc-design` as an isolated, uncommitted proposal branch for now; do not merge it into `origin/build` or touch the pending quadrotor build.
2. Treat this review as the requested design review. The model is suitable for a later vehicle-specific software prototype once the target and parameters are known.
3. Handle the generic angle-wrap defect as a separate correctness fix before Ackermann HLS integration. Handle `fastcos` separately and only before code begins using that path.
4. Before any HLS-performance claim is used to choose particles/iterations, reconcile the above accuracy-versus-timing variants and replace estimates with synthesis, co-sim, timing closure, and board measurements.
5. Preserve the old Create 3 worktree until its status and contents are checked and the user confirms it can be removed. Do not delete it as cleanup by assumption.

The current 3×24 Vivado build remains a separate user-directed task; this Ackermann proposal neither changes nor authorizes starting that build.
