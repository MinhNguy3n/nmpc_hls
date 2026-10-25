# GPT-6 Feedback: Angle-Wrap Fix on the 3×24 Build

## Recommendation

**Approve the two scoped commits** for the angle-wrap fix and its evidence, while keeping `overlay-3x24-wrap` unpromoted and fix6 promoted. The new board run is a convincing non-regression check for the existing five-second trajectory; the targeted C model tests and evaluator C/RTL co-simulation are the relevant evidence that the corrected crossing case works.

The unchanged board trajectories are expected: the tested trajectory does not cross the ±π residual boundary, and the hardware RNG restarts on programming. They show the fix did not perturb this nominal case, not that the board closed loop exercised the fix.

## Evidence and caveats

- The 12-case model regression compares the squared angle residual against an independent `std::remainder` reference, including nonzero references, positive and negative crossings, and a difference of 2.9π. This matches the implementation's documented one-correction contract, `|x_hat - ref| < 3π`.
- The evaluator testbench adds a targeted `3.0` versus `-3.0` yaw case. The reported co-simulation result (wrapped cost about 8.18 versus roughly 3,700 with the old branch) directly exercises the failure that the five-second HIL trajectory does not.
- The post-fix board results are bit-identical to the previous run across all five trials, with zero violations and timeouts. The aggregate RMSE remains 0.4235 m against the 0.425 m limit; this is still a narrow pass.
- Combined solver-plus-PS timing is 39.40 ms median, 39.53 ms p95, 39.92 ms p99, and 40.29 ms max. Two of 500 steps exceed 40 ms; none exceed 50 ms. This remains a measured hard-deadline pass, not a pass of the stricter 40 ms target on every step and not a worst-case guarantee.
- Scripted validation passes. The two Tcl checks remain skipped, not passed, because the board copy lacks the repository Tcl.
- WNS is +0.330 ns. Record it with the build evidence; do not use it to imply extra timing margin.

No additional board run longer than 8.75 seconds is required to accept this correction: the directed model test and C/RTL evaluator test cover the crossing, while the board run checks the non-regression path. The 10-seed PC regression can finish in the background. It need not block these commits, but record its outcome in the handoff when available.

## Commit scope and follow-up

1. Commit only the three HLS files: `hls_system.hpp`, `main_hls_model_unit_test.cpp`, and `main_hls_evaluate.cpp`.
2. Separately commit the handoff update with the new build/artifact hashes, WNS, C/RTL and scripted-validation results, the bit-identical board comparison, and both timing thresholds. Record that the 10-seed PC regression is still running if it has not completed.
3. Keep the unrelated SVG edit and `.claude/` worktree out of both commits. Do not promote or push the candidate as part of this approval.
4. After the PC screen finishes, update the handoff if its result changes or materially supplements the current evidence.

The fix is a prerequisite for longer heading runs and the Ackermann track. Keep physical-actuator testing gated until the timing and quality-margin decision is made.
