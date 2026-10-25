# GPT-6 Feedback: 3×24 Board Results

## Recommendation

Proceed with the scoped 3×24 implementation commits after addressing the evidence and manifest-safety items below. Keep **fix6 as the promoted overlay**. The 3×24 build is a promising, board-validated candidate, but its timing and quality margins are too narrow to treat it as a robust promotion yet.

## What the notebook supports

The saved 3×24 notebook records a 24-iteration solve with valid output and about 38.21 ms solver latency, plus five 100-step software-plant HIL trials. All 500 steps completed without timeouts, control-bound or control-step violations, or plant tilt-limit crossings. The worst recorded position error was about 0.696 m and worst tilt about 0.160 rad. The mean RMSE was 0.423488 m, just below the 0.425 m criterion; trial RMSE ranged up to 0.456658 m. Reaching the control-step limit 1–5 times in a trial is not a violation.

This is a narrow sample-mean pass, not strong evidence of a stable quality margin. It is broadly consistent with the 10-seed PC screen, but five board trials do not establish distributional equivalence. Keep the claim precise: **the board sample passed the agreed criteria; the RMSE margin is very small.**

The notebook's saved validation report passes the artifact, register-map, memory-layout, solver, numerical, constraint, and closed-loop checks. The two Tcl-based checks are skipped, not passed, because the repository Vivado Tcl was absent from the board copy.

## Timing interpretation and evidence

Separate the two timing thresholds:

- The reported worst solver-plus-PS time of 40.42 ms is below the 50 ms sample period in the 500 observed steps, so the run met the hard deadline.
- Three of 500 reported steps exceeded the stricter 40 ms target (50 ms with 20% margin). Therefore, it did **not** meet that target on every step. A 40.42 ms maximum is only 0.42 ms over the target and leaves limited measured margin to the deadline.

The notebook records `solve_call_seconds` in `solver_timing.csv`, but the raw `validation_logs/20261025T173937Z_rk4_100x5` archive is not present in this workspace; the saved notebook output shows solver and PS-overhead summaries separately. Before finalizing the timing claim, retrieve the archive from the board and independently calculate the row-wise combined distribution, including count above 40 ms, p95, max, and count above 50 ms. Do not add the separate solver maximum to the separate overhead maximum; those maxima may come from different steps. Keep the raw archive in the ignored validation-log area, not in Git.

Five hundred observed solves support feasibility, not a worst-case timing guarantee. Describe this as measured HIL evidence, not proof that every future invocation will meet the deadline.

## Pre-commit implementation check

The new manifest's `solver.iterations` value controls the size of the `bestfitness` PYNQ buffer. The package CLI accepts particle and iteration values as manually supplied arguments, while `NmpcOverlay` checks the reported iteration count only after the hardware run has written the output. If the manifest understates the bitstream's iteration count, the output buffer can be undersized before the mismatch is detected.

Before committing the manifest-size handling, validate solver metadata before allocation and ensure the output buffer cannot be undersized for the compiled hardware. Prefer binding the metadata to the build artifact; otherwise use a safe supported maximum for allocation and reject invalid metadata. Add a focused test for a manifest/bitstream iteration mismatch. This does not invalidate the current board result—the 3×24 artifact and manifest agree—but it matters for the general deployment path.

## Commit scope and order

Once the checks above are complete, make the proposed logical commits:

1. HLS configuration and testbench changes for 3×24.
2. Host-side manifest/configuration handling, validation, screening, and notebook source updates.
3. The build wrapper, handoff update, and dated executed 3×24 notebook as the run record.

Stage only the files belonging to these changes. In particular, exclude the modified `graphs/nmpc_pso_system_overview.svg`, unrelated notebook copies, `.claude/` worktrees, IP backups, and generated HLS logs. Do not push or promote the overlay as part of these commits.

Keep the angle-wrap correction as a separate change with its nonzero-reference regression tests. Keep `fastcos` and the Ackermann prototype isolated. Physical-actuator tests remain gated.
