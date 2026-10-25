# GPT-6 Feedback: 3×24 Commits and Follow-up

## Review

The raw timing archive resolves the earlier evidence gap. Row-wise `solve_call_seconds` for 500 solves gives a 39.34 ms median, 39.53 ms p95, 39.78 ms p99, and 40.40 ms max; three steps exceed 40 ms and none exceed 50 ms. This supports the precise claim that the measured run met the 50 ms deadline but missed the stricter 40 ms target on 3 of 500 steps. It is measured evidence, not a worst-case guarantee.

The five-board-trial quality sample passed the agreed criteria, but its mean RMSE of 0.4235 m is only 0.0015 m below the limit. Keep the narrow margin and limited sample size prominent.

The manifest-safety change addresses the pre-commit concern: the host validates solver metadata before hardware access and allocates the fixed 128-entry history buffer, matching the local BRAM boundary. The focused overlay suite passes: 11 tests.

## Recommendations

1. **Do not promote 3×24 yet.** The 40 ms target was not met on every observed step, and the RMSE pass is marginal. Keep fix6 as the promoted baseline unless the acceptance criterion is explicitly changed to the hard 50 ms period and the small quality margin is accepted. If the board is not being used for more candidate tests, restore fix6 with the guarded prep procedure before ordinary use.
2. **Proceed with the angle-wrap fix as the next correctness change.** Keep it separate, add the nonzero-reference and ±π-crossing regression cases, then rerun C simulation, co-simulation, synthesis, and board validation on a new overlay. Recheck 3×24 quality and timing with that evaluator before revisiting promotion. The current five-second spiral does not exercise the defect, but longer heading runs and Ackermann tracking can.
3. **Correct the stale timing text in the dated 3×24 notebook.** It still describes a solve as about 0.52 s, which is fix6-specific. This is a documentation-only edit; it does not require re-running the board, re-exporting results, or restaging the overlay. Leave the fix6 baseline notebook's corresponding text unchanged.
4. **No extra ignore-rule commit is needed.** The current checkout already contains `4be555b`, which ignores `hw/hls/logs/` and `hw/hls/ip_backup_*`.
5. **Defer `fastcos` repair until use.** Keep a focused failing test/backlog item, but do not perturb the active evaluator path merely to fix an unused helper.
6. **Keep Ackermann gated on vehicle data.** Before selecting a hardware variant or quoting a cycle budget, obtain the target vehicle, command interface, wheelbase, steering and speed limits, rates, actuator lag, and sample-time/latency constraints.
7. **Keep physical-actuator tests gated** until the model, timing margin, and safety checks are resolved.

## Repository-state note

The current workspace shows `HEAD` at `4be555b` and the tracking ref `origin/origin/build` at the same commit. That is consistent with there being no local commits ahead, but it does not by itself prove the remote's current state. If "not pushed" is a requirement, verify the remote before relying on that status. I did not fetch, push, stage, or alter the existing SVG or `.claude/` worktree.
