# Feedback on Claude's Latest Progress: `nmpc_solver_v2_fix4`

## Current status from the handoff

- Integrated fix4 build completed with WNS +0.301 ns, WHS +0.010 ns, and no failing endpoints.
- Netlist/BD reportedly contain the full fix set: FSM done polling, evaluator FIFO drain, detect `interaction` as plain input, random-stream any-ready fan-out, and corrected local-BRAM mapping/offsets.
- Staged artifacts match the build by SHA-256: `.bit` `c79aa9e2…`, `.hwh` `154fb64b…`, and `.bda` `323c0c1f…`; the `.ltx` hash is recorded on the host.
- The first fix4 board run completed all 100 iterations in about 0.50 s. `ap_ctrl=0xe`, `ap_return=100`, sentinels were overwritten, and `bestfitness` decreased from 4870.74 to 4554.46.
- The reported hardware hangs are resolved. The primary remaining work is numerical validation and safely promoting the overlay.

## Feedback and recommended next actions

1. **Validate numerics against a like-for-like software run.** Use exactly the same initial buffers, constraints, parameters, and random stream/seed if controllable. Compare the full `new_best` vector and final `bestfitness`, not just monotonic improvement and `du_max` bounds. Document the fixed-point/`half` tolerance and report per-element absolute/relative error.
2. **Check the reported results independently.** Confirm that `bestfitness` is the expected objective for the captured final control and that all constraints hold. Monotonicity and bounded control deltas are useful sanity checks, but do not alone prove equivalence or correctness.
3. **Handle `static w` before repeated-solve or closed-loop use.** Either add an explicit per-solve reset/initialization and test consecutive solves, or document and enforce the single-solve lifecycle. Do not let this statefulness contaminate software-vs-hardware comparisons.
4. **Promote only the verified fix4 overlay.** Keep `overlay-fsmfix`, `overlay-fix2`, and `overlay-fix3` marked superseded. Before changing the default overlay/notebook path, verify the exact `.bit`/`.hwh`/`.bda` hashes and corresponding XSA; retain a rollback copy. The `.ltx` is host-side for Hardware Manager, so record its build association even if it is not staged on the board.
5. **Commit a deliberate source-only change set.** Include intended RTL/HLS source, Tcl/BD, and self-checking cosim tests/scripts. Exclude generated build directories, packaged bitstreams, and backups unless the repository intentionally tracks those artifacts. Run the existing relevant checks before committing.
6. **Clarify stale historical notes.** The old local-BRAM defect section still contains wording that says wrong/degenerate results are expected until the map is fixed. Mark that text explicitly historical and point to the fix4 verification. The current next-step list is substantially updated; keep it consistent with the successful board run.
7. **Preserve the safe board procedure for future PL changes.** Continue `xmutil unloadapp`, cpuidle mitigation, UART logging, and short/necessary JTAG sessions. No additional ILA capture is needed unless numerical comparison points to a specific mismatch.

## Suggested completion record

- Matching software/hardware inputs, RNG behavior, and numerical comparison tolerance.
- Per-element `new_best` errors, final objective difference, and constraint checks.
- Consecutive-solve result after addressing or explicitly accounting for `static w`.
- Fix4 artifact hashes, XSA association, and which overlay path becomes the default.
- Exact source/test files committed and checks run.

## Feedback on fix5 progress (2026-10-23)

### What looks validated

- The stateless inertia-weight change is well motivated: the prior fixed-point decay/reset rounded across the expected iteration boundary, making later solves start at a different weight.
- `update` C/RTL co-sim checks repeatability for identical inputs/stream/iteration; evaluator and detect regressions pass.
- Integrated fix5 timing is positive with no failing endpoints, and the recorded bitstream/HWH/BDA/LTX identities give a useful deployment baseline.
- The reported board runs are self-consistent under the hardware's half-precision model: recomputed costs differ from the hardware result by at most about 0.010%, and the runs complete 100 iterations with constraints satisfied.

### Recommended decision and ordering

1. **First reconcile whether `step5_validate.sh` has passed.** The latest handoff still lists it as a next step, but the progress summary reports two fix5 board runs already checked. Update the handoff with the exact validation command/result, run count, artifact hash, and comparison tolerance. Do not assume the two logged runs equal the scripted two-consecutive-solve validation unless they used that script and captured the same JSON evidence.
2. **Promote fix5 with rollback if that validation is confirmed.** The static-weight correction is an implementation fix and the reported two-run evidence is encouraging. Keep the rollback copy and record the promoted artifact hashes. If the cost-model fidelity is a release acceptance criterion, promote only to a clearly labeled experimental/current FPGA overlay until that question is resolved.
3. **Commit the stateless-weight fix as a focused atomic change after validation.** Include only the intended HLS source, BD wiring, and regression test/script for `iteration`; avoid sweeping in unrelated dirty/generated files. The worktree has previously contained multiple independent debug changes, so stage by explicit paths and inspect the staged diff.
4. **Investigate the half-vs-float gap before claiming the optimization is correct for the plant.** Matching the hardware's half model proves implementation consistency, not that half is accurate. Do not assume float is ground truth either. Compare half, float, and preferably a higher-precision or independently trusted reference on identical controls and inputs. Log the state trajectory and objective terms per horizon step; find the first divergence and whether it crosses a constraint/penalty boundary. Check units, rounding, saturation, fast-sin approximation, and penalty logic.
5. **Keep the validator's reference explicit.** It should report both (a) hardware-vs-half consistency and (b) half/float/reference model discrepancy, rather than silently switching the single expected model. For release decisions, use the validated model required by the application.

### Handoff corrections

- Reconcile the current next-step item saying board validation is pending with the reported completed fix5 runs.
- The statement that float is “more accurate” should remain a hypothesis until checked against a higher-precision or trusted dynamics reference. The 133–134% objective difference is significant and warrants investigation, but by itself does not identify which model is physically correct.
- Clarify whether “promote fix5” means making it the default user-facing overlay or only promoting it as the verified hardware implementation. Those are different decisions while model fidelity remains open.

## Feedback on fast-sin / tilt-margin findings and fix6 (2026-10-23)

### Recommendation on fix5 promotion

**Wait for fix6 board validation; do not promote fix5 as the default or as an endorsed experimental controller.** The table-period/index mismatch is a functional model bug, and the fix5 solution crossed the true roll limit under the high-precision reference even though the hardware's half model predicted it just inside. Preserve fix5 as a rollback/debug artifact, but avoid presenting it as safe for control.

### Feedback on the fixes

- The sine-table diagnosis is convincing: 1023 samples fitted to a 1022-sample period but indexed as 1024 samples explains the phase-dependent error and the out-of-bounds index. Regenerating exactly 1024 samples per period, rounding the index, and masking the wrap address the root cause without adding interpolation cost.
- Before accepting the table fix, ensure tests cover negative angles, wrap boundaries, exact/near multiples of the table period, and the full operating range. Compare the generated C and synthesized table contents and verify rounding/wrap semantics match in both paths.
- The 0.01-rad tilt margin is a reasonable guard for the measured model error, but retain separate checks for (1) the tightened cost-function limit and (2) the true plant limit ±π/4. Test at, just inside, and just outside both thresholds. A lower cost caused by the margin is not itself proof that the predicted trajectory is safe.
- Keep the caveat that the double-precision REF uses the same equations and therefore checks numerical fidelity to those equations, not whether the equations match the physical plant. The fix5 discrepancy is large enough to justify a trusted independent reference or plant-level validation before claiming model correctness.

### Suggested fix6 sequence and acceptance criteria

1. Let evaluator co-simulation and the `nmpc_solver_costF` rebuild finish; record results and confirm synthesis uses the corrected table and margin.
2. Build the integrated fix6 design, inspect timing, and verify the full fix set. Hash the `.bit`, `.hwh`, `.bda`, `.ltx`, and XSA association; package/stage only matching artifacts.
3. Run `step5_validate.sh` on fix6 and preserve the JSON dumps. For each run report hardware-vs-half objective consistency, REF objective gap, and true ±π/4 plant-limit violations separately. Do not use total cost alone to classify safety because the hard penalty is discontinuous.
4. Promote fix6 only after at least the scripted repeated-solve validation passes, all true plant-limit checks pass, the overlay hashes match, and board health is confirmed. Keep a rollback copy and leave fix5 unpromoted.
5. Update the validator so it reports (a) hardware-vs-half consistency, (b) cost-model/reference discrepancy, and (c) true constraint violations as distinct results. State clearly that an equation-matching double reference is not a physical validation.

### Clarify the promotion question

The current recommendation is **wait for fix6**. A separate fix5 experimental promotion is not worthwhile when its known fast-sin defect can produce a true tilt-limit violation; retain the files for rollback and comparison instead.
