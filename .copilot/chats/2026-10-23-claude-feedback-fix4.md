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

## Feedback on the 100-step fix6 notebook run (2026-10-24)

### Evidence review

- **Correction after inspecting the full workspace notebook:** `app/kria-pynq/kv260_nmpc_demo_kv260.ipynb` has saved cell 16 outputs for five 100-step trials, cell 18 has the MATLAB comparison, and cell 20 records validation statuses and the run directory. The separate `kv260_nmpc_demo.ipynb` remains the 10-step/one-trial smoke test; retain both rather than treating them as conflicting results.
- Cell 16 reports five successful 100-step trials with zero timeouts, delta/bound violations, and plant-limit crossings. Aggregate position RMSE is 0.354217 ± 0.006398 m, max error 0.603886 ± 0.008877 m, peak tilt 0.143010 ± 0.001785 rad, and mean solve latency 0.522892 s. Cell 20 marks Tcl-only checks `skipped` with reasons and all checks that ran as pass.
- The full notebook outputs support Claude's summary. The separate attachment excerpt by itself did not expose the outputs, which caused my earlier incorrect statement that the 100-step results were not inspectable.
- The board archive is still at `/root/jupyter_notebooks/nmpc_hls/app/kria-pynq/validation_logs/20261023T220440Z_rk4_100x5`; copy it into the workspace for durable raw data. The notebook contains the rendered outputs, but board-side artifacts include per-trial histories.
- The controller uses the simulated plant and the same model equations; the RK4 plant is a better integration match than Euler, but this still does not establish physical-plant performance. At 0.523 s per solve versus a 50 ms sample period, this remains offline HIL, not real-time control.
- The controller uses the simulated plant and the same model equations; the RK4 plant is a better integration match than Euler, but this still does not establish physical-plant performance. Also, 0.523 s per solve is about 10.5 times the 50 ms sample period, so this remains offline HIL, not real-time control.

### Recommended order

1. **Archive the exact 100-step run first.** Preserve the executed notebook with outputs, state/control/reference histories, per-step timing and fitness, configuration (`plant_integrator`, sample time, horizon, initial state, reference), fix6 artifact hash, and a JSON/CSV metrics report. Include the final state at 5 s. The notebook already saves state/control history and timing, but ensure `closed_loop_metrics` and the exact run configuration are saved too.
2. **Fix reporting before treating the report as a pass/fail artifact.** Missing repo-only Tcl inputs should be `skipped`/`unavailable` with a reason, not `False` (which implies a failed check). Keep FPGA artifact/hash checks and actual runtime checks as pass/fail. Use a tri-state result or separate status/reason fields.
3. **Compare MATLAB with identical metric definitions and explicit data.** Use `drone_spiral_dir.mat`; document time alignment, sample count, initial/final-state inclusion, units, and the 25-step HLS versus 60-step MATLAB horizon/optimizer/weight differences. Present this as a descriptive comparison, not an apples-to-apples benchmark.
4. **Measure stochastic spread before changing the controller configuration.** Five 100-step trials are about 4.4 minutes of board solve time, not one minute total. Reset the simulated initial state and warm start for each trial; document that the free-running RNG continues and whether trials share one stream or reprogram/reset between trials. Report RMSE, maximum/final error, peak tilt, violations, and latency distribution per run and across runs.
5. **Then screen speed/quality candidates on the PC.** The 0.523 ms/evaluation estimate is a useful first-order budget, not a timing guarantee: startup/IO costs and HLS scheduling/resource use need measurement. A 5×25 configuration estimates around 65 ms serially; two parallel cost units could approach 33 ms only under ideal scaling. Include margin below the 50 ms deadline, and evaluate tracking/constraint metrics over multiple random runs before selecting a hardware rebuild.
6. **Keep physical tests gated.** Do not connect the result to actuators based on this perfect-model simulation alone. First meet the real-time budget and test model mismatch/disturbances in simulation; physical validation needs a separate safety plan and independent plant evidence.

### Decision

The fix6 five-trial baseline is now evidenced by the full notebook output. Preserve it as the reference, copy the raw board archive, then proceed with PC screening. Treat the MATLAB comparison as descriptive because optimizer, horizon, weights, and integrator differ. The central engineering gap remains real-time performance; physical-plant robustness is unvalidated.

## Feedback on five-trial baseline and PC configuration screening (2026-10-24)

### Baseline assessment

- The full workspace `app/kria-pynq/kv260_nmpc_demo_kv260.ipynb` contains the five-trial outputs, MATLAB comparison, and final status report. The separate 10-step smoke-test notebook remains distinct.
- Fetch the board's exact `validation_logs/20261023T220440Z_rk4_100x5/` output into the workspace before committing the summary. Keep per-trial states, controls, references, timing and configuration with the aggregate report. Preserve the earlier one-run archive as well.
- There is a small metric completeness issue in the attached `tracking_metrics`: `max_control_step` uses `np.diff(applied)` and omits the transition from initial `u_curr` to the first applied control. The loop's `delta_violation` counter does test that first transition, so use that counter for pass/fail or pass `u_curr` into `tracking_metrics` and prepend it when calculating the maximum.
- Clarify the statement that the 20-unit step limit is “hit” but has “no step-size violations”: distinguish reaching exactly `du_max` from exceeding it. The loop's `delta_violation` should mean strictly greater than the allowed bound plus tolerance.

### Feedback on the PC screening proposal

1. **Proceed with a PC closed-loop simulator before any rebuild.** It is the right next experiment for selecting particle/iteration settings; keep fix6 as the untouched baseline. Compile candidate binaries from isolated header copies, not by editing the live HLS source/config or generated IP in place.
2. **Treat the 10×100 PC check as distributional, not run-by-run reproduction.** The hardware AXI-stream RNG sequence differs from the PC RNG, and the board's five trials share a continuing stream. Compare repeated-run distributions and control/constraint metrics with matched initial conditions and the same RK4 plant, not individual control sequences. Record RNG implementation/seeding and whether each trial resets or continues its stream.
3. **Keep the hardware arithmetic for selection if feasible.** If `-O0` plus emulated half makes the full grid too slow, first compare the faster surrogate against the half model across a representative sample of states/controls, including near-limit trajectories and candidate rankings—not only the 10×100 aggregate. A surrogate that matches average cost but changes constraint crossings or ranking is not a safe screening model.
4. **Use the evaluation-count calculation only as a rough budget.** The measured 0.523 ms/evaluation includes some system overhead and will not necessarily scale linearly. Check HLS evaluator latency/throughput and fix6 resource reports, then include PYNQ/launch overhead and margin in the deadline estimate. Two/four parallel cost units may not provide ideal 2×/4× speedup because of memory ports, routing, timing, and shared resources.
5. **Choose candidates using more than mean RMSE.** For each setting and at least five deterministic PC seeds, report RMSE, max and final position error, peak tilt/true-limit crossings, absolute and delta-control violations, and latency estimate. Set a deadline criterion (with margin under 50 ms) as well as the proposed quality tolerance. Avoid accepting a setting on mean RMSE if a seed violates a hard constraint.
6. **Keep control-weight tuning separate.** The MATLAB R-weight difference is a plausible explanation for the more aggressive controls, but changing R during the particle/iteration screen would confound the comparison. First compare configurations at fixed fix6 weights; then run a separate weight-sensitivity study.
7. **The run is not real-time or physical validation.** The five-trial baseline is still a software-plant HIL simulation at roughly 10.5× the sample period. Physical tests should remain gated until real-time feasibility and model mismatch/safety tests are addressed.

### Decision

Proceed with the proposed PC screening after copying the five-trial board archive and fixing the first-control-step metric. Compare candidate quality statistically with the hardware-arithmetic model where practical; use HLS reports for architecture/latency estimates, and make no rebuild until one configuration meets both quality and deadline-margin criteria.

## Recheck of full five-trial notebook outputs and PC-screening proposal (2026-10-24)

### Verified in `kv260_nmpc_demo_kv260.ipynb`

- Cell 16 contains the five 100-step trial outputs. Each has 100 completed steps, zero timeouts, zero delta/bound violations, and `plant_limit_ok=True`.
- Across trials: RMSE mean 0.354217 m, std 0.006398 m (range 0.348958–0.363396); maximum position error mean 0.603886 m, std 0.008877 m; peak tilt mean 0.143010 rad, std 0.001785; mean latency 0.522892 s/solve, with max observed 0.523551 s.
- Four trials reached exactly `du_max=20`; that is an active constraint, not a violation. Trial 3 max step was 19.7793. Per-step `delta_violations` are all zero.
- Cell 18 compares the same 101 state samples and reports reference alignment within `7.1e-15 m`. It documents the material differences (25 vs 60 horizon, PSO vs fmincon, weights and RK4 vs Euler), so treat it as descriptive, not an apples-to-apples benchmark.
- Cell 20 correctly reports missing Tcl checks as skipped with reasons, while runtime/artifact/constraint checks that ran pass. The run archive is board-side at `validation_logs/20261023T220440Z_rk4_100x5/`.

### Recommendation on PC screening

Proceed with the proposed PC screen before any rebuild:

1. Calibrate the PC 10×100 hardware-arithmetic/RK4 closed loop against the board's **distribution**, not exact trajectories: PC and hardware use different RNG streams. Record seeding/stream policy and compare RMSE, max/final error, tilt, constraints, and latency inputs over multiple runs.
2. Build each particle/iteration candidate from isolated header copies and verify compile-time settings. Keep fix6 sources and overlay untouched until a candidate is selected.
3. Prefer the exact hardware arithmetic/half model. If a faster surrogate is necessary, validate it on representative trajectories near the angle limit and compare candidate rankings/constraint crossings—not just average objective.
4. Screen multiple seeds, requiring hard constraints and quality limits (including worst/percentile error, not only mean RMSE). Use the 50 ms deadline with margin.
5. Treat 0.523 ms/evaluation and ideal 2×/4× parallel speedups as rough estimates only. Refine with HLS cycle/resource reports, include host/launch overhead, and measure actual FPGA latency after the single selected rebuild.
6. Keep MATLAB weight tuning and plant-mismatch robustness as separate experiments so they do not confound particle/iteration screening.

The 5×100 experiment is a good validated fix6 software-plant baseline, but it is still about 10.5× slower than the 50 ms sample interval and is not physical-plant validation.

## Feedback on completed hardware-arithmetic PC screen and follow-up (2026-10-24)

### Assessment

- The `half`-arithmetic screen is complete and its 10×100 distribution is calibrated against the fix6 board baseline. This is substantially stronger evidence than the earlier `float` preview; use the `half` results for selection.
- The screened single-evaluator configurations do **not** meet a 50 ms deadline with 20% margin. In particular, 3×30 is estimated at 45–47 ms before PS-side overhead, leaving only 6–10% nominal margin; it does not meet the stated margin. 5×18 is both slower in quality and similarly over budget for that criterion.
- Preserve fix6 as the baseline and do not select or rebuild from the current screen alone. The open quality tolerance is a decision input, not something to infer from the table.

### Recommended next work

1. Agree on an acceptable tracking-quality degradation relative to fix6 and retain hard per-seed constraints as non-negotiable.
2. Screen the untested 3×20, 3×25, 4×20 and 2×40 settings with the validated `half` model and the existing multi-seed metrics. Their rough estimates are 30, 38, 40 and 40 ms respectively, before PS overhead; only 3×20 has meaningful apparent room under a 40 ms solver budget, and its quality still needs measurement.
3. Measure PS-side encoding, cache maintenance, launch and polling overhead on the board and include it in the 50 ms budget. Do not treat the linear cycle estimate as a timing result.
4. Evaluate the E≈150 / two-evaluator candidates separately. First establish HLS/Vivado feasibility, timing and shared-memory/port behavior; ideal 2× scaling is not evidence that either candidate will fit.
5. Keep fix6 untouched, and keep physical-actuator testing gated until real-time feasibility and mismatch/safety testing are established.

### Reply on `loginctl disable-linger`

The user has already run `loginctl disable-linger`; **leave lingering disabled**. Do not run `loginctl enable-linger` or change this setting. Disabling linger is not an immediate kill signal for an already-running `systemd --user` unit, but the user manager is no longer guaranteed to remain alive after the user's last session ends (or to start at boot solely for user services). The progress note says the PC screen is complete, so no lingering is needed for that finished run. If a future screen must survive logout, ask before changing this setting; otherwise keep a user session active for the duration or choose an explicitly managed alternative.
