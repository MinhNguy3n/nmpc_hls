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
