# KV260 NMPC: Real-Time Configuration Screening and Overlay-Load Guard

## Handoff for the next agent (current state, 2026-10-25)

This continues [2026-10-23-kv260-fsm-done-latch-fix.md](2026-10-23-kv260-fsm-done-latch-fix.md). That file still holds:

- the eight hardware fixes;
- the board scripts and the safe board procedure;
- the build flow and the hazards;
- the fix6 closed-loop HIL baseline;
- the first two PC screens (to 2026-10-24).

This file records what changed since then and what to do next.

### Status

**fix6** (bit `300bd966…`) is the validated, promoted baseline and stays untouched. It solves 10 particles × 100 iterations in **0.523 s**, against a 50 ms sample time. The goal now is one rebuild with a configuration that meets the deadline. The PC screen in the exact hardware arithmetic (`half` mode) is calibrated against the board. Across all three screens, **3 particles × 24 iterations (3×24)** is the only setting that meets both:

- the agreed quality tolerance;
- the 40 ms target (50 ms with 20 % margin), including the measured PS-side overhead.

It met both only narrowly on the PC, and the board confirmed it:

- **3×24 is rebuilt, staged as `overlay-3x24` and board-validated (2026-10-25)**, but **not promoted**. Five 100-step board trials passed the agreed quality criteria, with a very small RMSE margin (mean 0.4235 m against 0.425 m).
- Solver plus PS overhead per step: median 39.34 ms, max 40.40 ms. **No step exceeded the 50 ms deadline**, but **3 of 500 steps exceeded the stricter 40 ms target**: the margin is narrow, not a full pass. See "3×24 rebuild and board results".
- fix6 stays the promoted overlay. Physical-actuator tests stay gated.
- **The cost's angle-wrap bug is fixed** (commit `8c61f1b`) and rebuilt as `overlay-3x24-wrap`. On the board its 5 × 100 trials are bit-identical to the 3×24 run before the fix, so this is the build to use if 3×24 is ever promoted. See "Angle-wrap fix".

### What happened since the last handoff

1. **Board freeze and overlay-load guard (2026-10-25).**
   - **The freeze:** after the board had been rebooted, the notebook froze the whole board (no ping, no serial) right after printing the overlay paths. With `download=False`, `NmpcOverlay` writes the buffer addresses into the solver registers (`0xA0010000`, `nmpc_overlay.py` `_configure_arguments`). Without the fix6 design in the PL, nothing answers that access and the AXI bus stalls the SoC.
   - **Evidence:** after the power cycle, `step_prep.sh` had to unload `k26-starter-kits` from slot 0, so the PL held the starter app, not fix6.
   - **Fix (uncommitted, see below):**
     - `NmpcOverlay(download=True)` deletes any old marker before programming. After programming and register setup succeed, it writes `/run/nmpc_overlay_loaded.json` with the bitstream path, `bitstream_sha256`, `boot_id` and `programmed_utc`.
     - `NmpcOverlay(download=False)` raises a `RuntimeError` before any register access unless the marker matches the current boot (`/proc/sys/kernel/random/boot_id`) and the bitstream's SHA-256.
     - `/run` is cleared on reboot, so the marker cannot outlive the design it describes.
   - **Tests:**
     - PC, with a stubbed `pynq`: refusal without a marker; success after `download=True`; refusal for another bitstream; refusal after a boot-ID change.
     - Board: `step_prep.sh` wrote the marker at 2026-10-25T15:24:16Z (boot `8c89b500…`), and the 10-step notebook smoke test with `download=False` then passed.
2. **PS-side overhead measured on the board.**
   - The notebook now times each whole `solver.solve()` call. `ps_overhead_seconds` is the part outside `ap_start`→`ap_done`: input encoding, cache flush and invalidate, launch, polling exit and output decoding.
   - Run `validation_logs/20261025T143336Z_rk4_100x1` (100 solves): median **1.52 ms**, p95 1.63 ms, max **1.88 ms**. The 10-step smoke test gave median 1.57 ms and max 1.92 ms.
   - Budget **about 2 ms**.
   - The same 100-step run: RMSE 0.366 m, peak tilt 0.138 rad, no violations, latency p95 0.5231 s.
3. **NaN-to-fixed conversion verified in RTL.**
   - **The question:** `half`-mode solves print `assign NaN to fixed point value` when a particle's half-precision state derivative overflows. The derivative is copied into `_hw_top_real` (`ap_fixed<32,17,AP_RND_INF,AP_SAT_SYM>`) by a plain assignment (`one_step_prediction`, `hls_system.hpp`). That is `ap_fixed_base(double(d))`, which has no synthesis-only variant.
   - **The probe:** a dedicated HLS probe (`xck26`, 100 MHz) **passes C/RTL co-simulation**:

     | Input | Becomes |
     | --- | --- |
     | Every NaN (+, −, signaling) | +65535.99997 |
     | +inf | +65535.99997 |
     | −inf | −65535.99997 |
     | ±65504, 1.5 | exact |

   - **Conclusion:** the hardware handles these particles exactly as the PC screen does. The derivative saturates, the prediction diverges and the cost is very large. No trial with such events violated a constraint.
   - The probe sources were in the session scratchpad and are not in the repo; recreating them takes about 20 lines.
4. **Agreed quality tolerance** (user-approved selection criterion):
   - zero du/bound/tilt violations on every seed;
   - worst-seed peak tilt ≤ 0.2 rad;
   - mean position RMSE ≤ +20 % of the fix6 baseline (≤ 0.425 m);
   - worst-seed max error ≤ 0.75 m.
5. **Narrow PC screen** (`half`, 5 seeds, 100 steps; log `app/kria-pynq/validation_logs/pc_screen/screen_half3.log`).

### Candidate table (all `half` screens, 5 seeds; deadline target 40 ms)

The estimate is iterations × (particles × 50,067 + 337) cycles at 100 MHz. The upper value adds the 3.7 % that the HLS reports don't explain, and both values add the measured maximum PS overhead of 1.9 ms. It is an estimate, not a timing result.

| Setting | E | Mean RMSE (Δ vs 10×100) | Worst-seed RMSE | Worst max error | Worst tilt | Violations | Tolerance | Solver + overhead |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| board 10×100 (fix6) | 1000 | 0.354 | 0.363 | 0.614 | 0.145 | 0 | baseline | 525 ms measured |
| 3×30 | 90 | 0.407 (+15.2 %) | 0.415 | 0.665 | 0.146 | 0 | pass | 47.1–48.7 ms ✗ |
| 2×40 | 80 | 0.402 (+13.6 %) | 0.409 | 0.670 | 0.144 | 0 | pass | 42.1–43.6 ms ✗ |
| 3×25 | 75 | 0.416 (+17.5 %) | 0.436 | 0.688 | 0.146 | 0 | pass | 39.5–40.9 ms (borderline) |
| **3×24** | 72 | **0.417 (+17.8 %)** | 0.445 | 0.696 | 0.154 | 0 | **pass** | **38.0–39.4 ms ✓** |
| 2×35 | 70 | 0.426 (+20.3 %) | 0.435 | 0.719 | 0.147 | 0 | fail (RMSE) | 37.1–38.4 ms |
| 4×20 | 80 | 0.430 (+21.5 %) | 0.441 | 0.693 | 0.154 | 0 | fail (RMSE) | 42.0–43.5 ms |
| 3×20 | 60 | 0.435 (+22.9 %) | 0.463 | 0.711 | 0.157 | 0 | fail (RMSE) | 32.0–33.1 ms |

Notes:

- The PC 10×100 model matches the board distribution (RMSE 0.354 ± 0.009 vs 0.354 ± 0.006 m). Full tables are in the 2026-10-23 file, "PC screen results" and "Second PC screen".
- Iterations matter more than particles at equal evaluations.
- **Two-particle settings** (2×40, 2×35) have no randomly initialized particle: particle 0 is the warm start and particle 1 the stable zero. Their exploration under disturbances or model mismatch is untested, which is one more reason not to choose them.
- **3×24's margins are thin on both axes:**
  - about 0.6–2 ms under the 40 ms target;
  - quality from only 5 seeds, with a worst-seed RMSE of 0.445 m against 0.436 m for 3×25.

### 3×24 over 10 seeds (2026-10-25, `half`, log `screen_half4.log`)

| Seed | 1 | 2 | 3 | 4 | 5 | 6 | 7 | 8 | 9 | 10 |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| RMSE m | 0.445 | 0.420 | 0.408 | 0.397 | 0.414 | 0.441 | 0.419 | 0.440 | 0.393 | 0.434 |
| Max error m | 0.696 | 0.696 | 0.681 | 0.642 | 0.660 | 0.719 | 0.672 | 0.657 | 0.656 | 0.681 |
| Peak tilt rad | 0.154 | 0.148 | 0.139 | 0.139 | 0.147 | 0.154 | 0.142 | 0.149 | 0.139 | 0.144 |

- **Passes the tolerance on every criterion:**

  | Criterion | Limit | Result |
  | --- | --- | --- |
  | Violations | 0 on every seed | none (du, bound, plant limit) |
  | Worst-seed tilt | ≤ 0.2 rad | 0.154 rad |
  | Worst-seed max error | ≤ 0.75 m | 0.719 m |
  | Mean RMSE | ≤ 0.425 m | **0.421 m** (std 0.019, worst seed 0.445) |

  du_max is reached 2–5 times per trial and never exceeded. 12 NaN-to-fixed warnings.
- **The RMSE margin is statistically thin:** the 95 % confidence interval of the mean is 0.408–0.434 m, which contains the limit, and seeds 6–10 alone average 0.425 m. 3×24 is about +19 % RMSE against the board baseline, at the edge of the agreed +20 %.
- **3×24 remains the only screened setting that meets the tolerance and the 40 ms estimate together.** Whether to rebuild with it, accepting a quality margin within the seed-to-seed noise, is the decision for the user and reviewer.

### 3×24 rebuild and board results (2026-10-25)

**Build** (commits `c00ff1c` HLS configuration and testbenches, `494e033` host side):

- `config.hpp` sets `_n_S 3`, `_maxiter 24`. All three worker C/RTL co-simulations pass. The detect and update testbenches are scaled by the configuration and keep their fix6 values at 10×100.
- All five solver IPs were rebuilt; `init_s` had still been the August build. The fix6 IPs are backed up in `hw/hls/ip_backup_2026-10-25_pre_3x24/` (not in git).
- The evaluator costs 147,914 cycles per call, i.e. 49,305 per particle, identical to fix6 per particle. One iteration is 151,785 cycles (1.52 ms).
- Vivado: `hw/hdl/scripts/build_debug_variant.tcl nmpc_solver_v2_3x24` (this wrapper replaces the lost fix-build wrapper). WNS +0.414 ns, WHS +0.010 ns, no critical warnings; LUT 40.6 %, BRAM 23.6 %, the same as fix6.
- The netlist checks `k == 24` in the FSM and loops 3 particles in the evaluator. The audits pass: random fan-out present, broadcaster absent, register map unchanged, and the memory layout from the HWH fits.

| Artifact | SHA-256 prefix |
| --- | --- |
| XSA `hw/exported_platform/nmpc_solver_v2_3x24/nmpc_solver_v2.xsa` | `7fd04f889ec4` |
| bit | `f2cb0f319348` |
| hwh | `40acf348a03a` |
| bda | `323c0c1fe873` (unchanged) |
| ltx | `c30e9070` (unchanged) |

The package is `app/kria-pynq/overlay-3x24` (manifest `solver: {particles: 3, iterations: 24}`); on the board it is `~/nmpc_fsmfix/overlay-3x24`.

**Scripted validation** (`step5_validate.sh overlay-3x24`, two runs at 17:32 and 17:38 UTC):

- 24 iterations; 38.0–38.2 ms per solve.
- `validate_hw_run.py`: **VALIDATION PASS**.
  - Hardware vs its C model: 0.0003–0.0013 %.
  - Gap to the double-precision reference: 0.00–0.09 %.
  - True ±π/4 tilt limit holds (peak 0.746 rad).
  - Control constraints OK.
- The two runs are bit-identical, because the hardware RNG restarts on programming.

**Closed loop** (notebook `kv260_nmpc_demo_3x24_rk4_100x5_2026-10-25.ipynb`, run `validation_logs/20261025T173937Z_rk4_100x5`; RK4 software plant, 5 × 100 steps):

| Metric | Board 3×24 | PC 3×24 (10 seeds) | Requirement |
| --- | --- | --- | --- |
| Mean RMSE | 0.4235 ± 0.020 m | 0.421 ± 0.019 m | ≤ 0.425 m |
| Trial RMSE range | 0.404–0.457 m | 0.393–0.445 m | — |
| Worst max error | 0.696 m | 0.719 m | ≤ 0.75 m |
| Worst tilt | 0.160 rad | 0.154 rad | ≤ 0.2 rad |
| Violations, timeouts | 0, 0 (du_max reached 1–5 times per trial) | 0 | 0 on every seed |

The board sample passed the agreed criteria. The RMSE margin is very small, and five trials do not establish distributional equivalence with the PC screen.

**Timing.** The values below are computed row by row from the raw `solver_timing.csv` (500 solves); the separate solver and overhead maxima are not added together.

| Quantity | Median | p95 | p99 | Max |
| --- | --- | --- | --- | --- |
| Solver (`ap_start`→`ap_done`) | 38.03 ms | — | — | 38.16 ms |
| PS overhead | 1.30 ms | 1.44 ms | — | 2.35 ms |
| **Solver + overhead** | **39.34 ms** | 39.53 ms | 39.78 ms | **40.40 ms** |

- No step exceeded 50 ms; 3 steps exceeded the 40 ms target. The worst step leaves 19.2 % margin to the 50 ms deadline.
- The earlier run `20261025T173547Z` gave the same picture (max 40.42 ms, 3 steps over 40 ms).
- This is measured HIL evidence over 500 solves, not a worst-case timing guarantee.

### Angle-wrap fix (2026-10-25, commit `8c61f1b`)

- **The defect:** for angle states (`controlled_state == 2`), `one_step_error` used (2π − x̂) − ref when x̂ > π and ref < π. The wrapped residual is (x̂ − 2π) − ref; the two agree in the squared cost only for ref = 0. Angles below −π were not wrapped, and π was the half-precision model value (2π about 1.9 mrad off).
- **The fix:** compute x̂ − ref, then apply one conditional ∓2π into (−π, π]. π and 2π are now constants in the fixed-point top type. This is exact for |x̂ − ref| < 3π.
- **Tests:**
  - `main_hls_model_unit_test.cpp` has 12 regression cases: non-zero references of both signs, both sides of ±π, both crossings, unwrapped vs wrapped angles, and a 2.9π difference. 9 fail on the old code; all pass with the fix (`UNIT_PASS`).
  - The evaluator testbench (`main_hls_evaluate.cpp`) now compares each particle's RTL cost with the C cost function for yaw 3.0 against a −3.0 reference. C/RTL co-simulation passes: 8.179291 in both; the old residual would give about 3,700. The testbench never compared costs with C before, which is why the bug passed co-simulation.
- **Rebuild:**
  - Only the evaluator's RTL changed; the FSM, `init_s`, update and detect IPs are byte-identical to the 3×24 build. Evaluator latency is unchanged at 147,914 cycles. The IPs from before the fix are in `hw/hls/ip_backup_2026-10-25_pre_anglewrap/` (not in git).
  - Vivado project `nmpc_solver_v2_3x24_wrap`: WNS +0.330 ns, WHS +0.010 ns, no critical warnings; LUT 40.6 %, BRAM 23.6 %, DSP 6.2 %. The project's evaluator RTL matches the rebuilt IP.

  | Artifact | SHA-256 prefix |
  | --- | --- |
  | XSA `hw/exported_platform/nmpc_solver_v2_3x24_wrap/nmpc_solver_v2.xsa` | `cb5af291500e` |
  | bit | `ac0c8c37c3ca` |
  | hwh | `c8c77b56f7e2` |
  | bda | `323c0c1fe873` (unchanged) |
  | ltx | `c30e9070` (unchanged) |

  Packaged as `app/kria-pynq/overlay-3x24-wrap` (manifest 3×24, register map identical to 3×24); on the board it is `~/nmpc_fsmfix/overlay-3x24-wrap`.
- **Board re-validation** (programmed 19:14:48Z):
  - `step5_validate.sh overlay-3x24-wrap`: **VALIDATION PASS**, with dumps bit-identical to the 3×24 dumps before the fix.
  - Notebook run `validation_logs/20261025T191910Z_rk4_100x5`, 5 × 100 steps: every trial's states and controls are **bit-identical** to `20261025T173937Z`. Mean RMSE 0.4235 ± 0.0198 m (worst trial 0.457), worst max error 0.696 m, worst tilt 0.160 rad, no violations or timeouts. Same narrow pass of the 0.425 m criterion.
  - Timing, solver + PS overhead row by row over 500 solves:

    | Run | Median | p95 | p99 | Max | Steps > 40 ms | Steps > 50 ms |
    | --- | --- | --- | --- | --- | --- | --- |
    | 3×24 with the fix | 39.40 ms | 39.53 ms | 39.92 ms | 40.29 ms | 2 | 0 |
    | 3×24 before the fix | 39.34 ms | 39.53 ms | 39.78 ms | 40.40 ms | 3 | 0 |

    The solver median is 38.05 ms; the differences are run-to-run PS overhead.
  - The two Tcl-based checks are **skipped** (the repository Tcl is not on the board), not passed. All other checks pass.
- **What it shows:** the 5 s quadrotor runs never put the yaw difference across ±π, so the fix changes nothing there. On the PC, the 10×100 `float` trials are bit-identical too. The fix's effect is shown by the unit test and the co-simulation; a closed-loop demonstration would need a run beyond about 8.75 s.
- A 10-seed `half` 3×24 regression with the fixed code (`validation_logs/pc_screen_wrap/`) was still running when this was written. Add its result when it finishes.

### Next steps (priority order)

1. **Done:** the 10-seed 3×24 screen, the rebuild and the board validation (sections above). The screen command, for reference (seeds 6–10 reused the saved 1–5):

   ```bash
   cd app/kria-pynq
   python3 screen_pso_config.py --mode half --configs 3x24 --seeds 1-10 --steps 100 \
       --jobs 10 --out validation_logs/pc_screen --board validation_logs/20261023T220440Z_rk4_100x5
   ```

   Run it as a session background task. **Lingering is disabled on the PC and must stay disabled; ask the user before changing it.** The user keeps the session open for long runs.
2. **Done (record of the procedure): rebuild once with 3×24 (`_n_S 3`, `_maxiter 24`):**
   - set `_n_S 3`, `_maxiter 24` in `hw/hls/source/include/config.hpp` (CRLF; `_slope` follows `_maxiter` automatically);
   - check every HLS pragma depth and the BRAM layout that depends on `_n_S`/`_maxiter` (`pso_maxiter` depths; `bestfitness` region size in `nmpc_overlay.py` `MAX_ITERATIONS`, `HORIZON`, and the notebook);
   - rerun the three worker co-simulations;
   - rebuild the IPs and the integrated design (flow in the 2026-10-23 file, "Reproducing the build"); check timing; package; stage as a new overlay directory;
   - validate on the board as before (`step5_validate.sh`, `validate_hw_run.py`);
   - measure the real latency, then the closed loop with the notebook.

   `nmpc_overlay.py` and the notebook hard-code `MAX_ITERATIONS = 100` and "10 particles, 100 iterations": parameterize them before the board run.
3. **Keep fix6 promoted.** Promoting 3×24 is a separate decision for the user and reviewer, given the narrow quality and 40 ms margins. If it is promoted, use `overlay-3x24-wrap`: it has the angle-wrap fix. If it is promoted, `step6_promote.sh` first needs the overlay argument the other step scripts now take, and it must keep a rollback copy.
4. **Done (commit `8c61f1b`, see "Angle-wrap fix"): the angle-wrap fix, as a separate change** (reviewer's order). The cost's `controlled_state == 2` branch in `hls_system.hpp` is not a general wrapped residual: for x̂ > π, ref < π it uses (2π − x̂) − ref instead of (x̂ − 2π) − ref, and it ignores angles below −π. It is latent for the 5 s quadrotor runs (the yaw reference stays below about 0.8π) but would matter beyond about 8.75 s and for any heading-tracking model.
   - Add a cost regression test with non-zero references, both signs, both sides of ±π, and the crossings x̂ > π, ref < π and x̂ < −π, ref > −π.
   - Apply the smallest generic fix: subtract, then one conditional ±2π.
   - Rebuild the evaluator, run C simulation, co-simulation and a board re-validation.
5. **`fastcos`:** `local_cos` scales the angle and `fastcos` scales it again and truncates the index. The quadrotor avoids it with `local_sin(x + π/2)`. Add a test and fix it only before code starts using `local_cos`.
6. **Ackermann track** (proposal branch `ackermann-mpc-design` in `.claude/worktrees/agent-a3984fccbf29de483`, from `8caa65c`; uncommitted, isolated; reviews `2026-10-25-ackermann-mpc-pso-model-review.md` and `2026-10-25-claude-feedback-ackermann-review.md`):
   - The vehicle target, interface and parameters are needed from the user.
   - Then build one isolated HLS prototype of a chosen step variant: plain chord, nearest-entry fixed-point index, or interpolated chord. Its cycle estimates differ (10×100 is about 39.4 ms for the plain chord but about 43.3 ms interpolated), so quote no estimate without its variant.
   - Then run C simulation against the reference model, co-simulation, synthesis and timing before choosing a PSO size.
7. **Separate experiments afterwards** (do not mix them into the configuration choice):
   - control-weight sensitivity (MATLAB uses 0.02·u² vs R = 0.0005 here);
   - horizon length;
   - plant-mismatch and disturbance tests;
   - two-evaluator feasibility (HLS/Vivado, memory ports, timing) if more evaluations are needed.
8. Physical-actuator tests stay gated until real-time feasibility and the mismatch/safety tests are done.
9. The old Create 3 worktree (`.claude/worktrees/agent-a030184b931670f3b`, partial, based on an old commit) stays until the user confirms it can be removed.

### Board state (`ubuntu@10.42.0.29`; the user runs all sudo steps)

- **Boot:** booted 2026-10-25 (boot ID `8c89b500-4bdb-4c05-8624-38105689d692`). The user restored fix6 at 18:35Z; after the angle-wrap re-validation the PL holds **overlay-3x24-wrap** (bit `ac0c8c37c3ca`, programmed 19:14:48Z); the marker is present. `sudo bash ~/nmpc_fsmfix/step_prep.sh` (default `overlay-fix6`) restores fix6.
- **The power-cycle procedure is now mandatory, not optional:** power cycle → `sudo bash ~/nmpc_fsmfix/step_prep.sh` → notebook. Without `step_prep.sh` the guard refuses with a clear error instead of freezing the board.
- **Installed in the notebook tree** (`/root/jupyter_notebooks/nmpc_hls/app/kria-pynq/`, via `sudo bash ~/nmpc_fsmfix/hil_trials/archive_and_install.sh`):

  | File | SHA-256 prefix |
  | --- | --- |
  | `kv260_nmpc_demo.ipynb` | `e3c4c5cd3819` |
  | `kv260_nmpc_demo.py` | `0ffc6b53441a` |
  | `quadrotor.py` | `b2ea56acd503` |
  | `nmpc_overlay.py` (guard, manifest PSO size, safe buffer) | `6b49a985468b` |

  The script also packs run folders into `~/nmpc_fsmfix/runs/` for copying without sudo.
- **`~/nmpc_fsmfix/`:**
  - `nmpc_overlay.py` (`6b49a985468b`, as committed in `494e033`) is used by `step_prep.sh`, `step4_run.sh` and `step5_validate.sh`.
  - `step_prep.sh` logs the marker after programming (or `MISSING`).
  - `step_prep.sh [overlay_dir]` and `step5_validate.sh [overlay_dir]` take the overlay directory (default `overlay-fix6`); `step4_run.sh` and `step6_promote.sh` are still fix6-only.
  - `step5_validate.sh` saves only the fitness words actually written.
  - Staged overlays: `overlay-fix6` (promoted), and `overlay-3x24` and `overlay-3x24-wrap` (validated, not promoted).
  - `archive_and_install.sh` reinstalls the notebook when the board copy was edited (for example `overlay_dir`), keeping the edited copy as `kv260_nmpc_demo.ipynb.bak_<time>`.
  - Backups: `*.bak_premarker`, `*.bak_pre3x24`, `*.bak_maxbuf`.
  - These board-only scripts are not in the repo.
- **SSH from the PC:** the key has a passphrase. Use the agent the user starts at `SSH_AUTH_SOCK=/run/user/1000/ssh-agent.socket`; it does not survive a logout.

### Repository state

- **Committed** on the local branch `origin/build`, and **pushed by the user** to the GitHub branch literally named `origin/build` (`refs/heads/origin/build`; verified with `git ls-remote` on 2026-10-25 at `4be555b`). The remote's separate `build` branch (`fbb8878`) is older and unrelated. Committed: everything through the 3×24 commits `c00ff1c` (HLS), `494e033` (host) and the following docs/run-record commit. **`config.hpp` is now 3×24**; a fix6 rebuild needs `_n_S 10`, `_maxiter 100`.
- **Not to be committed:**
  - `graphs/nmpc_pso_system_overview.svg` (unrelated user change);
  - `hw/hls/ip_backup_2026-10-2*`;
  - the Vitis HLS and Vivado logs (`hw/hls/logs/`, build folders);
  - raw board downloads `app/kria-pynq/kv260_nmpc_demo_kv260*.ipynb` and `kv260_nmpc_demo_overlay_3x24.ipynb` (their content is in the dated copies);
  - the `.claude/` worktrees.
- **Run data (gitignored):** `app/kria-pynq/validation_logs/` holds:
  - the board runs: `20261023T…`, `20261025T143336Z_rk4_100x1`, and the 3×24 runs `20261025T173547Z_rk4_100x5` and `20261025T173937Z_rk4_100x5`;
  - the 3×24 validation dumps and prep logs in `3x24_validate/`;
  - `pc_screen/`: every `half`/`float` trial as JSON and NPZ, logs `screen_half*.log`.

### Hazards added since the last handoff

- **Never touch PL registers unless the PL holds the matching design in this boot.** The access hangs the whole SoC: no network, no serial; only a power cycle recovers it. The guard enforces this for `NmpcOverlay`; any other tool that maps `0xA000_0000` needs the same check.
- **The hardware RNG restarts from the same state when the PL is programmed:** the first solve after `step_prep.sh` was identical in two runs. Keep this in mind when comparing runs; PC and board still compare as distributions.
- **Under 12 parallel jobs a `half` solve takes about 2.5× its single-process time** (10×100: 144 s). Size long screens accordingly.
- **The overlay manifest's `solver` entry describes the bitstream, but the XSA does not record it:** pass the build's `_n_S`/`_maxiter` to `package_overlay.py` exactly. `NmpcOverlay` refuses out-of-range metadata and any `ap_return` mismatch, and its output buffer always holds 128 iterations.
- **The Vivado build is memory-hungry:** the user runs it in a separate terminal (3 jobs). The wrapper refuses an existing project folder; delete a partial one first.
