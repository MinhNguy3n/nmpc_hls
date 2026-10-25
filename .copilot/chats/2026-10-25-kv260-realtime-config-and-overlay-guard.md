# KV260 NMPC: Real-Time Configuration Screening and Overlay-Load Guard

## Handoff for the next agent (current state, 2026-10-25)

This continues [2026-10-23-kv260-fsm-done-latch-fix.md](2026-10-23-kv260-fsm-done-latch-fix.md). That file still holds:

- the eight hardware fixes;
- the board scripts and the safe board procedure;
- the build flow and the hazards;
- the fix6 closed-loop HIL baseline;
- the first two PC screens (to 2026-10-24).

This file records what changed since then and what to do next.

### Status in one paragraph

**fix6** (bit `300bd966…`) is the validated, promoted baseline and stays untouched. It solves 10 particles × 100 iterations in **0.523 s**, against a 50 ms sample time. The goal now is one rebuild with a configuration that meets the deadline. The PC screen in the exact hardware arithmetic (`half` mode) is calibrated against the board. Across all three screens, **3 particles × 24 iterations (3×24)** is the only setting that meets both:

- the agreed quality tolerance;
- the 40 ms target (50 ms with 20 % margin), including the measured PS-side overhead.

It meets both only narrowly. Next steps:

1. Run 3×24 for 10 seeds on the PC.
2. If it still passes, do one rebuild (`_n_S 3`, `_maxiter 24`).
3. Measure the real latency on the board.

Physical-actuator tests stay gated.

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

### Next steps (priority order)

1. **Extend 3×24 to 10 seeds** in `half` mode (about 30 min on 16 cores; seeds 6–10 reuse the saved 1–5):

   ```bash
   cd app/kria-pynq
   python3 screen_pso_config.py --mode half --configs 3x24 --seeds 1-10 --steps 100 \
       --jobs 10 --out validation_logs/pc_screen --board validation_logs/20261023T220440Z_rk4_100x5
   ```

   Run it as a session background task. **Lingering is disabled on the PC and must stay disabled; ask the user before changing it.** The user keeps the session open for long runs.
2. **If 3×24 still passes, rebuild once:**
   - set `_n_S 3`, `_maxiter 24` in `hw/hls/source/include/config.hpp` (CRLF; `_slope` follows `_maxiter` automatically);
   - check every HLS pragma depth and the BRAM layout that depends on `_n_S`/`_maxiter` (`pso_maxiter` depths; `bestfitness` region size in `nmpc_overlay.py` `MAX_ITERATIONS`, `HORIZON`, and the notebook);
   - rerun the three worker co-simulations;
   - rebuild the IPs and the integrated design (flow in the 2026-10-23 file, "Reproducing the build"); check timing; package; stage as a new overlay directory;
   - validate on the board as before (`step5_validate.sh`, `validate_hw_run.py`);
   - measure the real latency, then the closed loop with the notebook.

   `nmpc_overlay.py` and the notebook hard-code `MAX_ITERATIONS = 100` and "10 particles, 100 iterations": parameterize them before the board run.
3. Keep fix6 as the promoted overlay until the new build is validated; promote only with a rollback copy.
4. **Separate experiments afterwards** (do not mix them into the configuration choice):
   - control-weight sensitivity (MATLAB uses 0.02·u² vs R = 0.0005 here);
   - horizon length;
   - plant-mismatch and disturbance tests;
   - two-evaluator feasibility (HLS/Vivado, memory ports, timing) if more evaluations are needed.
5. Physical-actuator tests stay gated until real-time feasibility and the mismatch/safety tests are done.

### Board state (`ubuntu@10.42.0.29`; the user runs all sudo steps)

- **Boot:** booted 2026-10-25 (boot ID `8c89b500-4bdb-4c05-8624-38105689d692`). The PL holds fix6, programmed by `step_prep.sh` at 15:24:16Z; the marker is present.
- **The power-cycle procedure is now mandatory, not optional:** power cycle → `sudo bash ~/nmpc_fsmfix/step_prep.sh` → notebook. Without `step_prep.sh` the guard refuses with a clear error instead of freezing the board.
- **Installed in the notebook tree** (`/root/jupyter_notebooks/nmpc_hls/app/kria-pynq/`, via `sudo bash ~/nmpc_fsmfix/hil_trials/archive_and_install.sh`):

  | File | SHA-256 prefix |
  | --- | --- |
  | `kv260_nmpc_demo.ipynb` | `49c4ba5ae7b0` |
  | `kv260_nmpc_demo.py` | `be665c3c6c07` |
  | `quadrotor.py` | `b2ea56acd503` |
  | `nmpc_overlay.py` (guard) | `07d3b76f684b` |

  The script also packs run folders into `~/nmpc_fsmfix/runs/` for copying without sudo.
- **`~/nmpc_fsmfix/`:**
  - `nmpc_overlay.py` is the guarded version (`07d3b76f684b`), used by `step_prep.sh`, `step4_run.sh` and `step5_validate.sh`.
  - `step_prep.sh` now logs the marker after programming (or `MISSING`).
  - Backups: `*.bak_premarker`.
  - These board-only scripts are not in the repo.
- **SSH from the PC:** the key has a passphrase. Use the agent the user starts at `SSH_AUTH_SOCK=/run/user/1000/ssh-agent.socket`; it does not survive a logout.

### Repository state

- **Committed:** everything up to `6b12d8b` (HIL baseline, first and second PC screens, tolerance; all on `origin/build`, not pushed).
- **Uncommitted, waiting for the user's go-ahead:**
  - `app/kria-pynq/nmpc_overlay.py` (overlay-load guard);
  - `app/kria-pynq/kv260_nmpc_demo.ipynb` and `.py` (PS-overhead timing: `solve_call_seconds`, `ps_overhead_seconds`, per-trial median/max, run-level `ps_overhead_s`);
  - the pointer line in the 2026-10-23 handoff, this file, and the chats README;
  - board downloads `app/kria-pynq/kv260_nmpc_demo_kv260.ipynb` (100-step overhead run) and `kv260_nmpc_demo_kv260_10_steps.ipynb` (smoke test), to be kept as dated records.
- **Not to be committed:** `graphs/nmpc_pso_system_overview.svg` (unrelated user change) and `hw/hls/ip_backup_2026-10-23_*`.
- **Run data (gitignored):** `app/kria-pynq/validation_logs/` holds the board runs (`20261023T…`, `20261025T143336Z_rk4_100x1`) and `pc_screen/` (every `half`/`float` trial as JSON and NPZ, logs `screen_half*.log`).

### Hazards added since the last handoff

- **Never touch PL registers unless the PL holds the matching design in this boot.** The access hangs the whole SoC: no network, no serial; only a power cycle recovers it. The guard enforces this for `NmpcOverlay`; any other tool that maps `0xA000_0000` needs the same check.
- **The hardware RNG restarts from the same state when the PL is programmed:** the first solve after `step_prep.sh` was identical in two runs. Keep this in mind when comparing runs; PC and board still compare as distributions.
- **Under 12 parallel jobs a `half` solve takes about 2.5× its single-process time** (10×100: 144 s). Size long screens accordingly.
