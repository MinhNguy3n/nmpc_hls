# KV260 NMPC FSM Stall: Root Cause and Fix

## Session metadata

- Created: 2026-10-23 (Claude Code)
- Continues: [2026-08-18 KV260 NMPC ILA Overlay Debug](2026-08-18-kv260-ila-overlay-debug.md), [2026-09-13 summary](2026-09-13-kv260-ila-overlay-debug-summary.md)
- Target board: KV260, now at `ubuntu@10.42.0.29` (Xilinx MAC `00:0a:35:0f:24:00`). The SD card was reimaged: the SSH host key changed and key-based login is not yet installed.
- Toolchain: only Vivado/Vitis **2025.2** is installed now (`/tools/Xilinx/2025.2`). The `tcl_run/nmpc_solver_v2` project was built with 2025.1 and was left untouched.

## Root cause (verified in shipped RTL)

The FSM never leaves `wait_initialize` (state 2). The worker done inputs of `pso_fsm` were by-value `ap_none` scalars, so Vitis HLS sampled them **once, at `ap_start`**:

- `hw/hls/ip_backup_2026-10-23_pre_fsmfix/nmpc_solver_fsm/hdl/verilog/pso_fsm.v`: `init_s_ap_done`, `eval_s_ap_done`, `detect_min_ap_done` and `update_s_ap_done` are latched only under `if (ap_CS_fsm_state1)`.
- The `do { … } while(flag)` back-edge goes `state75 -> state2` and never revisits `state1`.
- `init_s_ap_done` is 0 at start, so `case 2` loops forever.

The workers use plain `ap_ctrl_hs`: `ap_done` is a combinational one-cycle pulse. The outer FSM iteration is at least 2 cycles, so a merely volatile `if (done)` could still miss it.

## Why the earlier evidence pointed at state 0

`debug_state` (AXI-Lite `0x18`) was useless for live observation. HLS collapsed the per-iteration writes into one `debug_state_ap_vld` at loop exit (`ap_CS_fsm_state75 & flag == 0`), so it read 0 for the whole solve, whatever the actual state. `fsm_phase: copy_inputs` was a measurement artifact. The ILA captures (the HP0/SmartConnect read path is healthy) remain valid.

Also note that commit `6e07abc` silently reverted the `debug_state` instrumentation from `hw/hls/source/src/hls_pso.cpp`, while the deployed IP still had it.

## Fix applied (`hw/hls/source/src/hls_pso.cpp`)

- Done inputs are now `volatile bool &`.
- Each wait state polls in `while (!done) { #pragma HLS PIPELINE II=1 }`. The generated `pso_fsm_Pipeline_VITIS_LOOP_*` modules read the raw input every clock.
- `debug_state` is restored as `volatile uint32_t &` on `s_axilite`. `debug_state_ap_vld` now fires at every loop head (`ap_CS_fsm_state3`), so `0x18` is live.
- C synthesis: 7.3 ns estimated at a 10 ns target; port list identical to the deployed IP, so the BD needs no changes.
- New IP is installed in `hw/hls/vitis_ip_repo/nmpc_solver_fsm{,.zip}`. The previous IP is in `hw/hls/ip_backup_2026-10-23_pre_fsmfix/`.

## Rebuild

- A fresh 2025.2 project is generated from `hw/hdl/nmpc_solver_v2_debug.tcl` (ILAs included) at `hw/hdl/build/nmpc_solver_v2_fsmfix`.
- Outputs go to `hw/exported_platform/nmpc_solver_v2_fsmfix/` (`nmpc_solver_v2.xsa`, `debug_nets.ltx`, `timing_summary.rpt`).
- Backup of the pre-fix ILA bitstream, probes and captures: `hw/hdl/tcl_run/nmpc_solver_v2/backup_2026-10-23_ila_debug_pre_fsmfix/`.
- **Build completed 2026-10-23.** synth and impl through `write_bitstream` passed. Timing met: WNS +0.951 ns, WHS +0.010 ns, 0 failing endpoints.
  - Bitstream: `hw/hdl/build/nmpc_solver_v2_fsmfix/nmpc_solver_v2_fsmfix.runs/impl_1/design_nmpc_solver_wrapper.bit`, SHA-256 `296af6a1cc3a85e98d2c232126447e6c6e356531e59aa9f1b249dfea363d9076`
  - Probes: `hw/exported_platform/nmpc_solver_v2_fsmfix/debug_nets.ltx`, SHA-256 `c30e9070c181854e6cd163014e93da3fba5b5af54dc43c0c1c36c98e92ed820b`. Both ILA cores are present (`ila_0` on HP0, `nmpc_solver/ila_0` on `pso_fsm_1/m_axi_nmpc_io`).
  - The synthesized `pso_fsm_1` contains the new `Pipeline_VITIS_LOOP_*` done-poll modules.

## Board bring-up log (2026-10-23)

- This is the same SD image as August; only the SSH host key changed. Board IP is `10.42.0.29`, and sudo needs a password (the user runs the root steps).
- Staging folder: `~/nmpc_fsmfix/`, holding `overlay-fsmfix/`, `nmpc_overlay.py`, step scripts and `logs/`.
- `xmutil unloadapp` succeeded: the starter dtbo was removed, and the board stayed stable for 2 min (240/240 pings).
- Programming `overlay-fsmfix` via PYNQ `download=True` **with the starter app unloaded** succeeded in 5 s. `ap_ctrl=0x4`, the board stayed healthy, and JTAG read back ILA UUIDs `3EF00B0E…` (HP0) and `3440A60D…` (FSM) matching the new LTX.
- **About 2 min after a Vivado HW Manager JTAG session** (connect at 1 MHz, refresh_hw_device, disconnect), Linux hit `watchdog: BUG: soft lockup - CPU#3 stuck … [sshd]` and dropped off the network. Suspected cause: the known ZynqMP issue where JTAG debug access combined with cpuidle hangs Linux. This likely also explains the August "crash after programming" episodes, during which Hardware Manager was connected.
- Mitigation: `step_prep.sh` disables cpuidle states other than state0 through sysfs before any JTAG access. The change does not persist across reboot. If that is not enough, add `cpuidle.off=1` to the kernel cmdline.

## First hardware run of the FSM fix (2026-10-23 14:07 board time)

- Prep: power cycle, then `step_prep.sh` (`xmutil unloadapp`, cpuidle `state1` disabled on all CPUs, PYNQ program). It was stable, and stayed stable for more than 10 min with a JTAG session attached and ILAs armed. The cpuidle mitigation appears effective.
- `step4_run.sh` (bounded run): live `debug_state` now works. The state went 0 → 2 immediately and stayed in **state 2 (`wait_initialize`) for 120 s**. `new_best` and `bestfitness` still held their sentinel values, and neither ILA triggered (no AW on `nmpc_io`).
- **Second root cause: random-stream deadlock.** `rand_wrapper_0` (free-running, started once) → `axis_broadcaster_0` → M00 to `initializeParticles_0/p_rand_port` and M01 to `updateParticlesWithD_0/p_rand_port`. The AXI4-Stream broadcaster only advances when **all** outputs are ready. `updateParticles` does not run during state 2, so the stream stalls and `initializeParticles` blocks forever on `__rand_port.read()` (`hls_pso.cpp:851`). The old FSM hung one step earlier (the frozen done sample), which masked this.
- The `*_ap_start` outputs were verified in RTL: they are set in the start state and held via `_preg` until after the poll loop, so the workers are started correctly.

## Second fix set (rebuild `nmpc_solver_v2_fix2`)

- `hw/hdl/src/axis_rand_fanout.v`: a one-generator, any-ready fan-out with `S_TREADY = M0_TREADY | M1_TREADY`, replacing `axis_broadcaster_0` in `nmpc_solver_v2_debug.tcl`.
- All 21 worker and FSM local-BRAM segments moved from `0xC0000000` to `0x00000000`. The `offset=off` ports (`x_mem`, `y_mem`, `v_mem`) start at 0, and the `offset=direct` constants are raw offsets. PS-side and HP0 maps are unchanged.
- `mem_bank` offsets respaced: `x_curr_local` 40→16, `xref_local` 40→64, `last_best` 440→1344, `local_uss` 120→200, `find_local` 0→512.

## Run of `overlay-fix2` (2026-10-23 14:44 board time, no JTAG)

- Bitstream SHA-256 `8ac18602158195702069da2ec7eacbe4e8845224e537fc66f94fdd4b8e86cb23`. Timing: WNS +0.903 ns, WHS +0.010 ns.
- The board hung once while idle with the previous overlay (between 14:12 and about 14:22, after the JTAG session was force-stopped). No UART log was captured.
- Result: state 2 → 3 → 4 within 5.6 ms. **The random-stream deadlock is fixed and `initializeParticles` completes.**
- New stall: **state 4 (`wait_evaluate`) for 120 s**. `eval_s_ap_start` is verified in RTL to be held high from state 3 through the poll, so `evaluateFitnessAndDetectLocalBest` is started but never asserts `ap_done`. Its csynth latency is about 3.8 ms (bounded), so an internal dataflow/FIFO deadlock in `nmpc_cost_function_topflow` (`hls_system.hpp`, FIFO depths `_system_Nx`/`_system_n_U`) or a stuck m_axi transaction is suspected.
- Next diagnostic: a standalone C/RTL cosim of the evaluator (no board needed). Its testbench `tb_eval.cpp` is in the scratch area and is not yet in the repo.

## Evaluator FIFO-imbalance fix and co-simulation (2026-10-23)

- Root cause of the state-4 hang: `nmpc_cost_function_topflow` (no DATAFLOW) used `#pragma HLS stream` arrays that ended each call holding 12 (`x_current`) and 4 (`uu_fifo`) words, so the next call blocked on a full FIFO. C simulation cannot show this.
- Fix (user-applied, structure corrected): the last horizon step skips `get_next_uu` and the `x_next`→`x_current` copy, and drains `x_next` into a `volatile` sink. The terminal cost is unaffected (`one_step_error` uses `Qf` at `i_step == N-1`).
- New repo testbench `hw/hls/source/src/main_hls_evaluate.cpp` (two calls, self-checking) and script `hw/hls/scripts/hls_script_evaluate_cosim.tcl`, run from `hw/hls`.
- Co-simulation result: **the RTL completes both calls with no deadlock** (3.63 ms per call at 8 ns). It **FAILS on values**: the RTL cost is ≥ 65535 (saturated `ap_fixed<32,17>`) for every particle, while the C model gives 5066.
- Why C and RTL disagree: `config.hpp` selects `ap_fixed<32,17,AP_RND_INF,AP_SAT_SYM>` and `half` (with `fastsin`/`fastcos`) only under `__SYNTHESIS__`; C simulation uses `float` everywhere. So C simulation is not a model of the hardware arithmetic. Defining `__SYNTHESIS__` for C simulation breaks `hls_math.h`.
- Next: add a project macro (e.g. `HW_ARITH_CSIM`) that selects the hardware types and fast sin/cos in C simulation, then find which term saturates (suspects: half overflow, the 1e4 out-of-limits penalty, fast sin/cos near 0).

## C vs RTL bisection of the evaluator cost (2026-10-23)

- Added an opt-in `-DHW_ARITH_CSIM` switch to `config.hpp`, `models/hls_arith_aux_functions.hpp` and `models/fast_sin_cos.hpp` (CRLF preserved). It selects the synthesized types, `H_MAX` and fast sin/cos in C simulation. Limitation: the `half` model does not compile in C simulation because the C++ `half` emulation has no `volatile` overloads and `__model` takes `volatile` ports. The repo cosim script therefore does not set the flag.
- Scratch bisection (C simulation, testbench `main_hls_evaluate.cpp`):

| Variant | Model | Top | sin | Cost |
| --- | --- | --- | --- | --- |
| float | float | float | math.h | 5066 |
| A | float | ap_fixed<32,17> | fast | 5066 |
| B (volatile stripped) | half | ap_fixed<32,17> | fast | 5066 |
| RTL cosim | half | ap_fixed<32,17> | fast | ≥ 65535 (saturated) |

- **Conclusion: the number formats and fastsin are not the cause.** The divergence is RTL-only. Suspects: `#pragma HLS stream` arrays (including `type=shared` in `one_step_prediction`) whose RTL access semantics differ from C array semantics (see HLS warning 200-1614), and `volatile` port handling in `__model`/`model()`.
- Minor bug found: `fastsin` indexes `table[1024]` (out of bounds) when the scaled angle is a negative multiple of 1024. `fastcos` double-scales its argument but is unused (the model computes cos as sin(x + π/2)).
- The existing harnesses are stale: `main_hls_model.cpp` redefines `_hw_top_real` (clash with `config.hpp`), and `cost_function_wrapper` fails scheduling (`SCHED 204-42`, FIFO `control_guess` random read).
- **Model probe** (a scratch top replicating `System::model()`: ap_fixed → volatile half arrays → `__model<half>` → ap_fixed): **the RTL returns NaN for every computed acceleration (`state_dot[6..11]`)** whenever the control is nonzero or the angles are nonzero, while C gives finite values. Pass-through outputs (`state_dot[0..5]`) also lose precision in RTL (0.2 → 0.2031, 0.1 → 0.1016, about 4 significant bits).
- A standalone ap_fixed ↔ half conversion probe is **correct in RTL**, so the conversions are not the cause.
- P1–P5 (pass-through wrapper, non-volatile, pragmas removed, explicit casts, casts via float) all showed the same NaN and precision loss.
- **Correction: the NaN, precision-loss and saturated-cost results were a cosim testbench artifact.** Without `HW_ARITH_CSIM`, C simulation uses `float` for `_hw_top_real`, while the RTL ports are `ap_fixed<32,17>`. Cosim passes the raw 32-bit patterns, so both sides misinterpret every value. Check: float 0.2 (`0x3E4CCCCD`) read as ap_fixed is 31897.6; half rounds it to 31904; that returns as `0x3E500000`, which is float **0.203125**, exactly the observed "0.2031". The scalar conversion probes were correct only because their testbench used `-DHW_ARITH_CSIM`.
- Fix: under `HW_ARITH_CSIM` (non-synthesis), `config.hpp` now keeps the top-level type `ap_fixed` (matching the RTL ports) and the model in `float` (avoids the C++ `half`/volatile compile problem). `hls_script_evaluate_cosim.tcl` now sets `-DUSE_FAST_SIN_COS -DHW_ARITH_CSIM` for the testbench.
- The deadlock result stands: both evaluator calls completed in RTL.
- **Rerun with matching types: evaluator C/RTL cosim PASS.** Both calls complete (3.66 ms each), all 10 particles cost 5066 in RTL, identical to C, and `TB_PASS`. The model probe RTL (half) matches C within about 0.3%, with no NaN. **The evaluator is validated.**
- The `nmpc_solver_costF` IP was rebuilt with HLS 2025.2 (same VLNV) and installed in `vitis_ip_repo`. Previous IP: `hw/hls/ip_backup_2026-10-23_pre_costFfix/`. HLS estimates 10.624 ns against the 8 ns target, versus 6.096 ns for the old 2025.1 build. The critical path is in `__model<half>` (unchanged code), so the change is tool-version related. Check Vivado timing at 100 MHz.
- Integrated bitstream build `hw/hdl/build/nmpc_solver_v2_fix3` (FSM fix, random-stream fan-out, BRAM map/offsets, evaluator FIFO fix) **completed**. Timing met: WNS +0.833 ns, WHS +0.006 ns, no critical warnings, so the HLS 10.6 ns estimate was pessimistic.
  - Bitstream SHA-256 `ca59b412fa0cfc5a7ca88ef19ea8da51754dcd9d937813f7c2073165a1a83d9d`. The LTX is unchanged (`c30e9070…`).
  - Packaged as `app/kria-pynq/overlay-fix3/`, staged on the board at `~/nmpc_fsmfix/overlay-fix3/`. `step_prep.sh` and `step4_run.sh` now point to it.

## Run of `overlay-fix3` (2026-10-23 16:41 board time)

- States 2 → 4 → 6 at 4.5 ms and 9.5 ms: **initialization and the evaluator now work on hardware.** The FSM then stuck in **state 6 (`wait_detect_minimum`)** for 120 s; sentinel values untouched.
- Root cause: `detectGlobalMinimum`'s `interaction` input was `ap_vld register`, wired to `pso_fsm_1/k` and `k_ap_vld`. The FSM pulses `k_ap_vld` for one cycle only when `k` is written (at FSM start and in state 8). Detect latches the pulse (`interaction_ap_vld_preg`) only while not idle, and it is always idle at that moment. So detect waits forever in its state 2, on every iteration.
- Fix (user choice): `#pragma HLS INTERFACE mode=ap_none port=interaction` in `hls_pso.cpp`, and the `pso_fsm_1_k_ap_vld` net removed from `nmpc_solver_v2_debug.tcl`. `k` is a held register output, stable when detect starts.
- `updateParticlesWithDuConstrains` has no handshake inputs beyond `ap_start` and the random AXIS port. Note: its inertia weight `static w` is never reset between solves (affects result quality, not hangs).
- Cosim support: `update`'s random port is now gated on `__SYNTHESIS__ || HW_ARITH_CSIM` (`hls_pso.hpp`, `hls_pso.cpp`). A C-simulation-only overload without the port fills it from `rand_real()`, so the software model compiles unchanged. Synthesis is unaffected.
- New self-checking testbenches and scripts: `main_hls_detect.cpp` / `hls_script_detect_cosim.tcl`, `main_hls_update.cpp` / `hls_script_update_cosim.tcl` (10 ns, matching the IP builds).
- The C-simulation helper is named `updateParticlesWithDuConstrains_swrng` (not an overload): cosim cannot wrap an overloaded top (`apatb_*` conflicting types). The software model calls it only under `HW_ARITH_CSIM && !__SYNTHESIS__`.
- **C/RTL cosim: all three PASS.**
  - detect: both calls write the correct `bestfitness` slot and `global_min` row, and nothing else.
  - update: the RTL checksums equal C exactly on both calls, all constraints hold, and the random stream is fully consumed.
  - evaluator: regression PASS.
- `nmpc_solver_global_minimum` IP rebuilt (`interaction` now a plain input, no `interaction_ap_vld`; 7.3 ns estimated at 10 ns) and installed. Previous IP: `hw/hls/ip_backup_2026-10-23_pre_detectfix/`.
- Integrated build `nmpc_solver_v2_fix4` (all fixes so far) completed. Timing met: WNS +0.301 ns, WHS +0.010 ns, no critical warnings.
  - Bitstream SHA-256 `c79aa9e21b4960b263cd7ca56a67d08f7a7dc125e57610604371c07a7f9a91ee`.
  - Packaged as `app/kria-pynq/overlay-fix4/` and staged at `~/nmpc_fsmfix/overlay-fix4/`. `step_prep.sh` and `step4_run.sh` now point to it.

## Resolved: local-BRAM DECERR map and overlapping offsets (historical)

Original defect: every worker and FSM address space mapped the local BRAMs at `0xC0000000`, while the `mem_bank` direct-offset constants were small raw byte offsets. The SmartConnect MMUs therefore returned DECERR, silently dropping writes and returning zeros on reads. Several offsets also overlapped: `x_curr_local = xref_local = 40`, `local_u_max = local_uss = 120`, `bestfitness = find_local = 0`.

Fixed in `nmpc_solver_v2_debug.tcl` (second fix set) and **verified in the fix4 block design**:
- All 21 local-BRAM segments are at `0x00000000`.
- Constants: `u_curr` 0, `x_curr` 16, `xref` 64, `last_best` 1344 (`current_mem`); `du_max` 40, `du_min` 80, `u_max` 120, `u_min` 160, `uss` 200 (`constraint_mem`); `bestfitness` 0, `find` 512, `global_best` 1280 (`pso_mem`).

## fix4 artifact audit (2026-10-23)

- Build `hw/hdl/build/nmpc_solver_v2_fix4`: timing met (WNS +0.301 ns, WHS +0.010 ns, 0 failing endpoints), no critical warnings.
- Netlist and block design contain the complete fix set:
  - FSM: 4 done-poll loops; `debug_state_ap_vld` in the loop head.
  - Evaluator: the `drain_x_next` FIFO fix.
  - Detect: `interaction` is a plain `input [31:0]` with no `interaction_ap_vld`, and the block design connects only `.interaction(pso_fsm_1_k)`.
  - Random stream: `axis_rand_fanout` present, no broadcaster.
  - Local-BRAM segments and offsets as above.
- Artifacts:

| File | SHA-256 | Staged on the board |
| --- | --- | --- |
| `design_nmpc_solver.bit` | `c79aa9e21b4960b263cd7ca56a67d08f7a7dc125e57610604371c07a7f9a91ee` | identical |
| `design_nmpc_solver.hwh` | `154fb64b91ef9dde1df86c3604d054a3a43a14502343d3cbf63f133033228764` | identical |
| `design_nmpc_solver.bda` | `323c0c1fe87320104050af3a377d26e06cebd7298bef8bc53d0e48a73d6da41b` | identical |
| `debug_nets.ltx` | `c30e9070c181854e6cd163014e93da3fba5b5af54dc43c0c1c36c98e92ed820b` | host only (Vivado HW Manager) |

- The board manifest's `source_xsa` is `hw/exported_platform/nmpc_solver_v2_fix4/nmpc_solver_v2.xsa`. Use only `overlay-fix4`: `overlay-fsmfix`, `overlay-fix2` and `overlay-fix3` are superseded.

## Run of `overlay-fix4` (2026-10-23 17:41 board time): FIRST COMPLETE SOLVE

- Prep: power cycle, then `step_prep.sh` (unload OK, cpuidle `state1` off, programmed in 5.88 s, `ap_ctrl=0x4`). The board was stable throughout. No JTAG session.
- Bounded run (`step4_run.sh`, inputs `x_curr=[7,10,0,…]`, `xref=0`, `u_curr=0`, `last_best=0`):
  - Sampled states: 2 → 4 → 6 → 8, then repeated 4/6/8 (3, 5 and 7 are too short for the ~1 ms poll), then **9 (`write_outputs`)** at t = 0.4987 s.
  - **`FINISHED`, `ap_ctrl=0xe`** (done, idle, ready), **`ap_return = 100` iterations**. Solve time ≈ 0.50 s, about 5 ms per iteration (S = 10 particles, Nh = Nu = 25).
  - `new_best` and `bestfitness` sentinels are fully overwritten (0/100 untouched each).
  - `bestfitness` is non-increasing: 4870.74 → 4868.34 → 4842.97 → … → 4554.46.
  - `new_best` stays within ±100, and every step changes by at most `du_max = 20` from the previous control (first row from `u_curr = 0`: `[20, 20, -19.92, 20]`).
- **All hardware hangs are resolved.** Root causes fixed in this session:
  1. FSM done inputs sampled once at `ap_start`, and one-cycle `ap_done` pulses (volatile + II=1 polling).
  2. Random-stream broadcaster deadlock (any-ready fan-out).
  3. Local-BRAM DECERR map and overlapping offsets.
  4. Evaluator FIFO imbalance across calls (drain on the last horizon step).
  5. Detect `interaction` `ap_vld` handshake that could never complete (`ap_none`).

## Next steps

1. Commit the source, testbench and BD changes (not yet committed). Generated build directories and backups stay out of git.
2. Make `overlay-fix4` the default overlay for `nmpc_overlay.py` and the notebook (`/root/jupyter_notebooks/nmpc_hls/app/kria-pynq/overlay`).
3. Validate results numerically against the C software model for the same inputs (compare `new_best` and the final `bestfitness` within fixed-point/half tolerance).
4. Reset or parameterize `update`'s `static w` per solve before closed-loop (multi-solve) use.
5. Performance: about 5 ms per iteration at 100 MHz. Profile the evaluator (dominant, about 4.8 ms) before comparing with the paper's reported timings, which used a different configuration (S = 5, kmax = 25 for the drone).
6. Board procedure for any PL change remains: power cycle, `xmutil unloadapp`, cpuidle `state1` off, program. Avoid long JTAG sessions.
