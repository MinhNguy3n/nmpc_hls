# KV260 NMPC FSM Stall: Root Cause and Fix

## Handoff for the next agent (current state, end of 2026-10-23)

Read this section first. Everything below it is the chronological log, kept for evidence.

### Status

- **The hardware solver works end to end on the KV260.** The current design is **fix6**: 100 PSO iterations in about 524 ms, `ap_done` asserts, results are written back, and it passed scripted board validation (`VALIDATION PASS`, see "Board validation record" and the fix6 run table under "fix6 build, validation, promotion and commits").
- **fix6 is promoted** as the notebook's default overlay (`/root/jupyter_notebooks/nmpc_hls/app/kria-pynq/overlay`), status VALIDATED. Scope: implementation consistency and numerical fidelity to the model equations; the true ±π/4 tilt constraint holds on the reference trajectory. **Physical plant validation is still pending.**
- Eight defects were found and fixed, each in its own commit: five that hung the FSM or silently corrupted data, the inertia-weight drift across solves, and two model-fidelity defects (sine table, tilt margin):

| Commit | Fix |
| --- | --- |
| `f02b57c` | FSM sampled the worker `ap_done` inputs once at `ap_start` → volatile, II=1 polling; live `debug_state` (AXI-Lite `0x18`) |
| `40a6719` | Random-stream broadcaster deadlock → any-ready fan-out `hw/hdl/src/axis_rand_fanout.v` |
| `664899d` | Local BRAMs mapped at `0xC0000000` (DECERR) and overlapping offsets → segments at `0x0`, offsets respaced |
| `5bf5ab5` | Evaluator FIFOs left full across calls → drain on the last horizon step (`hls_system.hpp`) |
| `61ba9ec` | `detect`'s `interaction` `ap_vld` handshake could never complete → `ap_none` |
| `842989b` | `update`'s `static w` drifted across solves → `w = w0 + slope * iteration`, wired from FSM `k` |
| `c102f6c` | Fast-sin table had 1023 samples of a 1022-sample period, indexed as 1024 → regenerated 1024 entries, rounded index, masked wrap |
| `4039b30` | Roll/pitch cost limits tightened by `_angle_limit_margin = 0.01` rad (`config.hpp`) |

  Supporting commits: `ab8c7a4` (`HW_ARITH_CSIM`), `307d261` (C-sim `rand_real`), `c05e7e8` (worker cosim testbenches), `f74e9cf` and `5760206` (validator), `8b569ab` (model unit tests), `79a539e` and `b8370ed` (docs). All are on branch `origin/build`, **not pushed**.

### Board state (`ubuntu@10.42.0.29`, sudo needs the user's password, so the user runs root steps)

- The overlay staged and promoted is `~/nmpc_fsmfix/overlay-fix6`: bit `300bd96693f57d02f6a7700e7797073175f785cd18b0dd989cd533aae3066b99`, hwh `386639a3dfc133dc8fbf962197c265a919febb1c10bb8d18bd4739ff020be4f7`, bda `323c0c1f…`, XSA `hw/exported_platform/nmpc_solver_v2_fix6/nmpc_solver_v2.xsa` (`5e10800e…`), LTX `c30e9070…` (host only).
- **Rollback caveat:** `overlay.rollback-20261023-194742` holds **fix5** (sine-table defect, can violate the tilt limit: **not safe for control**). `overlay.rollback-20261023-185807` holds the original August overlay, which hangs. Restore fix6 from `~/nmpc_fsmfix/overlay-fix6`. It is suggested to rename the fix5 copy to `overlay.rollback-fix5-UNSAFE` (needs sudo).
- **Board scripts exist only on the board** in `~/nmpc_fsmfix/` (not in the repo); logs go to `~/nmpc_fsmfix/logs/`:
  - `step_prep.sh` (root): `xmutil unloadapp` if the starter app is active; disables cpuidle `state1` on all CPUs (JTAG lockup mitigation, resets on reboot); programs the overlay via `NmpcOverlay(..., download=True)`.
  - `step4_run.sh` (root): one bounded solve with live FSM-state logging and sentinel checks.
  - `step5_validate.sh` (root): runs `step_prep.sh`, then two consecutive `NmpcOverlay.solve()` calls; writes `logs/06_validate_*.json` for `validate_hw_run.py`.
  - `step6_promote.sh` (root): hash-checked copy of `overlay-fix6` to the notebook overlay, with a rollback copy and `PROMOTION.txt`. It has already been run.
- The safe board procedure for any PL change: power cycle → `step_prep.sh` → run. Keep the UART (`screen /dev/ttyUSB1 115200`) open. Avoid long JTAG sessions: Vivado HW Manager connections caused Linux soft lockups; disabling cpuidle mitigated this.

### Reproducing the build (Vivado/Vitis 2025.2 only, `/tools/Xilinx/2025.2`)

- HLS IPs: `hw/hls/scripts/hls_script_pso.tcl` builds all five IPs into the **gitignored** `hw/hls/vitis_ip_repo/`, so rebuilt IPs are not in git. Settings: FSM 10 ns, costF 8 ns, others 10 ns; vendor `tu-dresden` for FSM/costF, `tu-dresden_turun-yliopisto` for the rest.
- Integrated design: source `hw/hdl/nmpc_solver_v2_debug.tcl` with `--origin_dir hw/hdl --project_name <name>` from `hw/hdl/build/`. Clear the 2025.1 incremental checkpoint (`INCREMENTAL_CHECKPOINT {}`, `AUTO_INCREMENTAL_CHECKPOINT 0`), then `update_ip_catalog -rebuild`, `validate_bd_design`, `generate_target`, synth, impl to `write_bitstream`, `write_debug_probes`, `write_hw_platform -fixed -include_bit`. (The wrapper used, `build_fix6.tcl`, was in a temporary scratch area and is not saved.) Then run `app/kria-pynq/package_overlay.py --xsa … --output overlay-<name>`.
- Checks, run from `hw/hls`: `vitis-run --mode hls --tcl scripts/hls_script_{evaluate,detect,update}_cosim.tcl` (all PASS); host unit test `src/main_hls_model_unit_test.cpp` (build command in its header, **-O0**); validator `python3 app/kria-pynq/validate_hw_run.py <dump.json> [--seeds N]`.

### Hazards learned the hard way

- **Cosim needs `-DHW_ARITH_CSIM`** in the testbench flags. Plain C simulation uses `float` top-level types while the RTL ports are `ap_fixed<32,17>`, so the bit patterns are misread (float 0.2 comes back as 0.203125) and correct RTL looks broken.
- **Host g++ 11 miscompiles >64-bit `ap_fixed` intermediates at -O1/-O2.** Build every host tool that uses ap_fixed at **-O0**.
- The C++ `half` emulation has no `volatile` operators: the half model only builds in C from a header copy without `volatile` (the validator does this) with `-DHW_MODEL_HALF_CSIM`, linked to `/tools/Xilinx/2025.2/lnx64/tools/fpo_v7_1/libIp_floating_point_v7_1_bitacc_cmodel.so` (RPATH, `--disable-new-dtags`).
- Several headers (`config.hpp`, `hls_pso.hpp`, model headers) use **CRLF**; keep it when editing.
- J is **discontinuous** at the roll/pitch cost limit (1e4 penalty), so small model differences can change J by about 6000. Classify safety from the plant-limit check (c), never from total cost.
- The double reference (REF) uses the same model equations, so it measures numerical fidelity only, not physical correctness.

### Open items (priority order)

1. **Physical plant validation:** a trusted independent dynamics reference or plant-level test before claiming the controller is correct for the drone.
2. Board hygiene: rename the fix5 rollback copy (above).
3. Performance profiling: about 5.2 ms per iteration, dominated by the evaluator (HLS estimate 10.6 ns against the 10 ns clock, yet Vivado meets timing with WNS +1.07 ns).
4. Optional: commit the board scripts and the Vivado build wrapper into the repo so the flow is reproducible without this note.

## Session metadata

- Created: 2026-10-23 (Claude Code); updated at the end of the session for the next agent.
- Continues: [2026-08-18 KV260 NMPC ILA Overlay Debug](2026-08-18-kv260-ila-overlay-debug.md), [2026-09-13 summary](2026-09-13-kv260-ila-overlay-debug-summary.md)
- Target board: KV260 at `ubuntu@10.42.0.29` (Xilinx MAC `00:0a:35:0f:24:00`; the August notes used 10.42.0.48). It is the same SD image as August; only the SSH host key changed. Key-based SSH login is installed for `ubuntu`; sudo still needs the user's password.
- Toolchain: only Vivado/Vitis **2025.2** is installed (`/tools/Xilinx/2025.2`). The old `tcl_run/nmpc_solver_v2` project (2025.1) was left untouched; all builds since are fresh projects under `hw/hdl/build/nmpc_solver_v2_fix*`.

## Chronological log

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

## Stateless inertia weight (fix5, 2026-10-23)

- Problem (simulated with the synthesized `ap_fixed<32,17>`): `update`'s `static w` decayed by `slope` per call and reset when `w + slope <= wf`. Rounding made w0 = 0.899994, slope = -0.00799561 and wf = 0.100006, so the reset fell at call 101, not 100. Only the first solve after programming used the intended schedule (0.900 → 0.108); each later solve started with w ≈ 0.10–0.12 for k iterations and drifted one iteration further per solve.
- Fix: `update` takes `int iteration` (`ap_none`) and computes `w = w0 + slope * iteration`. The static weight and its decay are removed. The block design wires `pso_fsm_1/k` to both `detectGlobalMinimum_0/interaction` and `updateParticlesWithD_0/iteration`. During state 7/8 of iteration i, `k = i`. The software model passes its loop counter `k`.
- `main_hls_update.cpp` now also checks statelessness: two calls with identical inputs, stream and iteration must give bit-identical outputs. **C/RTL cosim: update PASS** (repeat A = repeat B in RTL; iteration 0 unchanged from before). Detect and evaluator regression PASS.
- `nmpc_solver_update_s` IP rebuilt (7.3 ns estimated at 10 ns, `input [31:0] iteration`). Previous IP: `hw/hls/ip_backup_2026-10-23_pre_wfix/`.
- Build `nmpc_solver_v2_fix5`: timing met, WNS +0.993 ns, WHS +0.010 ns, no critical warnings.
  - The netlist wires `pso_fsm_1_k` to `.interaction` and `.iteration`.
  - Bitstream `3b8ac35e5af4f794b0d0b2682fb5c9490e13b3b3b80a0b919e60f5f86a8323e1`, HWH `bad68dc81fdc088f4719af321a4d9328e88f2810587d26b1d8d53f274a2f420e`, BDA `323c0c1f…` (unchanged), LTX `c30e9070…` (unchanged).
  - Staged as `~/nmpc_fsmfix/overlay-fix5`. `step_prep`, `step4_run`, `step5_validate` and `step6_promote` (with fix5 hashes) target it. **fix5 supersedes fix4.**

## Board validation record (scripted, 2026-10-23)

Both dumps come from `sudo bash ~/nmpc_fsmfix/step5_validate.sh`: a fresh program via `step_prep.sh`, then two consecutive `NmpcOverlay.solve()` calls with identical inputs (`x_curr=[7,10,0,…]`, `xref=0`, `u_curr=0`, `last_best=0`). They are archived in `.copilot/chats/logs/`.

| Dump | Overlay (bit SHA-256) | Prep log | Run | Iterations / time | Hardware J |
| --- | --- | --- | --- | --- | --- |
| `2026-10-23-kv260-fix4-validate.json` (board 06_validate_180001) | fix4 `c79aa9e2…` | 05_prep_175940 | 0 / 1 | 100 / 494.7 ms, 100 / 494.9 ms | 4554.4618 / 4580.2143 |
| `2026-10-23-kv260-fix5-validate.json` (board 06_validate_183929) | fix5 `3b8ac35e…` | 05_prep_183909 | 0 / 1 | 100 / 494.6 ms, 100 / 494.7 ms | 4585.6985 / 4613.5150 |

Host analysis (`app/kria-pynq/validate_hw_run.py`, 20 software seeds), with J(new_best) recomputed by the C evaluator:

| Run | Hardware J | Half model (hardware's model) | Float model | Constraints |
| --- | --- | --- | --- | --- |
| fix4 run 0 | 4554.46 | 4554.60 (0.003 %) | 4556.02 (0.034 %) | OK |
| fix4 run 1 | 4580.21 | 4580.66 (0.010 %) | 4581.09 (0.019 %) | OK |
| fix5 run 0 | 4585.70 | 4585.98 (0.006 %) | 10738.60 (134 %) | OK |
| fix5 run 1 | 4613.52 | 4613.79 (0.006 %) | 10765.17 (133 %) | OK |

- **Implementation consistency: verified.** Against the hardware's own half-precision model, every run is within 0.010 %. Tolerance: ap_fixed<32,17> top level plus half model; the residual comes from the half arithmetic the C emulation reproduces. Bounds, du_max steps and non-increasing bestfitness all hold.
- The half-model C build is currently a scratch build (volatile stripped, linked to `libIp_floating_point_v7_1_bitacc_cmodel`) and not yet in the repo validator.
- **Model fidelity: OPEN.** For the fix5 solutions, the half and float dynamics models disagree by about 2.3× in J. Neither is established as correct for the plant: compare against a higher-precision reference and find the first divergence before deciding.
- Software PSO reference (20 seeds, same inputs): float J 4383–4592 (median 4465); ap_fixed with float model 4387–4606 (median 4486). Hardware J sits at the 90–100th percentile, but these use different model precisions, so this is not yet a like-for-like quality comparison.
- Consecutive solves differ (new_best mean |diff| ≈ 28) because the free-running RNG continues between solves. In fix5 the inertia schedule is identical for every solve.

## Model fidelity: half vs float vs double reference (2026-10-23)

Scratch harness (`scratchpad/fidelity/fidelity.cpp`): it drives `System::nmpc_cost_function_lowflow` step by step, using headers with `volatile` stripped so `half` compiles in C, and links `libIp_floating_point_v7_1_bitacc_cmodel`. It logs the state, objective terms and roll/pitch limit flags per horizon step. Variants: REF (double, std::sin, the high-precision reference; it shares the model equations, so it does not check them), FLT (float, std::sin), DFS (double, fast sin), HWD/HWF (ap_fixed with double/float model, fast sin), HW (ap_fixed with half model, fast sin, matching the hardware).

| Control | REF J | FLT J | DFS J | HWD/HWF J | HW J (hardware) | Roll/pitch outside ±π/4 |
| --- | --- | --- | --- | --- | --- | --- |
| fix5 run 0 `new_best` | 10787.51 | 10787.51 | 10738.41 | 10738.60 | 4585.98 (4585.70) | step 24 in all variants except HW |
| fix4 run 0 `new_best` | 4528.91 | 4528.91 | 4555.43 | 4556.02 | 4554.60 (4554.46) | none |
| zero control | 5066 | 5066 | 5066 | 5066 | 5066 | none |

- **Mechanism.** The only finite state limits are roll and pitch at ±π/4 (0.78540). Outside them, the step error is weighted 1e4 instead of Q/Qf, so J is discontinuous at the limit. Under REF, fix5's solution reaches roll 0.79029 at the final step (24); HW predicts 0.78506, just inside. That single crossing explains the whole 2.3× gap. fix4's solution peaks at 0.78438 (REF), inside the limit.
- **Error budget** (max deviation over the horizon, fix5): fast sin vs REF gives x 0.120 m, roll 0.0047 rad, yaw 0.0068 rad; half vs double gives x 0.0014 m, roll 0.0005 rad. **The fast sin table dominates model error (about 10× half).** Every fast-sin variant departs from REF by step 4. Cause: a 1024-entry table (6.1 mrad step) indexed with `trunc(angle*512/π)`, neither rounded nor interpolated.
- **Plant correctness.** Against the reference, the fix5 hardware solution violates the roll limit by about 0.005 rad (0.29°) at the end of the horizon. PSO drives tilt to the limit, and model error decides which side it lands on. FLT equals REF here, so float with std::sin is a faithful stand-in for the reference in this regime.
- Also noted: `fastsin` can index `table[1024]` (out of bounds) when the scaled angle is a negative multiple of 1024; `fastcos` double-scales but is unused.

## Fast-sin root cause and fix (2026-10-23, decision: table/rounding fix plus margin)

- **Root cause of the model error:** the `fast_cossin_table` held **1023 samples of sin(2πk/1022)**, i.e. `np.linspace(0, 2π, 1023)` (max fit error 8.9e-16), declared `[1024]` so the last slot was 0. `fastsin` indexes it as 2π/1024 per entry, so the phase error grows with the angle: 3.1 mrad at π/2 (every cos is computed as sin(θ+π/2)) and 4.6 mrad near 3π/4. Interpolation could not help, which isolated the table as the cause. Constant rounding (Ts −2.4e-4, R −2.3 %) contributes only about 0.5 mrad.
- Measured on 14 controls (4 board + 10 software), max error vs REF (roll rad / x m):

| Variant | HW (half) | DFS (double) |
| --- | --- | --- |
| old table, truncation (shipped) | 0.00825 / 0.149 | 0.00745 / 0.147 |
| corrected table, truncation | 0.00282 / 0.038 | 0.00210 / 0.036 |
| **corrected table, rounding (chosen)** | **0.00148 / 0.0075** | 0.00070 / 0.0069 |
| corrected table, ap_fixed interpolation | 0.00121 / 0.0025 | 0.00049 / 0.0017 |
| exact sin (reference for the fast-sin path) | 0.00127 / 0.0023 | 0.00049 / 0.0017 |

- **Fix (a), `models/fast_sin_cos.hpp`:** the table is regenerated as sin(2πk/1024), k = 0..1023. `fastsin` rounds to the nearest entry and wraps with `& MASK` (correct for negative indices, removes the `table[1024]` read). Interpolation was not chosen: it adds a second table read and arithmetic in the dominant evaluator loop for less than the half-precision error.
- **Fix (b), `config.hpp`:** `_angle_limit_margin = 0.01` tightens the roll/pitch cost limits to ±(π/4 − 0.01) in both C and hardware.
- **Check:** software PSO with the new hardware arithmetic and margin (10 seeds) gives **0 plant-limit violations under REF**, with peak |roll/pitch| 0.7750–0.7759 against π/4 = 0.7854.
- Note: J remains discontinuous at the (now tightened) cost limit, so a REF-vs-HW J comparison can still jump by about 6000 when solutions sit on that limit. Validation must report plant-limit violations separately from the J gap.
- Rebuild in progress: evaluator cosim plus the `nmpc_solver_costF` IP (previous IP in `hw/hls/ip_backup_2026-10-23_pre_sinfix/`); then integrated build `fix6`.

## fix6 preparation, tests and validator (2026-10-23)

- **fix5 is not promoted** (decision with the review): its sine-table defect can produce a true tilt-limit violation. On the board, `step6_promote.sh` is a refusing placeholder; the fix5 script is kept as `step6_promote_fix5.DISABLED`, and the fix5 files are retained for rollback and comparison.
- **Host-compiler hazard:** host g++ 11 miscompiles Xilinx `ap_private` multi-word paths (>64-bit `ap_fixed` intermediates such as `(ap_fixed)system_pi*2 - x`) at -O1/-O2. UBSan reports shifts of negative values in `ap_private.h`, and the results vary with unrelated flags. **All host tools that use ap_fixed are built at -O0.** Earlier -O1/-O2 numbers were re-derived at -O0; the conclusions are unchanged (board consistency ≤ 0.010 %; fix5 REF plant violation at step 24; new model error ≤ 1.5 mrad).
- **Unit tests** `hw/hls/source/src/main_hls_model_unit_test.cpp` (-O0): table is 1024 entries = sin(2πk/1024) exactly; 21 index/wrap cases (negatives, ±0.5 ties, ±1024, ±2048, 1023.4–1023.6); local_sin error ≤ step/2 (0.00307) for float and double over [−4π, 4π] and 0.00315 for half over the operating range; penalty switch at, just inside and just outside the cost limit, plus the plant-limit classification. **UNIT_PASS.**
- **Synthesized ROMs (new costF IP):** the sine ROM has 1024 words with 0 mismatches against half(sin(2πk/1024)). The limit ROMs hold ±0.775391 for roll and pitch (π/4 − 0.01) and ±1000 elsewhere. HLS stores the upper limits as 21-bit words that are sign-extended to 25 bits and then zero-extended. Evaluator cosim **PASS**. HLS estimate 10.624 ns; latency 483–493 k cycles (about 6 % more for rounding).
- **Validator** `app/kria-pynq/validate_hw_run.py` (with `main_hls_validate.cpp` `trajectory` mode) builds float (REF), hw and half variants (the half build uses a `volatile`-free header copy, `-DHW_MODEL_HALF_CSIM` and the FPO library with RPATH). Per run it reports (a) hardware vs half-model consistency (tolerance 0.1 %), (b) the model gap vs REF (fidelity to the equations, not physical validation), (c) true ±π/4 violations on the REF trajectory, plus cost-limit crossings, control constraints and an optional `--seeds N` software comparison. Exit status is non-zero on any failure of (a), (c) or the control constraints.
- Negative test on the fix5 dump: (a) FAIL (old model vs current sources, as expected), (c) FAIL at step 24 (peaks 0.79029 / 0.79295) → `VALIDATION FAIL`, as intended.

## fix6 build, validation, promotion and commits (record)

1. **fix6 build done** (`hw/hdl/build/nmpc_solver_v2_fix6`): timing met, WNS +1.068 ns, WHS +0.010 ns, no critical warnings.
   - Full fix set verified in the netlist: FSM poll loops, evaluator drain, detect without `interaction_ap_vld`, update `iteration` port, `pso_fsm_1_k` driving `.interaction` and `.iteration`, fan-out present and no broadcaster.
   - Synthesized ROMs: the sine ROM has 1024 words with 0 mismatches; limits ±1000, roll/pitch ±0.775391.
   - Hashes: bit `300bd96693f57d02f6a7700e7797073175f785cd18b0dd989cd533aae3066b99`, hwh `386639a3dfc133dc8fbf962197c265a919febb1c10bb8d18bd4739ff020be4f7`, bda `323c0c1f…` (unchanged), ltx `c30e9070…` (unchanged), XSA `5e10800e7bdfe4668f2933056268b5de82f24e7c621b458765d6de0b6d3304cb` (`hw/exported_platform/nmpc_solver_v2_fix6/nmpc_solver_v2.xsa`).
   - Packaged as `app/kria-pynq/overlay-fix6`, staged on the board; `step_prep`, `step4_run` and `step5_validate` target fix6. The promotion placeholder still refuses.
2. **fix6 board validation PASSED** (`step5_validate.sh`; prep log at 19:44 board time, programmed in 4.66 s; dump `logs/2026-10-23-kv260-fix6-validate.json`, board `06_validate_194435.json`). `validate_hw_run.py --seeds 10` reports `VALIDATION PASS`:

| Run | Time | Hardware J | (a) half-model J | (b) REF J / max model error | (c) peak tilt (REF) / ±π/4 | SW percentile (float/hw) |
| --- | --- | --- | --- | --- | --- | --- |
| 0 | 523.9 ms | 4407.2700 | 4407.2500 (0.0005 %) | 4409.5406 (−0.05 %) / 1.20 mrad, 3.4 mm | 0.77503 / none | 10 / 20 |
| 1 | 524.1 ms | 4500.8849 | 4500.9049 (0.0004 %) | 4503.1168 (−0.05 %) / 1.03 mrad, 2.1 mm | 0.77413 / none | 60 / 70 |

   - Cost-limit crossings: none in either model. Control constraints OK. Software PSO, 10 seeds: float J 4367.91–4557.02, hw J 4359.06–4544.73. Solve time is about 6 % above fix5 (rounding), as predicted.
3. **fix6 PROMOTED** at 2026-10-23T19:47:42Z (`step6_promote.sh`, record `logs/07_promote_194742.txt`, also `PROMOTION.txt` inside the overlay). The installed overlay `/root/jupyter_notebooks/nmpc_hls/app/kria-pynq/overlay` has bit `300bd966…`, hwh `386639a3…`, bda `323c0c1f…`, manifest `1310d023…`, and source XSA `nmpc_solver_v2_fix6`. Status: VALIDATED for implementation consistency and numerical fidelity to the model equations, true ±π/4 met on the reference trajectory; **physical plant validation pending**.
   - **Rollback chain (caution):** fix5 had already been promoted as EXPERIMENTAL at 18:58:08Z (`logs/07_promote_185808.txt`), before the decision to wait for fix6. So:
     - `overlay.rollback-20261023-194742` = **fix5** (`3b8ac35e…`, sine-table defect, can violate the tilt limit: **not safe for control**);
     - `overlay.rollback-20261023-185807` = the original August overlay (hangs in the FSM).
     The rollback command printed by the fix6 promotion therefore reinstalls fix5. The safe source copy of fix6 is `~/nmpc_fsmfix/overlay-fix6`.
4. **Committed** (atomic, not pushed): `c102f6c` fast-sin table/rounding, `4039b30` angle margin, `8b569ab` model unit tests, `5760206` validator plus the `HW_MODEL_HALF_CSIM` opt-in, `b8370ed` docs and validation dumps. Not committed on purpose: the unrelated `graphs/nmpc_pso_system_overview.svg` change and the local `hw/hls/ip_backup_2026-10-23_*` IP backups.

Remaining work: see **Open items** in the handoff section at the top.
