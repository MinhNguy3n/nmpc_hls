# HLS bicycle model and PSO redesign switches: C level (2026-11-22)

**Status:** C level only. g++ builds of `hw/hls/source` and Python. No Vitis HLS synthesis, no cosim, no Vivado, no board. Every vehicle value is a simulation value (see `app/rc_car/params/*.yaml`). The check results are in `.copilot/chats/2026-11-22-claude-rc-car-reply-hls-bicycle.md`; the check is `app/rc_car/hls_bicycle_check.py`.

## 1. What was added (all pure additions: `git diff` has no removed line)

| File | Addition |
|---|---|
| `include/models/hls_bicycle.hpp` (new) | `__model`: x' = v cos ψ, y' = v sin ψ, ψ' = v κ. It is `hls_unicycle.hpp` with ψ' = v·κ: normalized controls mapped by multiplications, cos via `local_sin(ψ + π/2)`, every pragma on its own line. The constants between BEGIN/END markers are generated. |
| `include/config.hpp` | A generated `BICYCLE_CONFIG` block (before the chain's `#endif`) and a generated switch block (before the chain). Generator: `app/rc_car/gen_bicycle_config.py` (`--check`, `--write`). κ_max = tan(δ_max)/L from `reely_new1.yaml`. |
| `include/hls_model.hpp` | `#elif defined(BICYCLE_CONFIG)` → `models/hls_bicycle.hpp` |
| `include/hls_system.hpp` | `rd_stage_u`, `rd_one_step_u_error`, `rd_one_step_error_pf`; two new parameters of `nmpc_cost_function_lowflow` and `_topflow`; the existing calls wrapped in `#if`/`#else`/`#endif` |
| `src/hls_pso.cpp`, `include/hls_pso.hpp` | `rd_initializeWarmStart` (R1); per-knot rate windows (R2) in the initialization, the stable-zero particle and the update; the `rd_aux` parameter of `execute`, `initializeParticles_set`, `evaluateFitnessAndDetectLocalBest` (plus `rd_u_curr`) and `pso_fsm` (plus `rd_aux_local`) |
| `src/hls_nonlinear_solver.cpp`, `.hpp` | The `rd_aux` top-level port (s_axilite + m_axi) and its local copy; under R2, the full `_Nh*_Nx` reference copy |
| `src/main_hls_bicycle_test.cpp` (new) | The C test driver: model, cost and solve modes |

### Switches (`-DPSO_RD_…`, all off by default; defined for `BICYCLE_CONFIG` only, `#error` otherwise)

| Switch | Meaning | Python counterpart |
|---|---|---|
| `PSO_RD_R1` | Warm start: particle 0 = `last_best` (unchanged; the host sends the shifted previous best); particle 2 = the host's extra start (the feed-forward) when `rd_aux[0]` = 1; then max(1, n_S/5) perturbed copies of particle 0, projected onto the rate window and the box. **Uniform** noise of std 0.25 × window (the hardware stream is uniform; the Gazebo candidate used normal noise of the same std) | `Redesign.r1`; `Problem.perturb_dist = "uniform"` for the matched check |
| `PSO_RD_R2` | Move blocking: `_Nu` = 5 knots per input at stages 0, 6, 12, 18, 24; stage input = z_j + a (z_{j+1} − z_j); per-knot rate window = du × gap | `Redesign.r2 = 5` |
| `PSO_RD_R3` | Path-frame position cost: e_lat = c dy − s dx, e_lon = c dx + s dy with the host's per-stage (cos, sin) of ψ_ref; weights 3160 / 316 (terminal × 10) | `Redesign.r3` |
| `PSO_RD_R4` | Increment cost (W_s (u_k − u_{k−1}))², u_{−1} = u_curr; s_dv = 1, s_dκ = 10 (R4) | `Redesign.r4`, `s_dv` |
| `PSO_RD_UREF` | Effort about the host's per-stage reference control (W_r (u − u_ref))² instead of R (u − uss)² | the section 9.3 cost (q_v, r_κ) of every simulated NMPC arm |
| `PSO_RD_ALL` | All five: the simulated candidate controller | `redesign_smooth` |

**The candidate is `PSO_RD_ALL` = R1+R2+R3+R4+UREF.** The brief (`2026-11-22-claude-to-claude-rc-car-bh-review-next.md`) named only R1–R4. `PSO_RD_UREF` was added here, and the STOP H review accepted it.

**Why UREF is separate from R1–R4:** the simulated controllers (phase 4 and the redesign) always used the section 9.3 cost: a speed and curvature reference plus the increments. The deployed HLS cost never had either; it has R·(u − uss)². So "R1–R4 on" alone would not be the simulated controller. `PSO_RD_R4` carries both increment weights (s_dv = 1 as in phase 4, s_dκ = 10 the R4 value). `PSO_RD_UREF` carries q_v and r_κ.

**Square-root weights:** in normalized units r_κ (κ_max/100)² = 1.8e-5 is below one LSB of `ap_fixed<32,17>` (3.05e-5). The new terms therefore store √weight (`_rd_Wr`, `_rd_Ws`) and add (W e)²; the weighted error is rounded once to the top type.

**Stage A vs the simulated Stage B:** the HLS model is integrated by the existing RK4 (`one_step_prediction`), as the brief asks ("the unicycle header with ψ' = v·κ"). The Python controller in Gazebo predicted with the chord step (the October note's Stage B). The check measures the effect on the cost and the solver result.

## 2. The matching change list for `arc-shared-rollout/hw/hls/proto_arc/eval_multi.cpp` (not edited; owner: the ARC session)

**The patch:** `docs/rc_car/arc_bicycle_eval.patch` implements this list. Its header has the files, the build and the check order. It is not applied; the ARC session applies it after 27 Nov.

The interleaved evaluator replaces `evaluateFitnessAndDetectLocalBest`. With switches off nothing changes. With them on:

1. **Signature and ports:** add `_pso_hw_real rd_u_curr[_pso_n_U]` and `_pso_hw_real rd_aux[_RD_AUX_LEN]` after `local_find` under `#ifdef PSO_RD_ANY`, with the two `m_axi … bundle=current_mem` pragmas copied from `hls_pso.cpp`. Load both into local arrays (`ucur_l[EV_NU]`; `uref_l[EV_NH][EV_NU]` and `cs_l[EV_NH][2]` partitioned on dim 2) next to `load_ref`.
2. **R2 input (`uu[j] = cg[p][step*EV_NU + j]`):** under `PSO_RD_R2`, use `uu[j] = z0 + a*(z1 − z0)` with `z0 = cg[p][_rd_knot_j[step]*EV_NU + j]`, `z1` the next knot and `a = _rd_knot_a[step]`, one rounding to `top_t`. `EV_PART = _Nu*_n_U` then holds 10 words per particle, not 50. The comment "Nu == Nh" no longer holds under R2.
3. **Control error at s == 0 (`u_error_term`):** a new `u_error_term_rd(Jui_in, uu, uref, uprev, j)` with the expressions of `System::rd_one_step_u_error`. `uprev` is the previous step's `uu` per particle: keep a `uprev[EV_S][EV_NU]` array, initialized from `ucur_l` in `load_state` and written after the s == 0 term. This array is a new inter-iteration dependence on row p only, like `Jui`: add it to the `DEPENDENCE … inter false` list.
4. **State error at s == 3 (`state_error_term`):** under `PSO_RD_R3` the rotation needs both position components of the new state at once. Compute `xn` for i = 0, 1, 2 first (the existing loop already has all three in `xs[p][i]` after the update), then `ex`, `ey`, `e_lat`, `e_lon` as in `System::rd_one_step_error_pf`, and accumulate `Ji[p][0] += Q_lat e_lat²`, `Ji[p][1] += Q_lon e_lon²`, with i = 2 unchanged. That is 4 multiplies and 2 adds per particle and step, with no new transcendental.
5. **Generated headers:** `gen_headers.sh` gets a `bicycle` case (as `unicycle`):
   - model file `models/hls_bicycle.hpp`, guard `HLS_BICYCLE_HPP`;
   - the overlay files `config.hpp`, `hls_model.hpp` and `models/hls_bicycle.hpp` from this worktree;
   - the same pure-addition check, which these files pass (O3).
   - The switch defines must reach the eval build (`-DPSO_RD_ALL` or individual switches) together with `-DBICYCLE_CONFIG`.
6. **R1 does not touch the evaluator.** It changes `initializeParticles_set` only (deployed sources).
7. **Verification order** (the unicycle pattern): eval csim against `main_hls_bicycle_test cost` (bit-exact expected, the same expressions); then the RTL comparison (`cmp_eval_rtl.sh`); then cosim.

## 3. Host ABI note (`app/kria-pynq/nmpc_overlay.py`; no code changed)

1. **Model sizes:** `NX = 3`, `NU = 2`, `HORIZON = 25`.
   - The decision buffers `last_best`/`new_best` become `(_Nu, NU)`: `(5, 2)` under R2, `(25, 2)` otherwise.
   - Keep `xref` at `(HORIZON, NX)`. Under R2 the wrapper now copies all `_Nh*_Nx` values (the old copy took `_Nu*_Nx`, a latent assumption that `_Nu = _Nh`).
   - `MAX_PARTICLES = 8192 // (Nu * NU * 4)`.
2. **One new input buffer, `rd_aux` (int32, `_RD_AUX_LEN` words, the same fixed-point encoding):**
   - `[0]` n_extra: 1.0 if word 1… holds an extra start, else 0;
   - `[1 : 1 + Nu·2]` the extra start: the projected feed-forward of the reference, normalized, at the knots under R2 (`nmpc_engine.to_decision`);
   - `[… : … + Nh·2]` u_ref per stage, normalized: z_v = (v_ref − v_c)/v_h·100 and z_κ = κ_ref/κ_max·100;
   - `[… : … + Nh·2]` (cos ψ_ref,k, sin ψ_ref,k) per stage, computed in double on the host from the local-frame reference heading.
   - Length 111 under R2 (`1 + 10 + 50 + 50`), 151 without.
   - Reference packing: `hls_bicycle_emul.pack`.
3. **Register map:** a new pointer argument after `bestfitness` in `ARGUMENT_REGISTERS` (`pso_fsm` gets `rd_aux` and `rd_aux_local`). Read its offset from the new manifest; do not assume 0x40.
4. **Warm-start buffer:**
   - `last_best` stays particle 0.
   - With R1 the host sends the projected shifted previous best there whenever one exists (not the cheaper of it and the feed-forward); the feed-forward goes to `rd_aux` slot 1 with n_extra = 1.
   - On the first tick (no previous plan) the host sends the cheaper candidate and n_extra = 0, as `NmpcController.prepare` does.
5. **Knot layout (R2):**
   - knots at stages 0, 6, 12, 18, 24;
   - the returned `new_best` holds the 5 knots;
   - the host expands it with the same interpolation (`nmpc_engine.to_full`) before applying u_0 (= knot 0) and before the double-precision `check_controls`.
6. **Random stream:**
   - R1 draws `_rd_n_perturb × Nu × NU` extra numbers in the initialization (20 for 10 particles under R2);
   - the `axis_rand_fanout` count per solve grows by that number;
   - the update's count `2·n_S·Nu·NU` shrinks 5× under R2.
7. **Manifest fields:**
   - `design_revision` +1;
   - `model: "bicycle"`;
   - `switches: ["R1", "R2", "R3", "R4", "UREF"]`;
   - `nu` (knots) and `knots: [0, 6, 12, 18, 24]`;
   - `rd_aux_len`;
   - `perturb: {"dist": "uniform", "std_fraction": 0.25}`;
   - `weights` (the generated `_Q`, `_Qf`, `_rd_Wr`, `_rd_Ws`, `_rd_Q_pf`, `_rd_Qf_pf`);
   - `kappa_max`, `v_range`, and the hash of the generated config block (so a host with other vehicle values refuses the bitstream).
8. **`encode_fixed` uses `ceil`:** a +1 LSB bias (3e-5) on every input. The C checks used round-to-nearest. It is negligible, but the host and these checks differ by that LSB.
9. **Block design:**
   - `current_mem` grows by `_RD_AUX_LEN + n_U` words;
   - the `mem_bank` address constants must be re-laid out;
   - all five IP cores are rebuilt together (October note section 10).

## 4. Open issues in the existing sources (found at STOP H; not fixed, by decision of the STOP H review)

Each fix changes the switches-off outputs, so it goes into a separate change, checked on its own (O1/O2 re-baselined), after the ARC submission.

| # | Issue | Where | One-line fix |
|---|---|---|---|
| 1 | `execute()` returns an uninitialized J: it reads `bestfitness[_pso_maxiter-1]`, a local array that is never written (`local_bestfitness` is). `nonlinear_solver_wrapper`'s J and `main_hls_validate solve`'s "J" line are garbage; the deployed FSM path (`bestfitness_top`) is correct | `src/hls_pso.cpp`, `execute()` | `J[0] = local_bestfitness[_pso_maxiter-1];` |
| 2 | `nonlinear_solver_wrapper` copies only `_Nu*_Nx` reference values into `my_xref[_Nx*_Nh]`: correct only while `_Nu = _Nh` (fixed under `PSO_RD_R2` by an extra copy) | `src/hls_nonlinear_solver.cpp` | `memcpy_loop_rolled<_hw_top_real, float, _Nh*_Nx>(my_xref, (float *)xref );` |
| 3 | `encode_fixed` rounds with `ceil`: a +1 LSB (3.05e-5) bias on every host input; the C checks round to nearest | `app/kria-pynq/nmpc_overlay.py` | `v = np.asarray(values, dtype=np.float64); scaled = np.sign(v) * np.floor(np.abs(v) * FIXED_SCALE + 0.5)` (round half away from zero, as AP_RND_INF) |
| 4 | `initializeStableZero` stores the pre-box value in `x_ant_tmp` (harmless while `_uss` is inside the box; October note §9.5) | `src/hls_pso.cpp` | `x_ant_tmp[i] = local_x[index*part_S + idx];` (after the box clamp) |

