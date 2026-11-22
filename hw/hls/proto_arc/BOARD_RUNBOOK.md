# Full-solver integration of the interleaved evaluator: build and board runbook (2026-11-20)

The user runs every step; agents prepare and check. All builds stay in this worktree (`arc-shared-rollout`). The main checkout's `hw/hls/vitis_ip_repo`, overlays and exported platforms are not touched. Nothing is promoted on the board: `overlay-fix6` stays promoted.

## What is built

| Overlay | PSO size | Purpose |
|---|---|---|
| `overlay-engine_3x24` | 3 particles × 24 iterations | End-to-end equivalence and speed. Same S × K as `overlay-3x24-wrap`, so the board outputs must be **bit-identical** to that build. Only the evaluator IP differs. |
| `overlay-engine_10x40` | 10 × 40 | The larger swarm from the PC screen. Closed-loop quality and timing. There is no bit-identical baseline. |

**Contents of each build:**
- the four unchanged solver IPs from `source/`, with `_n_S` and `_maxiter` set in a header copy;
- `nmpc_solver_costF` from `proto_arc/eval_multi.cpp` at II 8, with the same VLNV `tu-dresden:hls:evaluateFitnessAndDetectLocalBest:1.0`. Its 188 top-level ports and port/parameter declarations are identical to the deployed evaluator's;
- the same ILA debug block design as the 3×24 baselines (`build_engine_vivado.tcl`, a copy of `build_debug_variant.tcl`). The one change is a synthesis pre-hook (`synth.maxThreads 2`) against memory exhaustion.

## 1. Build on the PC

Run in tmux, one configuration at a time: both share `hw/hls/vitis_ip_repo`. Close VS Code if memory is short.

```bash
tmux new -s engine
cd /home/minh/Documents/UTU-Doctorate/nmpc_hls/.claude/worktrees/arc-shared-rollout/hw/hls/proto_arc
systemd-run --user --scope -p MemoryMax=10G -p MemorySwapMax=0 \
  ./build_engine_overlay.sh 3 24 engine_3x24 all 8 3 2>&1 | tee build_engine_3x24.log
systemd-run --user --scope -p MemoryMax=10G -p MemorySwapMax=0 \
  ./build_engine_overlay.sh 10 40 engine_10x40 all 8 3 2>&1 | tee build_engine_10x40.log
```

**Stages:**
- `hls` takes about 20–30 min.
- `vivado` takes about 1–2 h. The project goes to `hw/hdl/build/<name>`, and the XSA, timing and utilization to `hw/exported_platform/<name>/`.
- `package` writes the overlay to `app/kria-pynq/overlay-<name>`.

To rerun a single stage, put `hls`, `vivado` or `package` in place of `all`. A failed Vivado stage needs `hw/hdl/build/<name>` deleted first.

**Check after each build:** WNS ≥ 0 in `hw/exported_platform/<name>/timing_summary.rpt`, and no critical warnings in `hw/hdl/build/vivado_<name>.log`.

## 2. Board: stage, program, validate (KV260 `ubuntu@10.42.0.29`; the user runs sudo)

**Host-key check first.** On 2026-11-20 the board's SSH host key no longer matched `~/.ssh/known_hosts` (line 62), and SSH refused to connect. Verify the new key before trusting it:
- on the board's serial console, run `ssh-keygen -lf /etc/ssh/ssh_host_rsa_key.pub`;
- compare it with `SHA256:+Ks+0vO+518o8Lk6DcjZVhmMTaxtxFamZEXdFYbqgL4`.

Only if they match, run `ssh-keygen -f ~/.ssh/known_hosts -R 10.42.0.29`. If the board was re-imaged, `~/nmpc_fsmfix/` (the step scripts and overlays) may be gone; check before going further.

```bash
W=/home/minh/Documents/UTU-Doctorate/nmpc_hls/.claude/worktrees/arc-shared-rollout
scp -r $W/app/kria-pynq/overlay-engine_3x24 ubuntu@10.42.0.29:~/nmpc_fsmfix/
ssh ubuntu@10.42.0.29
  sudo bash ~/nmpc_fsmfix/step_prep.sh overlay-engine_3x24       # programs the PL, writes the marker
  sudo bash ~/nmpc_fsmfix/step5_validate.sh overlay-engine_3x24  # fixed-input solves -> 06_validate_*.json
```

Copy the new `05_prep_*.txt` and `06_validate_*.json` (the script prints their paths) to `app/kria-pynq/validation_logs/engine_3x24_validate/` in the main checkout. Then, on the PC, from the main checkout:

```bash
V=app/kria-pynq/validation_logs
/usr/bin/python3 app/kria-pynq/validate_hw_run.py $V/engine_3x24_validate/06_validate_*.json --particles 3 --iterations 24
/usr/bin/python3 $W/hw/hls/proto_arc/compare_board_runs.py dump \
  $V/engine_3x24_validate/06_validate_*.json $V/3x24_wrap_validate/06_validate_191454.json
```

**Expected:** VALIDATION PASS, then `DUMP MATCH`, with the per-solve times of the engine and the baseline (38.0–38.2 ms).

## 3. Board: closed loop (notebook, 5 × 100 steps, RK4 software plant)

1. In the board's Jupyter, open `kv260_nmpc_demo.ipynb` and set `overlay_dir` to `overlay-engine_3x24`.
2. Run 5 trials × 100 steps, as for `20261025T191910Z_rk4_100x5`.
3. Run `sudo bash ~/nmpc_fsmfix/hil_trials/archive_and_install.sh`. It packs the run into `~/nmpc_fsmfix/runs/`.
4. Copy the run folder to `app/kria-pynq/validation_logs/`.
5. Compare:

```bash
/usr/bin/python3 $W/hw/hls/proto_arc/compare_board_runs.py loop $V/<engine run folder> $V/20261025T191910Z_rk4_100x5
```

**Expected for 3×24:** `HISTORIES MATCH` (states and controls bit-identical to the 3×24-wrap run), with the timing distribution of both runs.

**For 10×40:** repeat steps 2–3 with `overlay-engine_10x40` and `--particles 10 --iterations 40`. There is no baseline to compare with, so run `compare_board_runs.py loop <run>` for the timing, and use `closed_loop_trials.csv` for RMSE, max error, tilt and violations. Compare these with the PC 10×40 screen as distributions.

**After the measurements:** restore the promoted baseline with `sudo bash ~/nmpc_fsmfix/step_prep.sh` (its default is `overlay-fix6`). Nothing is promoted.

## Predictions (not results; do not quote as measurements)

These come from cosim evaluator cycles plus the HLS estimates for the other phases, at 100 MHz, and exclude PS overhead:
- **3 × 24:** about 24 × (21.5k + 3.9k) cycles ≈ 6 ms per solve, against 38.0 ms measured for 3×24-wrap.
- **10 × 40:** about 40 × (28.7k + 10.9k update + 0.3k detect) ≈ 16 ms per solve.
