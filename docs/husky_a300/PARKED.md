# Husky work: parked 2026-11-21 (not cancelled)

The study moves to the Reely NEW1 RC car (prompt `.copilot/chats/2026-11-21-claude-to-claude-husky-agent-rc-car-prompt.md`). All Husky files stay where they are. This page records what was done, what is open, and how to resume.

## Where things are

- **Worktree:** `.claude/worktrees/agent-a7f53c7be12d59b73`
- **Branch:** `husky-unicycle-mpc`
  - committed: `468c747`, `870e3fd` (offline harness, `r_omega` tooling, plan rows; 110 tests);
  - nothing is pushed or merged.
- **Gazebo SIL:**
  - Docker `nmpc-jazzy`, with the scripts in `~/nmpc_sim/docker/`;
  - plan `docs/husky_a300/simulation_and_integration_plan.md`;
  - model note `docs/husky_a300/unicycle_mpc_pso_model.md`.

## Status of the ARC-prompt tasks (`.copilot/chats/2026-11-20-claude-to-claude-husky-agent-prompt.md`)

| Task | Status | Files (uncommitted unless stated) |
|---|---|---|
| A: unicycle model in HLS (`UNICYCLE_CONFIG`) | **Done, accepted** (the ARC session reran `hls_unicycle_check.py`: all PASS). Keep the allocation pragmas. | `hw/hls/source/include/config.hpp`, `hls_model.hpp`, `models/hls_unicycle.hpp`, `hw/hls/source/src/main_hls_unicycle_test.cpp`, `app/husky/hls_unicycle_check.py`; reply `.copilot/chats/2026-11-21-claude-husky-reply-unicycle-hls.md` |
| B: A200 G0 checklist | **Done, accepted**, plus §4b OptiTrack, battery and firmware rows, and the Thor note. **Not yet run on the robot.** | `docs/husky_a300/a200_g0_checklist.md` |
| C: A200 + OptiTrack design | Not started | — |
| D: A200 delta list (plan §3.4) | Not started | — |

`UNICYCLE_CONFIG` and `hls_unicycle.hpp` belong to the ARC paper (due 2026-11-27): **don't edit them** while the car work runs.

## Open items

1. **G0 on the A200:**
   - run the checklist **on the Thor** (the onboard computer, which drives the MCU over the PL2303 cable), never through that cable from another machine;
   - fill §5 rows 1–41, including §4b (Motive, protocol, up axis, rates, rigid body, bridge, network, floor area), battery voltage and firmware.
2. **Thor vs KV260:**
   - recommended: keep the Thor as the robot computer and add the KV260 as a solver over wired Ethernet;
   - decision open with the user, supervisor and lab.
3. **Weight resolution (raised with GPT-6, Update 5 of `2026-11-19-claude-to-gpt6-husky-sim-checks-and-sil.md`):**
   - `ap_fixed<32,17>` stores `R_omega = 5e-5` as 6.10e-5 and `5e-6` as 0;
   - recommended: common weight scaling plus hardware-representable candidates;
   - **no weight changes before 27 Nov**; awaiting GPT-6.
4. **Gazebo weight pair (5e-5 vs 5e-6, 12 runs):** prepared, parked by item 3.
5. **Out-of-bounds writes found during Task A** (reply §6, not fixed; outside the worktree's scope):
   - `main_hls_evaluate.cpp` writes `x_curr[5]` and `xref[k*Nx+5]`;
   - `tb_eval.cpp` `make_state` writes `x[3..11]` when `Nx = 3`;
   - the ARC scripts hard-code `SNIFFBOT_CONFIG`.
6. **Delay compensation** (actuation-prediction mismatch, `cmd_vel_out` 3/12/33 ms after the tick): GPT-6 asked for a separate review; not started.
7. **Truth parser lag (found 2026-11-22 in the RC-car work; Husky code unchanged):** the shared parser `identify_effective_track.parse_pose_v_text` hands over each Gazebo pose only when the next message starts, one frame (10 ms at 100 Hz) late. Scoring uses the stamps and is unaffected. **The earlier Husky SIL runs did not get this delay in their feedback:** `sil/sil_node.py` feeds the controller only the EKF odometry (`platform/odom/filtered`, or the SIL estimator), and the truth is "logged only, never fed back" (checked 2026-11-22). Any future use of the parsed truth as a feedback signal would add the 10 ms. The car uses `app/rc_car/gz_pose_stream.py` instead.

## How to resume

1. Re-read this page, the plan's latest rows and the last GPT-6 feedback in `.copilot/chats/`.
2. Check the worktree: `git status` should show the Task A/B files above as modified or untracked, and nothing else from the Husky work.
3. Run `/usr/bin/python3 -m pytest app/husky -q -p no:cacheprovider` (expected: 110 passed) and `/usr/bin/python3 app/husky/hls_unicycle_check.py` (all PASS; g++ only, no Vitis HLS).
4. Continue with the G0 results (item 1), then Tasks C and D. Keep A200 values `PLACEHOLDER_` until they are read off the robot.
