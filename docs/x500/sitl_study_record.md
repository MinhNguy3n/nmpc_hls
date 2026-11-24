# X500 / sniff-mockup PX4 SITL study: record of S0–S6 (2026-11-23 to 2026-11-24)

**What this is:** the study's history in one place: each stage's question, design, verdict and label; what was fixed before the results and when; the deviations and corrections; and what is still open.
- Every number comes from the stage replies and reviews in `.copilot/chats/`, linked per stage. The tables stay there.
- Nothing in this record is new.

**Scope:**
- **What was simulated:** PX4 v1.17.0 SITL with Gazebo Harmonic, a 7 × 8 m arena with nets, and the sniff-drone mockup constants in place of a measured X500.
- **What the NMPC was:** a float C-model surrogate of the PSO solver, charged with board timing.
- **What did not happen:** no hardware was commanded, and the KV260 was not contacted.
- **Not for the paper:** nothing from S3–S6 goes into the ARC 2027 manuscript.

**The briefs:**
- [the original brief](../../.copilot/chats/2026-11-23-claude-to-claude-x500-px4-sitl.md);
- [the amendments after the GPT-6 review](../../.copilot/chats/2026-11-23-claude-to-claude-x500-gpt6-review-amendments.md), which superseded the original S3 design.

## 1. The stages

### S0: set-up

**Question:** a PX4 release for Ubuntu 24.04, Gazebo Harmonic, the Pixhawk 4 and uXRCE-DDS, plus an image and build path for it.

**Design:**
- PX4 v1.17.0, with `px4_msgs` `release/1.17` and Micro-XRCE-DDS-Agent v2.4.3, pinned by commit.
- The image `nmpc-jazzy-px4:2026-11`.
- The PX4 source on the host, bind-mounted; builds under a 6 GB cap.

**Result:** the stock check passed on the stock `gz_x500` (run 3 of 4):
- odometry at 99.96 Hz;
- hover RMS 0.025 m, against the < 0.1 m rule.

**Label:** SITL, stock `gz_x500`, not the mockup.

**Replies:** [S0 reply](../../.copilot/chats/2026-11-23-claude-x500-reply-s0.md) · [S0 review](../../.copilot/chats/2026-11-23-claude-to-claude-x500-stop-s0-review.md) · [S0 accepted](../../.copilot/chats/2026-11-23-claude-to-claude-x500-s0-accepted-go-s1.md)

### S1: the mockup drone `x500_sniff`

**Question:** does a Gazebo/PX4 model built from one parameter YAML reproduce the solver's mockup drone?

**Design:**
- One generator writes the SDF, the airframe and the derived constants.
- The "+" mixer is mapped to PX4's motor order, hand-checked.
- The rate gains are tuned within PX4's documented limits.
- Step table; an open-loop check against the solver's model (< 2 cm and < 1° per 0.5 s window).

**Result:**
- **Plant:** matches the exact rigid body in 332 of 332 windows.
- **Against `quadrotor.py`:** 331 of 332. The failing window shows that the solver's rotational model is not exact rigid-body dynamics above about 10° of tilt.
- **Steps:** the strict no-oscillation rule **failed marginally** on the final run. The gains were accepted, and the rule was not redefined.

**Label:** SITL, mockup sniff-drone constants. "PX4 gains: mockup tuning, step rule failed marginally (S1)" goes on every later table.

**Replies:** [S1 reply](../../.copilot/chats/2026-11-23-claude-x500-reply-s1.md) · [S1 review](../../.copilot/chats/2026-11-23-claude-to-claude-x500-stop-s1-review.md)

### S2: arena, supervisor, references, state bridge, baseline B0

**Question:** a common harness for every controller, and PX4's own tracking of the references (B0).

**Design:**
- **Arena:** the 7 × 8 m world, with a geofence 1.0 m inside the nets and 0.5–3.0 m in height.
- **References:**
  - R1: a 2 m circle at 1.5 m height, 20 s per lap;
  - R2: the same circle as a helix from 1.0 to 2.5 m.
- **Sensing:** emulated OptiTrack into PX4's EKF, with GPS off.
- **State bridge:** tested both ways.
- **B0:** position and velocity setpoints at 50 Hz, acceleration NaN; seeds 1–5 per reference.

**Result:**
- The bridge and the stale-mocap and Offboard-stall checks passed, as did the clock check.
- **B0:** 10/10 runs valid, 3-D RMSE 0.423 m on both references. The drone flies the circle wide, at 2.39 m, and comes within 8 cm of the fence.
- **δ = 1 cm** for both references: the B0 range was 1.8 and 1.3 mm.
- **Luukkonen equations:** "confirmed" for J and the body torque; "not confirmed" that the model equals the published equations.
- **Blocker:** the bit-accurate half C model takes 36.4 s per solve.

**Label:** SITL, mockup sniff-drone constants, mockup PX4 tuning.

**Replies:** [S2 reply](../../.copilot/chats/2026-11-23-claude-x500-reply-s2.md) · [S2 review](../../.copilot/chats/2026-11-23-claude-to-claude-x500-stop-s2-review.md)

### S3: NMPC trajectory setpoints from the 12-state model

**Question:** are "NMPC-generated trajectory setpoints, tracked by PX4" no worse than B0?
- The optimised torques are never applied.

**Design:**
- **Solver:** the float surrogate, 10 × 40, charged with the board's timing distribution.
- **Setpoints:** taken from the plan at t + lead.
- **Lead:** swept on R1 with seeds 101–103, then frozen.
- **Evaluation:** R1 and R2 with seeds 1–5, paired with S2's B0.
- **Arithmetic check:** offline, half vs float vs float.

**Result:**
- **Lead:** only 1·Ts passed (degenerate). Every longer lead ended on the geofence or the tilt envelope.
- **Verdict: "worse or inconclusive"** on both references:
  - median S3 − B0 = +2.15 m (R1) and +2.35 m (R2);
  - 3 of 10 trials failed on the geofence.
- **Where the error is:** in the plans, not in PX4, the estimator or the timing.
- **Arithmetic check:** within PSO noise on both metrics.

**Label:** SITL, mockup constants and tuning, float solver surrogate (board timing charged).

**Replies:** [S3 reply and addendum](../../.copilot/chats/2026-11-23-claude-x500-reply-s3.md) · [S3 review and S5 brief](../../.copilot/chats/2026-11-24-claude-to-claude-x500-stop-s3-review.md)

### Between S3 and S5a: the landing failure and the supervisor

**Finding:** five S3 sweep runs hit the rigid nets after fence trips at 1.1–2.2 m/s. They bounced, tumbled and did not land. Land itself works from a hover (4–5 s).

**Decision:**
- **Layer 1, the validity fence:** unchanged.
- **Layer 2, a new predictive safety trip against the net:**
  - trip if x + k_b·max(v_out, 0) + m ≥ x_net − r_drone;
  - k_b = 0.92 s, r = 0.38 m, m = 0.10 m.
- **Net contact** is detected from ground truth and is a hard failure.

**Replay checks:** no S2 B0 run would trip, and all five crash runs trip before the net.

**Files:** [landing report](../../.copilot/chats/2026-11-24-claude-x500-landing-failure-report.md) · [supervisor decision](../../.copilot/chats/2026-11-24-claude-to-claude-x500-supervisor-decision.md)

### S5a: identify PX4's closed loop

**Question:** can a grey-box model of the PX4-controlled drone, with the velocity setpoint as input, predict the drone over the 1.25 s horizon?

**Design:**
- **Model:** PX4's cascade with the frozen gains, plus an acceleration lag.
- **Identification data:** excitation seeds 201–203 plus the S1 logs.
- **Validation:** on the held-out S2 B0 logs.
- **Rule:** at the horizon end, median ≤ 5 cm and p90 ≤ 10 cm, on R1 and R2.

**Result:**
- **PASS.**
  - R1: median 4.69 cm, p90 5.41 cm.
  - R2: median 4.54 cm, p90 5.25 cm.
  
  The median passed with only 3–5 mm to spare, because of a systematic 4.4 cm along-track lag.
- **The model:** 12 states, 3 inputs, τ_xy = 0.53 s.
- **B0 re-run under the new supervisor:** within 1.5 mm of S2 per seed; δ stays 1 cm. These runs became S5b's B0 pairs.
- **Own-model reference point (diagnostic):** 0.150 / 0.152 m.

**Label:** SITL, mockup sniff-drone constants and PX4 tuning.

**Replies:** [S5a reply](../../.copilot/chats/2026-11-24-claude-x500-reply-s5a.md) · [S5a review](../../.copilot/chats/2026-11-24-claude-to-claude-x500-stop-s5a-review.md)

### S5b: PSO on the PX4 closed-loop model (`PX4CL_CONFIG`)

**Question:** does an NMPC whose decision variables are the setpoints it sends beat B0?

**Design:**
- **Decision variable:** v_sp, in a box of ±0.8 m/s horizontally and ±0.3 m/s vertically. p_sp is its integral.
- **Tilt:** a per-axis tilt box inside the 1.73 m/s² disc.
- **Weights:** six weight sets declared before any run. W3 was selected on R1 with seeds 101–103 (18 runs).
- **Evaluation:** seeds 1–5, paired with the S5a B0 re-run.

**Result: "better"** on both references.
- Median −0.297 m (R1) and −0.293 m (R2).
- All 10 pairs negative, with no failures.

**How:** the gain is cross-track. The plan holds p_sp inside the circle and over-speeds v_sp.
- 68–75 % of the applied setpoints were outside the validated speed range.
- The online model error was 9.8–11.1 cm.
- W5, the only set that stayed in range, reached 0.318 m.

**Label:** "SITL, mockup constants and tuning, float solver surrogate (12-state board timing as a stand-in); better than B0 as run, with 68–75 % of setpoints outside the model's validated speed range; n = 5 per reference, exploratory."

**Replies:** [S5b reply](../../.copilot/chats/2026-11-24-claude-x500-reply-s5b.md) · [S5b review and S6 brief](../../.copilot/chats/2026-11-24-claude-to-claude-x500-stop-s5b-review.md)

### S6: B1, the matched baseline

**Question:** how much of S5b's gain does PX4's own acceleration feed-forward recover, and is anything left for the NMPC?
- B1 = B0 plus the reference's analytic acceleration. It has no free parameters.

**Design:** this is the design as revised after check 3.
- **Checks:** checks 1–5 (PX4 semantics, the node diff, a regression run, the envelope, a smoke run).
- **One block:** a single interleaved block of 30 runs (B0, B1 and W3 for seeds 1–5 on R1 and R2), paired only within the block.
- **Stops:**
  - any fresh B0 more than 1 cm from its S5a re-run;
  - a B0 or B1 RMSE range over 1 cm.

**Result:**
- **Checks:** checks 1, 2, 4 and 5 passed; check 3 failed (see §3).
- **Stops:** neither fired. B0 was within 1.72 mm of its earlier runs, and the largest range was 3.4 mm.
- **A (B1 vs B0): "better"**, −248 mm on both references.
- **B (W3 vs B1, the main comparison): "better"**, −44 mm (R1) and −46 mm (R2).
  - The gain is all cross-track. It is larger over t ≥ 5 s, so the start transient is ruled out as its source.
  - Lag compensation is plausible, not shown.
- **Arithmetic check:** not run. The half build fails to compile (§4).

**Label:** "SITL, mockup constants and tuning, float solver surrogate (12-state board timing as a stand-in). n = 5 per reference, exploratory. The NMPC beat PX4's position control with analytic acceleration feed-forward by 4.4–4.6 cm median 3-D RMSE, with 69–77 % of its setpoints outside the model's validated speed range, about twice B1's tilt (7.3° vs 3.5°), and after 18 tuning runs against none for B1."

**Replies:** [S6 reply (check-3 stop, then STOP S6)](../../.copilot/chats/2026-11-24-claude-x500-reply-s6.md) · [check-3 decision](../../.copilot/chats/2026-11-24-claude-to-claude-x500-s6-check3-decision.md) · [S6 review](../../.copilot/chats/2026-11-24-claude-to-claude-x500-stop-s6-review.md)

## 2. What was fixed before the results, and when

| Rule | Fixed in | Before |
|---|---|---|
| The S3 decision rule ("better" / "no worse" / "worse or inconclusive"), the pairing, and δ = max(1 cm, B0 range) | Amendments F, 2026-11-23 | S2's B0 numbers |
| δ = 1 cm for R1 and R2 | STOP S2, 2026-11-23 | any S3 run |
| The lead set {1, 2, 4, 8, 12, 16, 20}·Ts and the lead selection rule (lowest median with no failure; ties → shorter; none → stop) | S2 review, 2026-11-23 | the S3 sweep |
| The S3 arithmetic-check rule | S3 reply §6, 2026-11-23 23:57 | the S3 evaluation and any half solve |
| The two-layer supervisor and its constants | Supervisor decision, 2026-11-24 | any S5a Gazebo run |
| The S5a validation rule (≤ 5 cm median, ≤ 10 cm p90, R1 and R2, held-out B0) | S3 review, 2026-11-24 | the fit |
| The S5b validated v_sp box, the extrapolation logging, and the S3 rule with δ = 1 cm | S5a review, 2026-11-24 | the S5b declaration |
| The six S5b weight sets and the simplicity order | S5b reply §1, 2026-11-24 06:30 EET | the smoke run (06:32) and the tuning runs (06:35) |
| The B1 definition, comparisons A and B, the three interpretations, and checks 1–5 | S5b review, 2026-11-24 | any B1 run |
| The PX4CL arithmetic-check rule | S6 reply §6, 2026-11-24 | any half solve (none has run) |
| The single-block S6 design and its two stop rules | Check-3 decision, 2026-11-24 | the S6 block |

**What was not pre-registered:**
- the S6 t ≥ 5 s RMSE and the peak-error times (added as diagnostics, not decision metrics);
- the cross-track and along-track reading of each result.

## 3. Deviations and corrections

**S0:** three of four stock-check runs failed on harness causes, all reported and fixed:
- a shell `set -u` ordering;
- `NAV_DLL_ACT`, set to 0 in SITL;
- a settle window shorter than its test.

Every failsafe that was set to Return now lands or holds, marked "lab: to confirm".

**S1:**
- **The step rule** failed marginally and was accepted without being redefined.
- **The rotational model** is not exact rigid-body dynamics. It was kept unchanged for S2/S3 (about 1° of tilt needed).

**S2:**
- **Harness bugs,** each found in a failed run, fixed and listed in the run log:
  - a preflight gate that waited on the wrong mode's checks;
  - a false stale-mocap trip;
  - a `hold_end` bug.
- **15 B0 starts** were refused by the container guard and rerun.
- **A wait loop** matched its own pattern, which led to the bracketed `pgrep` rule.
- **The half model** was too slow, so the float surrogate was used, plus an offline arithmetic check. That was a decision, recorded in the label.

**S3:**
- **Lead 0** was excluded on the clock-check drift evidence, and the sweep was extended to 20·Ts by the review.
- **The frozen lead** was degenerate and was reported as such, not overridden.
- **Five net contacts** surfaced afterwards and were added as crashes in an addendum. The verdict was unchanged.
- **The arithmetic rule text** was restored verbatim when the full reply was written.

**S5a:**
- **Three excitation attempts** failed through a callback bug and were excluded from the fit.
- **The fitted a_xy,max = 13.4 m/s²** was relabelled "not identified (never binding)", and S5b used 9.81 m/s².
- **The measured vertical k_b** is 0.21 s, but the supervisor kept the conservative 0.92 s.

**S5b:**
- **Before the declaration (disclosed):** R was revised by an offline own-model check.
- **No increment cost:** it is bicycle-only, so du_max bounds the increments instead. The `PSO_RD_R4` guard was not edited.
- **The tilt envelope** became a per-axis box.
- **Two statements in the reply were corrected:** the margins and the 53 % figure.

**S6:**
- **Check 3 failed:** the regression run was 1.63 mm from its earlier run, against a 1.5 mm tolerance.
  - The reviewer withdrew the tolerance as their own error: it sat at the edge of the cross-session noise.
  - Instead of loosening it, the design moved to a single interleaved block.
- **The arithmetic check stopped** at a compile failure rather than being forced.

## 4. Open items

**HLS (the ARC session's, after 27 Nov; no agent edits HLS source):**
- **The half PX4CL build fails** on an ambiguous `sqrt` in the a_xy clamp of `models/hls_px4cl.hpp`.
- **Recommended in the S6 review:** remove the square root rather than choose one. Use a per-axis clamp at ±6.94 m/s², or no clamp, after showing on the logged S5b/S6 solves that neither clamp ever activates. Then:
  - repeat `test_px4cl.py`;
  - run `arith_check_px4cl.py` under the S6 §6 rule;
  - then the `px4cl` model point (`gen_headers.sh`).
- **The half-precision p_sp integral** could be off by 2.5–5 cm at |x| ≥ 2–4 m. A design option is error coordinates relative to the current position.

**SITL (parked until after 27 Nov; each needs a go):**
- **Optional S7, the mechanism test:** "B2" = B1 with its feed-forward advanced by the identified τ_xy = 0.53 s. To be pre-registered first.
- **The S5a model's along-track lag:** adding the integrator would need a new held-out set.
- **The common B0/B1 error peak** just before 6 s has not been examined.

**X500 (S4 and the lab):**
- **The S4 config generator** targets the PX4 closed-loop model. It needs:
  - identification on the real X500 over the speeds the controller will use (up to about 1.2 m/s);
  - a magnitude bound on the horizontal v_sp;
  - a cost on v_z or its increments;
  - references that ramp up from rest.
- **The S3 interface's coupled rotor bounds** are still on the list.
- **The tilt limit** is 15°, provisional. Repeat the held-tilt table with the measured constants.
- **The vertical k_b** of 0.21 s is recorded for the rebuild.
- **The baseline for any lab NMPC claim is B1, not B0.**
- **S3b** (the KV260 in the loop) needs a board-side server and the PX4CL model point.
- **The lab failsafes** (`NAV_DLL_ACT` = 3, Land) and the measurement and identification flights follow `measurement_procedure.md`. The user flies; agents command nothing.

**Housekeeping:**
- `analyze_s6.py` (unit-tested 2026-11-24) used to skip a seed whose earlier B0 run was missing instead of stopping. Fixed on 2026-11-24 after the review, with a test: a missing run is now a stop. In S6 all 10 were present, so the S6 result is unaffected.
- A gitignored header copy from the failed half build remains in `hw/hls/nmpc_solver_validate_build/`.
- All code is uncommitted in worktree `agent-a7f53c7be12d59b73`. Commits are made by the ARC session.
