#!/usr/bin/env bash
# Phase 4b batch of the RC car study (SIMULATION ONLY). Run by the user, ONE BLOCK PER ASSIGNED PC WINDOW:
#
#   bash app/rc_car/checks/run_phase4_batch.sh B1      # from the worktree root
#
# Blocks (wall-time estimates from the step 4a smoke runs; see docs/rc_car/simulation_plan.md section 7.8):
#   B1 (~0.45 h, 75 episodes) RTF-invariance check (4 episodes) + tick-timing smoke (1 per size) FIRST,
#                             then the baseline neighbourhood re-run (new delta_max) + NMPC weight tuning (7.7)
#                             at the factors the check decides -> then STOP: the agent applies the two
#                             selection rules and freezes the values
#   B2 (~0.9 h, 100 episodes) core, realistic_guess NMPC + the baseline core (both profiles)
#   B3 (~0.8 h, 65 episodes)  core, ideal NMPC + the float spot check (10x40, figure-eight, 5 seeds)
#   B4 (~0.9 h, 80 episodes)  compensation on/off, 10 mm contact point, speed sweep (cut list: sweep first)
# Total 320 episodes, ~3.1 h wall (+ ~15 % contingency).
#   BR (~0.85 h, 80 episodes + C1-C6) PSO-redesign confirmation (plan section 7.10): C1-C6 with the
#                             1.5 kg payload first, the baseline core re-run (RTF 1.0), the frozen
#                             redesign (variant:=redesign) core at 10x40 (0.5x) and 3x24 (0.35x), and the
#                             rule-dropped smooth variant (variant:=redesign_smooth, 10x40, 0.5x)
#
# Real-time factors: per arm (baseline 1.0; NMPC 3x24 0.35, 10x40 0.5, 10x60 0.4) if B1's RTF check
# passes; otherwise ONE common factor, 0.35, for every arm (STOP 4a review section 3). B2-B4 read the
# decision from ~/nmpc_sim/logs/rc_car_phase4_batch/B1/rtf_decision.json.
#
# Every container is throwaway (--rm, --memory 4g) and starts only if no Vivado/Vitis/xsim/csim job
# runs (the ARC cosims take precedence); the block stops at the first such job. One container at a time.
set -u
BLOCK=${1:?usage: run_phase4_batch.sh B1|B2|B3|B4|BR|BH}
W=$(cd "$(dirname "$0")/../../.." && pwd)
LOG=~/nmpc_sim/logs/rc_car_phase4_batch/$BLOCK
mkdir -p "$LOG"
IMG=nmpc-jazzy-sim:2026-10
SEEDS="1 2 3 4 5"
CORE="line circle figure_eight lane_change"

heavy() { pgrep -af '[v]ivado|[v]itis|[x]sim|[c]sim'; }

DECISION=~/nmpc_sim/logs/rc_car_phase4_batch/B1/rtf_decision.json
# f DEFAULT -> the factor to use: DEFAULT if the RTF check passed, 0.35 if it failed
f() {
  if [ -f "$DECISION" ] && python3 -c "import json,sys; sys.exit(0 if json.load(open('$DECISION'))['pass'] else 1)"; then
    echo "$1"
  else
    echo 0.35
  fi
}
# fb DEFAULT -> the baseline's factor. Nav2's controller_server paces its loop on WALL time
# (rclcpp::WallRate, found in B1 2026-11-22: 50 Hz in sim time at RTF 0.4), so the baseline runs at its
# intended 20 Hz only at RTF 1.0. With B1/rtf_policy.json {"baseline_rtf": 1.0} (written after the
# user's decision) the baseline uses that factor and the NMPC its per-arm factors; without the file,
# the RTF-check decision applies to both (f).
POLICY=~/nmpc_sim/logs/rc_car_phase4_batch/B1/rtf_policy.json
fb() {
  if [ -f "$POLICY" ]; then python3 -c "import json; print(json.load(open('$POLICY'))['baseline_rtf'])"; else f "$1"; fi
}
fn() {
  if [ -f "$POLICY" ]; then echo "$1"; else f "$1"; fi
}
need_decision() {
  [ -f "$DECISION" ] || { echo "STOP: run block B1 first (no RTF decision at $DECISION)"; exit 3; }
}

# run NAME PROFILE RTF CONTACT ARM_LAUNCH "TRACK_RUN ARGS..."   (ARM_LAUNCH: baseline | nmpc:SIZE:COMP:ARITH)
run() {
  local name=$1 profile=$2 rtf=$3 contact=$4 arm=$5 targs=$6
  if heavy; then echo "STOP: a Vivado/Vitis/xsim/csim job is running; $name not started"; exit 2; fi
  mkdir -p "$LOG/$name"
  local armcmd
  if [ "$arm" = baseline ]; then
    armcmd="ros2 launch /w/app/rc_car/launch/baseline_rpp.launch.py > /out/arm.log 2>&1 &"
  else
    IFS=: read -r _ size comp arith variant <<< "$arm"
    armcmd="ros2 launch /w/app/rc_car/launch/nmpc.launch.py size:=$size profile:=$profile compensation:=$comp arith:=$arith variant:=${variant:-phase4} > /out/arm.log 2>&1 &"
  fi
  echo "=== $(date +%H:%M:%S) $name ($profile, rtf $rtf, contact '${contact}', $arm)"
  timeout 7200 docker run --rm --memory 4g -e GZ_IP=127.0.0.1 -e ROS_DOMAIN_ID=77 -v "$W":/w:ro -v "$LOG/$name":/out \
    --entrypoint bash $IMG -c "
    source /opt/ros/jazzy/setup.bash; cd /tmp
    ros2 launch /w/app/rc_car/launch/rc_car_stack.launch.py profile:=$profile mocap_profile:=realistic_guess rtf:=$rtf ${contact:+contact_width:=$contact} > /out/stack.log 2>&1 &
    sleep 30
    $armcmd
    sleep 12
    python3 /w/app/rc_car/checks/track_run.py --arm ${arm%%:*} --profile $profile --rtf $rtf --label $name --out /out $targs
    kill %1 %2; pkill -f nmpc_node.py; sleep 2" 2>&1 | grep -E "^(baseline|nmpc)|Traceback|Error" | cut -c1-260
}

case $BLOCK in
B1)
  # 1. RTF-invariance check (review section 3) and the tick-timing smoke check (review 2.1), first
  run rtfcheck_baseline_rtf1.0 realistic_guess 1.0 "" baseline "--paths figure_eight --seeds 1"
  run rtfcheck_baseline_rtf0.4 realistic_guess 0.4 "" baseline "--paths figure_eight --seeds 1"
  run rtfcheck_nmpc10x40_rtf0.5 realistic_guess 0.5 "" nmpc:10x40:true:half "--paths lane_change figure_eight --seeds 1"
  run rtfcheck_nmpc10x40_rtf0.4 realistic_guess 0.4 "" nmpc:10x40:true:half "--paths figure_eight --seeds 1"
  run smoke_nmpc3x24 realistic_guess 0.35 "" nmpc:3x24:true:half "--paths lane_change --seeds 1"
  run smoke_nmpc10x60 realistic_guess 0.4 "" nmpc:10x60:true:half "--paths lane_change --seeds 1"
  python3 "$W/app/rc_car/checks/rtf_check.py" "$LOG"   # writes $LOG/rtf_decision.json (PASS or FAIL)
  # 2. the tuning runs, at the decided factors
  run b0_baseline_neighbourhood realistic_guess "$(f 1.0)" "" baseline \
      "--paths s_curve --seeds 100 101 --grid lookahead_time=0.15,0.2,0.25 min_lookahead_dist=0.1,0.15,0.2"
  run nmpc_tuning_10x40 realistic_guess "$(f 0.5)" "" nmpc:10x40:true:half \
      "--paths s_curve --seeds 100 101 --grid q_pos=10,31.6,100,316,1000 q_psi=1,3.16,10,31.6,100"
  ;;
B2)
  need_decision
  run core_baseline_realistic realistic_guess "$(fb 1.0)" "" baseline "--paths $CORE --seeds $SEEDS"
  run core_baseline_ideal ideal "$(fb 1.0)" "" baseline "--paths $CORE --seeds $SEEDS"
  run core_nmpc10x40_realistic realistic_guess "$(fn 0.5)" "" nmpc:10x40:true:half "--paths $CORE --seeds $SEEDS"
  run core_nmpc3x24_realistic realistic_guess "$(fn 0.35)" "" nmpc:3x24:true:half "--paths $CORE --seeds $SEEDS"
  run core_nmpc10x60_realistic realistic_guess "$(fn 0.4)" "" nmpc:10x60:true:half "--paths $CORE --seeds $SEEDS"
  ;;
B3)
  need_decision
  run core_nmpc10x40_ideal ideal "$(fn 0.5)" "" nmpc:10x40:true:half "--paths $CORE --seeds $SEEDS"
  run core_nmpc3x24_ideal ideal "$(fn 0.35)" "" nmpc:3x24:true:half "--paths $CORE --seeds $SEEDS"
  run core_nmpc10x60_ideal ideal "$(fn 0.4)" "" nmpc:10x60:true:half "--paths $CORE --seeds $SEEDS"
  run float_check_10x40 realistic_guess "$(fn 0.5)" "" nmpc:10x40:true:float "--paths figure_eight --seeds $SEEDS"
  ;;
B4)
  need_decision
  run comp_off_10x40 realistic_guess "$(fn 0.5)" "" nmpc:10x40:false:half "--paths figure_eight --speeds 1.0 1.5 --seeds $SEEDS"
  run comp_on15_10x40 realistic_guess "$(fn 0.5)" "" nmpc:10x40:true:half "--paths figure_eight --speeds 1.5 --seeds $SEEDS"
  run comp_off_3x24 realistic_guess "$(fn 0.35)" "" nmpc:3x24:false:half "--paths figure_eight --speeds 1.0 1.5 --seeds $SEEDS"
  run comp_on15_3x24 realistic_guess "$(fn 0.35)" "" nmpc:3x24:true:half "--paths figure_eight --speeds 1.5 --seeds $SEEDS"
  run contact10mm_baseline realistic_guess "$(fb 1.0)" 0.01 baseline "--paths figure_eight --seeds $SEEDS"
  run contact10mm_nmpc10x40 realistic_guess "$(fn 0.5)" 0.01 nmpc:10x40:true:half "--paths figure_eight --seeds $SEEDS"
  run sweep_baseline realistic_guess "$(fb 1.0)" "" baseline "--paths arcs_and_lines --speeds 0.5 1.0 1.5 2.0 --seeds $SEEDS"
  run sweep_nmpc10x40 realistic_guess "$(fn 0.5)" "" nmpc:10x40:true:half "--paths arcs_and_lines --speeds 0.5 1.0 1.5 2.0 --seeds $SEEDS"
  ;;
BR)
  if heavy; then echo "STOP: a Vivado/Vitis/xsim/csim job is running"; exit 2; fi
  mkdir -p "$LOG/c1c6_payload_1p5kg"
  echo "=== $(date +%H:%M:%S) C1-C6 with the 1.5 kg payload"
  timeout 1200 docker run --rm --memory 4g -e GZ_IP=127.0.0.1 -e ROS_DOMAIN_ID=77 -v "$W":/w:ro -v "$LOG/c1c6_payload_1p5kg":/out \
    --entrypoint bash $IMG -c "
    source /opt/ros/jazzy/setup.bash; cd /tmp
    ros2 launch /w/app/rc_car/launch/rc_car_sim.launch.py > /out/launch.log 2>&1 &
    sleep 30
    python3 /w/app/rc_car/checks/bringup_checks.py --out /out --label payload_1p5kg
    kill %1; sleep 2" 2>&1 | grep -E "^C[1-6]" | cut -c1-200
  run br_baseline_realistic realistic_guess 1.0 "" baseline "--paths $CORE --seeds $SEEDS"
  run br_nmpc10x40_redesign realistic_guess 0.5 "" nmpc:10x40:true:half:redesign "--paths $CORE --seeds $SEEDS"
  run br_nmpc3x24_redesign realistic_guess 0.35 "" nmpc:3x24:true:half:redesign "--paths $CORE --seeds $SEEDS"
  # second NMPC arm, NOT SELECTED BY THE RULE; run for steering smoothness (STOP R review): 10x40 only
  run br_nmpc10x40_redesign_smooth realistic_guess 0.5 "" nmpc:10x40:true:half:redesign_smooth "--paths $CORE --seeds $SEEDS"
  ;;
BH)
  # STOP BR review (2026-11-22): item 1, payload pairing: the frozen phase 4 10x40 NMPC at 1.5 kg, seeds 1-5;
  # item 2, held-out check: the carried-forward smoother variant and the baseline on NEW seeds 6-10
  run bh_nmpc10x40_phase4_1p5kg realistic_guess 0.5 "" nmpc:10x40:true:half:phase4 "--paths $CORE --seeds $SEEDS"
  run bh_heldout_nmpc10x40_redesign_smooth realistic_guess 0.5 "" nmpc:10x40:true:half:redesign_smooth "--paths $CORE --seeds 6 7 8 9 10"
  run bh_heldout_baseline realistic_guess 1.0 "" baseline "--paths $CORE --seeds 6 7 8 9 10"
  ;;
*) echo "unknown block $BLOCK"; exit 1 ;;
esac
echo "=== $(date +%H:%M:%S) block $BLOCK done; logs in $LOG"
