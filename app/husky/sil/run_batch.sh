#!/bin/bash
# Stage 3a SIL screen in the A300 simulation: every path x PSO size x predictor gain x seed.
#
# Run INSIDE the container, with the headless simulation and the IMU bridge already up (plan
# section 2.1). The script sets the real-time factor itself (V15: set_physics applies the whole
# Physics message, so gravity and the step size are sent too; omitted fields become 0) and checks
# that the robot rests on the floor. Before each run the robot is put back at the origin with Gazebo's
# set_pose; odometry is not reset by that, which does not matter: the node attaches the path to
# its feedback pose at arming and the metrics attach it to the true pose. The batch stops at the
# first run that never took ownership, lost it, or in which the robot tilted over (metrics
# validity), because every later run would start from a broken state. Every other rejection
# (published constraint violation, stop fallback, C_sim misses > 1 %, tracking thresholds) is
# printed, written to batch_summary.csv with all metrics, and the batch continues: a rejected
# run is never a passing candidate.
#
# Feedback: FEEDBACK/IMU/ESTIMATOR select the SIL estimator (default: the simulation-only
# velocity + IMU-yaw-rate EKF, variant "driver", of launch/a300_sil_estimator.launch.py, which
# must be running); FEEDBACK=platform/odom/filtered IMU=sensors/imu_0/data is the generated EKF.
#
# Path placement (GPT-6 2026-11-19): START="x y yaw" (world) is where the robot is put before every
# run; SIZE (radius, m) and SPEED (m/s) define the paths. Before the first run every path's swept
# robot circle is checked against the warehouse obstacles (sil/clearance.py, >= MIN_CLEARANCE m).
# Primary (low curvature): SIZE=2 SPEED=0.2 START="0 3.9 1.5708" (the default);
# stress case: SIZE=1 SPEED=0.3, same start.
#
#   bash sil/run_batch.sh             # 36 runs at the primary geometry
#   PATHS=circle CFGS=7x30 GAINS=plain SEEDS=1 bash sil/run_batch.sh  # a subset
set -u
H=$(cd "$(dirname "$0")/.." && pwd)
OUT=${OUT:-$HOME/logs/sil_batch_$(date +%Y%m%d_%H%M)}
PATHS=${PATHS:-"circle figure8"}
CFGS=${CFGS:-"4x50 7x30 9x24"}
GAINS=${GAINS:-"plain sim-derived"}
SEEDS=${SEEDS:-"1 2 3"}
WORLD=${WORLD:-warehouse}
RTF=${RTF:-0.25}
STEP=${STEP:-0.003}  # the warehouse world's max_step_size
MAX_WALL=${MAX_WALL:-1800}
START=${START:-"0 3.9 1.5708"}
SIZE=${SIZE:-2}
SPEED=${SPEED:-0.2}
MIN_CLEARANCE=${MIN_CLEARANCE:-1.0}
read -r SX SY SYAW <<< "$START"
QZ=$(python3 -c "import math; print(math.sin($SYAW / 2))"); QW=$(python3 -c "import math; print(math.cos($SYAW / 2))")
FEEDBACK=${FEEDBACK:-sil/driver/odom/filtered}
IMU=${IMU:-sil/driver/imu}
ESTIMATOR=${ESTIMATOR:-"SIL EKF: wheel v_x + IMU yaw rate, gyro variance 2.75e-6 (driver default)"}

gz_srv() { gz service -s "/world/$WORLD/$1" --reqtype "$2" --reptype gz.msgs.Boolean --timeout 3000 --req "$3" > /dev/null; }
robot_z() {
  timeout 10 gz topic -e -t "/world/$WORLD/dynamic_pose/info" -n 1 \
    | awk '/name: "a300_00000\/robot"/{f=1} f && /^ *z:/{print $2; exit}'
}

reset_pose() {
  sleep 2  # the robot stops on the mux and controller timeouts after the node stops publishing
  gz_srv set_pose gz.msgs.Pose "name: \"a300_00000/robot\", position: {x: $SX, y: $SY, z: 0.15}, orientation: {z: $QZ, w: $QW}" || return 1
  sleep 4  # about 1 s of simulation at RTF 0.25 to settle
}

cd "$H" || exit 1
for path in $PATHS; do
  python3 -m sil.clearance --path "$path" --size "$SIZE" --speed "$SPEED" --start $START --min "$MIN_CLEARANCE" \
    || { echo "path $path not collision-clear at START=$START: batch not started"; exit 3; }
done
gz_srv set_physics gz.msgs.Physics "max_step_size: $STEP, real_time_factor: $RTF, gravity: {z: -9.8}" \
  || { echo "set_physics failed"; exit 3; }
gz_srv set_pose gz.msgs.Pose 'name: "a300_00000/robot", position: {z: 0.6}, orientation: {w: 1}'  # drop test
sleep 8
z=$(robot_z)
python3 -c "import sys; sys.exit(0 if 0.10 < float(sys.argv[1]) < 0.20 else 1)" "${z:-9}" \
  || { echo "drop test failed (robot z = $z m after 8 s): gravity or physics wrong, see plan V15"; exit 3; }
echo "physics: RTF $RTF, step $STEP s, gravity on (robot at rest, z = $z m)"
mkdir -p "$OUT"
echo "SIL batch: $PATHS x $CFGS x $GAINS x seeds $SEEDS -> $OUT"
for path in $PATHS; do for cfg in $CFGS; do for gain in $GAINS; do for seed in $SEEDS; do
  reset_pose || { echo "set_pose failed: batch stopped"; exit 3; }
  python3 -m sil.sil_node --sim --path "$path" --size "$SIZE" --speed "$SPEED" --pso "$cfg" --gain "$gain" --seed "$seed" --auto-arm \
      --rtf "$RTF" --max-wall "$MAX_WALL" --out-dir "$OUT" --feedback-topic "$FEEDBACK" --imu-topic "$IMU" \
      --estimator "$ESTIMATOR" --ros-args -p use_sim_time:=true > "$OUT/node.log" 2>&1
  run=$(ls -td "$OUT"/*/ | head -1)
  mv "$OUT/node.log" "$run/node.log"
  python3 -m sil.metrics "$run" --out "$run/report.json" > /dev/null
  summary=$(python3 -m sil.batch_summary "$run/report.json" "$OUT/batch_summary.csv")
  echo "$(date +%T) $(basename "$run") $summary"
  case "$summary" in
    *"never owned"*|*"ownership lost"*) echo "the node did not keep ownership: batch stopped"; exit 2 ;;
    *"robot tipped"*|*"validity unknown"*) echo "the robot tilted over (or no tilt data): batch stopped"; exit 2 ;;
  esac
done; done; done; done
echo "done: $OUT"
