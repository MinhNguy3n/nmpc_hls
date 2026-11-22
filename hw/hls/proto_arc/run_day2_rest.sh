#!/usr/bin/env bash
# Rest of the day-2 queue without the deployed S = 10 cosims, which need about 7 GB (they thrash at a
# 4.5 GB cap): the new-evaluator S = 10 seed-22 stored-cost cosim (for a later comparison), then Vivado
# out-of-context implementation. Same rules as run_day2_queue.sh: one job at a time, memory guard.
set -uo pipefail
here="$(cd "$(dirname "$0")" && pwd)"
set +u; source /tools/Xilinx/2025.2/Vitis/settings64.sh >/dev/null 2>&1; set -u
avail_gb() { awk '/MemAvailable/ {print int($2/1048576)}' /proc/meminfo; }
wait_ok() { while [ "$(avail_gb)" -lt "${QUEUE_MIN_GB:-5}" ]; do sleep 60; done; }
run() { local dir="$1"; shift; mkdir -p "$dir"
    ( cd "$dir" && env "$@" nice -n 5 vitis-run --mode hls --tcl "$here/hls_eval.tcl" > run.log 2>&1; echo "exit $?" >> run.log ); }
echo "rest start $(date +%H:%M)"
"$here/gen_eval_headers.sh" 10 >/dev/null
wait_ok; echo "$(date +%H:%M) cosim new S=10 seed=22"
run "$here/stored_gate/S10/seed22/new" EVAL_DUT=new PROTO_S=10 PROTO_II=8 PROTO_TRIALS=1 PROTO_SEED=22 PROTO_CALLS=1 \
    PROTO_STORED="$here/stored_j0/j0_S10_s22.dump" PROTO_NODEADLOCK=1
grep -q "co-simulation finished" "$here/stored_gate/S10/seed22/new/run.log" && echo "  new S=10 seed=22 finished" || echo "  new S=10 seed=22 INCOMPLETE"
for job in "new 10 8" "new 10 16" "ref 10 8"; do
    set -- $job
    wait_ok; echo "$(date +%H:%M) OOC impl $1 S=$2 II=$3"
    run "$here/ooc/$1_S$2_II$3" EVAL_DUT=$1 PROTO_S=$2 PROTO_II=$3 PROTO_COSIM=0 PROTO_EXPORT=1
    tail -1 "$here/ooc/$1_S$2_II$3/run.log"
done
echo "rest end $(date +%H:%M)"
