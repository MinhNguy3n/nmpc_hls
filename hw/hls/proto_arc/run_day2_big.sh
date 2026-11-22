#!/usr/bin/env bash
# Day-2 jobs that need about 7 GB: the deployed-evaluator S = 10 stored-cost cosims (they thrash at a
# 4.5 GB cap) and Vivado out-of-context implementation (vitis_hls + two Vivado processes). Waits until
# QUEUE_MIN_GB (default 8) is free before each job; run in a 7 GB-capped user unit.
set -uo pipefail
here="$(cd "$(dirname "$0")" && pwd)"
set +u; source /tools/Xilinx/2025.2/Vitis/settings64.sh >/dev/null 2>&1; set -u
avail_gb() { awk '/MemAvailable/ {print int($2/1048576)}' /proc/meminfo; }
wait_ok() { while [ "$(avail_gb)" -lt "${QUEUE_MIN_GB:-8}" ]; do sleep 60; done; }
run() { local dir="$1"; shift; rm -rf "$dir"; mkdir -p "$dir"
    ( cd "$dir" && env "$@" nice -n 5 vitis-run --mode hls --tcl "$here/hls_eval.tcl" > run.log 2>&1; echo "exit $?" >> run.log ); }
echo "big start $(date +%H:%M); waiting for ${QUEUE_MIN_GB:-8} GB"
"$here/gen_eval_headers.sh" 10 >/dev/null
for seed in 1 22; do
    base="$here/stored_gate/S10/seed$seed"; j0="$here/stored_j0/j0_S10_s$seed.dump"
    wait_ok; echo "$(date +%H:%M) cosim ref S=10 seed=$seed"
    run "$base/ref" EVAL_DUT=ref PROTO_S=10 PROTO_II=8 PROTO_TRIALS=1 PROTO_SEED=$seed PROTO_CALLS=1 \
        PROTO_STORED="$j0" PROTO_NODEADLOCK=1
    n="$base/new/rtl_out.txt"; r="$base/ref/rtl_out.txt"
    if ! grep -q "co-simulation finished" "$base/new/run.log" || ! grep -q "co-simulation finished" "$base/ref/run.log"; then
        v="INCOMPLETE (cosim did not finish; dump is not RTL)"
    elif [ -s "$n" ] && cmp -s "$n" "$r"; then v="IDENTICAL ($(wc -l < "$n") words)"; else v="DIFFERENT"; fi
    upd=$(join <(grep '^t0 c0 find' "$j0" | awk '{print $4, $5}' | sort) <(grep '^t0 c1 find' "$n" 2>/dev/null | awk '{print $4, $5}' | sort) | awk '$2 != $3' | wc -l)
    lat() { grep -h Verilog "$1"/proj_*/sol/sim/report/*cosim.rpt 2>/dev/null | awk -F'|' '{print $4}' | tr -d ' '; }
    echo "S=10 seed=$seed stored-cost gate RTL(new) vs RTL(deployed): $v; RTL local-best updates $upd of 10; latency new $(lat "$base/new"), deployed $(lat "$base/ref")"
done
for job in "new 10 8" "new 10 16" "ref 10 8"; do
    set -- $job
    wait_ok; echo "$(date +%H:%M) OOC impl $1 S=$2 II=$3"
    run "$here/ooc/$1_S$2_II$3" EVAL_DUT=$1 PROTO_S=$2 PROTO_II=$3 PROTO_COSIM=0 PROTO_EXPORT=1
    tail -1 "$here/ooc/$1_S$2_II$3/run.log"
done
echo "big end $(date +%H:%M)"
