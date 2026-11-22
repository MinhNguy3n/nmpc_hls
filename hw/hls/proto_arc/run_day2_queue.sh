#!/usr/bin/env bash
# Day-2 follow-up queue (GPT-6 review): (1) the stored-cost update/retain RTL gate, 1 call per cosim,
# new vs deployed, S = 3 and 10, seeds 1 and 22; (2) Vivado out-of-context implementation of the new
# evaluator (S = 10, II 8 and II 16) and the deployed one (S = 10). One job at a time; each job waits
# until the PC screen is done and enough memory is free. Run inside a MemoryMax-capped user unit.
set -uo pipefail
here="$(cd "$(dirname "$0")" && pwd)"
set +u; source /tools/Xilinx/2025.2/Vitis/settings64.sh >/dev/null 2>&1; set -u
avail_gb() { awk '/MemAvailable/ {print int($2/1048576)}' /proc/meminfo; }
wait_ok() {  # $1 = GB needed (QUEUE_MIN_GB overrides)
    local need="${QUEUE_MIN_GB:-$1}"
    while systemctl --user is-active -q arc-screen-half || [ "$(avail_gb)" -lt "$need" ]; do sleep 60; done
}
run() {  # dir, then env assignments
    local dir="$1"; shift
    mkdir -p "$dir"
    ( cd "$dir" && env "$@" nice -n 5 vitis-run --mode hls --tcl "$here/hls_eval.tcl" > run.log 2>&1; echo "exit $?" >> run.log )
}
echo "queue start $(date +%H:%M); waiting for the screen and memory"
for S in 3 10; do "$here/gen_eval_headers.sh" "$S" >/dev/null; done
for S in 3 10; do
    for seed in 1 22; do
        base="$here/stored_gate/S$S/seed$seed"; j0="$here/stored_j0/j0_S${S}_s$seed.dump"
        for dut in new ref; do
            wait_ok 8
            echo "$(date +%H:%M) cosim $dut S=$S seed=$seed"
            run "$base/$dut" EVAL_DUT=$dut PROTO_S=$S PROTO_II=8 PROTO_TRIALS=1 PROTO_SEED=$seed PROTO_CALLS=1 \
                PROTO_STORED="$j0" PROTO_NODEADLOCK=1
        done
        n="$base/new/rtl_out.txt"; r="$base/ref/rtl_out.txt"
        if ! grep -q "co-simulation finished" "$base/new/run.log" || ! grep -q "co-simulation finished" "$base/ref/run.log"; then
            v="INCOMPLETE (cosim did not finish; dump is not RTL)"
        elif [ -s "$n" ] && cmp -s "$n" "$r"; then v="IDENTICAL ($(wc -l < "$n") words)"; else v="DIFFERENT"; fi
        # Updates in the RTL: stored best (J0) vs the RTL's local_find after the call.
        upd=$(join <(grep '^t0 c0 find' "$j0" | awk '{print $4, $5}' | sort) <(grep '^t0 c1 find' "$n" 2>/dev/null | awk '{print $4, $5}' | sort) | awk '$2 != $3' | wc -l)
        lat() { grep -h Verilog "$1"/proj_*/sol/sim/report/*cosim.rpt 2>/dev/null | awk -F'|' '{print $4}' | tr -d ' '; }
        echo "S=$S seed=$seed stored-cost gate RTL(new) vs RTL(deployed): $v; RTL local-best updates $upd of $S; latency new $(lat "$base/new"), deployed $(lat "$base/ref")"
    done
done
for job in "new 10 8" "new 10 16" "ref 10 8"; do
    set -- $job
    wait_ok 8
    echo "$(date +%H:%M) OOC impl $1 S=$2 II=$3"
    run "$here/ooc/$1_S$2_II$3" EVAL_DUT=$1 PROTO_S=$2 PROTO_II=$3 PROTO_COSIM=0 PROTO_EXPORT=1
    tail -1 "$here/ooc/$1_S$2_II$3/run.log"
done
echo "queue end $(date +%H:%M)"
