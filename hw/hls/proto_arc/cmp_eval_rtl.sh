#!/usr/bin/env bash
# RTL-vs-RTL evaluator gate: cosim the new evaluator and the deployed one on the same tb_eval
# inputs, compare their raw output dumps.
#   ./cmp_eval_rtl.sh OUTDIR "10:8 3:8" [TRIALS]
set -euo pipefail
here="$(cd "$(dirname "$0")" && pwd)"
out="$(mkdir -p "$1" && cd "$1" && pwd)"; pairs="$2"; trials="${3:-4}"
set +u; source /tools/Xilinx/2025.2/Vitis/settings64.sh >/dev/null 2>&1; set -u
launch() {  # dut S II
    local run="$out/$1_S$2_II$3"; mkdir -p "$run"
    ( cd "$run" && EVAL_DUT="$1" PROTO_S="$2" PROTO_II="$3" PROTO_TRIALS="$trials" \
        nice -n 5 vitis-run --mode hls --tcl "$here/hls_eval.tcl" > run.log 2>&1; echo "exit $?" >> run.log ) &
    # SEQ=1 runs one simulation at a time (an S=10 evaluator xsim can exceed 5 GB of RAM).
    if [ "${SEQ:-0}" = 1 ]; then wait; fi
}
refs=""
for p in $pairs; do
    s="${p%%:*}"; ii="${p##*:}"
    "$here/gen_eval_headers.sh" "$s" >/dev/null
    launch new "$s" "$ii"
    case " $refs " in *" $s "*) ;; *) refs="$refs $s"; launch ref "$s" 8;; esac
done
wait
for p in $pairs; do
    s="${p%%:*}"; ii="${p##*:}"
    new="$out/new_S${s}_II${ii}"; ref="$out/ref_S${s}_II8"
    lat() { grep -h Verilog "$1"/proj_*/sol/sim/report/*cosim.rpt 2>/dev/null | awk -F'|' '{print $4"/"$5"/"$6}' | tr -d ' '; }
    if ! grep -q "co-simulation finished" "$new/run.log" || ! grep -q "co-simulation finished" "$ref/run.log"; then
        v="INCOMPLETE (cosim did not finish; dump is not RTL)"
    elif [ -s "$new/rtl_out.txt" ] && cmp -s "$new/rtl_out.txt" "$ref/rtl_out.txt"; then
        v="IDENTICAL ($(wc -l < "$new/rtl_out.txt") words)"; else v="DIFFERENT"; fi
    echo "S=$s II=$ii evaluator RTL(new) vs RTL(deployed): $v"
    echo "    cosim latency min/avg/max per call: new $(lat "$new"), deployed $(lat "$ref")"
done
