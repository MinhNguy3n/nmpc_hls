#!/usr/bin/env bash
# Vivado OOC for the day-2 comparison, one job at a time: HLS csynth (if not done), then a single
# Vivado process on sol/syn/verilog (ooc_vivado.tcl). Waits for QUEUE_MIN_GB (default 6) free per job.
set -uo pipefail
here="$(cd "$(dirname "$0")" && pwd)"
set +u; source /tools/Xilinx/2025.2/Vitis/settings64.sh >/dev/null 2>&1; set -u
avail_gb() { awk '/MemAvailable/ {print int($2/1048576)}' /proc/meminfo; }
wait_ok() { while [ "$(avail_gb)" -lt "${QUEUE_MIN_GB:-6}" ]; do sleep 60; done; }
echo "ooc start $(date +%H:%M); waiting for ${QUEUE_MIN_GB:-6} GB"
"$here/gen_eval_headers.sh" 10 >/dev/null
for job in "new 10 8" "new 10 16" "ref 10 8"; do
    set -- $job; d="$here/ooc/$1_S$2_II$3"; v="$d/proj_eval_$1_S$2_II$3/sol/syn/verilog"
    if [ ! -d "$v" ]; then
        wait_ok; echo "$(date +%H:%M) csynth $1 S=$2 II=$3"; mkdir -p "$d"
        ( cd "$d" && EVAL_DUT=$1 PROTO_S=$2 PROTO_II=$3 PROTO_COSIM=0 nice -n 5 vitis-run --mode hls \
            --tcl "$here/hls_eval.tcl" > run.log 2>&1 )
    fi
    wait_ok; echo "$(date +%H:%M) vivado OOC $1 S=$2 II=$3"
    ( cd "$d" && nice -n 5 vivado -mode batch -nojournal -log vivado_ooc.log -source "$here/ooc_vivado.tcl" \
        -tclargs "$v" "$d/vivado_ooc" 8.0 > /dev/null 2>&1 )
    grep -h "OOC_DONE" "$d/vivado_ooc.log" || echo "  FAILED (see $d/vivado_ooc.log)"
done
echo "ooc end $(date +%H:%M)"
