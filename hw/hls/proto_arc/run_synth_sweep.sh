#!/usr/bin/env bash
# Launch one Vitis HLS synthesis per (S, II) in its own run directory, in parallel.
#   ./run_synth_sweep.sh "10:16 10:8 10:4"
set -euo pipefail
here="$(cd "$(dirname "$0")" && pwd)"
set +u; source /tools/Xilinx/2025.2/Vitis/settings64.sh >/dev/null 2>&1; set -u
command -v vitis-run >/dev/null || { echo "vitis-run not found" >&2; exit 1; }
[ -d "$here/gen/include" ] || "$here/gen_headers.sh"
for pair in ${1:-"10:16 10:8 10:4"}; do
    S="${pair%%:*}"; II="${pair##*:}"
    run="${RUN_ROOT:-$here/runs}/S${S}_II${II}"
    mkdir -p "$run"
    ( cd "$run" && PROTO_S="$S" PROTO_II="$II" PROTO_COSIM="${PROTO_COSIM:-0}" \
        nice -n 5 vitis-run --mode hls --tcl "$here/hls_rk4_multi.tcl" > "$run/run.log" 2>&1; \
      echo "exit $?" >> "$run/run.log" ) &
done
wait
