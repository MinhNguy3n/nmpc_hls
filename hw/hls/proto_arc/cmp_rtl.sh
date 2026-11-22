#!/usr/bin/env bash
# Run cosim of the new design and of the original serial predictor (rk4_ref.cpp) on the same
# testbench inputs, then compare their raw RTL output words.
#   ./cmp_rtl.sh OUTDIR "10:8 10:16 1:8"     (ref is run once per distinct S)
set -euo pipefail
here="$(cd "$(dirname "$0")" && pwd)"
out="$(mkdir -p "$1" && cd "$1" && pwd)"; pairs="$2"
refs=""; for p in $pairs; do s="${p%%:*}"; case " $refs " in *" $s:8 "*) ;; *) refs="$refs $s:8";; esac; done
RUN_ROOT="$out/new" PROTO_COSIM=1 "$here/run_synth_sweep.sh" "$pairs" &
RUN_ROOT="$out/ref" PROTO_COSIM=1 PROTO_DUT=rk4_ref.cpp "$here/run_synth_sweep.sh" "$refs" &
wait
for p in $pairs; do
    s="${p%%:*}"; ii="${p##*:}"
    new="$out/new/S${s}_II${ii}/rtl_out.txt"; ref="$out/ref/S${s}_II8/rtl_out.txt"
    lat_new=$(grep -h Verilog "$out"/new/S${s}_II${ii}/proj_*/sol/sim/report/*cosim.rpt | awk -F'|' '{print $4}' | tr -d ' ')
    lat_ref=$(grep -h Verilog "$out"/ref/S${s}_II8/proj_*/sol/sim/report/*cosim.rpt | awk -F'|' '{print $4}' | tr -d ' ')
    # A dump is RTL output only if the cosim post-check ran: the testbench's C pre-run writes the
    # same file first, so an interrupted cosim leaves C outputs behind.
    done_new=$(grep -c "co-simulation finished" "$out/new/S${s}_II${ii}/run.log" || true)
    done_ref=$(grep -c "co-simulation finished" "$out/ref/S${s}_II8/run.log" || true)
    if [ "$done_new" = 0 ] || [ "$done_ref" = 0 ]; then verdict="INCOMPLETE (cosim did not finish; dump is not RTL)"
    elif [ -s "$new" ] && cmp -s "$new" "$ref"; then verdict="IDENTICAL ($(wc -l < "$new") words)"; else verdict="DIFFERENT"; fi
    echo "S=$s II=$ii: RTL(new) vs RTL(orig serial): $verdict; cosim latency new $lat_new, orig $lat_ref cycles"
    grep -h "words compared" "$out"/new/S${s}_II${ii}/run.log | tail -1 | sed 's/^/    new RTL vs C: /'
done
