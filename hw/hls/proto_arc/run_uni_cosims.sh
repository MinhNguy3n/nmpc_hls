#!/usr/bin/env bash
# Unicycle model point (ARC paper): RTL-vs-RTL gate of the interleaved evaluator (eval_multi.cpp,
# II 8) against the deployed evaluator (source/src/hls_pso.cpp), both built with UNICYCLE_CONFIG
# (gen_headers.sh PROTO_MODEL=unicycle), at S = 3 and S = 10. Strictly one simulation at a time.
#   ./run_uni_cosims.sh OUTDIR
# Per S: (1) seed 4242, 1 trial = 3 calls; (2) the stored-cost update/retain test, fed by a C dump
# of the same seed. Afterwards: OUTDIR/summary.txt (verdicts, cosim latencies, csynth resources).
# Run it in tmux under a memory cap, e.g.:
#   systemd-run --user --scope -p MemoryMax=8G -p MemorySwapMax=0 ./run_uni_cosims.sh <OUTDIR>
set -euo pipefail
here="$(cd "$(dirname "$0")" && pwd)"
out="$(mkdir -p "$1" && cd "$1" && pwd)"
if pgrep -af '[g]z sim|[r]un_phase4_batch' > /dev/null; then
    echo "refusing: a Gazebo batch is running (one heavy job at a time)" >&2; exit 1
fi
export PROTO_MODEL=unicycle PROTO_NODEADLOCK=1 SEQ=1
X=/tools/Xilinx/2025.2
log() { echo "[$(date +%H:%M:%S)] $*"; }

# C dumps (half arithmetic) for the stored-cost inputs; also the C-level equality check.
for S in 3 10; do
    "$here/gen_eval_headers.sh" "$S" > /dev/null
    inc="$here/gen/uni_S$S/include"
    for d in new ref; do
        if [ $d = new ]; then src=("$here/eval_multi.cpp"); else
            src=("$here/../source/src/hls_pso.cpp" "$here/../source/src/hls_nonlinear_solver.cpp"); fi
        g++ -std=c++11 -O0 -w -D__VITIS__ -DPSO_CONFIG -DUNICYCLE_CONFIG -DHW_ARITH_CSIM -DUSE_FAST_SIN_COS \
            -DHW_MODEL_HALF_CSIM -DPROTO_II=8 -I"$here" -I"$inc" -I"$inc/models" -I"$X/Vitis/include" \
            -o "$out/c_${d}_S$S" "$here/tb_eval.cpp" "$here/../source/src/aux_functions.cpp" "${src[@]}" \
            -L"$X/lnx64/tools/fpo_v7_1" -lIp_floating_point_v7_1_bitacc_cmodel -Wl,--disable-new-dtags \
            -Wl,-rpath,"$X/lnx64/tools/fpo_v7_1"
        "$out/c_${d}_S$S" 1 "$out/c_${d}_S$S.dump" 4242 > /dev/null
    done
    cmp -s "$out/c_new_S$S.dump" "$out/c_ref_S$S.dump" \
        && log "S=$S C(new) vs C(deployed), seed 4242: IDENTICAL" \
        || { log "S=$S C(new) vs C(deployed): DIFFERENT, stopping"; exit 1; }
done

for S in 3 10; do
    log "S=$S cosim pair, seed 4242 (new, then deployed)"
    PROTO_SEED=4242 "$here/cmp_eval_rtl.sh" "$out/S${S}_seed4242" "$S:8" 1 | tee "$out/S${S}_seed4242.verdict"
    log "S=$S cosim pair, stored-cost update/retain test"
    PROTO_SEED=4242 PROTO_CALLS=1 PROTO_STORED="$out/c_ref_S$S.dump" \
        "$here/cmp_eval_rtl.sh" "$out/S${S}_stored" "$S:8" 1 | tee "$out/S${S}_stored.verdict"
    grep -h "stored-cost mode" "$out/S${S}_stored"/*/run.log | head -2 | tee -a "$out/S${S}_stored.verdict" || true
done

{
    echo "Unicycle (UNICYCLE_CONFIG) evaluator gate, II 8, xck26 8 ns, $(date -Is)"
    echo "Overlay files: $(cat "$here/gen/uni_src/OVERLAY.from")"; sed 's/^/  /' "$here/gen/uni_src/OVERLAY.sha256"
    cat "$out"/S*_seed4242.verdict "$out"/S*_stored.verdict
    for r in "$out"/S*_seed4242/*/proj_*/sol/syn/report/csynth.rpt; do
        # Top row of the module table: latency (cycles), est. slack, BRAM, DSP, FF, LUT.
        echo "--- $r"; sed -n 17,18p "$r"; grep -m1 "^|+ evaluateFitnessAndDetectLocalBest " "$r" || true
    done
} > "$out/summary.txt"
log "done: $out/summary.txt"
