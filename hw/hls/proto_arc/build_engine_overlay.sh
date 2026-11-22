#!/usr/bin/env bash
# Full-solver build with the interleaved evaluator (ARC 2027), in this worktree only:
#   1. hls      header copy with _n_S = S and _maxiter = K; the five IPs into hw/hls/vitis_ip_repo
#               (four unchanged from source/, nmpc_solver_costF from eval_multi.cpp), unpacked;
#               a copy is kept in hw/hls/ip_engine_<NAME>/.
#   2. vivado   hw/hdl/scripts/build_debug_variant.tcl <NAME> <JOBS> (same ILA debug design as the
#               3x24 baselines) -> hw/exported_platform/<NAME>/.
#   3. package  app/kria-pynq/package_overlay.py -> app/kria-pynq/overlay-<NAME> (manifest S x K).
# Usage: build_engine_overlay.sh <S> <K> <NAME> [hls|vivado|resume|package|all|resume+package] [II] [JOBS]
#   JOBS: parallel Vivado runs (default 1: with 3, three IP synth runs plus the parent exceeded an 8 GB cap)
#   e.g. build_engine_overlay.sh 3 24 engine_3x24 all 8 3
# The main checkout (its vitis_ip_repo, overlays and exported platforms) is never touched.
set -euo pipefail
S="$1"; K="$2"; NAME="$3"; STAGE="${4:-all}"; II="${5:-8}"; JOBS="${6:-1}"
here="$(cd "$(dirname "$0")" && pwd)"
hls="$(cd "$here/.." && pwd)"; repo="$(cd "$hls/../.." && pwd)"
case "$repo" in */.claude/worktrees/*) ;; *) echo "refusing: not inside a .claude/worktrees checkout ($repo)" >&2; exit 1;; esac
set +u; source /tools/Xilinx/2025.2/Vitis/settings64.sh >/dev/null 2>&1; set -u
log() { echo "[$(date +%H:%M:%S)] $*"; }

stage_hls() {
    local cfg="$here/gen/cfg_${NAME}/include"
    rm -rf "$here/gen/cfg_${NAME}"; mkdir -p "$here/gen/cfg_${NAME}"
    cp -r "$hls/source/include" "$cfg"
    # CRLF-safe: only the number is replaced, the line ending is kept.
    sed -i -E "s/^(#define _n_S[[:space:]]+)[0-9]+/\1$S/; s/^(#define _maxiter[[:space:]]+)[0-9]+/\1$K/" "$cfg/config.hpp"
    grep -qE "^#define _n_S[[:space:]]+$S\b" "$cfg/config.hpp" && grep -qE "^#define _maxiter[[:space:]]+$K\b" "$cfg/config.hpp" \
        || { echo "config patch failed" >&2; exit 1; }
    diff -r "$hls/source/include" "$cfg" | grep -E '^[<>]' | grep -vE '_n_S|_maxiter' && { echo "config copy differs beyond _n_S/_maxiter" >&2; exit 1; } || true
    "$here/gen_eval_headers.sh" "$S" >/dev/null
    sed -i -E "s/^(#define _maxiter[[:space:]]+)[0-9]+/\1$K/" "$here/gen/S$S/include/config.hpp"
    local ipr="$hls/vitis_ip_repo" work="$hls/engine_build/$NAME"
    mkdir -p "$ipr" "$work"
    log "HLS: five IPs for ${S}x${K}, evaluator II $II -> $ipr"
    ( cd "$work" && ENGINE_CFG_INC="$cfg" ENGINE_EVAL_INC="$here/gen/S$S/include" ENGINE_II="$II" ENGINE_IP_REPO="$ipr" \
        vitis-run --mode hls --tcl "$here/hls_script_engine_ips.tcl" > "$work/hls.log" 2>&1 ) \
        || { echo "HLS failed, see $work/hls.log" >&2; exit 1; }
    for ip in nmpc_solver_fsm nmpc_solver_init_s nmpc_solver_update_s nmpc_solver_costF nmpc_solver_global_minimum; do
        [ -s "$ipr/$ip.zip" ] || { echo "missing $ipr/$ip.zip" >&2; exit 1; }
        rm -rf "$ipr/$ip"; mkdir -p "$ipr/$ip"; unzip -q -o "$ipr/$ip.zip" -d "$ipr/$ip"
    done
    # The evaluator IP must come from eval_multi.cpp (its particle pipeline), not from hls_pso.cpp.
    grep -rq "particle_loop" "$ipr/nmpc_solver_costF/hdl/verilog/" || { echo "costF IP is not the interleaved evaluator" >&2; exit 1; }
    rm -rf "$hls/ip_engine_$NAME"; cp -r "$ipr" "$hls/ip_engine_$NAME"
    # Informational only (2025.2 csynth.rpt has no "Latency (cycles)" line; read the XML), never fatal.
    echo "  nmpc_solver_costF latency (HLS): $(grep -oE '<(Best|Worst)-caseLatency>[0-9]+' \
        "$work/nmpc_solver_costF/solution_system/syn/report/csynth.xml" | head -2 | sed 's/.*>//' | paste -sd- || true) cycles"
    log "HLS done"
}

stage_vivado() {
    # The block design reads hw/hls/vitis_ip_repo; make sure it holds this configuration's IPs.
    [ -d "$hls/ip_engine_$NAME" ] || { echo "run the hls stage for $NAME first" >&2; exit 1; }
    for ip in nmpc_solver_fsm nmpc_solver_init_s nmpc_solver_update_s nmpc_solver_costF nmpc_solver_global_minimum; do
        cmp -s "$hls/ip_engine_$NAME/$ip.zip" "$hls/vitis_ip_repo/$ip.zip" \
            || { echo "vitis_ip_repo/$ip.zip is not the $NAME build; rerun the hls stage" >&2; exit 1; }
    done
    # The project script imports the block-design wrapper (and a 2025.1 checkpoint that
    # build_debug_variant.tcl then clears) from hw/hdl/tcl_run/nmpc_solver_v2, which is gitignored and
    # exists only in the main checkout; import_files only copies from it, so link it read-only.
    # The block design also needs the RNG IP (tu-dresden_turun-yliopisto:hls:rand_wrapper:1.0). It does
    # not depend on S or K and is not rebuilt here: copy the prebuilt one the deployed overlays use
    # from the main checkout's (gitignored) vitis_ip_repo.
    local main_rng="$repo/../../../hw/hls/vitis_ip_repo/pseudorand_stream"
    grep -q "<spirit:vendor>tu-dresden_turun-yliopisto</spirit:vendor>" "$main_rng/component.xml" \
        && grep -q "<spirit:name>rand_wrapper</spirit:name>" "$main_rng/component.xml" \
        || { echo "missing the rand_wrapper IP in $main_rng" >&2; exit 1; }
    rm -rf "$hls/vitis_ip_repo/pseudorand_stream"
    cp -r "$main_rng" "$main_rng.zip" "$hls/vitis_ip_repo/"
    local main_tcl_run="$repo/../../../hw/hdl/tcl_run"
    [ -e "$repo/hw/hdl/tcl_run" ] || ln -s "$(cd "$main_tcl_run" && pwd)" "$repo/hw/hdl/tcl_run"
    [ -f "$repo/hw/hdl/tcl_run/nmpc_solver_v2/nmpc_solver_v2.gen/sources_1/bd/design_nmpc_solver/hdl/design_nmpc_solver_wrapper.v" ] \
        || { echo "missing the block-design wrapper under hw/hdl/tcl_run" >&2; exit 1; }
    [ ! -e "$repo/hw/hdl/build/$NAME" ] || { echo "hw/hdl/build/$NAME exists; delete it first" >&2; exit 1; }
    mkdir -p "$repo/hw/hdl/build"   # Vivado does not create the directory of its -log file
    log "Vivado: $NAME, $JOBS jobs (takes about 1-2 h); log: $repo/hw/hdl/build/vivado_$NAME.log"
    ( cd "$repo" && vivado -mode batch -nojournal -log "$repo/hw/hdl/build/vivado_$NAME.log" \
        -source hw/hls/proto_arc/build_engine_vivado.tcl -tclargs "$NAME" "$JOBS" > /dev/null )
    grep -A6 "Design Timing Summary" "$repo/hw/exported_platform/$NAME/timing_summary.rpt" | tail -1 \
        || echo "  (WNS line not found; read $repo/hw/exported_platform/$NAME/timing_summary.rpt)"
    log "Vivado done: $repo/hw/exported_platform/$NAME/"
}

stage_resume() {
    # Re-run only the synthesis runs that did not finish (e.g. killed out of memory), then implement
    # and export. The project reads hw/hls/vitis_ip_repo, so first put this configuration's own IPs
    # (kept in hw/hls/ip_engine_<NAME> by the hls stage) back there, and check them.
    [ -f "$repo/hw/hdl/build/$NAME/$NAME.xpr" ] || { echo "no project hw/hdl/build/$NAME to resume" >&2; exit 1; }
    [ -d "$hls/ip_engine_$NAME" ] || { echo "missing hw/hls/ip_engine_$NAME" >&2; exit 1; }
    for ip in nmpc_solver_fsm nmpc_solver_init_s nmpc_solver_update_s nmpc_solver_costF nmpc_solver_global_minimum; do
        rm -rf "$hls/vitis_ip_repo/$ip" "$hls/vitis_ip_repo/$ip.zip"
        cp -r "$hls/ip_engine_$NAME/$ip" "$hls/ip_engine_$NAME/$ip.zip" "$hls/vitis_ip_repo/"
        cmp -s "$hls/ip_engine_$NAME/$ip.zip" "$hls/vitis_ip_repo/$ip.zip" \
            && diff -rq "$hls/ip_engine_$NAME/$ip" "$hls/vitis_ip_repo/$ip" > /dev/null \
            || { echo "restoring $ip for $NAME failed" >&2; exit 1; }
    done
    grep -rq "particle_loop" "$hls/vitis_ip_repo/nmpc_solver_costF/hdl/verilog/" || { echo "costF IP is not the interleaved evaluator" >&2; exit 1; }
    log "Vivado resume: $NAME, $JOBS jobs; log: $repo/hw/hdl/build/vivado_${NAME}_resume.log"
    ( cd "$repo" && vivado -mode batch -nojournal -log "$repo/hw/hdl/build/vivado_${NAME}_resume.log" \
        -source hw/hls/proto_arc/resume_engine_vivado.tcl -tclargs "$NAME" "$JOBS" > /dev/null )
    grep -A6 "Design Timing Summary" "$repo/hw/exported_platform/$NAME/timing_summary.rpt" | tail -1 \
        || echo "  (WNS line not found; read $repo/hw/exported_platform/$NAME/timing_summary.rpt)"
    log "Vivado done: $repo/hw/exported_platform/$NAME/"
}

stage_package() {
    local out="$repo/app/kria-pynq/overlay-$NAME"
    log "Package: $out (${S}x${K})"
    ( cd "$repo" && "$repo/../../../.venv/bin/python3" app/kria-pynq/package_overlay.py \
        --xsa "hw/exported_platform/$NAME/nmpc_solver_v2.xsa" --output "$out" --particles "$S" --iterations "$K" )
    ( cd "$out" && find . -type f | sort | xargs sha256sum | sed 's/^/  /' )
}

case "$STAGE" in
    hls) stage_hls ;; vivado) stage_vivado ;; resume) stage_resume ;; package) stage_package ;;
    all) stage_hls; stage_vivado; stage_package ;;
    resume+package) stage_resume; stage_package ;;
    *) echo "unknown stage $STAGE" >&2; exit 2 ;;
esac
