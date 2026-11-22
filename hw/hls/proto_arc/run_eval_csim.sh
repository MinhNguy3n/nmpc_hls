#!/usr/bin/env bash
# C-level evaluator equivalence in the hardware's arithmetic (half model, bit-accurate FPO):
# tb_eval linked with eval_multi.cpp vs tb_eval linked with the deployed hls_pso.cpp, same
# inputs, raw output dumps compared with cmp.
#   ./run_eval_csim.sh S [II] [trials]
set -euo pipefail
here="$(cd "$(dirname "$0")" && pwd)"
S="$1"; II="${2:-8}"; TRIALS="${3:-10}"
XILINX="${XILINX:-/tools/Xilinx/2025.2}"
"$here/gen_eval_headers.sh" "$S" >/dev/null
inc="$here/gen/S$S/include"
mkdir -p "$here/build"
flags=(-std=c++11 -O0 -w -D__VITIS__ -DPSO_CONFIG -DSNIFFBOT_CONFIG -DHW_ARITH_CSIM -DUSE_FAST_SIN_COS
       -DHW_MODEL_HALF_CSIM -DPROTO_II="$II" -I"$here" -I"$inc" -I"$inc/models" -I"$XILINX/Vitis/include")
libs=(-L"$XILINX/lnx64/tools/fpo_v7_1" -lIp_floating_point_v7_1_bitacc_cmodel -Wl,--disable-new-dtags
      -Wl,-rpath,"$XILINX/lnx64/tools/fpo_v7_1")
common=("$here/tb_eval.cpp" "$here/../source/src/aux_functions.cpp")
g++ "${flags[@]}" -o "$here/build/eval_new_S$S" "${common[@]}" "$here/eval_multi.cpp" "${libs[@]}" &
g++ "${flags[@]}" -o "$here/build/eval_ref_S$S" "${common[@]}" "$here/../source/src/hls_pso.cpp" \
    "$here/../source/src/hls_nonlinear_solver.cpp" "${libs[@]}" &
wait
"$here/build/eval_new_S$S" "$TRIALS" "$here/build/eval_new_S$S.dump" &
"$here/build/eval_ref_S$S" "$TRIALS" "$here/build/eval_ref_S$S.dump" &
wait
if cmp -s "$here/build/eval_new_S$S.dump" "$here/build/eval_ref_S$S.dump"; then
    echo "S=$S II=$II C(new) vs C(deployed source): IDENTICAL ($(wc -l < "$here/build/eval_new_S$S.dump") words)"
else
    echo "S=$S II=$II C(new) vs C(deployed source): DIFFERENT"
    diff "$here/build/eval_new_S$S.dump" "$here/build/eval_ref_S$S.dump" | head -6
    exit 1
fi
