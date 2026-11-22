#!/usr/bin/env bash
# Fast C simulation of the prototype against the original predictor, without Vitis HLS:
# the same compiler flags and bit-accurate floating-point library as validate_hw_run.py's
# "half" build (the hardware's own arithmetic).
#   ./run_csim.sh [S] [II] [trials]
set -euo pipefail
here="$(cd "$(dirname "$0")" && pwd)"
S="${1:-10}"; II="${2:-8}"; TRIALS="${3:-200}"
XILINX="${XILINX:-/tools/Xilinx/2025.2}"
[ -d "$here/gen/include" ] || "$here/gen_headers.sh"
mkdir -p "$here/build"
bin="$here/build/tb_rk4_multi_S${S}_II${II}"
g++ -std=c++11 -O0 -w -D__VITIS__ -DPSO_CONFIG -DSNIFFBOT_CONFIG \
    -DHW_ARITH_CSIM -DUSE_FAST_SIN_COS -DHW_MODEL_HALF_CSIM \
    -DPROTO_S="$S" -DPROTO_II="$II" \
    -I"$here" -I"$here/gen/include" -I"$here/gen/include/models" -I"$XILINX/Vitis/include" \
    -o "$bin" "$here/tb_rk4_multi.cpp" "$here/rk4_multi.cpp" "$here/../source/src/aux_functions.cpp" \
    -L"$XILINX/lnx64/tools/fpo_v7_1" -lIp_floating_point_v7_1_bitacc_cmodel \
    -Wl,--disable-new-dtags -Wl,-rpath,"$XILINX/lnx64/tools/fpo_v7_1"
"$bin" "$TRIALS"
