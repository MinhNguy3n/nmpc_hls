#!/usr/bin/env bash
# Generate the prototype's header set from hw/hls/source/include without editing it.
#
#   gen/include/                 copy of source/include with `volatile` removed (the C++ half
#                                emulation has no volatile operators; same as validate_hw_run.py)
#   gen/include/models/model_pipe.hpp
#                                the quadrotor model (hls_sniffbot.hpp) renamed __model_pipe, with
#                                every `#pragma HLS` line deleted so the model can be inlined into
#                                a pipelined loop. Only pragma lines are removed: every arithmetic
#                                expression is byte-identical, so the operation order (and thus
#                                the half-precision result) is the one the fix6/3x24 IP computes.
set -euo pipefail
here="$(cd "$(dirname "$0")" && pwd)"
src="$here/../source/include"
out="$here/gen/include"

rm -rf "$out"
mkdir -p "$here/gen"
cp -r "$src" "$out"
find "$out" -name '*.hpp' -exec sed -i -E 's/\bvolatile //g' {} +

sed -E \
    -e '/^[[:space:]]*#pragma HLS/d' \
    -e 's/\bvolatile //g' \
    -e 's/HLS_SNIFFBOT_HPP/HLS_SNIFFBOT_PIPE_HPP/g' \
    -e 's/>void __model\(/>void __model_pipe(/' \
    "$src/models/hls_sniffbot.hpp" > "$out/models/model_pipe.hpp"

# Guard: the generated model must differ from the original only in deleted pragma lines,
# `volatile`, the include guard and the function name.
diff <(sed -E -e '/^[[:space:]]*#pragma HLS/d' -e 's/\bvolatile //g' "$src/models/hls_sniffbot.hpp") \
     <(sed -E -e 's/HLS_SNIFFBOT_PIPE_HPP/HLS_SNIFFBOT_HPP/g' -e 's/__model_pipe\(/__model(/' "$out/models/model_pipe.hpp") \
    > /dev/null || { echo "model_pipe.hpp differs from hls_sniffbot.hpp beyond pragmas" >&2; exit 1; }
grep -q '__model_pipe(' "$out/models/model_pipe.hpp" || { echo "rename failed" >&2; exit 1; }
echo "generated $out (model_pipe.hpp: $(grep -c '' "$out/models/model_pipe.hpp") lines)"
