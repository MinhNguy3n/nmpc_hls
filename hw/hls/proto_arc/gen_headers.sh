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
#
# PROTO_MODEL=unicycle (default: sniffbot) builds the same for the unicycle model point:
#   gen/uni_src/include          source/include plus the Husky agent's UNICYCLE_CONFIG files
#                                (config.hpp, hls_model.hpp, models/hls_unicycle.hpp) from UNI_SRC
#                                (default: its worktree). The overlay must be a pure addition to
#                                config.hpp and hls_model.hpp; the hashes are written next to it.
#   gen/uni_include/             as gen/include, with model_pipe.hpp made from hls_unicycle.hpp.
set -euo pipefail
here="$(cd "$(dirname "$0")" && pwd)"
model="${PROTO_MODEL:-sniffbot}"
src="$here/../source/include"

case "$model" in
sniffbot)
    out="$here/gen/include"; mfile="$src/models/hls_sniffbot.hpp"; guard=HLS_SNIFFBOT_HPP ;;
unicycle)
    uni_src="${UNI_SRC:-$here/../../../../agent-a7f53c7be12d59b73/hw/hls/source/include}"
    [ -f "$uni_src/models/hls_unicycle.hpp" ] || { echo "no hls_unicycle.hpp under $uni_src" >&2; exit 1; }
    base="$here/gen/uni_src/include"
    rm -rf "$here/gen/uni_src"; mkdir -p "$here/gen/uni_src"
    cp -r "$src" "$base"
    for f in config.hpp hls_model.hpp; do
        # Pure addition: no line of the ARC source may be removed or changed.
        if diff "$src/$f" "$uni_src/$f" | grep -q '^<'; then
            echo "$uni_src/$f changes existing lines of source/include/$f" >&2; exit 1
        fi
        cp "$uni_src/$f" "$base/$f"
    done
    cp "$uni_src/models/hls_unicycle.hpp" "$base/models/hls_unicycle.hpp"
    ( cd "$uni_src" && sha256sum config.hpp hls_model.hpp models/hls_unicycle.hpp ) > "$here/gen/uni_src/OVERLAY.sha256"
    echo "$uni_src" > "$here/gen/uni_src/OVERLAY.from"
    src="$base"
    out="$here/gen/uni_include"; mfile="$src/models/hls_unicycle.hpp"; guard=HLS_UNICYCLE_HPP ;;
*) echo "unknown PROTO_MODEL=$model" >&2; exit 2 ;;
esac

rm -rf "$out"
mkdir -p "$here/gen"
cp -r "$src" "$out"
find "$out" -name '*.hpp' -exec sed -i -E 's/\bvolatile //g' {} +

sed -E \
    -e '/^[[:space:]]*#pragma HLS/d' \
    -e 's/\bvolatile //g' \
    -e "s/${guard}/${guard%_HPP}_PIPE_HPP/g" \
    -e 's/>void __model\(/>void __model_pipe(/' \
    "$mfile" > "$out/models/model_pipe.hpp"

# Guard: the generated model must differ from the original only in deleted pragma lines,
# `volatile`, the include guard and the function name.
diff <(sed -E -e '/^[[:space:]]*#pragma HLS/d' -e 's/\bvolatile //g' "$mfile") \
     <(sed -E -e "s/${guard%_HPP}_PIPE_HPP/${guard}/g" -e 's/__model_pipe\(/__model(/' "$out/models/model_pipe.hpp") \
    > /dev/null || { echo "model_pipe.hpp differs from $(basename "$mfile") beyond pragmas" >&2; exit 1; }
grep -q '__model_pipe(' "$out/models/model_pipe.hpp" || { echo "rename failed" >&2; exit 1; }
echo "generated $out (model_pipe.hpp: $(grep -c '' "$out/models/model_pipe.hpp") lines, from $(basename "$mfile"))"
