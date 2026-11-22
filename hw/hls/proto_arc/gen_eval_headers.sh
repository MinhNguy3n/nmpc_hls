#!/usr/bin/env bash
# Header sets for the evaluator comparison at swarm size S:
#   gen/S<S>/include       volatile removed + model_pipe.hpp (as gen_headers.sh), _n_S = S.
#                          Used by eval_multi.cpp and by the half-model C builds of both designs.
#   gen/ref_S<S>/include   an exact copy of source/include with ONLY `#define _n_S` changed
#                          (volatile kept): the deployed costF sources, for the RTL reference.
#                          With S equal to the configured _n_S (3) it is byte-identical.
set -euo pipefail
here="$(cd "$(dirname "$0")" && pwd)"
S="$1"
src="$here/../source/include"
[ -f "$here/gen/include/models/model_pipe.hpp" ] || "$here/gen_headers.sh" >/dev/null
for kind in "S$S" "ref_S$S"; do
    out="$here/gen/$kind/include"
    rm -rf "$here/gen/$kind"; mkdir -p "$here/gen/$kind"
    if [ "$kind" = "S$S" ]; then cp -r "$here/gen/include" "$out"; else cp -r "$src" "$out"; fi
    sed -i -E "s/^(#define _n_S[[:space:]]+)[0-9]+/\1$S/" "$out/config.hpp"
    grep -qE "^#define _n_S[[:space:]]+$S\b" "$out/config.hpp" || { echo "_n_S patch failed" >&2; exit 1; }
done
diff -r "$src" "$here/gen/ref_S$S/include" | grep -E '^[<>]' | grep -v '_n_S' && { echo "ref copy differs beyond _n_S" >&2; exit 1; } || true
echo "gen/S$S and gen/ref_S$S ready"
