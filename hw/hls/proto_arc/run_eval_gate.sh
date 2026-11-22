#!/usr/bin/env bash
# Evaluator RTL gate in short, memory-bounded pieces: for each seed, cosim the new and the deployed
# evaluator (one at a time, 1 trial = 3 calls each) and compare the RTL dumps.
#   ./run_eval_gate.sh OUTDIR S II "SEED1 SEED2 ..."
set -euo pipefail
here="$(cd "$(dirname "$0")" && pwd)"
out="$1"; S="$2"; II="$3"; seeds="$4"
for seed in $seeds; do
    PROTO_SEED="$seed" PROTO_NODEADLOCK=1 SEQ=1 "$here/cmp_eval_rtl.sh" "$out/seed$seed" "$S:$II" 1
done
