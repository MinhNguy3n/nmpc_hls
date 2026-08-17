#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

for tool in vitis vivado python3; do
    if ! command -v "$tool" >/dev/null 2>&1; then
        printf 'Required tool is not on PATH: %s\n' "$tool" >&2
        exit 1
    fi
done

cd "$repo_root/hw/hls"
vitis-run --mode hls --tcl scripts/hls_script_pso.tcl
vitis-run --mode hls --tcl scripts/hls_script_pseudorand.tcl

cd "$repo_root/hw/hdl"
vivado -mode batch -notrace -source scripts/build_nmpc_solver_overlay.tcl

cd "$repo_root"
python3 app/kria-pynq/package_overlay.py \
    --xsa hw/exported_platform/design_nmpc_solver_wrapper.xsa \
    --output app/kria-pynq/overlay

printf 'Packaged KV260 overlay in %s\n' "$repo_root/app/kria-pynq/overlay"
