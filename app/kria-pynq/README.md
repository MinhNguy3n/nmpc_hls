# KV260 NMPC PYNQ Overlay

This directory contains the board-side driver and notebook for the Sniffbot
quadrotor NMPC overlay. It packages the current
`hw/exported_platform/nmpc_solver.xsa` handoff for deployment on a KV260.

## Corrected hardware contract

The current `design_npmc_solver.hwh` and `design_npmc_solver.bda` expose these
PS-visible register segments:

| Address | Block | Purpose |
| --- | --- | --- |
| `0xA0000000` | AXI GPIO | Debug/status only |
| `0xA0030000` | PSO FSM control | `ap_start`, `ap_done`, `ap_return` |
| `0xA0040000` | PSO FSM arguments | Six DDR buffer addresses |

The packager reads the two PSO FSM aperture bases and ranges from the XSA's BDA
and stores them in `manifest.json`; the driver uses those values instead of
hard-coded MMIO bases. CMA buffers must be below `0x80000000`, which is the
mapped HP0 DDR-low window used by the solver.

Top-level arrays use `ap_fixed<32,17>`, so the host exchanges signed Q17.15
words, not IEEE `float32` values. Buffer shapes are:

| Buffer | Shape |
| --- | --- |
| Current control | `(4,)` |
| Current state | `(12,)` |
| Reference horizon | `(25, 12)` |
| Warm-start control | `(25, 4)` |
| Optimized control | `(25, 4)` |
| Fitness history | `(100,)` |

The PL returns an optimized control trajectory. It does not return a next-state
vector. `quadrotor_step()` applies the first control to the same Euler-discrete
Sniffbot model used by the HLS cost function to obtain the predicted next state.

## Build on the development machine

Source a compatible AMD toolchain first. Vivado and Vitis HLS 2025.1 are the
expected versions for the current project export.

```bash
source /tools/Xilinx/Vivado/2025.1/settings64.sh
./hw/build_kv260_overlay.sh
```

Package the generated platform for PYNQ:

```text
python app/kria-pynq/package_overlay.py \
	--xsa hw/exported_platform/nmpc_solver.xsa \
	--output app/kria-pynq/overlay
```

The output contains a matching `nmpc_solver.bit`, `nmpc_solver.hwh`, and
`manifest.json`. The packager validates the exported `pso_fsm_0` interfaces and
worker completion ports, then records the BDA-derived control apertures.

## Run on Kria PYNQ

Transfer `app/kria-pynq` to the KV260, open its notebook in Jupyter, and run the
cells in order. The notebook allocates physically contiguous DDR buffers,
programs the overlay, submits one NMPC solve, and computes the predicted next
state. It also includes a short closed-loop simulation cell.

The driver rejects any CMA allocation at or above `0x80000000`, because the HLS
pointer ports and HP0 mapping are currently 32-bit. A timeout usually indicates
an HLS/Vivado artifact mismatch or a stalled subordinate core; the exception
includes the last `AP_CTRL` value.
