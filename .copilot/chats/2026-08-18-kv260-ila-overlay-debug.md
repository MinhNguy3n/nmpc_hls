# KV260 NMPC ILA Overlay Debug Handoff

## Session metadata

- Created: 2026-08-19
- Source chat session: `3697f6ce-3e89-48c3-8aec-de2cdbb2af37`
- Repository-local transcript: `logs/2026-08-18-kv260-ila-overlay-debug.raw.jsonl`
- Captured terminal output: `logs/2026-08-18-kv260-ila-overlay-debug.terminal-output.txt`
- Repository: `/home/minh/Documents/UTU-Doctorate/nmpc_hls`
- Target board: KV260 at `ubuntu@10.42.0.48`
- Toolchain: Vivado/Vitis 2025.1 (`vivado2025`, `vitis2025`)

## Goal

Identify why the NMPC solver FSM stalls during its first AXI DMA read from PS DDR through the SmartConnect and PS HP0 path.

## Current hardware and overlay state

- The ILA-enabled debug platform has been built and deployed.
- Board overlay directory: `/root/jupyter_notebooks/nmpc_hls/app/kria-pynq/overlay`
- Deployed manifest is revision `4`; bitstream, HWH, and BDA SHA-256 hashes were verified against it.
- Manifest maps `pso_fsm_1` control to `0xA0000000` and arguments to `0xA0010000`.
- The package source XSA reported by the remote manifest was `/home/ubuntu/nmpc_hls/hw/exported_platform/nmpc_solver_v2.xsa`. Confirm that this is the latest ILA-enabled export before rebuilding or redeploying.
- The current debug Tcl is `hw/hdl/nmpc_solver_v2_debug.tcl`.
- The design includes ILA probes on both `pso_fsm_1/m_axi_nmpc_io` and `axi_smc_1/M00_AXI`.

## Latest verified runtime result

A bounded external PYNQ transaction was executed on the deployed overlay. It initialized Q17.15 CMA buffers, flushed them, wrote `ap_start`, and read only the safe FSM registers for five seconds.

```json
{
  "ap_ctrl": 1,
  "ap_return": 0,
  "fsm_state": 0,
  "fsm_phase": "copy_inputs"
}
```

The FSM accepted `ap_start` but remained in state `0` (`copy_inputs`) for the entire five-second observation. This is the expected ILA trigger transaction and reproduces the previous timeout without running the full 120-second notebook solve.

## 2026-08-19 ILA continuation

Physical JTAG is now available through the FT4232H interface. The chain is stable when the hardware-target frequency is reduced from 15 MHz to 1 MHz. Vivado validates the resident design against:

- Bitstream: `hw/hdl/tcl_run/nmpc_solver_v2/nmpc_solver_v2.runs/impl_1/design_nmpc_solver_wrapper.bit`
- Probes: `hw/hdl/tcl_run/nmpc_solver_v2/nmpc_solver_v2.runs/impl_1/debug_nets.ltx`
- Bitstream SHA-256: `d1229e05c6bc0629b3c5ac96c46c30dc18feee579491cff5303ce84f99f63a95`
- Probe SHA-256: `40ede91cb2b458670acfbe730f79227f6e95c003e26873dd85a5527ee62af0af`

Both expected ILA UUIDs were discovered. A 1024-cycle steady-state snapshot was captured from each AXI boundary after the original stalled transaction:

| Signal | FSM master | SmartConnect M00 / HP0 |
| --- | --- | --- |
| `ARVALID` | `0` for all samples | `0` for all samples |
| `ARREADY` | `1` for all samples | `1` for all samples |
| `ARADDR` | `0x03e59180` | `0x0000003e59180` |
| `ARLEN` / `ARSIZE` / `ARBURST` | `3` / `2` / `INCR` | `3` / `2` / `INCR` |
| `RVALID` | `0` for all samples | `0` for all samples |
| `RREADY` | `1` for all samples | `1` for all samples |

Saved waveforms:

- `.copilot/chats/logs/2026-08-19-kv260-ila-fsm-master-live.csv`
- `.copilot/chats/logs/2026-08-19-kv260-ila-fsm-master-live.ila`
- `.copilot/chats/logs/2026-08-19-kv260-ila-hp0-live.csv`
- `.copilot/chats/logs/2026-08-19-kv260-ila-hp0-live.ila`

The generated HLS read adapter deasserts external `ARVALID` after accepting the address and then waits for `RVALID`, so this late steady-state snapshot could not distinguish a missing response from a completed response. The transaction-qualified capture below falsified the missing-response hypothesis.

### Transaction-qualified ILA capture

The overlay was successfully programmed from Vivado Hardware Manager, followed by `sudo xmutil unloadapp` on the KV260. Running notebook Cell 14 then triggered both ILAs. The capture artifacts are:

- `hw/hdl/tcl_run/nmpc_solver_v2/iladata_1.csv` and `.ila`: SmartConnect `axi_smc_1/M00_AXI` at PS HP0
- `hw/hdl/tcl_run/nmpc_solver_v2/ILA_1_waveform.png`
- `hw/hdl/tcl_run/nmpc_solver_v2/iladata_2.csv` and `.ila`: `pso_fsm_1/m_axi_nmpc_io` before SmartConnect
- `hw/hdl/tcl_run/nmpc_solver_v2/ILA_2_waveform.png`

ILA 1 used trigger position 512 and ILA 2 used trigger position 128. After aligning those trigger samples:

- All 21 read-address handshakes occur at identical relative cycles and addresses on both interfaces.
- No cycle has `ARVALID = 1` with `ARREADY = 0`.
- All 271 response beats visible in both captures match exactly in data, response, and `RLAST`.
- Data reaches the FSM-side ILA exactly five PL clocks after the HP0-side ILA.
- `RREADY` remains asserted and every valid `RRESP` is `OKAY` (`0`).
- The FSM-side capture contains all 316 returned words with no read-data backpressure.

The 21 requests are exactly the first three state-0 input copies:

| Input | Address range | AXI bursts | Returned words |
| --- | --- | --- | --- |
| `u_curr` | `0x02882000` | 1 x 4 beats | 4 |
| `x_curr` | `0x02B06000` | 1 x 12 beats | 12 |
| `xref` | `0x02B07000` through `0x02B07480` | 18 x 16 beats + 1 x 12 beats | 300 |

The returned Q17.15 data is correct: `u_curr` is four zeros, `x_curr` decodes to `[7.0, 10.0, 0.0, ...]`, and all 300 `xref` words are present and plausible. The SmartConnect/HP0/DDR read path is therefore healthy and is not the source of the timeout.

The absence of a `last_best` request in this capture is expected, not yet evidence of a stall. Generated HLS reports show the rolled `xref` copy controller has fixed latency 901 clocks (300 iterations at three clocks each). Its first data reaches the FSM at sample 390, so nominal completion is near sample 1291; ILA 2 ends at sample 1023. The capture stops roughly 268 clocks before the `xref` copy should finish draining writes to `current_mem`.

Required next ILA capture:

1. Preserve the working programming sequence: program the bitstream and probes in Vivado Hardware Manager, then run `sudo xmutil unloadapp` on the KV260 before the notebook transaction.
2. Trigger late on the final `xref` burst. For this allocation use `ARVALID = 1`, `ARADDR = 0x02B07480`, and optionally `ARLEN = 0x0B`, with AND trigger matching. If CMA addresses change, use `xref.physical_address + 0x480` instead.
3. Use trigger position 128 and capture depth 1024. This provides enough post-trigger samples to observe the expected `last_best` request near the end of the rolled `xref` copy.
4. Add an ILA AXI monitor to `pso_fsm_1/m_axi_current_mem` (the `pso_fsm_1_m_axi_current_mem` net feeding `smartconnect_0/S02_AXI`). Capture `AWVALID`, `AWREADY`, `AWADDR`, `AWLEN`, `WVALID`, `WREADY`, `WLAST`, `BVALID`, `BREADY`, and `BRESP`.
5. If `last_best` still does not launch, use the current-memory trace to distinguish a blocked write-data handshake from a missing write response. The HP0-side ILA is no longer needed for this question.

### New pre-transaction failure

After a clean physical power cycle, programming the exact matching bitstream caused Linux Ethernet and UART responsiveness to fail before any solver MMIO access or `ap_start`. This occurred through both PYNQ/FPGA Manager and direct Vivado JTAG programming. Both ILAs remained untriggered, so no NMPC AXI read caused this failure.

The direct interrupt path is not responsible: JTAG reads showed the HLS `GIE`, `IER`, and `ISR` registers at `0xA0000004`, `0xA0000008`, and `0xA000000C` were all zero. The ZynqMP reset reason read `0x00000021` (external POR plus software reset), although the software-reset bit may be sticky from an earlier manual XSDB reset.

The leading hypothesis is missing PS-to-PL reset sequencing during dynamic configuration. The implemented `proc_sys_reset` polarity is correct (`C_EXT_RESET_HIGH=0`), but it requires `pl_resetn0` to pulse low. Xilinx's ZynqMP full-configuration helper performs this through GPIO5 EMIO 92-95:

- Direction register: `0xFF0A0344`, set mask `0xF0000000`
- Assert resets low: write `0x0FFF0000` to `0xFF0A002C`
- Release resets high: write `0x0FFFF000` to `0xFF0A002C`
- Bank 5 output latch: `0xFF0A0054`
- Bank 5 input readback: `0xFF0A0074`

Do not use `0xFF0A0064` to verify Bank 5; that address is `DATA_1_RO`. `OEN_5` is at `0xFF0A0348`, although Xilinx's `XFpga_PsPlGpioResetsLow/High` helper modifies only `DIRM_5` and `MASK_DATA_5_MSW`.

### Reset-sequencing experiment result

The reset-sequencing experiment was attempted after another clean power cycle. Before the write, `DIRM_5` was already `0xF0000000`. Writing `0x0FFF0000` to `MASK_DATA_5_MSW` caused Ethernet and UART responsiveness to disappear before the bitstream was programmed. The release write latched successfully (`DATA_5 = 0xF0000000`), but Linux did not recover. A53 #0 was found at BootROM address `0xFFFEA144`, confirming a PS reset rather than an NMPC or AXI transaction stall.

The boot PL application must therefore be quiesced before asserting fabric resets or replacing the PL image. The current package has no `.dtbo`, and its HWH reports AXI shutdown pins disabled. PYNQ's `EmbeddedDevice.download()` calls `shutdown()` and then writes FPGA Manager, but it has no shutdown metadata for a boot-time Kria application it did not load.

### `xmutil unload` follow-up

After another power cycle, `xmutil unload k26-starter-kits` reported that the starter application was deactivated. A subsequent reset-release write (`0x0FFFF000` to `0xFF0A002C`) was successful even though a script incorrectly checked `0xFF0A0064`. Correct readback showed:

- `DATA_5` at `0xFF0A0054`: `0xF0000000` (all four release latches high)
- `DIRM_5` at `0xFF0A0344`: `0xF0000000`
- `DATA_5_RO` at `0xFF0A0074`: `0x00000000`
- `OEN_5` at `0xFF0A0348`: `0x00000000`

No reset-low write and no PL programming occurred in this follow-up. Ping initially responded after unload, then degraded and stopped entirely. JTAG showed A53 cores still executing in Linux kernel virtual-address space rather than BootROM. Resuming a core left stopped by an overlapping XSDB diagnostic did not restore networking. The unloaded platform state is therefore not stable enough to proceed, and competing interactive XSDB clients should be closed during the next run.

Required next experiment after a physical power cycle:

1. Confirm Linux responds at `10.42.0.48`; connect only with `ssh ubuntu@10.42.0.48` and let the user type passwords directly.
2. Close extra interactive XSDB clients. Keep one `hw_server` and use short-lived scripted clients only.
3. Before unload, capture `xmutil --help`, `xmutil listapps`, the active application, configfs device-tree overlays, FPGA bridges, FPGA Manager state, `ip address`, and recent kernel messages. Open the UART console before unload so kernel output survives an SSH loss.
4. Run the exact unload command supported by that `xmutil` version from the authenticated SSH session, preserving its output and immediately capturing the same state again.
5. Verify ping, SSH, and UART remain healthy for a sustained interval after unload. Do not assert reset or program PL if networking degrades.
6. Only after the unloaded state is demonstrably stable, perform a platform-supported full configuration. Do not repeat direct EMIO reset writes against an unstable or active boot application.
7. Attach `debug_nets.ltx`, arm both ILAs on `ARVALID = 1`, and only then run `NmpcOverlay("overlay", download=False)` for the bounded transaction.

Do not call `Overlay(..., download=True)` again until this reset-sequenced programming check succeeds.

## Safe diagnostic interfaces

Use only the `pso_fsm_1` control and argument MMIO regions exposed by `NmpcOverlay`.

- `app/kria-pynq/nmpc_overlay.py`
  - `FSM_INSTANCE = "pso_fsm_1"`
  - `AP_DEBUG_STATE = 0x18`
  - `NmpcOverlay.runtime_status()` returns safe diagnostic information.
- `app/kria-pynq/package_overlay.py`
  - Revision `4`; expects `/nmpc_solver/pso_fsm_1`.
- `hw/hls/source/src/hls_pso.cpp`
  - `debug_state` is exposed through the FSM AXI-Lite control interface.

Do **not** access direct AXI-GPIO status or worker-control MMIO windows from PYNQ. Earlier direct accesses caused a SIGBUS and terminated the Jupyter kernel.

## Required next diagnostic step

Retrieve the existing ILA waveform through Vivado Hardware Manager using a physical JTAG connection, then inspect the following signals on both sides of the SmartConnect/HP0 boundary:

- `ARVALID`, `ARREADY`, `ARADDR`, `ARLEN`, `ARSIZE`, `ARBURST`
- `RVALID`, `RREADY`, `RRESP`

Interpretation:

| Observation | Likely cause / next action |
| --- | --- |
| `ARVALID = 0` at FSM | FSM never issues the read: inspect HLS FSM state and AXI pointer mapping. |
| FSM `ARVALID = 1`, but `ARREADY = 0` | SmartConnect or HP0 bridge does not accept the read: inspect HP0 clock, reset, and configuration. |
| Address accepted, no `RVALID` | HP0/DDR interconnect path hangs. |
| Non-zero `RRESP` | Investigate address, protection, cache, or PS memory configuration. |

No JTAG target was visible from the local `hw_server`, and the KV260 did not expose XVC or a board-side `hw_server`; the ILA must therefore be retrieved via a physical JTAG connection available to Vivado Hardware Manager.

## Reproducible bounded probe

Run on the development host; it avoids Jupyter and touches no unsafe apertures:

```bash
ssh -tt ubuntu@10.42.0.48 'sudo bash -lc "source /etc/profile.d/pynq_venv.sh; export XILINX_XRT=/usr; cd /root/jupyter_notebooks/nmpc_hls/app/kria-pynq; python3 - <<\"PY\"
import json
import time
import numpy as np
from nmpc_overlay import NmpcOverlay, encode_fixed

solver = NmpcOverlay(\"overlay\")
try:
    solver._buffers[\"u_curr\"][:] = encode_fixed(np.zeros(4))
    solver._buffers[\"x_curr\"][:] = encode_fixed(np.array([7.0, 10.0] + [0.0] * 10))
    solver._buffers[\"xref\"][:] = encode_fixed(np.zeros((25, 12)))
    solver._buffers[\"last_best\"][:] = encode_fixed(np.zeros((25, 4)))
    solver._buffers[\"new_best\"].fill(0)
    solver._buffers[\"bestfitness\"].fill(0)
    for buffer in solver._buffers.values():
        buffer.flush()
    solver.control.write(0x00, 0x01)
    started = time.monotonic()
    while time.monotonic() - started < 5.0:
        print(json.dumps(solver.runtime_status()))
        time.sleep(1)
finally:
    solver.close()
PY"'
```

## Related completed work

- Corrected worker-start routing in the block design.
- Closed timing on the standard build: routed WNS `+0.606 ns` after evaluator HLS target was set to 8 ns.
- Added safe FSM stage instrumentation.
- HLS VLNV consistency for future rebuilds:
  - `nmpc_solver_fsm` and `nmpc_solver_costF`: `tu-dresden`.
  - init/update/detect/random IP: `tu-dresden_turun-yliopisto`.

## Files to inspect first

- `hw/hdl/nmpc_solver_v2_debug.tcl`
- `app/kria-pynq/nmpc_overlay.py`
- `hw/hls/source/src/hls_pso.cpp`
- `app/kria-pynq/package_overlay.py`
