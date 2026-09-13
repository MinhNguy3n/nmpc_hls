# KV260 NMPC ILA debug summary (agent memory token)

This note is a compact handoff for downstream agents. It summarizes verified status, evidence, and the safest next steps.

## Current status

The original theory that the PS HP0/SmartConnect DDR read path is hanging is no longer supported by the captured ILA evidence. The captured traffic shows that the first three input-copy reads (`u_curr`, `x_curr`, `xref`) complete normally across the external memory path. The remaining issue is more likely in the subsequent FSM logic around write acceptance / `current_mem` staging, or in the missing `last_best` read launch after the rolled `xref` copy completes.

## Verified evidence

Captured ILA artifacts:

- [hw/hdl/tcl_run/nmpc_solver_v2/iladata_1.csv](hw/hdl/tcl_run/nmpc_solver_v2/iladata_1.csv)
- [hw/hdl/tcl_run/nmpc_solver_v2/iladata_2.csv](hw/hdl/tcl_run/nmpc_solver_v2/iladata_2.csv)
- [hw/hdl/tcl_run/nmpc_solver_v2/ILA_1_waveform.png](hw/hdl/tcl_run/nmpc_solver_v2/ILA_1_waveform.png)
- [hw/hdl/tcl_run/nmpc_solver_v2/ILA_2_waveform.png](hw/hdl/tcl_run/nmpc_solver_v2/ILA_2_waveform.png)

Key verified facts from the CSVs:

- There are 21 read requests total.
- The request sequence matches exactly on both interfaces.
- No cycle shows `ARVALID = 1` with `ARREADY = 0`.
- No valid response shows `RRESP != 0` (all are `OKAY`).
- `RREADY` remains asserted; there is no read-data backpressure.
- The FSM-side trace contains all 316 accepted words.
- The SmartConnect adds a constant 5-cycle latency from HP0 to the FSM-side ILA.

The observed request set is exactly the first three state-0 copies:

- `u_curr`: address `0x02882000`, 1 burst x 4 beats
- `x_curr`: address `0x02B06000`, 1 burst x 12 beats
- `xref`: `0x02B07000` through `0x02B07480`, 18 bursts of 16 + 1 burst of 12 beats

Decoded Q17.15 results are consistent with the intended values:

- `u_curr` = four zeros
- `x_curr` begins with `[7.0, 10.0, 0, ...]`
- all 300 `xref` words are present and plausible

This means the HP0/DDR/SmartConnect path is healthy and is not the primary timeout cause.

## Why `last_best` is still missing

The missing `last_best` read is not yet proof of a stall. Generated HLS reports show the rolled `xref` copy loop has fixed latency 901 cycles (300 iterations at 3 cycles per iteration). The first `xref` response is observed at sample 390 in the FSM-side ILA, which means the copy should complete around sample 1291. The current capture stops around sample 1023, so it still does not reach the end of the `xref` transfer. In other words, the capture is short of the full copy completion window.

The current HLS copy ordering is explicitly in [hw/hls/source/src/hls_pso.cpp](hw/hls/source/src/hls_pso.cpp#L390-L394):

- copy `u_curr`
- copy `x_curr`
- copy `xref`
- copy `last_best`

The generated RTL confirms the copy loop trip counts:

- loop 0: 4 words
- loop 1: 12 words
- loop 2: 300 words
- loop 3: 100 words

The missing fourth copy is therefore expected until a deeper capture reaches the tail of the `xref` transfer. The real issue is likely after the data path has been proven healthy, in the FSM write-back / current-memory path or in a state transition waiting for `current_mem` completion before launching the final copy.

## Important non-root-cause finding

A separate reset-related issue appeared when the board was power-cycled and the matching bitstream was reprogrammed. In that state, Linux networking and UART responsiveness failed before any solver MMIO transaction. The bitstream itself was not the cause of an AXI read stall; the failure was a PS/PL reset sequencing problem. This is not the same as the original DDR-read timeout.

The direct interrupt path is not implicated: the HLS interrupt registers (`GIE`, `IER`, `ISR`) were zero, and the reset reason register was consistent with a PS reset event.

This strongly suggests a PS-to-PL reset assertion/release issue or an unstable unload state, not a broken HP0 memory pipeline.

## Safe guidance for next diagnostic work

Do not re-run direct AXI/GPIO MMIO access or unsafe worker MMIO. Only use the safe interfaces exposed by the overlay:

- [app/kria-pynq/nmpc_overlay.py](app/kria-pynq/nmpc_overlay.py)
- [app/kria-pynq/package_overlay.py](app/kria-pynq/package_overlay.py)
- [hw/hls/source/src/hls_pso.cpp](hw/hls/source/src/hls_pso.cpp)
- [hw/hdl/nmpc_solver_v2_debug.tcl](hw/hdl/nmpc_solver_v2_debug.tcl)

## Recommended next capture

The next ILA should be placed on the `pso_fsm_1/m_axi_current_mem` interface, not on HP0. The goal is to inspect the write side that follows the read copies:

- `AWVALID`, `AWREADY`, `AWADDR`, `AWLEN`
- `WVALID`, `WREADY`, `WLAST`
- `BVALID`, `BREADY`, `BRESP`

Trigger conditions should be late and specific:

- trigger on final `xref` burst: `ARVALID = 1`, `ARADDR = 0x02B07480`, optionally `ARLEN = 0x0B`
- use capture depth 1024 and trigger position 128
- keep both the external read ILA and the `current_mem` ILA active during the bounded transaction

If the final `last_best` launch still does not occur, the `current_mem` trace will distinguish:

1. blocked write-data handshake
2. missing write response
3. FSM waiting after the copy loop without issuing the next read

## Recovery plan

1. Power-cycle the KV260.
2. Confirm SSH connectivity to `ubuntu@10.42.0.48` is stable.
3. Close extra XSDB sessions; keep one `hw_server` and short-lived scripted clients only.
4. Before unloading, collect `xmutil --help`, `xmutil listapps`, overlay state, IP state, and kernel logs.
5. Run a supported `xmutil unload` from the SSH session.
6. Verify networking remains healthy before any PL programming.
7. Only then program the design using the supported FPGA-manager path.
8. Attach the debug probes and run the bounded `NmpcOverlay("overlay", download=False)` transaction.

Do not re-run `Overlay(..., download=True)` until a stable unload/programming sequence is confirmed.

## Bottom line

The current evidence says the HP0/DDR read path is functioning correctly for the first three input-copy reads. The outstanding issue is downstream of the external memory path: the real next investigation target is the `current_mem` write/response flow or the state transition after the rolled `xref` copy before `last_best` is launched. The reset issue should be treated separately and stabilized before any further PL reprogramming.
