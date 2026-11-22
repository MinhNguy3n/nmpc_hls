# Vivado out-of-context results (2026-11-20, 04:33-05:01)

Flow: `ooc_vivado.tcl` run by `run_day2_ooc.sh`, launched by the user over ssh in a 7 GB-capped scope.
- Vivado 2025.2, non-project mode, xck26-sfvc784-2LV-c, out of context.
- `create_clock -period 8.0` on `ap_clk`; no I/O constraints.
- Floating-point IP synthesised in-line with the top; `synth.maxThreads 1`.
- Default synthesis, opt_design, place_design and route_design (no phys_opt).
- RTL: the HLS `sol/syn/verilog` of each project (`ooc/<job>/proj_*`).

Post-route utilisation, of 117,120 LUTs, 234,240 FFs, 1,248 DSPs and 144 BRAM tiles:

| Design | CLB LUTs | Registers | DSPs | BRAM tiles | WNS at 8 ns | WHS | Failing endpoints |
|---|---|---|---|---|---|---|---|
| New evaluator, S = 10, II 8 | 38,787 (33.1 %) | 37,569 (16.0 %) | 190 (15.2 %) | 4 | +0.006 ns | +0.035 ns | 0 |
| New evaluator, S = 10, II 16 | 35,029 (29.9 %) | 32,613 (13.9 %) | 104 (8.3 %) | 4 | +0.021 ns | +0.037 ns | 0 |
| Deployed evaluator, S = 10 | 11,891 (10.2 %) | 9,970 (4.3 %) | 52 (4.2 %) | 3 | +0.110 ns | +0.031 ns | 0 |

- **Timing:** all three meet 8 ns (125 MHz), with no failing setup or hold endpoints, so they also meet the 100 MHz system clock. The HLS clock estimates (6.15 ns new, 10.62 ns deployed) were pessimistic for the deployed design. The new II 8 design meets 8 ns by only 6 ps; at 10 ns it has about 2 ns of margin.
- **Logs:** no critical warnings in any `vivado_ooc.log`.
- **Post-synthesis LUTs:** 42,492, 37,571 and 13,050. Routing optimisation reduced them.
- **Latency (cycles per call):**
  - new II 8: cosim 28,327–29,083 (3 calls); stored-cost runs 28,201 and 28,453;
  - deployed: cosim 483,780–486,062;
  - new II 16: HLS estimate only, 35,396–36,496; it was not cosimulated.
- **Ratios vs the deployed evaluator, post-route:**
  - new II 8: 3.3× LUTs, 3.8× FFs, 3.7× DSPs, for 16.7–17.2× fewer cycles;
  - new II 16: 2.9× LUTs, 3.3× FFs, 2.0× DSPs.
