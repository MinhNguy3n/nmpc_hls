# BH comparison tables (SIMULATION, emulated solver, assumed and approximate parameters; 1.5 kg payload; phase 4 NMPC on seeds 1-5, held-out smoother variant and baseline on seeds 6-10; 2026-11-22)

Median (range) over seeds.

| arm | comp | arith | profile | path | v | contact | seeds | xte_rmse [mm] | xte_max [mm] | head_rmse [mrad] | head_max [mrad] | completion | safety | misses | stale | fallbacks | overruns | busy | steer_lim [%] | rate_lim [%] | slip_rms [mm/s] | cost_final |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| baseline | - | - | realistic_guess | circle | 1.0 | 70mm | 5 | 2.4 (2.3-2.4) | 5.2 (5.0-5.3) | 14.3 (13.9-14.9) | 35.5 (32.3-37.2) | 1.00 | 0 | - | - | - | - | - | 0.0 | 2.4 (1.7-3.2) | 13.3 (13.0-13.3) | - |
| baseline | - | - | realistic_guess | figure_eight | 1.0 | 70mm | 5 | 2.1 (2.1-2.1) | 5.6 (5.2-6.2) | 12.8 (12.6-12.8) | 40.4 (35.3-44.0) | 1.00 | 0 | - | - | - | - | - | 0.0 | 1.8 (1.5-3.2) | 11.4 (11.2-11.5) | - |
| baseline | - | - | realistic_guess | lane_change | 1.0 | 70mm | 5 | 0.8 (0.7-0.8) | 2.5 (1.8-2.9) | 5.5 (5.3-6.3) | 18.8 (12.9-27.6) | 1.00 | 0 | - | - | - | - | - | 0.0 | 2.3 (1.2-3.4) | 3.7 (3.6-3.9) | - |
| baseline | - | - | realistic_guess | line | 1.0 | 70mm | 5 | 0.3 (0.3-0.4) | 1.0 (0.8-1.2) | 3.2 (2.8-4.2) | 10.1 (7.8-13.5) | 1.00 | 0 | - | - | - | - | - | 0.0 | 1.5 (1.2-2.0) | 0.0 (0.0-0.0) | - |
| nmpc10x40 | on | half | realistic_guess | circle | 1.0 | 70mm | 5 | 6.3 (5.8-6.9) | 12.1 (9.6-12.4) | 13.3 (12.0-13.9) | 39.9 (38.0-45.7) | 1.00 | 0 | 0 | 0 (0-1) | 0 | 0 | 0 | 0.0 (0.0-0.3) | 9.6 (8.9-12.3) | 12.1 (11.7-12.5) | 1.0 (0.7-1.5) |
| nmpc10x40 | on | half | realistic_guess | figure_eight | 1.0 | 70mm | 5 | 5.8 (5.4-5.9) | 13.2 (11.7-13.5) | 11.6 (10.8-12.7) | 50.9 (39.1-53.9) | 1.00 | 0 | 0 | 0 (0-1) | 0 | 0 (0-1) | 0 | 0.0 | 9.6 (8.8-12.8) | 10.4 (9.9-10.8) | 0.5 (0.3-0.5) |
| nmpc10x40 | on | half | realistic_guess | lane_change | 1.0 | 70mm | 5 | 2.0 (1.8-2.4) | 5.6 (4.3-5.8) | 3.3 (2.8-4.2) | 13.0 (9.5-22.2) | 1.00 | 0 | 0 | 0 | 0 | 0 | 0 | 0.0 | 8.4 (6.6-8.9) | 3.2 (3.2-3.5) | 0.1 (0.1-0.2) |
| nmpc10x40 | on | half | realistic_guess | line | 1.0 | 70mm | 5 | 0.9 (0.6-1.4) | 1.7 (1.2-2.9) | 2.4 (2.1-3.7) | 10.3 (5.7-24.6) | 1.00 | 0 | 0 | 0 | 0 | 0 | 0 | 0.0 | 7.0 (6.0-9.3) | 0.0 (0.0-0.0) | 0.2 (0.1-0.3) |
| nmpc10x40_redesign_smooth | on | half | realistic_guess | circle | 1.0 | 70mm | 5 | 6.5 (6.3-6.6) | 11.8 (11.3-13.3) | 14.9 (14.8-15.2) | 34.7 (32.2-36.6) | 1.00 | 0 | 0 | 0 (0-1) | 0 | 0 | 0 | 0.0 | 0.4 (0.3-0.8) | 14.0 (13.9-14.0) | 0.2 (0.2-0.4) |
| nmpc10x40_redesign_smooth | on | half | realistic_guess | figure_eight | 1.0 | 70mm | 5 | 5.2 (5.1-5.6) | 13.1 (11.6-13.3) | 12.9 (12.9-13.4) | 38.7 (36.6-44.7) | 1.00 | 0 | 0 | 0 | 0 | 0 | 0 | 0.0 | 0.6 (0.2-0.9) | 11.6 (11.5-11.9) | 0.2 (0.1-0.3) |
| nmpc10x40_redesign_smooth | on | half | realistic_guess | lane_change | 1.0 | 70mm | 5 | 1.7 (1.3-1.9) | 4.3 (3.0-4.8) | 5.6 (4.6-7.9) | 17.7 (16.0-22.3) | 1.00 | 0 | 0 | 0 | 0 | 0 | 0 | 0.0 | 0.7 (0.7-1.5) | 3.4 (3.1-3.6) | 0.1 (0.1-0.3) |
| nmpc10x40_redesign_smooth | on | half | realistic_guess | line | 1.0 | 70mm | 5 | 0.8 (0.4-0.9) | 1.7 (1.3-1.8) | 2.3 (2.0-3.3) | 6.2 (5.3-8.3) | 1.00 | 0 | 0 | 0 | 0 | 0 | 0 | 0.0 | 0.5 (0.3-0.6) | 0.0 (0.0-0.0) | 0.0 (0.0-0.1) |

## Paired differences, NMPC - baseline (same seeds)

| arm | comp | arith | profile | path | v | contact | seeds | d xte_rmse [mm] | d xte_max [mm] | d head_rmse [mrad] | d head_max [mrad] | d steer_lim [%] | d rate_lim [%] | d slip_rms [mm/s] |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| nmpc10x40 | on | half | realistic_guess | circle | 1.0 | 70mm | 0 | - | - | - | - | - | - | - |
| nmpc10x40 | on | half | realistic_guess | figure_eight | 1.0 | 70mm | 0 | - | - | - | - | - | - | - |
| nmpc10x40 | on | half | realistic_guess | lane_change | 1.0 | 70mm | 0 | - | - | - | - | - | - | - |
| nmpc10x40 | on | half | realistic_guess | line | 1.0 | 70mm | 0 | - | - | - | - | - | - | - |
| nmpc10x40_redesign_smooth | on | half | realistic_guess | circle | 1.0 | 70mm | 5 | 4.09 (3.85-4.24) | 6.41 (6.00-8.38) | 0.58 (0.06-0.92) | -2.37 (-3.29-4.28) | 0.00 | -1.92 (-2.83--0.89) | 0.67 (0.62-0.97) |
| nmpc10x40_redesign_smooth | on | half | realistic_guess | figure_eight | 1.0 | 70mm | 5 | 3.14 (3.02-3.50) | 6.98 (5.42-8.09) | 0.19 (0.06-0.74) | 0.22 (-6.94-9.42) | 0.00 | -1.33 (-2.33--0.86) | 0.40 (0.14-0.46) |
| nmpc10x40_redesign_smooth | on | half | realistic_guess | lane_change | 1.0 | 70mm | 5 | 0.98 (0.48-1.02) | 1.67 (0.46-2.49) | 0.25 (-1.23-1.63) | -1.90 (-5.68-4.84) | 0.00 | -1.55 (-2.62-0.03) | -0.50 (-0.50--0.07) |
| nmpc10x40_redesign_smooth | on | half | realistic_guess | line | 1.0 | 70mm | 5 | 0.36 (0.12-0.45) | 0.60 (0.50-0.84) | -0.73 (-2.04-0.07) | -2.69 (-8.17--0.75) | 0.00 | -0.94 (-1.62--0.53) | -0.02 (-0.03--0.02) |
