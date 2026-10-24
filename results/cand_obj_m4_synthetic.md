# cand_obj M4 on the synthetic tuning suites

> Final review: these are the pre-fix (M4a) builds. cand_obj after the final-review fixes (S4 sticky lock, S8 truncation
> guard, S5 grid merge) on suite_v2: `results/final_review_fixes.md` section 1 (recall unchanged, precision +0.004).

Systems: `simple`, `cand_obj M3` (frozen build of commit 89317b2, verified to reproduce the M3 Isaac map byte-for-byte),
`cand_obj M4` (this milestone, commit f8a78c0, `config/cand_obj.cfg` == built-in defaults), `cand_grid` (unchanged since M3).
Runs: `treemapper_tools.evaluate` with the M4 build (`/data/work/build_m4a`) for simple / cand_obj M4 / cand_grid and the frozen build
(`/data/work/build_m4a_m3`) for cand_obj M3; raw outputs under `/data/work/eval/m4a/final` and `/data/work/eval/m4a/final_m3`.
Worst = worst seed (min for recall / precision / IoU / sprayed, max otherwise). Paired rows: diff = M4 - other per seed,
t = mean / (std / sqrt n), |t| < 2 is within seed noise; W / L / T counts seeds where M4 is better / worse / equal.
Run-to-run noise is 0 (deterministic), so every difference between builds on one seed is real; seed-to-seed spread is the noise floor.

Both suites were used for tuning (section 10 of ); the held-out verdict is in .

## suite_v2 (fixed-hover generator, 15 missions, seeds 200-204)

### easy (n = 5): mean / worst

| Metric | simple | cand_obj M3 | cand_obj M4 | cand_grid |
|---|---|---|---|---|
| recall (final) | 0.886 / 0.733 | 1 / 1 | 1 / 1 | 1 / 1 |
| precision (final) | 0.815 / 0.706 | 1 / 1 | 1 / 1 | 1 / 1 |
| recall (per-frame mean) | 0.843 / 0.738 | 0.908 / 0.891 | 0.892 / 0.889 | 0.887 / 0.885 |
| mean IoU (final) | 0.577 / 0.522 | 0.898 / 0.884 | 0.905 / 0.895 | 0.859 / 0.851 |
| min IoU (final) | 0.295 / 0.178 | 0.819 / 0.723 | 0.857 / 0.845 | 0.739 / 0.699 |
| neighbours swallowed (final) | 1.800 / 3 | 0 / 0 | 0 / 0 | 0 / 0 |
| split trees (final) | 1.800 / 3 | 0 / 0 | 0 / 0 | 0 / 0 |
| false trees (final) | 1.200 / 3 | 0 / 0 | 0 / 0 | 0 / 0 |
| centre RMSE m (final) | 1.035 / 1.345 | 0.175 / 0.215 | 0.141 / 0.162 | 0.171 / 0.207 |
| height RMSE m (final) | 0.169 / 0.208 | 0.055 / 0.102 | 0.054 / 0.066 | 0.061 / 0.081 |
| height bias m (final) | -0.070 / -0.011 | 0.010 / 0.097 | -0.012 / 0.060 | -0.013 / 0.069 |
| spill (final) | 0.461 / 0.525 | 0.064 / 0.073 | 0.050 / 0.060 | 0.073 / 0.089 |
| neighbour spill max (final) | 0.174 / 0.307 | 0.001 / 0.005 | 0 / 0 | 0.005 / 0.023 |
| centre jumps | 7.600 / 9 | 1.800 / 4 | 0 / 0 | 0 / 0 |
| height jumps | 0.600 / 1 | 0.200 / 1 | 0 / 0 | 0 / 0 |
| ID switches after lock | 0 / 0 | 0 / 0 | 0 / 0 | 0 / 0 |
| SAFETY total violations | 0 / 0 | 0 / 0 | 0 / 0 | 0 / 0 |
| implausible-height frames | 0 / 0 | 0 / 0 | 0 / 0 | 0 / 0 |
| truth trees sprayed | 3.400 / 2 | 4 / 4 | 4 / 4 | 4 / 4 |
| scan latency mean s | 0.559 / 0.600 | 1.090 / 1.160 | 1.185 / 1.300 | 1.443 / 1.536 |
| max height drift since lock m (worst) | 0.258 | 0.071 | 0 | 0.159 |
| max centre drift since lock m (worst) | 0.599 | 0.077 | 0.109 | 0.256 |
| target height range in own descent m (worst) | 0.280 | 0.068 | 0 | 0.144 |
| matched trees with abs height error > 1 m (of which under-estimates) / matched | 0 (0) / 61 | 0 (0) / 69 | 0 (0) / 69 | 0 (0) / 69 |
| max abs height error m | 0.446 | 0.140 | 0.121 | 0.134 |
| cost ms / frame, mean (worst max) | 0.517 (1.015) | 0.929 (25.054) | 0.972 (1.528) | 0.805 (1.785) |

#### Paired: cand_obj M4 vs cand_obj M3 on easy (diff = M4 - cand_obj M3; win = M4 better)

| Metric | mean diff | t | W / L / T |
|---|---|---|---|
| recall (final) | +0.000 | +0.0 | 0 / 0 / 5 |
| precision (final) | +0.000 | +0.0 | 0 / 0 / 5 |
| recall (per-frame mean) | -0.016 | -4.0 | 0 / 5 / 0 |
| mean IoU (final) | +0.007 | +3.4 | 5 / 0 / 0 |
| min IoU (final) | +0.038 | +1.6 | 3 / 1 / 1 |
| neighbours swallowed (final) | +0.000 | +0.0 | 0 / 0 / 5 |
| split trees (final) | +0.000 | +0.0 | 0 / 0 / 5 |
| false trees (final) | +0.000 | +0.0 | 0 / 0 / 5 |
| centre RMSE m (final) | -0.034 | -2.7 | 4 / 1 / 0 |
| height RMSE m (final) | -0.001 | -0.1 | 2 / 3 / 0 |
| spill (final) | -0.014 | -5.4 | 5 / 0 / 0 |
| neighbour spill max (final) | -0.001 | -1.0 | 1 / 0 / 4 |
| centre jumps | -1.800 | -2.7 | 4 / 0 / 1 |
| height jumps | -0.200 | -1.0 | 1 / 0 / 4 |
| ID switches after lock | +0.000 | +0.0 | 0 / 0 / 5 |
| SAFETY total violations | +0.000 | +0.0 | 0 / 0 / 5 |
| implausible-height frames | +0.000 | +0.0 | 0 / 0 / 5 |
| truth trees sprayed | +0.000 | +0.0 | 0 / 0 / 5 |
| scan latency mean s | +0.095 | +1.7 | 2 / 3 / 0 |

#### Paired: cand_obj M4 vs cand_grid on easy (diff = M4 - cand_grid; win = M4 better)

| Metric | mean diff | t | W / L / T |
|---|---|---|---|
| recall (final) | +0.000 | +0.0 | 0 / 0 / 5 |
| precision (final) | +0.000 | +0.0 | 0 / 0 / 5 |
| recall (per-frame mean) | +0.005 | +2.4 | 5 / 0 / 0 |
| mean IoU (final) | +0.045 | +6.3 | 5 / 0 / 0 |
| min IoU (final) | +0.117 | +5.4 | 5 / 0 / 0 |
| neighbours swallowed (final) | +0.000 | +0.0 | 0 / 0 / 5 |
| split trees (final) | +0.000 | +0.0 | 0 / 0 / 5 |
| false trees (final) | +0.000 | +0.0 | 0 / 0 / 5 |
| centre RMSE m (final) | -0.030 | -2.8 | 5 / 0 / 0 |
| height RMSE m (final) | -0.007 | -2.0 | 4 / 1 / 0 |
| spill (final) | -0.023 | -3.2 | 5 / 0 / 0 |
| neighbour spill max (final) | -0.005 | -1.0 | 1 / 0 / 4 |
| centre jumps | +0.000 | +0.0 | 0 / 0 / 5 |
| height jumps | +0.000 | +0.0 | 0 / 0 / 5 |
| ID switches after lock | +0.000 | +0.0 | 0 / 0 / 5 |
| SAFETY total violations | +0.000 | +0.0 | 0 / 0 / 5 |
| implausible-height frames | +0.000 | +0.0 | 0 / 0 / 5 |
| truth trees sprayed | +0.000 | +0.0 | 0 / 0 / 5 |
| scan latency mean s | -0.258 | -6.2 | 5 / 0 / 0 |

### medium (n = 5): mean / worst

| Metric | simple | cand_obj M3 | cand_obj M4 | cand_grid |
|---|---|---|---|---|
| recall (final) | 0.391 / 0.188 | 1 / 1 | 0.976 / 0.933 | 0.965 / 0.933 |
| precision (final) | 0.439 / 0.250 | 0.981 / 0.905 | 0.978 / 0.941 | 0.987 / 0.933 |
| recall (per-frame mean) | 0.383 / 0.225 | 0.892 / 0.874 | 0.871 / 0.816 | 0.858 / 0.827 |
| mean IoU (final) | 0.305 / 0.260 | 0.775 / 0.731 | 0.791 / 0.769 | 0.751 / 0.731 |
| min IoU (final) | 0.150 / 0.112 | 0.522 / 0.331 | 0.533 / 0.321 | 0.452 / 0.193 |
| neighbours swallowed (final) | 12.800 / 15 | 0.600 / 2 | 0.400 / 1 | 0.600 / 2 |
| split trees (final) | 0.200 / 1 | 0.400 / 2 | 0.400 / 1 | 0.200 / 1 |
| false trees (final) | 9 / 12 | 0 / 0 | 0 / 0 | 0 / 0 |
| centre RMSE m (final) | 1.974 / 2.469 | 0.385 / 0.478 | 0.407 / 0.548 | 0.562 / 0.759 |
| height RMSE m (final) | 0.688 / 1.048 | 0.141 / 0.219 | 0.187 / 0.377 | 0.154 / 0.218 |
| height bias m (final) | 0.012 / 0.207 | 0.100 / 0.171 | 0.079 / 0.171 | 0.065 / 0.198 |
| spill (final) | 0.669 / 0.728 | 0.171 / 0.193 | 0.154 / 0.167 | 0.167 / 0.188 |
| neighbour spill max (final) | 0.336 / 0.452 | 0.228 / 0.373 | 0.223 / 0.385 | 0.277 / 0.468 |
| centre jumps | 13.400 / 15 | 5.400 / 11 | 0 / 0 | 1.400 / 2 |
| height jumps | 1.200 / 4 | 0.400 / 2 | 0 / 0 | 0 / 0 |
| ID switches after lock | 0.200 / 1 | 0 / 0 | 0 / 0 | 0 / 0 |
| SAFETY total violations | 0 / 0 | 0 / 0 | 0 / 0 | 0 / 0 |
| implausible-height frames | 0 / 0 | 0 / 0 | 0 / 0 | 0 / 0 |
| truth trees sprayed | 3.200 / 2 | 5 / 5 | 4.800 / 4 | 5 / 4 |
| scan latency mean s | 0.760 / 1.086 | 1.537 / 2.313 | 1.547 / 2.287 | 1.855 / 2.494 |
| max height drift since lock m (worst) | 0.498 | 0.729 | 0.040 | 0.365 |
| max centre drift since lock m (worst) | 1.951 | 0.861 | 0.481 | 0.638 |
| target height range in own descent m (worst) | 0.432 | 0.446 | 0 | 0.494 |
| matched trees with abs height error > 1 m (of which under-estimates) / matched | 5 (3) / 32 | 0 (0) / 83 | 1 (0) / 81 | 0 (0) / 80 |
| max abs height error m | 1.600 | 0.352 | 1.501 | 0.747 |
| cost ms / frame, mean (worst max) | 0.478 (1.116) | 0.915 (1.734) | 0.967 (1.860) | 0.953 (2.341) |

#### Paired: cand_obj M4 vs cand_obj M3 on medium (diff = M4 - cand_obj M3; win = M4 better)

| Metric | mean diff | t | W / L / T |
|---|---|---|---|
| recall (final) | -0.024 | -1.6 | 0 / 2 / 3 |
| precision (final) | -0.003 | -0.2 | 1 / 1 / 3 |
| recall (per-frame mean) | -0.021 | -1.5 | 2 / 3 / 0 |
| mean IoU (final) | +0.016 | +1.7 | 4 / 1 / 0 |
| min IoU (final) | +0.011 | +0.2 | 2 / 3 / 0 |
| neighbours swallowed (final) | -0.200 | -1.0 | 1 / 0 / 4 |
| split trees (final) | +0.000 | +0.0 | 1 / 1 / 3 |
| false trees (final) | +0.000 | +0.0 | 0 / 0 / 5 |
| centre RMSE m (final) | +0.022 | +0.5 | 2 / 3 / 0 |
| height RMSE m (final) | +0.046 | +1.0 | 2 / 3 / 0 |
| spill (final) | -0.018 | -4.7 | 5 / 0 / 0 |
| neighbour spill max (final) | -0.005 | -0.2 | 4 / 1 / 0 |
| centre jumps | -5.400 | -3.6 | 5 / 0 / 0 |
| height jumps | -0.400 | -1.0 | 1 / 0 / 4 |
| ID switches after lock | +0.000 | +0.0 | 0 / 0 / 5 |
| SAFETY total violations | +0.000 | +0.0 | 0 / 0 / 5 |
| implausible-height frames | +0.000 | +0.0 | 0 / 0 / 5 |
| truth trees sprayed | -0.200 | -1.0 | 0 / 1 / 4 |
| scan latency mean s | +0.010 | +0.2 | 3 / 2 / 0 |

#### Paired: cand_obj M4 vs cand_grid on medium (diff = M4 - cand_grid; win = M4 better)

| Metric | mean diff | t | W / L / T |
|---|---|---|---|
| recall (final) | +0.011 | +0.4 | 2 / 1 / 2 |
| precision (final) | -0.008 | -0.4 | 1 / 2 / 2 |
| recall (per-frame mean) | +0.013 | +0.6 | 3 / 2 / 0 |
| mean IoU (final) | +0.040 | +7.2 | 5 / 0 / 0 |
| min IoU (final) | +0.082 | +3.0 | 4 / 1 / 0 |
| neighbours swallowed (final) | -0.200 | -1.0 | 1 / 0 / 4 |
| split trees (final) | +0.200 | +0.5 | 1 / 2 / 2 |
| false trees (final) | +0.000 | +0.0 | 0 / 0 / 5 |
| centre RMSE m (final) | -0.155 | -2.6 | 4 / 1 / 0 |
| height RMSE m (final) | +0.032 | +0.9 | 2 / 3 / 0 |
| spill (final) | -0.014 | -1.4 | 4 / 1 / 0 |
| neighbour spill max (final) | -0.055 | -2.2 | 4 / 1 / 0 |
| centre jumps | -1.400 | -5.7 | 5 / 0 / 0 |
| height jumps | +0.000 | +0.0 | 0 / 0 / 5 |
| ID switches after lock | +0.000 | +0.0 | 0 / 0 / 5 |
| SAFETY total violations | +0.000 | +0.0 | 0 / 0 / 5 |
| implausible-height frames | +0.000 | +0.0 | 0 / 0 / 5 |
| truth trees sprayed | -0.200 | -0.5 | 1 / 2 / 2 |
| scan latency mean s | -0.308 | -1.2 | 3 / 2 / 0 |

### hard (n = 5): mean / worst

| Metric | simple | cand_obj M3 | cand_obj M4 | cand_grid |
|---|---|---|---|---|
| recall (final) | 0.216 / 0.083 | 0.899 / 0.792 | 0.886 / 0.792 | 0.741 / 0.704 |
| precision (final) | 0.184 / 0.069 | 0.983 / 0.950 | 0.984 / 0.955 | 0.951 / 0.895 |
| recall (per-frame mean) | 0.223 / 0.116 | 0.802 / 0.705 | 0.786 / 0.703 | 0.659 / 0.626 |
| mean IoU (final) | 0.263 / 0.201 | 0.609 / 0.561 | 0.621 / 0.572 | 0.627 / 0.610 |
| min IoU (final) | 0.129 / 0.100 | 0.296 / 0.108 | 0.361 / 0.306 | 0.285 / 0.233 |
| neighbours swallowed (final) | 26.600 / 35 | 3.400 / 6 | 2.800 / 5 | 6 / 8 |
| split trees (final) | 0.200 / 1 | 0.600 / 2 | 0.400 / 1 | 0.200 / 1 |
| false trees (final) | 23 / 27 | 0 / 0 | 0 / 0 | 0.800 / 2 |
| centre RMSE m (final) | 2.184 / 3.763 | 0.803 / 1.077 | 0.729 / 0.949 | 1.150 / 1.399 |
| height RMSE m (final) | 0.817 / 1.197 | 0.436 / 0.768 | 0.224 / 0.296 | 0.559 / 0.894 |
| height bias m (final) | 0.022 / 0.620 | 0.012 / 0.225 | 0.097 / 0.182 | 0.134 / 0.326 |
| spill (final) | 0.821 / 0.850 | 0.266 / 0.305 | 0.250 / 0.274 | 0.280 / 0.342 |
| neighbour spill max (final) | 0.297 / 0.455 | 0.457 / 0.585 | 0.371 / 0.473 | 0.426 / 0.502 |
| centre jumps | 28.600 / 32 | 7 / 9 | 0 / 0 | 5.400 / 8 |
| height jumps | 3 / 4 | 1.600 / 3 | 0 / 0 | 0 / 0 |
| ID switches after lock | 0 / 0 | 0 / 0 | 0 / 0 | 0 / 0 |
| SAFETY total violations | 86.800 / 168 | 0 / 0 | 0 / 0 | 0 / 0 |
| implausible-height frames | 86.800 / 168 | 0 / 0 | 0 / 0 | 0 / 0 |
| truth trees sprayed | 2.600 / 1 | 5.800 / 5 | 5.800 / 5 | 6.200 / 5 |
| scan latency mean s | 0.752 / 1.026 | 2.366 / 3.323 | 2.573 / 3.908 | 4.710 / 7.730 |
| max height drift since lock m (worst) | 0.970 | 1.075 | 0.040 | 1.043 |
| max centre drift since lock m (worst) | 2.164 | 0.571 | 0.685 | 2.668 |
| target height range in own descent m (worst) | 0.582 | 0.584 | 0 | 0.757 |
| matched trees with abs height error > 1 m (of which under-estimates) / matched | 4 (2) / 26 | 7 (6) / 111 | 0 (0) / 109 | 7 (1) / 91 |
| max abs height error m | 1.541 | 2.567 | 0.854 | 2.643 |
| cost ms / frame, mean (worst max) | 0.439 (1.252) | 0.935 (5.708) | 0.983 (2.348) | 1.024 (2.766) |

#### Paired: cand_obj M4 vs cand_obj M3 on hard (diff = M4 - cand_obj M3; win = M4 better)

| Metric | mean diff | t | W / L / T |
|---|---|---|---|
| recall (final) | -0.013 | -0.5 | 1 / 1 / 3 |
| precision (final) | +0.001 | +0.0 | 2 / 2 / 1 |
| recall (per-frame mean) | -0.015 | -0.7 | 1 / 4 / 0 |
| mean IoU (final) | +0.012 | +0.9 | 2 / 3 / 0 |
| min IoU (final) | +0.064 | +1.2 | 3 / 2 / 0 |
| neighbours swallowed (final) | -0.600 | -1.2 | 3 / 1 / 1 |
| split trees (final) | -0.200 | -0.3 | 2 / 2 / 1 |
| false trees (final) | +0.000 | +0.0 | 0 / 0 / 5 |
| centre RMSE m (final) | -0.074 | -1.0 | 3 / 2 / 0 |
| height RMSE m (final) | -0.212 | -2.2 | 5 / 0 / 0 |
| spill (final) | -0.017 | -2.2 | 4 / 1 / 0 |
| neighbour spill max (final) | -0.086 | -2.2 | 5 / 0 / 0 |
| centre jumps | -7.000 | -12.8 | 5 / 0 / 0 |
| height jumps | -1.600 | -4.0 | 5 / 0 / 0 |
| ID switches after lock | +0.000 | +0.0 | 0 / 0 / 5 |
| SAFETY total violations | +0.000 | +0.0 | 0 / 0 / 5 |
| implausible-height frames | +0.000 | +0.0 | 0 / 0 / 5 |
| truth trees sprayed | +0.000 | +0.0 | 0 / 0 / 5 |
| scan latency mean s | +0.207 | +1.4 | 1 / 4 / 0 |

#### Paired: cand_obj M4 vs cand_grid on hard (diff = M4 - cand_grid; win = M4 better)

| Metric | mean diff | t | W / L / T |
|---|---|---|---|
| recall (final) | +0.144 | +4.3 | 5 / 0 / 0 |
| precision (final) | +0.033 | +1.2 | 3 / 1 / 1 |
| recall (per-frame mean) | +0.127 | +3.9 | 5 / 0 / 0 |
| mean IoU (final) | -0.006 | -0.6 | 3 / 2 / 0 |
| min IoU (final) | +0.075 | +2.3 | 4 / 1 / 0 |
| neighbours swallowed (final) | -3.200 | -2.9 | 5 / 0 / 0 |
| split trees (final) | +0.200 | +0.5 | 1 / 2 / 2 |
| false trees (final) | -0.800 | -2.1 | 3 / 0 / 2 |
| centre RMSE m (final) | -0.420 | -5.9 | 5 / 0 / 0 |
| height RMSE m (final) | -0.335 | -2.5 | 5 / 0 / 0 |
| spill (final) | -0.030 | -1.2 | 4 / 1 / 0 |
| neighbour spill max (final) | -0.055 | -2.7 | 4 / 1 / 0 |
| centre jumps | -5.400 | -7.2 | 5 / 0 / 0 |
| height jumps | +0.000 | +0.0 | 0 / 0 / 5 |
| ID switches after lock | +0.000 | +0.0 | 0 / 0 / 5 |
| SAFETY total violations | +0.000 | +0.0 | 0 / 0 / 5 |
| implausible-height frames | +0.000 | +0.0 | 0 / 0 / 5 |
| truth trees sprayed | -0.400 | -1.6 | 0 / 2 / 3 |
| scan latency mean s | -2.137 | -1.9 | 4 / 1 / 0 |


## suite_v1 (15 missions, seeds 0-4; descent hover clears only the target, D30)

Sources for this table: cand_obj M4 from the configuration sweep run C (`/data/work/eval/m4a/v1_C`, same binary and the
shipped defaults, no overrides; the three-system suite_v1 rerun had not finished when this file was written), cand_obj M3
from `/data/work/eval/m4a/final_m3`, simple and cand_grid from the M3 result files (`results/cand_obj_synthetic.json`,
`results/cand_grid_synthetic.json`; same code, deterministic).

### easy (n = 5): mean / worst

| Metric | simple | cand_obj M3 | cand_obj M4 | cand_grid |
|---|---|---|---|---|
| recall (final) | 0.899 / 0.786 | 1 / 1 | 1 / 1 | 1 / 1 |
| precision (final) | 0.889 / 0.857 | 1 / 1 | 1 / 1 | 1 / 1 |
| recall (per-frame mean) | 0.835 / 0.736 | 0.900 / 0.892 | 0.891 / 0.882 | 0.885 / 0.871 |
| mean IoU (final) | 0.582 / 0.492 | 0.903 / 0.895 | 0.913 / 0.904 | 0.875 / 0.862 |
| min IoU (final) | 0.195 / 0.135 | 0.802 / 0.752 | 0.866 / 0.825 | 0.775 / 0.733 |
| neighbours swallowed (final) | 2.200 / 4 | 0 / 0 | 0 / 0 | 0 / 0 |
| split trees (final) | 1 / 2 | 0 / 0 | 0 / 0 | 0 / 0 |
| false trees (final) | 1.600 / 3 | 0 / 0 | 0 / 0 | 0 / 0 |
| centre RMSE m (final) | 1.244 / 1.503 | 0.140 / 0.161 | 0.125 / 0.146 | 0.146 / 0.193 |
| height RMSE m (final) | 0.247 / 0.387 | 0.056 / 0.093 | 0.059 / 0.110 | 0.063 / 0.126 |
| height bias m (final) | -0.115 / 0.001 | -0.013 / 0.060 | -0.038 / 0.032 | -0.032 / 0.037 |
| spill (final) | 0.431 / 0.530 | 0.062 / 0.071 | 0.047 / 0.054 | 0.062 / 0.073 |
| neighbour spill max (final) | 0.288 / 0.370 | 0.004 / 0.011 | 0.004 / 0.010 | 0.021 / 0.098 |
| centre jumps | 8.600 / 11 | 3.200 / 4 | 0 / 0 | 0 / 0 |
| height jumps | 0 / 0 | 0.600 / 2 | 0 / 0 | 0 / 0 |
| ID switches after lock | 0 / 0 | 0 / 0 | 0 / 0 | 0 / 0 |
| SAFETY total violations | 0 / 0 | 0 / 0 | 0 / 0 | 0 / 0 |
| implausible-height frames | 0 / 0 | 0 / 0 | 0 / 0 | 0 / 0 |
| truth trees sprayed | 3.200 / 2 | 4 / 4 | 4 / 4 | 4 / 4 |
| scan latency mean s | 0.596 / 0.677 | 1.190 / 1.300 | 1.166 / 1.323 | 1.474 / 1.529 |
| max height drift since lock m (worst) | 0.281 | 0.106 | 0 | 0.184 |
| max centre drift since lock m (worst) | 0.336 | 0.088 | 0.129 | 0.305 |
| target height range in own descent m (worst) | 0.225 | 0.099 | 0 | 0.195 |
| matched trees with abs height error > 1 m (of which under-estimates) / matched | 1 (1) / 63 | 0 (0) / 70 | 0 (0) / 70 | 0 (0) / 70 |
| max abs height error m | 1.200 | 0.137 | 0.188 | 0.208 |
| cost ms / frame, mean (worst max) | 0.521 (0.941) | 0.897 (1.282) | 1.056 (26.935) | 0.784 (1.967) |

#### Paired: cand_obj M4 vs cand_obj M3 on easy (diff = M4 - cand_obj M3; win = M4 better)

| Metric | mean diff | t | W / L / T |
|---|---|---|---|
| recall (final) | +0.000 | +0.0 | 0 / 0 / 5 |
| precision (final) | +0.000 | +0.0 | 0 / 0 / 5 |
| recall (per-frame mean) | -0.009 | -2.1 | 1 / 4 / 0 |
| mean IoU (final) | +0.010 | +6.9 | 5 / 0 / 0 |
| min IoU (final) | +0.064 | +5.1 | 5 / 0 / 0 |
| neighbours swallowed (final) | +0.000 | +0.0 | 0 / 0 / 5 |
| split trees (final) | +0.000 | +0.0 | 0 / 0 / 5 |
| false trees (final) | +0.000 | +0.0 | 0 / 0 / 5 |
| centre RMSE m (final) | -0.016 | -2.0 | 4 / 1 / 0 |
| height RMSE m (final) | +0.003 | +0.4 | 2 / 3 / 0 |
| spill (final) | -0.015 | -7.6 | 5 / 0 / 0 |
| neighbour spill max (final) | -0.000 | -1.4 | 3 / 1 / 1 |
| centre jumps | -3.200 | -8.6 | 5 / 0 / 0 |
| height jumps | -0.600 | -1.5 | 2 / 0 / 3 |
| ID switches after lock | +0.000 | +0.0 | 0 / 0 / 5 |
| SAFETY total violations | +0.000 | +0.0 | 0 / 0 / 5 |
| implausible-height frames | +0.000 | +0.0 | 0 / 0 / 5 |
| truth trees sprayed | +0.000 | +0.0 | 0 / 0 / 5 |
| scan latency mean s | -0.024 | -0.6 | 3 / 2 / 0 |

#### Paired: cand_obj M4 vs cand_grid on easy (diff = M4 - cand_grid; win = M4 better)

| Metric | mean diff | t | W / L / T |
|---|---|---|---|
| recall (final) | +0.000 | +0.0 | 0 / 0 / 5 |
| precision (final) | +0.000 | +0.0 | 0 / 0 / 5 |
| recall (per-frame mean) | +0.007 | +1.9 | 5 / 0 / 0 |
| mean IoU (final) | +0.038 | +9.5 | 5 / 0 / 0 |
| min IoU (final) | +0.091 | +4.7 | 5 / 0 / 0 |
| neighbours swallowed (final) | +0.000 | +0.0 | 0 / 0 / 5 |
| split trees (final) | +0.000 | +0.0 | 0 / 0 / 5 |
| false trees (final) | +0.000 | +0.0 | 0 / 0 / 5 |
| centre RMSE m (final) | -0.022 | -1.5 | 4 / 1 / 0 |
| height RMSE m (final) | -0.004 | -1.0 | 3 / 2 / 0 |
| spill (final) | -0.015 | -3.1 | 4 / 1 / 0 |
| neighbour spill max (final) | -0.017 | -0.9 | 1 / 2 / 2 |
| centre jumps | +0.000 | +0.0 | 0 / 0 / 5 |
| height jumps | +0.000 | +0.0 | 0 / 0 / 5 |
| ID switches after lock | +0.000 | +0.0 | 0 / 0 / 5 |
| SAFETY total violations | +0.000 | +0.0 | 0 / 0 / 5 |
| implausible-height frames | +0.000 | +0.0 | 0 / 0 / 5 |
| truth trees sprayed | +0.000 | +0.0 | 0 / 0 / 5 |
| scan latency mean s | -0.308 | -5.9 | 5 / 0 / 0 |

### medium (n = 5): mean / worst

| Metric | simple | cand_obj M3 | cand_obj M4 | cand_grid |
|---|---|---|---|---|
| recall (final) | 0.496 / 0.300 | 0.980 / 0.900 | 0.990 / 0.950 | 0.968 / 0.944 |
| precision (final) | 0.527 / 0.429 | 0.989 / 0.947 | 1 / 1 | 0.989 / 0.944 |
| recall (per-frame mean) | 0.462 / 0.279 | 0.874 / 0.792 | 0.879 / 0.840 | 0.853 / 0.833 |
| mean IoU (final) | 0.373 / 0.328 | 0.761 / 0.737 | 0.787 / 0.772 | 0.767 / 0.749 |
| min IoU (final) | 0.135 / 0.105 | 0.513 / 0.352 | 0.574 / 0.539 | 0.516 / 0.455 |
| neighbours swallowed (final) | 14.600 / 19 | 1 / 3 | 1 / 2 | 1 / 1 |
| split trees (final) | 0.400 / 1 | 0.200 / 1 | 0 / 0 | 0 / 0 |
| false trees (final) | 9.600 / 13 | 0 / 0 | 0 / 0 | 0.200 / 1 |
| centre RMSE m (final) | 2.058 / 2.846 | 0.456 / 0.664 | 0.388 / 0.509 | 0.526 / 0.694 |
| height RMSE m (final) | 0.555 / 0.822 | 0.188 / 0.258 | 0.131 / 0.290 | 0.175 / 0.315 |
| height bias m (final) | 0.056 / 0.252 | 0.000 / 0.217 | 0.017 / 0.208 | -0.029 / 0.147 |
| spill (final) | 0.640 / 0.688 | 0.188 / 0.208 | 0.166 / 0.181 | 0.180 / 0.214 |
| neighbour spill max (final) | 0.391 / 0.445 | 0.235 / 0.374 | 0.224 / 0.323 | 0.256 / 0.318 |
| centre jumps | 12.200 / 13 | 8.600 / 10 | 0 / 0 | 0.800 / 2 |
| height jumps | 0.400 / 1 | 0.200 / 1 | 0 / 0 | 0 / 0 |
| ID switches after lock | 0.200 / 1 | 0 / 0 | 0 / 0 | 0 / 0 |
| SAFETY total violations | 0 / 0 | 0 / 0 | 0 / 0 | 7.400 / 37 |
| implausible-height frames | 0 / 0 | 0 / 0 | 0 / 0 | 7.400 / 37 |
| truth trees sprayed | 2.800 / 1 | 5 / 5 | 5 / 5 | 5 / 5 |
| scan latency mean s | 0.821 / 1.294 | 1.826 / 2.883 | 1.950 / 2.826 | 2.491 / 3.678 |
| max height drift since lock m (worst) | 0.515 | 0.383 | 0 | 0.352 |
| max centre drift since lock m (worst) | 1.819 | 0.243 | 0.172 | 0.392 |
| target height range in own descent m (worst) | 0.561 | 0.303 | 0 | 0.367 |
| matched trees with abs height error > 1 m (of which under-estimates) / matched | 4 (1) / 45 | 0 (0) / 90 | 0 (0) / 91 | 1 (1) / 89 |
| max abs height error m | 2.207 | 0.922 | 0.958 | 1.186 |
| cost ms / frame, mean (worst max) | 0.511 (0.996) | 0.931 (1.808) | 1.010 (1.930) | 0.959 (2.317) |

#### Paired: cand_obj M4 vs cand_obj M3 on medium (diff = M4 - cand_obj M3; win = M4 better)

| Metric | mean diff | t | W / L / T |
|---|---|---|---|
| recall (final) | +0.010 | +1.0 | 1 / 0 / 4 |
| precision (final) | +0.011 | +1.0 | 1 / 0 / 4 |
| recall (per-frame mean) | +0.006 | +0.5 | 2 / 3 / 0 |
| mean IoU (final) | +0.026 | +2.7 | 4 / 1 / 0 |
| min IoU (final) | +0.061 | +1.3 | 4 / 1 / 0 |
| neighbours swallowed (final) | +0.000 | +0.0 | 1 / 1 / 3 |
| split trees (final) | -0.200 | -1.0 | 1 / 0 / 4 |
| false trees (final) | +0.000 | +0.0 | 0 / 0 / 5 |
| centre RMSE m (final) | -0.068 | -1.8 | 4 / 1 / 0 |
| height RMSE m (final) | -0.057 | -1.7 | 4 / 1 / 0 |
| spill (final) | -0.022 | -7.7 | 5 / 0 / 0 |
| neighbour spill max (final) | -0.011 | -0.8 | 2 / 2 / 1 |
| centre jumps | -8.600 | -12.7 | 5 / 0 / 0 |
| height jumps | -0.200 | -1.0 | 1 / 0 / 4 |
| ID switches after lock | +0.000 | +0.0 | 0 / 0 / 5 |
| SAFETY total violations | +0.000 | +0.0 | 0 / 0 / 5 |
| implausible-height frames | +0.000 | +0.0 | 0 / 0 / 5 |
| truth trees sprayed | +0.000 | +0.0 | 0 / 0 / 5 |
| scan latency mean s | +0.124 | +0.4 | 1 / 4 / 0 |

#### Paired: cand_obj M4 vs cand_grid on medium (diff = M4 - cand_grid; win = M4 better)

| Metric | mean diff | t | W / L / T |
|---|---|---|---|
| recall (final) | +0.022 | +1.6 | 2 / 0 / 3 |
| precision (final) | +0.011 | +1.0 | 1 / 0 / 4 |
| recall (per-frame mean) | +0.026 | +2.0 | 4 / 1 / 0 |
| mean IoU (final) | +0.021 | +2.4 | 4 / 1 / 0 |
| min IoU (final) | +0.058 | +1.5 | 4 / 1 / 0 |
| neighbours swallowed (final) | +0.000 | +0.0 | 2 / 2 / 1 |
| split trees (final) | +0.000 | +0.0 | 0 / 0 / 5 |
| false trees (final) | -0.200 | -1.0 | 1 / 0 / 4 |
| centre RMSE m (final) | -0.138 | -3.8 | 5 / 0 / 0 |
| height RMSE m (final) | -0.044 | -1.1 | 4 / 1 / 0 |
| spill (final) | -0.015 | -1.1 | 3 / 2 / 0 |
| neighbour spill max (final) | -0.031 | -1.0 | 3 / 2 / 0 |
| centre jumps | -0.800 | -1.6 | 2 / 0 / 3 |
| height jumps | +0.000 | +0.0 | 0 / 0 / 5 |
| ID switches after lock | +0.000 | +0.0 | 0 / 0 / 5 |
| SAFETY total violations | -7.400 | -1.0 | 1 / 0 / 4 |
| implausible-height frames | -7.400 | -1.0 | 1 / 0 / 4 |
| truth trees sprayed | +0.000 | +0.0 | 0 / 0 / 5 |
| scan latency mean s | -0.542 | -2.3 | 4 / 1 / 0 |

### hard (n = 5): mean / worst

| Metric | simple | cand_obj M3 | cand_obj M4 | cand_grid |
|---|---|---|---|---|
| recall (final) | 0.271 / 0.087 | 0.864 / 0.826 | 0.823 / 0.783 | 0.778 / 0.652 |
| precision (final) | 0.250 / 0.087 | 0.972 / 0.950 | 0.962 / 0.909 | 0.969 / 0.947 |
| recall (per-frame mean) | 0.262 / 0.106 | 0.767 / 0.726 | 0.728 / 0.689 | 0.697 / 0.585 |
| mean IoU (final) | 0.219 / 0.144 | 0.574 / 0.537 | 0.593 / 0.545 | 0.634 / 0.602 |
| min IoU (final) | 0.120 / 0.101 | 0.294 / 0.140 | 0.308 / 0.194 | 0.420 / 0.365 |
| neighbours swallowed (final) | 25.200 / 29 | 2.200 / 4 | 2 / 3 | 3.800 / 5 |
| split trees (final) | 0.200 / 1 | 0.800 / 1 | 0.600 / 2 | 0 / 0 |
| false trees (final) | 22 / 26 | 0 / 0 | 0.200 / 1 | 0.600 / 1 |
| centre RMSE m (final) | 2.646 / 3.366 | 0.856 / 1.119 | 0.810 / 1.035 | 0.767 / 0.856 |
| height RMSE m (final) | 0.828 / 1.375 | 0.591 / 0.912 | 0.562 / 0.895 | 0.342 / 0.422 |
| height bias m (final) | 0.278 / 1.029 | 0.008 / 0.513 | 0.013 / 0.464 | 0.013 / 0.251 |
| spill (final) | 0.830 / 0.859 | 0.306 / 0.336 | 0.287 / 0.325 | 0.282 / 0.324 |
| neighbour spill max (final) | 0.436 / 0.637 | 0.436 / 0.581 | 0.365 / 0.502 | 0.430 / 0.541 |
| centre jumps | 33.200 / 36 | 6.400 / 12 | 0 / 0 | 2.200 / 7 |
| height jumps | 2.200 / 4 | 1 / 3 | 0 / 0 | 0 / 0 |
| ID switches after lock | 0 / 0 | 0 / 0 | 0 / 0 | 0 / 0 |
| SAFETY total violations | 132 / 225 | 0 / 0 | 0 / 0 | 17 / 43 |
| implausible-height frames | 132 / 225 | 0 / 0 | 0 / 0 | 17 / 43 |
| truth trees sprayed | 2.600 / 2 | 5.200 / 4 | 5.400 / 4 | 5.200 / 4 |
| scan latency mean s | 0.744 / 0.926 | 2.226 / 3.600 | 2.607 / 4.745 | 1.907 / 3.933 |
| max height drift since lock m (worst) | 0.895 | 1.402 | 0 | 2.240 |
| max centre drift since lock m (worst) | 3.528 | 0.873 | 2.281 | 0.857 |
| target height range in own descent m (worst) | 0.712 | 1.402 | 0 | 1.840 |
| matched trees with abs height error > 1 m (of which under-estimates) / matched | 11 (5) / 32 | 6 (3) / 102 | 5 (2) / 97 | 1 (0) / 92 |
| max abs height error m | 2.294 | 2.997 | 2.880 | 1.118 |
| cost ms / frame, mean (worst max) | 0.455 (1.248) | 0.916 (2.287) | 1.019 (2.471) | 0.998 (2.761) |

#### Paired: cand_obj M4 vs cand_obj M3 on hard (diff = M4 - cand_obj M3; win = M4 better)

| Metric | mean diff | t | W / L / T |
|---|---|---|---|
| recall (final) | -0.042 | -1.8 | 0 / 3 / 2 |
| precision (final) | -0.010 | -0.6 | 1 / 3 / 1 |
| recall (per-frame mean) | -0.038 | -2.0 | 0 / 5 / 0 |
| mean IoU (final) | +0.018 | +2.0 | 4 / 1 / 0 |
| min IoU (final) | +0.014 | +0.3 | 3 / 2 / 0 |
| neighbours swallowed (final) | -0.200 | -1.0 | 1 / 0 / 4 |
| split trees (final) | -0.200 | -0.5 | 2 / 1 / 2 |
| false trees (final) | +0.200 | +1.0 | 0 / 1 / 4 |
| centre RMSE m (final) | -0.046 | -0.7 | 3 / 2 / 0 |
| height RMSE m (final) | -0.029 | -0.7 | 3 / 2 / 0 |
| spill (final) | -0.019 | -5.8 | 5 / 0 / 0 |
| neighbour spill max (final) | -0.072 | -1.5 | 4 / 1 / 0 |
| centre jumps | -6.400 | -3.5 | 5 / 0 / 0 |
| height jumps | -1.000 | -1.6 | 2 / 0 / 3 |
| ID switches after lock | +0.000 | +0.0 | 0 / 0 / 5 |
| SAFETY total violations | +0.000 | +0.0 | 0 / 0 / 5 |
| implausible-height frames | +0.000 | +0.0 | 0 / 0 / 5 |
| truth trees sprayed | +0.200 | +0.5 | 2 / 1 / 2 |
| scan latency mean s | +0.381 | +1.0 | 1 / 4 / 0 |

#### Paired: cand_obj M4 vs cand_grid on hard (diff = M4 - cand_grid; win = M4 better)

| Metric | mean diff | t | W / L / T |
|---|---|---|---|
| recall (final) | +0.045 | +0.9 | 2 / 2 / 1 |
| precision (final) | -0.007 | -0.3 | 2 / 3 / 0 |
| recall (per-frame mean) | +0.032 | +0.7 | 2 / 3 / 0 |
| mean IoU (final) | -0.041 | -2.0 | 1 / 4 / 0 |
| min IoU (final) | -0.112 | -2.0 | 1 / 4 / 0 |
| neighbours swallowed (final) | -1.800 | -3.1 | 5 / 0 / 0 |
| split trees (final) | +0.600 | +1.5 | 0 / 2 / 3 |
| false trees (final) | -0.400 | -1.0 | 3 / 1 / 1 |
| centre RMSE m (final) | +0.043 | +0.5 | 2 / 3 / 0 |
| height RMSE m (final) | +0.220 | +2.4 | 1 / 4 / 0 |
| spill (final) | +0.005 | +0.2 | 3 / 2 / 0 |
| neighbour spill max (final) | -0.065 | -3.1 | 5 / 0 / 0 |
| centre jumps | -2.200 | -1.8 | 4 / 0 / 1 |
| height jumps | +0.000 | +0.0 | 0 / 0 / 5 |
| ID switches after lock | +0.000 | +0.0 | 0 / 0 / 5 |
| SAFETY total violations | -17.000 | -2.1 | 3 / 0 / 2 |
| implausible-height frames | -17.000 | -2.1 | 3 / 0 / 2 |
| truth trees sprayed | +0.200 | +0.4 | 1 / 1 / 3 |
| scan latency mean s | +0.700 | +1.9 | 1 / 4 / 0 |

