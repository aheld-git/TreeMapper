# cand_obj M4 on the held-out suite (30 missions, seeds 100-109 x easy / medium / hard; not used for parameter tuning)

> Caveat (final review): this suite was inspected several times (M3 selection, this M4a verdict, the M4c real-profile
> check, the R14 scorer-fix effect), so it is a validation set, no longer an unbiased test. Use a fresh seed range for
> future claims. Machine-readable numbers: `results/cand_obj_m4_holdout.json`.

Systems: `simple`, `cand_obj M3` (frozen build of commit 89317b2, verified to reproduce the M3 Isaac map byte-for-byte),
`cand_obj M4` (this milestone, commit f8a78c0, `config/cand_obj.cfg` == built-in defaults), `cand_grid` (unchanged since M3).
Runs: `treemapper_tools.evaluate` with the M4 build (`/data/work/build_m4a`) for simple / cand_obj M4 / cand_grid and the frozen build
(`/data/work/build_m4a_m3`) for cand_obj M3; raw outputs under `/data/work/eval/m4a/final` and `/data/work/eval/m4a/final_m3`.
Worst = worst seed (min for recall / precision / IoU / sprayed, max otherwise). Paired rows: diff = M4 - other per seed,
t = mean / (std / sqrt n), |t| < 2 is within seed noise; W / L / T counts seeds where M4 is better / worse / equal.
Run-to-run noise is 0 (deterministic), so every difference between builds on one seed is real; seed-to-seed spread is the noise floor.

Tuning used suite_v1, suite_v2 (fixed hover) and the recording only; this suite was run once, after the configuration was frozen.

### easy (n = 10): mean / worst

| Metric | simple | cand_obj M3 | cand_obj M4 | cand_grid |
|---|---|---|---|---|
| recall (final) | 0.966 / 0.867 | 1 / 1 | 1 / 1 | 1 / 1 |
| precision (final) | 0.861 / 0.778 | 0.993 / 0.929 | 0.988 / 0.938 | 1 / 1 |
| recall (per-frame mean) | 0.902 / 0.790 | 0.899 / 0.886 | 0.893 / 0.883 | 0.887 / 0.880 |
| mean IoU (final) | 0.583 / 0.551 | 0.903 / 0.895 | 0.908 / 0.892 | 0.872 / 0.848 |
| min IoU (final) | 0.274 / 0.167 | 0.848 / 0.775 | 0.840 / 0.672 | 0.761 / 0.661 |
| neighbours swallowed (final) | 1.800 / 3 | 0 / 0 | 0 / 0 | 0 / 0 |
| split trees (final) | 1.500 / 2 | 0.100 / 1 | 0.200 / 1 | 0 / 0 |
| false trees (final) | 0.800 / 2 | 0 / 0 | 0 / 0 | 0 / 0 |
| centre RMSE m (final) | 1.048 / 1.249 | 0.141 / 0.162 | 0.140 / 0.214 | 0.153 / 0.262 |
| height RMSE m (final) | 0.169 / 0.240 | 0.050 / 0.068 | 0.038 / 0.055 | 0.056 / 0.110 |
| height bias m (final) | -0.064 / 0.025 | 0.019 / 0.057 | 0.001 / 0.039 | -0.005 / 0.034 |
| spill (final) | 0.434 / 0.482 | 0.060 / 0.071 | 0.049 / 0.058 | 0.064 / 0.079 |
| neighbour spill max (final) | 0.208 / 0.304 | 0.000 / 0.003 | 0.000 / 0.003 | 0.017 / 0.087 |
| centre jumps | 7.500 / 11 | 2.500 / 6 | 0 / 0 | 0 / 0 |
| height jumps | 0 / 0 | 0.100 / 1 | 0 / 0 | 0 / 0 |
| ID switches after lock | 0 / 0 | 0 / 0 | 0 / 0 | 0 / 0 |
| SAFETY total violations | 0 / 0 | 0 / 0 | 0 / 0 | 0 / 0 |
| implausible-height frames | 0 / 0 | 0 / 0 | 0 / 0 | 0 / 0 |
| truth trees sprayed | 3.600 / 3 | 4 / 4 | 4 / 4 | 4 / 4 |
| scan latency mean s | 0.564 / 0.629 | 1.172 / 1.247 | 1.142 / 1.200 | 1.400 / 1.600 |
| max height drift since lock m (worst) | 0.865 | 0.173 | 0 | 0.254 |
| max centre drift since lock m (worst) | 0.857 | 0.139 | 0.159 | 0.305 |
| target height range in own descent m (worst) | 0.459 | 0.150 | 0 | 0.167 |
| matched trees with abs height error > 1 m (of which under-estimates) / matched | 0 (0) / 141 | 0 (0) / 146 | 0 (0) / 146 | 0 (0) / 146 |
| max abs height error m | 0.651 | 0.159 | 0.110 | 0.240 |
| cost ms / frame, mean (worst max) | 0.546 (24.015) | 0.937 (1.816) | 1.018 (21.088) | 0.851 (16.605) |

#### Paired: cand_obj M4 vs cand_obj M3 on easy (diff = M4 - cand_obj M3; win = M4 better)

| Metric | mean diff | t | W / L / T |
|---|---|---|---|
| recall (final) | +0.000 | +0.0 | 0 / 0 / 10 |
| precision (final) | -0.005 | -0.5 | 1 / 2 / 7 |
| recall (per-frame mean) | -0.006 | -1.7 | 3 / 7 / 0 |
| mean IoU (final) | +0.004 | +1.2 | 8 / 2 / 0 |
| min IoU (final) | -0.008 | -0.3 | 4 / 4 / 2 |
| neighbours swallowed (final) | +0.000 | +0.0 | 0 / 0 / 10 |
| split trees (final) | +0.100 | +0.6 | 1 / 2 / 7 |
| false trees (final) | +0.000 | +0.0 | 0 / 0 / 10 |
| centre RMSE m (final) | -0.001 | -0.1 | 6 / 4 / 0 |
| height RMSE m (final) | -0.012 | -2.5 | 8 / 2 / 0 |
| spill (final) | -0.011 | -10.0 | 10 / 0 / 0 |
| neighbour spill max (final) | -0.000 | -1.3 | 2 / 1 / 7 |
| centre jumps | -2.500 | -5.2 | 10 / 0 / 0 |
| height jumps | -0.100 | -1.0 | 1 / 0 / 9 |
| ID switches after lock | +0.000 | +0.0 | 0 / 0 / 10 |
| SAFETY total violations | +0.000 | +0.0 | 0 / 0 / 10 |
| implausible-height frames | +0.000 | +0.0 | 0 / 0 / 10 |
| truth trees sprayed | +0.000 | +0.0 | 0 / 0 / 10 |
| scan latency mean s | -0.030 | -1.0 | 7 / 3 / 0 |

#### Paired: cand_obj M4 vs cand_grid on easy (diff = M4 - cand_grid; win = M4 better)

| Metric | mean diff | t | W / L / T |
|---|---|---|---|
| recall (final) | +0.000 | +0.0 | 0 / 0 / 10 |
| precision (final) | -0.013 | -1.5 | 0 / 2 / 8 |
| recall (per-frame mean) | +0.006 | +3.1 | 10 / 0 / 0 |
| mean IoU (final) | +0.036 | +7.1 | 10 / 0 / 0 |
| min IoU (final) | +0.079 | +3.4 | 9 / 1 / 0 |
| neighbours swallowed (final) | +0.000 | +0.0 | 0 / 0 / 10 |
| split trees (final) | +0.200 | +1.5 | 0 / 2 / 8 |
| false trees (final) | +0.000 | +0.0 | 0 / 0 / 10 |
| centre RMSE m (final) | -0.013 | -0.9 | 5 / 5 / 0 |
| height RMSE m (final) | -0.017 | -3.0 | 10 / 0 / 0 |
| spill (final) | -0.015 | -5.4 | 10 / 0 / 0 |
| neighbour spill max (final) | -0.017 | -1.5 | 3 / 0 / 7 |
| centre jumps | +0.000 | +0.0 | 0 / 0 / 10 |
| height jumps | +0.000 | +0.0 | 0 / 0 / 10 |
| ID switches after lock | +0.000 | +0.0 | 0 / 0 / 10 |
| SAFETY total violations | +0.000 | +0.0 | 0 / 0 / 10 |
| implausible-height frames | +0.000 | +0.0 | 0 / 0 / 10 |
| truth trees sprayed | +0.000 | +0.0 | 0 / 0 / 10 |
| scan latency mean s | -0.258 | -7.1 | 10 / 0 / 0 |

### medium (n = 10): mean / worst

| Metric | simple | cand_obj M3 | cand_obj M4 | cand_grid |
|---|---|---|---|---|
| recall (final) | 0.393 / 0.235 | 0.977 / 0.938 | 0.960 / 0.824 | 0.961 / 0.882 |
| precision (final) | 0.478 / 0.250 | 0.979 / 0.895 | 0.995 / 0.950 | 0.989 / 0.938 |
| recall (per-frame mean) | 0.383 / 0.225 | 0.870 / 0.826 | 0.850 / 0.732 | 0.846 / 0.787 |
| mean IoU (final) | 0.315 / 0.199 | 0.755 / 0.715 | 0.779 / 0.731 | 0.771 / 0.758 |
| min IoU (final) | 0.151 / 0.111 | 0.430 / 0.147 | 0.523 / 0.232 | 0.529 / 0.423 |
| neighbours swallowed (final) | 14.300 / 18 | 1 / 3 | 1 / 3 | 0.700 / 2 |
| split trees (final) | 0.100 / 1 | 0.400 / 2 | 0.100 / 1 | 0.200 / 1 |
| false trees (final) | 10 / 15 | 0 / 0 | 0 / 0 | 0 / 0 |
| centre RMSE m (final) | 2.137 / 3.864 | 0.523 / 0.714 | 0.441 / 0.656 | 0.514 / 0.721 |
| height RMSE m (final) | 0.586 / 1.125 | 0.257 / 0.818 | 0.232 / 0.549 | 0.153 / 0.260 |
| height bias m (final) | 0.114 / 0.974 | 0.022 / 0.189 | 0.064 / 0.246 | 0.017 / 0.168 |
| spill (final) | 0.679 / 0.764 | 0.169 / 0.196 | 0.158 / 0.185 | 0.153 / 0.178 |
| neighbour spill max (final) | 0.362 / 0.516 | 0.306 / 0.526 | 0.288 / 0.529 | 0.234 / 0.393 |
| centre jumps | 15 / 19 | 7.400 / 10 | 0 / 0 | 0.700 / 2 |
| height jumps | 0.800 / 2 | 1.100 / 3 | 0 / 0 | 0 / 0 |
| ID switches after lock | 0.100 / 1 | 0 / 0 | 0 / 0 | 0 / 0 |
| SAFETY total violations | 6.500 / 65 | 0 / 0 | 0 / 0 | 0 / 0 |
| implausible-height frames | 6.500 / 65 | 0 / 0 | 0 / 0 | 0 / 0 |
| truth trees sprayed | 2.600 / 1 | 5.100 / 5 | 5.100 / 5 | 5.100 / 5 |
| scan latency mean s | 1.178 / 4.031 | 1.847 / 2.920 | 2.024 / 3.771 | 2.383 / 4.440 |
| max height drift since lock m (worst) | 0.483 | 1.366 | 0.820 | 0.700 |
| max centre drift since lock m (worst) | 1.998 | 0.469 | 0.260 | 0.850 |
| target height range in own descent m (worst) | 0.318 | 0.618 | 0 | 0.492 |
| matched trees with abs height error > 1 m (of which under-estimates) / matched | 9 (4) / 72 | 4 (2) / 178 | 5 (0) / 175 | 0 (0) / 175 |
| max abs height error m | 2.024 | 2.531 | 2.088 | 0.927 |
| cost ms / frame, mean (worst max) | 0.494 (15.668) | 0.967 (25.014) | 1.049 (18.791) | 0.964 (2.225) |

#### Paired: cand_obj M4 vs cand_obj M3 on medium (diff = M4 - cand_obj M3; win = M4 better)

| Metric | mean diff | t | W / L / T |
|---|---|---|---|
| recall (final) | -0.018 | -1.2 | 1 / 3 / 6 |
| precision (final) | +0.016 | +1.2 | 3 / 1 / 6 |
| recall (per-frame mean) | -0.020 | -1.5 | 2 / 8 / 0 |
| mean IoU (final) | +0.024 | +4.0 | 10 / 0 / 0 |
| min IoU (final) | +0.093 | +1.9 | 7 / 3 / 0 |
| neighbours swallowed (final) | +0.000 | +0.0 | 0 / 0 / 10 |
| split trees (final) | -0.300 | -1.2 | 3 / 1 / 6 |
| false trees (final) | +0.000 | +0.0 | 0 / 0 / 10 |
| centre RMSE m (final) | -0.082 | -3.6 | 9 / 1 / 0 |
| height RMSE m (final) | -0.025 | -0.6 | 7 / 3 / 0 |
| spill (final) | -0.011 | -3.5 | 9 / 1 / 0 |
| neighbour spill max (final) | -0.018 | -0.9 | 7 / 3 / 0 |
| centre jumps | -7.400 | -13.2 | 10 / 0 / 0 |
| height jumps | -1.100 | -4.0 | 8 / 0 / 2 |
| ID switches after lock | +0.000 | +0.0 | 0 / 0 / 10 |
| SAFETY total violations | +0.000 | +0.0 | 0 / 0 / 10 |
| implausible-height frames | +0.000 | +0.0 | 0 / 0 / 10 |
| truth trees sprayed | +0.000 | +0.0 | 0 / 0 / 10 |
| scan latency mean s | +0.177 | +1.2 | 1 / 9 / 0 |

#### Paired: cand_obj M4 vs cand_grid on medium (diff = M4 - cand_grid; win = M4 better)

| Metric | mean diff | t | W / L / T |
|---|---|---|---|
| recall (final) | -0.002 | -0.1 | 3 / 3 / 4 |
| precision (final) | +0.006 | +0.6 | 2 / 1 / 7 |
| recall (per-frame mean) | +0.004 | +0.3 | 7 / 3 / 0 |
| mean IoU (final) | +0.009 | +0.9 | 6 / 4 / 0 |
| min IoU (final) | -0.005 | -0.1 | 5 / 5 / 0 |
| neighbours swallowed (final) | +0.300 | +0.8 | 3 / 4 / 3 |
| split trees (final) | -0.100 | -0.6 | 2 / 1 / 7 |
| false trees (final) | +0.000 | +0.0 | 0 / 0 / 10 |
| centre RMSE m (final) | -0.073 | -1.8 | 8 / 2 / 0 |
| height RMSE m (final) | +0.079 | +1.6 | 5 / 5 / 0 |
| spill (final) | +0.005 | +0.8 | 4 / 6 / 0 |
| neighbour spill max (final) | +0.054 | +1.1 | 5 / 5 / 0 |
| centre jumps | -0.700 | -3.3 | 6 / 0 / 4 |
| height jumps | +0.000 | +0.0 | 0 / 0 / 10 |
| ID switches after lock | +0.000 | +0.0 | 0 / 0 / 10 |
| SAFETY total violations | +0.000 | +0.0 | 0 / 0 / 10 |
| implausible-height frames | +0.000 | +0.0 | 0 / 0 / 10 |
| truth trees sprayed | +0.000 | +0.0 | 0 / 0 / 10 |
| scan latency mean s | -0.359 | -1.4 | 7 / 3 / 0 |

### hard (n = 10): mean / worst

| Metric | simple | cand_obj M3 | cand_obj M4 | cand_grid |
|---|---|---|---|---|
| recall (final) | 0.105 / 0 | 0.866 / 0.815 | 0.831 / 0.750 | 0.758 / 0.679 |
| precision (final) | 0.117 / 0 | 0.987 / 0.957 | 1 / 1 | 0.952 / 0.850 |
| recall (per-frame mean) | 0.114 / 0.007 | 0.777 / 0.724 | 0.745 / 0.675 | 0.678 / 0.609 |
| mean IoU (final) | 0.344 / 0.173 | 0.593 / 0.550 | 0.621 / 0.597 | 0.640 / 0.597 |
| min IoU (final) | 0.268 / 0.108 | 0.275 / 0.132 | 0.312 / 0.116 | 0.375 / 0.304 |
| neighbours swallowed (final) | 29.300 / 41 | 3.600 / 7 | 2.800 / 5 | 5.300 / 8 |
| split trees (final) | 0.100 / 1 | 0.500 / 1 | 0.100 / 1 | 0.200 / 1 |
| false trees (final) | 18.700 / 26 | 0 / 0 | 0 / 0 | 0.800 / 3 |
| centre RMSE m (final) | 2.097 / 3.775 | 0.798 / 0.958 | 0.754 / 0.885 | 0.875 / 1.247 |
| height RMSE m (final) | 0.859 / 1.278 | 0.480 / 0.742 | 0.405 / 0.595 | 0.420 / 0.581 |
| height bias m (final) | 0.389 / 0.916 | 0.109 / 0.357 | 0.183 / 0.403 | 0.133 / 0.390 |
| spill (final) | 0.807 / 0.866 | 0.249 / 0.300 | 0.233 / 0.283 | 0.238 / 0.306 |
| neighbour spill max (final) | 0.286 / 0.632 | 0.443 / 0.584 | 0.441 / 0.566 | 0.461 / 0.612 |
| centre jumps | 24 / 29 | 6.300 / 11 | 0 / 0 | 3.700 / 7 |
| height jumps | 2.900 / 6 | 0.800 / 3 | 0 / 0 | 0 / 0 |
| ID switches after lock | 0 / 0 | 0 / 0 | 0 / 0 | 0 / 0 |
| SAFETY total violations | 160.500 / 331 | 0 / 0 | 0 / 0 | 0 / 0 |
| implausible-height frames | 160.500 / 331 | 0 / 0 | 0 / 0 | 0 / 0 |
| truth trees sprayed | 1.600 / 0 | 5.500 / 3 | 5.500 / 4 | 5.800 / 5 |
| scan latency mean s | 0.800 / 1.373 | 2.238 / 3.577 | 2.510 / 4.604 | 2.655 / 5.365 |
| max height drift since lock m (worst) | 1.255 | 1.906 | 1.208 | 2 |
| max centre drift since lock m (worst) | 4.769 | 1.652 | 1.129 | 0.977 |
| target height range in own descent m (worst) | 0.976 | 1.792 | 0 | 1.600 |
| matched trees with abs height error > 1 m (of which under-estimates) / matched | 9 (2) / 27 | 10 (5) / 224 | 7 (1) / 215 | 5 (3) / 196 |
| max abs height error m | 2.139 | 2.296 | 1.926 | 2.177 |
| cost ms / frame, mean (worst max) | 0.432 (1.353) | 0.959 (24.370) | 0.960 (2.415) | 1.017 (2.778) |

#### Paired: cand_obj M4 vs cand_obj M3 on hard (diff = M4 - cand_obj M3; win = M4 better)

| Metric | mean diff | t | W / L / T |
|---|---|---|---|
| recall (final) | -0.035 | -2.2 | 2 / 6 / 2 |
| precision (final) | +0.013 | +2.0 | 3 / 0 / 7 |
| recall (per-frame mean) | -0.033 | -2.3 | 3 / 7 / 0 |
| mean IoU (final) | +0.028 | +3.6 | 10 / 0 / 0 |
| min IoU (final) | +0.037 | +0.8 | 6 / 4 / 0 |
| neighbours swallowed (final) | -0.800 | -2.8 | 6 / 0 / 4 |
| split trees (final) | -0.400 | -1.8 | 5 / 1 / 4 |
| false trees (final) | +0.000 | +0.0 | 0 / 0 / 10 |
| centre RMSE m (final) | -0.044 | -1.3 | 7 / 3 / 0 |
| height RMSE m (final) | -0.076 | -1.9 | 7 / 3 / 0 |
| spill (final) | -0.017 | -4.5 | 9 / 1 / 0 |
| neighbour spill max (final) | -0.001 | -0.1 | 6 / 4 / 0 |
| centre jumps | -6.300 | -7.5 | 10 / 0 / 0 |
| height jumps | -0.800 | -2.4 | 5 / 0 / 5 |
| ID switches after lock | +0.000 | +0.0 | 0 / 0 / 10 |
| SAFETY total violations | +0.000 | +0.0 | 0 / 0 / 10 |
| implausible-height frames | +0.000 | +0.0 | 0 / 0 / 10 |
| truth trees sprayed | +0.000 | +0.0 | 2 / 3 / 5 |
| scan latency mean s | +0.272 | +1.2 | 3 / 7 / 0 |

#### Paired: cand_obj M4 vs cand_grid on hard (diff = M4 - cand_grid; win = M4 better)

| Metric | mean diff | t | W / L / T |
|---|---|---|---|
| recall (final) | +0.073 | +4.4 | 8 / 0 / 2 |
| precision (final) | +0.048 | +3.3 | 7 / 0 / 3 |
| recall (per-frame mean) | +0.067 | +4.3 | 9 / 1 / 0 |
| mean IoU (final) | -0.019 | -1.7 | 3 / 7 / 0 |
| min IoU (final) | -0.064 | -1.5 | 3 / 7 / 0 |
| neighbours swallowed (final) | -2.500 | -3.4 | 8 / 1 / 1 |
| split trees (final) | -0.100 | -0.6 | 2 / 1 / 7 |
| false trees (final) | -0.800 | -2.4 | 5 / 0 / 5 |
| centre RMSE m (final) | -0.120 | -1.9 | 7 / 3 / 0 |
| height RMSE m (final) | -0.015 | -0.4 | 7 / 3 / 0 |
| spill (final) | -0.005 | -0.2 | 4 / 6 / 0 |
| neighbour spill max (final) | -0.020 | -0.6 | 4 / 6 / 0 |
| centre jumps | -3.700 | -5.4 | 9 / 0 / 1 |
| height jumps | +0.000 | +0.0 | 0 / 0 / 10 |
| ID switches after lock | +0.000 | +0.0 | 0 / 0 / 10 |
| SAFETY total violations | +0.000 | +0.0 | 0 / 0 / 10 |
| implausible-height frames | +0.000 | +0.0 | 0 / 0 / 10 |
| truth trees sprayed | -0.300 | -0.7 | 3 / 5 / 2 |
| scan latency mean s | -0.144 | -0.4 | 4 / 6 / 0 |

