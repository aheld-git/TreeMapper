# Shipped cand_obj (M4) vs the simple baseline on the held-out suite, paired by seed

Extracted by M5 (no new runs) from the M4a final evaluation that produced `results/cand_obj_m4_holdout.md`:
`/data/work/eval/m4a/final/holdout_{easy,medium,hard}.md` (run by `/data/work/eval/m4a/final_runs.sh final
<M4 build of commit f8a78c0> simple,cand_obj,cand_grid simple`, i.e. `treemapper_tools.evaluate --reference simple`
once per level on `/data/work/holdout/suite_holdout`, 30 missions, seeds 100-109, never used for tuning).
`cand_obj` below is the shipped M4 build with `config/cand_obj.cfg`. `results/cand_obj_m4_holdout.md` already holds the paired comparisons
against the frozen M3 cand_obj and against cand_grid; this one adds the shipped system's own per-level card with the
**seed-to-seed spread** (std over the 10 datasets) and its **paired comparison against `simple`**.
sha256 (first 16 hex) of the source JSONs: easy 9b8f539f84d277a8, hard ececd16c104eead6, medium b3fd534c0e6dfcb7.

Reading: "worst" = worst seed (min for recall / precision / IoU / sprayed, max otherwise). Paired: diff = cand_obj -
simple per seed; t = mean diff / (std diff / sqrt n), |t| < 2 is within seed noise; win / loss / tie counts seeds where
cand_obj is better / worse / equal. A metric that is undefined for one system on a seed (for example IoU when `simple`
matches no tree, hard level) drops that pair, so a paired mean can differ slightly from the card's mean (hard mean IoU
0.618 over 8 pairs vs 0.621 over 10 datasets). Run-to-run noise is 0 (deterministic), so the std columns are the floor.

### easy
#### cand_obj M4, held-out easy: mean / worst / std over the 10 datasets (std = seed-to-seed spread)

| Metric | mean | worst | std over datasets |
|---|---|---|---|
| recall (final) | 1 | 1 | 0 |
| precision (final) | 0.988 | 0.938 | 0.025 |
| published / truth (final) | 14.800 | n/a | 0.872 |
| recall (per-frame mean) | 0.893 | 0.883 | 0.007 |
| mean IoU (final) | 0.908 | 0.892 | 0.008 |
| min IoU (final) | 0.840 | 0.672 | 0.062 |
| merged entries (final) | 0 | 0 | 0 |
| neighbours swallowed (final) | 0 | 0 | 0 |
| split trees (final) | 0.200 | 1 | 0.400 |
| false trees (final) | 0 | 0 | 0 |
| centre RMSE m (final) | 0.140 | 0.214 | 0.033 |
| height bias m (final) | 0.001 | n/a | 0.027 |
| height RMSE m (final) | 0.038 | 0.055 | 0.011 |
| spill (final) | 0.049 | 0.058 | 0.005 |
| neighbour spill max (final) | 0.000 | 0.003 | 0.001 |
| ID switches after lock | 0 | 0 | 0 |
| geometry retreats | 0 | 0 | 0 |
| centre jumps | 0 | 0 | 0 |
| height jumps | 0 | 0 | 0 |
| IDs vanished while held by nav | 0 | 0 | 0 |
| SAFETY total violations | 0 | 0 | 0 |
|   undefined-height frames | 0 | 0 | 0 |
|   new trees after scan | 0 | 0 | 0 |
|   non-target changes in descent | 0 | 0 | 0 |
|   sprayed-tree changes | 0 | 0 | 0 |
|   new tree inside sprayed | 0 | 0 | 0 |
|   invalid-entry frames | 0 | 0 | 0 |
|   duplicate-ID frames | 0 | 0 | 0 |
|   implausible-height frames | 0 | 0 | 0 |
|   locked ID vanished | 0 | 0 | 0 |
| truth trees sprayed | 4 | 4 | 0 |
| truth trees sprayed twice | 0 | 0 | 0 |
| scan latency mean s | 1.142 | 1.200 | 0.042 |

Per dataset:

| dataset | recall (final) | precision (final) | published / truth (final) | recall (per-frame mean) | mean IoU (final) | min IoU (final) | merged entries (final) | neighbours swallowed (final) | split trees (final) | false trees (final) | centre RMSE m (final) | height bias m (final) | safety |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| easy_s100 | 1 | 1 | 15 | 0.883 | 0.908 | 0.854 | 0 | 0 | 0 | 0 | 0.131 | 0.038 | 0 |
| easy_s101 | 1 | 1 | 15 | 0.888 | 0.912 | 0.893 | 0 | 0 | 0 | 0 | 0.119 | 0.025 | 0 |
| easy_s102 | 1 | 1 | 15 | 0.892 | 0.913 | 0.879 | 0 | 0 | 0 | 0 | 0.118 | -0.033 | 0 |
| easy_s103 | 1 | 1 | 13 | 0.889 | 0.920 | 0.861 | 0 | 0 | 0 | 0 | 0.100 | -0.001 | 0 |
| easy_s104 | 1 | 1 | 14 | 0.901 | 0.910 | 0.884 | 0 | 0 | 0 | 0 | 0.111 | 0.004 | 0 |
| easy_s105 | 1 | 1 | 15 | 0.885 | 0.914 | 0.847 | 0 | 0 | 0 | 0 | 0.124 | -0.017 | 0 |
| easy_s106 | 1 | 0.938 | 16 | 0.897 | 0.893 | 0.792 | 0 | 0 | 1 | 0 | 0.165 | -0.047 | 0 |
| easy_s107 | 1 | 1 | 14 | 0.895 | 0.910 | 0.843 | 0 | 0 | 0 | 0 | 0.166 | 0.039 | 0 |
| easy_s108 | 1 | 1 | 15 | 0.891 | 0.904 | 0.876 | 0 | 0 | 0 | 0 | 0.154 | -0.001 | 0 |
| easy_s109 | 1 | 0.938 | 16 | 0.908 | 0.892 | 0.672 | 0 | 0 | 1 | 0 | 0.214 | 0.004 | 0 |

#### Paired: cand_obj M4 vs simple, held-out easy (diff = cand_obj - simple, same seed)

10 pairs. For metrics with a direction, **win** = cand_obj better than simple. t = mean diff / (std diff / sqrt(n)); |t| below about 2 is within seed noise.

| Metric | simple mean | cand_obj mean | mean diff | std diff | t | win / loss / tie |
|---|---|---|---|---|---|---|
| recall (final) | 0.966 | 1 | +0.034 | 0.048 | +2.2 | 4 / 0 / 6 |
| precision (final) | 0.861 | 0.988 | +0.126 | 0.032 | +12.5 | 10 / 0 / 0 |
| published / truth (final) | 16.400 | 14.800 | -1.600 | 0.843 | -6.0 | - |
| recall (per-frame mean) | 0.902 | 0.893 | -0.009 | 0.052 | -0.6 | 4 / 6 / 0 |
| mean IoU (final) | 0.583 | 0.908 | +0.324 | 0.023 | +44.4 | 10 / 0 / 0 |
| min IoU (final) | 0.274 | 0.840 | +0.566 | 0.104 | +17.2 | 10 / 0 / 0 |
| merged entries (final) | 1.500 | 0 | -1.500 | 0.527 | -9.0 | 10 / 0 / 0 |
| neighbours swallowed (final) | 1.800 | 0 | -1.800 | 0.919 | -6.2 | 10 / 0 / 0 |
| split trees (final) | 1.500 | 0.200 | -1.300 | 0.483 | -8.5 | 10 / 0 / 0 |
| false trees (final) | 0.800 | 0 | -0.800 | 0.919 | -2.8 | 5 / 0 / 5 |
| centre RMSE m (final) | 1.048 | 0.140 | -0.907 | 0.141 | -20.4 | 10 / 0 / 0 |
| height bias m (final) | -0.064 | 0.001 | +0.065 | 0.026 | +8.0 | - |
| height RMSE m (final) | 0.169 | 0.038 | -0.131 | 0.030 | -13.9 | 10 / 0 / 0 |
| spill (final) | 0.434 | 0.049 | -0.385 | 0.031 | -39.9 | 10 / 0 / 0 |
| neighbour spill max (final) | 0.208 | 0.000 | -0.208 | 0.068 | -9.6 | 10 / 0 / 0 |
| ID switches after lock | 0 | 0 | 0 | 0 | +0.0 | 0 / 0 / 10 |
| geometry retreats | 0 | 0 | 0 | 0 | +0.0 | 0 / 0 / 10 |
| centre jumps | 7.500 | 0 | -7.500 | 2.369 | -10.0 | 10 / 0 / 0 |
| height jumps | 0 | 0 | 0 | 0 | +0.0 | 0 / 0 / 10 |
| IDs vanished while held by nav | 0 | 0 | 0 | 0 | +0.0 | 0 / 0 / 10 |
| SAFETY total violations | 0 | 0 | 0 | 0 | +0.0 | 0 / 0 / 10 |
|   undefined-height frames | 0 | 0 | 0 | 0 | +0.0 | 0 / 0 / 10 |
|   new trees after scan | 0 | 0 | 0 | 0 | +0.0 | 0 / 0 / 10 |
|   non-target changes in descent | 0 | 0 | 0 | 0 | +0.0 | 0 / 0 / 10 |
|   sprayed-tree changes | 0 | 0 | 0 | 0 | +0.0 | 0 / 0 / 10 |
|   new tree inside sprayed | 0 | 0 | 0 | 0 | +0.0 | 0 / 0 / 10 |
|   invalid-entry frames | 0 | 0 | 0 | 0 | +0.0 | 0 / 0 / 10 |
|   duplicate-ID frames | 0 | 0 | 0 | 0 | +0.0 | 0 / 0 / 10 |
|   implausible-height frames | 0 | 0 | 0 | 0 | +0.0 | 0 / 0 / 10 |
|   locked ID vanished | 0 | 0 | 0 | 0 | +0.0 | 0 / 0 / 10 |
| truth trees sprayed | 3.600 | 4 | +0.400 | 0.516 | +2.4 | 4 / 0 / 6 |
| truth trees sprayed twice | 0 | 0 | 0 | 0 | +0.0 | 0 / 0 / 10 |
| scan latency mean s | 0.564 | 1.142 | +0.578 | 0.058 | +31.6 | 0 / 10 / 0 |


### medium
#### cand_obj M4, held-out medium: mean / worst / std over the 10 datasets (std = seed-to-seed spread)

| Metric | mean | worst | std over datasets |
|---|---|---|---|
| recall (final) | 0.960 | 0.824 | 0.053 |
| precision (final) | 0.995 | 0.950 | 0.015 |
| published / truth (final) | 17.600 | n/a | 1.960 |
| recall (per-frame mean) | 0.850 | 0.732 | 0.045 |
| mean IoU (final) | 0.779 | 0.731 | 0.026 |
| min IoU (final) | 0.523 | 0.232 | 0.127 |
| merged entries (final) | 1 | 3 | 1 |
| neighbours swallowed (final) | 1 | 3 | 1 |
| split trees (final) | 0.100 | 1 | 0.300 |
| false trees (final) | 0 | 0 | 0 |
| centre RMSE m (final) | 0.441 | 0.656 | 0.115 |
| height bias m (final) | 0.064 | n/a | 0.098 |
| height RMSE m (final) | 0.232 | 0.549 | 0.170 |
| spill (final) | 0.158 | 0.185 | 0.014 |
| neighbour spill max (final) | 0.288 | 0.529 | 0.133 |
| ID switches after lock | 0 | 0 | 0 |
| geometry retreats | 0 | 0 | 0 |
| centre jumps | 0 | 0 | 0 |
| height jumps | 0 | 0 | 0 |
| IDs vanished while held by nav | 0 | 0 | 0 |
| SAFETY total violations | 0 | 0 | 0 |
|   undefined-height frames | 0 | 0 | 0 |
|   new trees after scan | 0 | 0 | 0 |
|   non-target changes in descent | 0 | 0 | 0 |
|   sprayed-tree changes | 0 | 0 | 0 |
|   new tree inside sprayed | 0 | 0 | 0 |
|   invalid-entry frames | 0 | 0 | 0 |
|   duplicate-ID frames | 0 | 0 | 0 |
|   implausible-height frames | 0 | 0 | 0 |
|   locked ID vanished | 0 | 0 | 0 |
| truth trees sprayed | 5.100 | 5 | 0.300 |
| truth trees sprayed twice | 0 | 0 | 0 |
| scan latency mean s | 2.024 | 3.771 | 0.724 |

Per dataset:

| dataset | recall (final) | precision (final) | published / truth (final) | recall (per-frame mean) | mean IoU (final) | min IoU (final) | merged entries (final) | neighbours swallowed (final) | split trees (final) | false trees (final) | centre RMSE m (final) | height bias m (final) | safety |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| medium_s100 | 0.824 | 1 | 14 | 0.732 | 0.781 | 0.559 | 1 | 1 | 0 | 0 | 0.344 | 0.051 | 0 |
| medium_s101 | 0.938 | 1 | 15 | 0.824 | 0.775 | 0.482 | 3 | 3 | 0 | 0 | 0.493 | 0.139 | 0 |
| medium_s102 | 1 | 1 | 20 | 0.871 | 0.822 | 0.570 | 0 | 0 | 0 | 0 | 0.343 | 0.157 | 0 |
| medium_s103 | 1 | 1 | 18 | 0.877 | 0.816 | 0.732 | 0 | 0 | 0 | 0 | 0.248 | -0.092 | 0 |
| medium_s104 | 1 | 1 | 19 | 0.877 | 0.796 | 0.582 | 0 | 0 | 0 | 0 | 0.477 | -0.039 | 0 |
| medium_s105 | 0.950 | 0.950 | 20 | 0.843 | 0.731 | 0.232 | 2 | 2 | 1 | 0 | 0.656 | 0.099 | 0 |
| medium_s106 | 1 | 1 | 18 | 0.892 | 0.751 | 0.448 | 1 | 1 | 0 | 0 | 0.442 | -0.025 | 0 |
| medium_s107 | 0.941 | 1 | 16 | 0.853 | 0.769 | 0.533 | 1 | 1 | 0 | 0 | 0.488 | 0.246 | 0 |
| medium_s108 | 1 | 1 | 19 | 0.894 | 0.787 | 0.643 | 0 | 0 | 0 | 0 | 0.356 | 0.007 | 0 |
| medium_s109 | 0.944 | 1 | 17 | 0.835 | 0.766 | 0.451 | 2 | 2 | 0 | 0 | 0.565 | 0.096 | 0 |

#### Paired: cand_obj M4 vs simple, held-out medium (diff = cand_obj - simple, same seed)

10 pairs. For metrics with a direction, **win** = cand_obj better than simple. t = mean diff / (std diff / sqrt(n)); |t| below about 2 is within seed noise.

| Metric | simple mean | cand_obj mean | mean diff | std diff | t | win / loss / tie |
|---|---|---|---|---|---|---|
| recall (final) | 0.393 | 0.960 | +0.567 | 0.098 | +18.3 | 10 / 0 / 0 |
| precision (final) | 0.478 | 0.995 | +0.517 | 0.132 | +12.4 | 10 / 0 / 0 |
| published / truth (final) | 15.300 | 17.600 | +2.300 | 2.751 | +2.6 | - |
| recall (per-frame mean) | 0.383 | 0.850 | +0.467 | 0.098 | +15.1 | 10 / 0 / 0 |
| mean IoU (final) | 0.315 | 0.779 | +0.465 | 0.083 | +17.7 | 10 / 0 / 0 |
| min IoU (final) | 0.151 | 0.523 | +0.372 | 0.163 | +7.2 | 10 / 0 / 0 |
| merged entries (final) | 5.100 | 1 | -4.100 | 2.025 | -6.4 | 10 / 0 / 0 |
| neighbours swallowed (final) | 14.300 | 1 | -13.300 | 3.129 | -13.4 | 10 / 0 / 0 |
| split trees (final) | 0.100 | 0.100 | 0 | 0.471 | +0.0 | 1 / 1 / 8 |
| false trees (final) | 10 | 0 | -10 | 2.667 | -11.9 | 10 / 0 / 0 |
| centre RMSE m (final) | 2.137 | 0.441 | -1.696 | 0.914 | -5.9 | 10 / 0 / 0 |
| height bias m (final) | 0.114 | 0.064 | -0.050 | 0.277 | -0.6 | - |
| height RMSE m (final) | 0.586 | 0.232 | -0.354 | 0.296 | -3.8 | 8 / 2 / 0 |
| spill (final) | 0.679 | 0.158 | -0.521 | 0.041 | -40.4 | 10 / 0 / 0 |
| neighbour spill max (final) | 0.362 | 0.288 | -0.073 | 0.162 | -1.4 | 7 / 3 / 0 |
| ID switches after lock | 0.100 | 0 | -0.100 | 0.316 | -1.0 | 1 / 0 / 9 |
| geometry retreats | 0 | 0 | 0 | 0 | +0.0 | 0 / 0 / 10 |
| centre jumps | 15 | 0 | -15 | 2.981 | -15.9 | 10 / 0 / 0 |
| height jumps | 0.800 | 0 | -0.800 | 0.789 | -3.2 | 6 / 0 / 4 |
| IDs vanished while held by nav | 0 | 0 | 0 | 0 | +0.0 | 0 / 0 / 10 |
| SAFETY total violations | 6.500 | 0 | -6.500 | 20.555 | -1.0 | 1 / 0 / 9 |
|   undefined-height frames | 0 | 0 | 0 | 0 | +0.0 | 0 / 0 / 10 |
|   new trees after scan | 0 | 0 | 0 | 0 | +0.0 | 0 / 0 / 10 |
|   non-target changes in descent | 0 | 0 | 0 | 0 | +0.0 | 0 / 0 / 10 |
|   sprayed-tree changes | 0 | 0 | 0 | 0 | +0.0 | 0 / 0 / 10 |
|   new tree inside sprayed | 0 | 0 | 0 | 0 | +0.0 | 0 / 0 / 10 |
|   invalid-entry frames | 0 | 0 | 0 | 0 | +0.0 | 0 / 0 / 10 |
|   duplicate-ID frames | 0 | 0 | 0 | 0 | +0.0 | 0 / 0 / 10 |
|   implausible-height frames | 6.500 | 0 | -6.500 | 20.555 | -1.0 | 1 / 0 / 9 |
|   locked ID vanished | 0 | 0 | 0 | 0 | +0.0 | 0 / 0 / 10 |
| truth trees sprayed | 2.600 | 5.100 | +2.500 | 1.354 | +5.8 | 9 / 0 / 1 |
| truth trees sprayed twice | 0 | 0 | 0 | 0 | +0.0 | 0 / 0 / 10 |
| scan latency mean s | 1.178 | 2.024 | +0.846 | 0.562 | +4.8 | 1 / 9 / 0 |


### hard
#### cand_obj M4, held-out hard: mean / worst / std over the 10 datasets (std = seed-to-seed spread)

| Metric | mean | worst | std over datasets |
|---|---|---|---|
| recall (final) | 0.831 | 0.750 | 0.044 |
| precision (final) | 1 | 1 | 0 |
| published / truth (final) | 21.500 | n/a | 1.204 |
| recall (per-frame mean) | 0.745 | 0.675 | 0.037 |
| mean IoU (final) | 0.621 | 0.597 | 0.021 |
| min IoU (final) | 0.312 | 0.116 | 0.105 |
| merged entries (final) | 2.600 | 5 | 1.114 |
| neighbours swallowed (final) | 2.800 | 5 | 1.166 |
| split trees (final) | 0.100 | 1 | 0.300 |
| false trees (final) | 0 | 0 | 0 |
| centre RMSE m (final) | 0.754 | 0.885 | 0.087 |
| height bias m (final) | 0.183 | n/a | 0.181 |
| height RMSE m (final) | 0.405 | 0.595 | 0.090 |
| spill (final) | 0.233 | 0.283 | 0.033 |
| neighbour spill max (final) | 0.441 | 0.566 | 0.057 |
| ID switches after lock | 0 | 0 | 0 |
| geometry retreats | 0 | 0 | 0 |
| centre jumps | 0 | 0 | 0 |
| height jumps | 0 | 0 | 0 |
| IDs vanished while held by nav | 0 | 0 | 0 |
| SAFETY total violations | 0 | 0 | 0 |
|   undefined-height frames | 0 | 0 | 0 |
|   new trees after scan | 0 | 0 | 0 |
|   non-target changes in descent | 0 | 0 | 0 |
|   sprayed-tree changes | 0 | 0 | 0 |
|   new tree inside sprayed | 0 | 0 | 0 |
|   invalid-entry frames | 0 | 0 | 0 |
|   duplicate-ID frames | 0 | 0 | 0 |
|   implausible-height frames | 0 | 0 | 0 |
|   locked ID vanished | 0 | 0 | 0 |
| truth trees sprayed | 5.500 | 4 | 0.806 |
| truth trees sprayed twice | 0 | 0 | 0 |
| scan latency mean s | 2.510 | 4.604 | 0.980 |

Per dataset:

| dataset | recall (final) | precision (final) | published / truth (final) | recall (per-frame mean) | mean IoU (final) | min IoU (final) | merged entries (final) | neighbours swallowed (final) | split trees (final) | false trees (final) | centre RMSE m (final) | height bias m (final) | safety |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| hard_s100 | 0.880 | 1 | 22 | 0.782 | 0.606 | 0.316 | 2 | 3 | 0 | 0 | 0.825 | 0.315 | 0 |
| hard_s101 | 0.880 | 1 | 22 | 0.796 | 0.636 | 0.356 | 1 | 1 | 1 | 0 | 0.736 | 0.361 | 0 |
| hard_s102 | 0.852 | 1 | 23 | 0.762 | 0.599 | 0.302 | 3 | 3 | 0 | 0 | 0.739 | 0.148 | 0 |
| hard_s103 | 0.833 | 1 | 20 | 0.749 | 0.663 | 0.432 | 3 | 3 | 0 | 0 | 0.653 | 0.141 | 0 |
| hard_s104 | 0.792 | 1 | 19 | 0.709 | 0.598 | 0.272 | 2 | 2 | 0 | 0 | 0.803 | 0.168 | 0 |
| hard_s105 | 0.852 | 1 | 23 | 0.766 | 0.610 | 0.116 | 3 | 4 | 0 | 0 | 0.870 | 0.082 | 0 |
| hard_s106 | 0.880 | 1 | 22 | 0.775 | 0.597 | 0.263 | 5 | 5 | 0 | 0 | 0.885 | 0.329 | 0 |
| hard_s107 | 0.778 | 1 | 21 | 0.702 | 0.635 | 0.181 | 3 | 3 | 0 | 0 | 0.750 | -0.258 | 0 |
| hard_s108 | 0.750 | 1 | 21 | 0.675 | 0.637 | 0.420 | 3 | 3 | 0 | 0 | 0.625 | 0.403 | 0 |
| hard_s109 | 0.815 | 1 | 22 | 0.730 | 0.634 | 0.459 | 1 | 1 | 0 | 0 | 0.656 | 0.144 | 0 |

#### Paired: cand_obj M4 vs simple, held-out hard (diff = cand_obj - simple, same seed)

10 pairs. For metrics with a direction, **win** = cand_obj better than simple. t = mean diff / (std diff / sqrt(n)); |t| below about 2 is within seed noise.

| Metric | simple mean | cand_obj mean | mean diff | std diff | t | win / loss / tie |
|---|---|---|---|---|---|---|
| recall (final) | 0.105 | 0.831 | +0.726 | 0.088 | +25.9 | 10 / 0 / 0 |
| precision (final) | 0.117 | 1 | +0.883 | 0.079 | +35.5 | 10 / 0 / 0 |
| published / truth (final) | 20.500 | 21.500 | +1 | 5.099 | +0.6 | - |
| recall (per-frame mean) | 0.114 | 0.745 | +0.631 | 0.082 | +24.2 | 10 / 0 / 0 |
| mean IoU (final) | 0.344 | 0.618 | +0.274 | 0.178 | +4.4 | 7 / 1 / 0 |
| min IoU (final) | 0.268 | 0.293 | +0.024 | 0.175 | +0.4 | 4 / 4 / 0 |
| merged entries (final) | 3.700 | 2.600 | -1.100 | 1.663 | -2.1 | 6 / 1 / 3 |
| neighbours swallowed (final) | 29.300 | 2.800 | -26.500 | 5.949 | -14.1 | 10 / 0 / 0 |
| split trees (final) | 0.100 | 0.100 | 0 | 0.471 | +0.0 | 1 / 1 / 8 |
| false trees (final) | 18.700 | 0 | -18.700 | 4.270 | -13.8 | 10 / 0 / 0 |
| centre RMSE m (final) | 2.097 | 0.773 | -1.325 | 1.114 | -3.4 | 7 / 1 / 0 |
| height bias m (final) | 0.389 | 0.133 | -0.256 | 0.570 | -1.3 | - |
| height RMSE m (final) | 0.859 | 0.390 | -0.469 | 0.318 | -4.2 | 7 / 1 / 0 |
| spill (final) | 0.807 | 0.233 | -0.575 | 0.023 | -77.5 | 10 / 0 / 0 |
| neighbour spill max (final) | 0.286 | 0.443 | +0.157 | 0.196 | +2.3 | 1 / 7 / 0 |
| ID switches after lock | 0 | 0 | 0 | 0 | +0.0 | 0 / 0 / 10 |
| geometry retreats | 0 | 0 | 0 | 0 | +0.0 | 0 / 0 / 10 |
| centre jumps | 24 | 0 | -24 | 3.944 | -19.2 | 10 / 0 / 0 |
| height jumps | 2.900 | 0 | -2.900 | 1.912 | -4.8 | 9 / 0 / 1 |
| IDs vanished while held by nav | 0 | 0 | 0 | 0 | +0.0 | 0 / 0 / 10 |
| SAFETY total violations | 160.500 | 0 | -160.500 | 83.810 | -6.1 | 10 / 0 / 0 |
|   undefined-height frames | 0 | 0 | 0 | 0 | +0.0 | 0 / 0 / 10 |
|   new trees after scan | 0 | 0 | 0 | 0 | +0.0 | 0 / 0 / 10 |
|   non-target changes in descent | 0 | 0 | 0 | 0 | +0.0 | 0 / 0 / 10 |
|   sprayed-tree changes | 0 | 0 | 0 | 0 | +0.0 | 0 / 0 / 10 |
|   new tree inside sprayed | 0 | 0 | 0 | 0 | +0.0 | 0 / 0 / 10 |
|   invalid-entry frames | 0 | 0 | 0 | 0 | +0.0 | 0 / 0 / 10 |
|   duplicate-ID frames | 0 | 0 | 0 | 0 | +0.0 | 0 / 0 / 10 |
|   implausible-height frames | 160.500 | 0 | -160.500 | 83.810 | -6.1 | 10 / 0 / 0 |
|   locked ID vanished | 0 | 0 | 0 | 0 | +0.0 | 0 / 0 / 10 |
| truth trees sprayed | 1.600 | 5.500 | +3.900 | 1.370 | +9.0 | 10 / 0 / 0 |
| truth trees sprayed twice | 0 | 0 | 0 | 0 | +0.0 | 0 / 0 / 10 |
| scan latency mean s | 0.800 | 2.510 | +1.710 | 0.939 | +5.8 | 0 / 10 / 0 |

