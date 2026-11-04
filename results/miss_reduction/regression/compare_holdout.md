## Paired: after vs before (difference = after - before, same dataset = same seed)

30 pairs. For metrics with a direction, **win** = after better than before. t = mean diff / (std diff / sqrt(n)); |t| below about 2 is within seed noise.

| Metric | before mean | after mean | mean diff | std diff | t | win / loss / tie |
|---|---|---|---|---|---|---|
| recall (final) | 0.930 | 0.930 | 0 | 0 | +0.0 | 0 / 0 / 30 |
| precision (final) | 0.998 | 0.998 | 0 | 0 | +0.0 | 0 / 0 / 30 |
| published / truth (final) | 17.900 | 17.900 | 0 | 0 | +0.0 | - |
| recall (per-frame mean) | 0.828 | 0.828 | 0 | 0 | +0.0 | 0 / 0 / 30 |
| mean IoU (final) | 0.770 | 0.770 | 0 | 0 | +0.0 | 0 / 0 / 30 |
| min IoU (final) | 0.565 | 0.565 | 0 | 0 | +0.0 | 0 / 0 / 30 |
| merged entries (final) | 1.200 | 1.200 | 0 | 0 | +0.0 | 0 / 0 / 30 |
| neighbours swallowed (final) | 1.267 | 1.267 | 0 | 0 | +0.0 | 0 / 0 / 30 |
| split trees (final) | 0.067 | 0.067 | 0 | 0 | +0.0 | 0 / 0 / 30 |
| false trees (final) | 0 | 0 | 0 | 0 | +0.0 | 0 / 0 / 30 |
| centre RMSE m (final) | 0.443 | 0.425 | -0.018 | 0.056 | -1.8 | 19 / 11 / 0 |
| height bias m (final) | 0.083 | 0.083 | 0 | 0 | +0.0 | - |
| height RMSE m (final) | 0.225 | 0.225 | 0 | 0 | +0.0 | 0 / 0 / 30 |
| spill (final) | 0.146 | 0.146 | 0 | 0 | +0.0 | 0 / 0 / 30 |
| neighbour spill max (final) | 0.243 | 0.243 | 0 | 0 | +0.0 | 0 / 0 / 30 |
| ID switches after lock | 0 | 0 | 0 | 0 | +0.0 | 0 / 0 / 30 |
| geometry retreats | 0 | 0 | 0 | 0 | +0.0 | 0 / 0 / 30 |
| centre jumps | 0 | 0 | 0 | 0 | +0.0 | 0 / 0 / 30 |
| height jumps | 0 | 0 | 0 | 0 | +0.0 | 0 / 0 / 30 |
| IDs vanished while held by nav | 0 | 0 | 0 | 0 | +0.0 | 0 / 0 / 30 |
| SAFETY total violations | 0 | 0 | 0 | 0 | +0.0 | 0 / 0 / 30 |
|   undefined-height frames | 0 | 0 | 0 | 0 | +0.0 | 0 / 0 / 30 |
|   new trees after scan | 0 | 0 | 0 | 0 | +0.0 | 0 / 0 / 30 |
|   non-target changes in descent | 0 | 0 | 0 | 0 | +0.0 | 0 / 0 / 30 |
|   sprayed-tree changes | 0 | 0 | 0 | 0 | +0.0 | 0 / 0 / 30 |
|   new tree inside sprayed | 0 | 0 | 0 | 0 | +0.0 | 0 / 0 / 30 |
|   invalid-entry frames | 0 | 0 | 0 | 0 | +0.0 | 0 / 0 / 30 |
|   duplicate-ID frames | 0 | 0 | 0 | 0 | +0.0 | 0 / 0 / 30 |
|   implausible-height frames | 0 | 0 | 0 | 0 | +0.0 | 0 / 0 / 30 |
|   locked ID vanished | 0 | 0 | 0 | 0 | +0.0 | 0 / 0 / 30 |
| truth trees sprayed | 4.867 | 4.933 | +0.067 | 0.254 | +1.4 | 2 / 0 / 28 |
| truth trees sprayed twice | 0 | 0 | 0 | 0 | +0.0 | 0 / 0 / 30 |
| scan latency mean s | 1.934 | 1.934 | 0 | 0 | +0.0 | 0 / 0 / 30 |

Mean paired difference per group:

| group (n) | recall (final) | precision (final) | mean IoU (final) | neighbours swallowed (final) | false trees (final) | centre RMSE m (final) | height RMSE m (final) | spill (final) | SAFETY total violations |
|---|---|---|---|---|---|---|---|---|---|
| easy (10) | 0 | 0 | 0 | 0 | 0 | +0.015 | 0 | 0 | 0 |
| hard (10) | 0 | 0 | 0 | 0 | 0 | -0.023 | 0 | 0 | 0 |
| medium (10) | 0 | 0 | 0 | 0 | 0 | -0.046 | 0 | 0 | 0 |

Per seed (after - before):

| dataset | recall (final) | precision (final) | mean IoU (final) | neighbours swallowed (final) | false trees (final) | centre RMSE m (final) | height RMSE m (final) | spill (final) | SAFETY total violations |
|---|---|---|---|---|---|---|---|---|---|
| easy_s100 | 0 | 0 | 0 | 0 | 0 | +0.029 | 0 | 0 | 0 |
| easy_s101 | 0 | 0 | 0 | 0 | 0 | -0.002 | 0 | 0 | 0 |
| easy_s102 | 0 | 0 | 0 | 0 | 0 | +0.040 | 0 | 0 | 0 |
| easy_s103 | 0 | 0 | 0 | 0 | 0 | +0.021 | 0 | 0 | 0 |
| easy_s104 | 0 | 0 | 0 | 0 | 0 | +0.031 | 0 | 0 | 0 |
| easy_s105 | 0 | 0 | 0 | 0 | 0 | -0.004 | 0 | 0 | 0 |
| easy_s106 | 0 | 0 | 0 | 0 | 0 | -0.021 | 0 | 0 | 0 |
| easy_s107 | 0 | 0 | 0 | 0 | 0 | -0.006 | 0 | 0 | 0 |
| easy_s108 | 0 | 0 | 0 | 0 | 0 | +0.026 | 0 | 0 | 0 |
| easy_s109 | 0 | 0 | 0 | 0 | 0 | +0.032 | 0 | 0 | 0 |
| hard_s100 | 0 | 0 | 0 | 0 | 0 | -0.048 | 0 | 0 | 0 |
| hard_s101 | 0 | 0 | 0 | 0 | 0 | +0.095 | 0 | 0 | 0 |
| hard_s102 | 0 | 0 | 0 | 0 | 0 | -0.039 | 0 | 0 | 0 |
| hard_s103 | 0 | 0 | 0 | 0 | 0 | -0.032 | 0 | 0 | 0 |
| hard_s104 | 0 | 0 | 0 | 0 | 0 | -0.070 | 0 | 0 | 0 |
| hard_s105 | 0 | 0 | 0 | 0 | 0 | -0.022 | 0 | 0 | 0 |
| hard_s106 | 0 | 0 | 0 | 0 | 0 | -0.118 | 0 | 0 | 0 |
| hard_s107 | 0 | 0 | 0 | 0 | 0 | +0.055 | 0 | 0 | 0 |
| hard_s108 | 0 | 0 | 0 | 0 | 0 | +0.001 | 0 | 0 | 0 |
| hard_s109 | 0 | 0 | 0 | 0 | 0 | -0.049 | 0 | 0 | 0 |
| medium_s100 | 0 | 0 | 0 | 0 | 0 | +0.047 | 0 | 0 | 0 |
| medium_s101 | 0 | 0 | 0 | 0 | 0 | -0.016 | 0 | 0 | 0 |
| medium_s102 | 0 | 0 | 0 | 0 | 0 | -0.097 | 0 | 0 | 0 |
| medium_s103 | 0 | 0 | 0 | 0 | 0 | -0.031 | 0 | 0 | 0 |
| medium_s104 | 0 | 0 | 0 | 0 | 0 | -0.161 | 0 | 0 | 0 |
| medium_s105 | 0 | 0 | 0 | 0 | 0 | +0.026 | 0 | 0 | 0 |
| medium_s106 | 0 | 0 | 0 | 0 | 0 | -0.018 | 0 | 0 | 0 |
| medium_s107 | 0 | 0 | 0 | 0 | 0 | -0.043 | 0 | 0 | 0 |
| medium_s108 | 0 | 0 | 0 | 0 | 0 | -0.096 | 0 | 0 | 0 |
| medium_s109 | 0 | 0 | 0 | 0 | 0 | -0.070 | 0 | 0 | 0 |


Run-to-run max diff (TM_REPEAT) before / after, per metric:

