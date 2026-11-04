## Paired: after vs before (difference = after - before, same dataset = same seed)

1 pairs. For metrics with a direction, **win** = after better than before. t = mean diff / (std diff / sqrt(n)); |t| below about 2 is within seed noise.

| Metric | before mean | after mean | mean diff | std diff | t | win / loss / tie |
|---|---|---|---|---|---|---|
| recall (final) | 1 | 1 | 0 | n/a | n/a | 0 / 0 / 1 |
| precision (final) | 1 | 1 | 0 | n/a | n/a | 0 / 0 / 1 |
| published / truth (final) | 9 | 9 | 0 | n/a | n/a | - |
| recall (per-frame mean) | 0.738 | 0.738 | 0 | n/a | n/a | 0 / 0 / 1 |
| mean IoU (final) | 0.867 | 0.867 | 0 | n/a | n/a | 0 / 0 / 1 |
| min IoU (final) | 0.817 | 0.817 | 0 | n/a | n/a | 0 / 0 / 1 |
| merged entries (final) | 0 | 0 | 0 | n/a | n/a | 0 / 0 / 1 |
| neighbours swallowed (final) | 0 | 0 | 0 | n/a | n/a | 0 / 0 / 1 |
| split trees (final) | 0 | 0 | 0 | n/a | n/a | 0 / 0 / 1 |
| false trees (final) | 0 | 0 | 0 | n/a | n/a | 0 / 0 / 1 |
| centre RMSE m (final) | 0.191 | 0.255 | +0.064 | n/a | n/a | 0 / 1 / 0 |
| height bias m (final) | n/a | n/a | n/a | n/a | n/a | n/a |
| height RMSE m (final) | n/a | n/a | n/a | n/a | n/a | n/a |
| spill (final) | 0.064 | 0.064 | 0 | n/a | n/a | 0 / 0 / 1 |
| neighbour spill max (final) | 0.056 | 0.056 | 0 | n/a | n/a | 0 / 0 / 1 |
| ID switches after lock | 0 | 0 | 0 | n/a | n/a | 0 / 0 / 1 |
| geometry retreats | 0 | 0 | 0 | n/a | n/a | 0 / 0 / 1 |
| centre jumps | 0 | 0 | 0 | n/a | n/a | 0 / 0 / 1 |
| height jumps | 0 | 0 | 0 | n/a | n/a | 0 / 0 / 1 |
| IDs vanished while held by nav | 0 | 0 | 0 | n/a | n/a | 0 / 0 / 1 |
| SAFETY total violations | 0 | 1 | +1 | n/a | n/a | 0 / 1 / 0 |
|   undefined-height frames | 0 | 0 | 0 | n/a | n/a | 0 / 0 / 1 |
|   new trees after scan | 0 | 0 | 0 | n/a | n/a | 0 / 0 / 1 |
|   non-target changes in descent | 0 | 1 | +1 | n/a | n/a | 0 / 1 / 0 |
|   sprayed-tree changes | 0 | 0 | 0 | n/a | n/a | 0 / 0 / 1 |
|   new tree inside sprayed | 0 | 0 | 0 | n/a | n/a | 0 / 0 / 1 |
|   invalid-entry frames | 0 | 0 | 0 | n/a | n/a | 0 / 0 / 1 |
|   duplicate-ID frames | 0 | 0 | 0 | n/a | n/a | 0 / 0 / 1 |
|   implausible-height frames | 0 | 0 | 0 | n/a | n/a | 0 / 0 / 1 |
|   locked ID vanished | 0 | 0 | 0 | n/a | n/a | 0 / 0 / 1 |
| truth trees sprayed | 7 | 7 | 0 | n/a | n/a | 0 / 0 / 1 |
| truth trees sprayed twice | 0 | 0 | 0 | n/a | n/a | 0 / 0 / 1 |
| scan latency mean s | 2.319 | 2.319 | 0 | n/a | n/a | 0 / 0 / 1 |

Per seed (after - before):

| dataset | recall (final) | precision (final) | mean IoU (final) | neighbours swallowed (final) | false trees (final) | centre RMSE m (final) | height RMSE m (final) | spill (final) | SAFETY total violations |
|---|---|---|---|---|---|---|---|---|---|
| isaac_2026-10-23-15-34-29 | 0 | 0 | 0 | 0 | 0 | +0.064 | n/a | 0 | +1 |


Run-to-run max diff (TM_REPEAT) before / after, per metric:

