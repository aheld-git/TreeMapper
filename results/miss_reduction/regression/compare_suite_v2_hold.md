## Paired: after vs before (difference = after - before, same dataset = same seed)

15 pairs. For metrics with a direction, **win** = after better than before. t = mean diff / (std diff / sqrt(n)); |t| below about 2 is within seed noise.

| Metric | before mean | after mean | mean diff | std diff | t | win / loss / tie |
|---|---|---|---|---|---|---|
| recall (final) | 0.954 | 0.954 | 0 | 0 | +0.0 | 0 / 0 / 15 |
| precision (final) | 0.991 | 0.991 | 0 | 0 | +0.0 | 0 / 0 / 15 |
| published / truth (final) | 17.467 | 17.467 | 0 | 0 | +0.0 | - |
| recall (per-frame mean) | 0.849 | 0.849 | 0 | 0 | +0.0 | 0 / 0 / 15 |
| mean IoU (final) | 0.773 | 0.773 | 0 | 0 | +0.0 | 0 / 0 / 15 |
| min IoU (final) | 0.586 | 0.586 | 0 | 0 | +0.0 | 0 / 0 / 15 |
| merged entries (final) | 1.067 | 1.067 | 0 | 0 | +0.0 | 0 / 0 / 15 |
| neighbours swallowed (final) | 1.067 | 1.067 | 0 | 0 | +0.0 | 0 / 0 / 15 |
| split trees (final) | 0.200 | 0.200 | 0 | 0 | +0.0 | 0 / 0 / 15 |
| false trees (final) | 0 | 0 | 0 | 0 | +0.0 | 0 / 0 / 15 |
| centre RMSE m (final) | 0.426 | 0.426 | 0 | 0 | +0.0 | 0 / 0 / 15 |
| height bias m (final) | 0.055 | 0.055 | 0 | 0 | +0.0 | - |
| height RMSE m (final) | 0.155 | 0.155 | 0 | 0 | +0.0 | 0 / 0 / 15 |
| spill (final) | 0.151 | 0.151 | 0 | 0 | +0.0 | 0 / 0 / 15 |
| neighbour spill max (final) | 0.198 | 0.198 | 0 | 0 | +0.0 | 0 / 0 / 15 |
| ID switches after lock | 0 | 0 | 0 | 0 | +0.0 | 0 / 0 / 15 |
| geometry retreats | 0 | 0 | 0 | 0 | +0.0 | 0 / 0 / 15 |
| centre jumps | 0 | 0 | 0 | 0 | +0.0 | 0 / 0 / 15 |
| height jumps | 0 | 0 | 0 | 0 | +0.0 | 0 / 0 / 15 |
| IDs vanished while held by nav | 0 | 0 | 0 | 0 | +0.0 | 0 / 0 / 15 |
| SAFETY total violations | 0 | 0 | 0 | 0 | +0.0 | 0 / 0 / 15 |
|   undefined-height frames | 0 | 0 | 0 | 0 | +0.0 | 0 / 0 / 15 |
|   new trees after scan | 0 | 0 | 0 | 0 | +0.0 | 0 / 0 / 15 |
|   non-target changes in descent | 0 | 0 | 0 | 0 | +0.0 | 0 / 0 / 15 |
|   sprayed-tree changes | 0 | 0 | 0 | 0 | +0.0 | 0 / 0 / 15 |
|   new tree inside sprayed | 0 | 0 | 0 | 0 | +0.0 | 0 / 0 / 15 |
|   invalid-entry frames | 0 | 0 | 0 | 0 | +0.0 | 0 / 0 / 15 |
|   duplicate-ID frames | 0 | 0 | 0 | 0 | +0.0 | 0 / 0 / 15 |
|   implausible-height frames | 0 | 0 | 0 | 0 | +0.0 | 0 / 0 / 15 |
|   locked ID vanished | 0 | 0 | 0 | 0 | +0.0 | 0 / 0 / 15 |
| truth trees sprayed | 4.867 | 4.867 | 0 | 0 | +0.0 | 0 / 0 / 15 |
| truth trees sprayed twice | 0 | 0 | 0 | 0 | +0.0 | 0 / 0 / 15 |
| scan latency mean s | 1.795 | 1.795 | 0 | 0 | +0.0 | 0 / 0 / 15 |

Mean paired difference per group:

| group (n) | recall (final) | precision (final) | mean IoU (final) | neighbours swallowed (final) | false trees (final) | centre RMSE m (final) | height RMSE m (final) | spill (final) | SAFETY total violations |
|---|---|---|---|---|---|---|---|---|---|
| easy (5) | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 |
| hard (5) | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 |
| medium (5) | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 |

Per seed (after - before):

| dataset | recall (final) | precision (final) | mean IoU (final) | neighbours swallowed (final) | false trees (final) | centre RMSE m (final) | height RMSE m (final) | spill (final) | SAFETY total violations |
|---|---|---|---|---|---|---|---|---|---|
| easy_s200 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 |
| easy_s201 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 |
| easy_s202 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 |
| easy_s203 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 |
| easy_s204 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 |
| hard_s200 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 |
| hard_s201 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 |
| hard_s202 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 |
| hard_s203 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 |
| hard_s204 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 |
| medium_s200 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 |
| medium_s201 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 |
| medium_s202 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 |
| medium_s203 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 |
| medium_s204 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 |


Run-to-run max diff (TM_REPEAT) before / after, per metric:

- recall (final): 0.0 / None
- precision (final): 0.0 / None
- published / truth (final): 0.0 / None
- recall (per-frame mean): 0.0 / None
- mean IoU (final): 0.0 / None
- min IoU (final): 0.0 / None
- merged entries (final): 0.0 / None
- neighbours swallowed (final): 0.0 / None
- split trees (final): 0.0 / None
- false trees (final): 0.0 / None
- centre RMSE m (final): 0.0 / None
- height bias m (final): 0.0 / None
- height RMSE m (final): 0.0 / None
- spill (final): 0.0 / None
- neighbour spill max (final): 0.0 / None
- ID switches after lock: 0.0 / None
- geometry retreats: 0.0 / None
- centre jumps: 0.0 / None
- height jumps: 0.0 / None
- IDs vanished while held by nav: 0.0 / None
- SAFETY total violations: 0.0 / None
-   undefined-height frames: 0.0 / None
-   new trees after scan: 0.0 / None
-   non-target changes in descent: 0.0 / None
-   sprayed-tree changes: 0.0 / None
-   new tree inside sprayed: 0.0 / None
-   invalid-entry frames: 0.0 / None
-   duplicate-ID frames: 0.0 / None
-   implausible-height frames: 0.0 / None
-   locked ID vanished: 0.0 / None
- truth trees sprayed: 0.0 / None
- truth trees sprayed twice: 0.0 / None
- scan latency mean s: 0.0 / None
