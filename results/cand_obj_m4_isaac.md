# cand_obj M4 on the recorded Isaac mission (9 trees, 7 hops)

> Caveat (final review): the recording was **used during tuning** (M3 and M4a tuned on it); these numbers are not an
> unbiased test. Machine-readable numbers: `results/cand_obj_m4_isaac.json`.

Systems: `simple`, `cand_obj M3` (frozen build of commit 89317b2, verified to reproduce the M3 Isaac map byte-for-byte),
`cand_obj M4` (this milestone, commit f8a78c0, `config/cand_obj.cfg` == built-in defaults), `cand_grid` (unchanged since M3).
Runs: `treemapper_tools.evaluate` with the M4 build (`/data/work/build_m4a`) for simple / cand_obj M4 / cand_grid and the frozen build
(`/data/work/build_m4a_m3`) for cand_obj M3; raw outputs under `/data/work/eval/m4a/final` and `/data/work/eval/m4a/final_m3`.
Worst = worst seed (min for recall / precision / IoU / sprayed, max otherwise). Paired rows: diff = M4 - other per seed,
t = mean / (std / sqrt n), |t| < 2 is within seed noise; W / L / T counts seeds where M4 is better / worse / equal.
Run-to-run noise is 0 (deterministic), so every difference between builds on one seed is real; seed-to-seed spread is the noise floor.

Height error rows are n/a by design (truth heights are a constant 4.0 m fallback; measured canopy tops 5.4-5.9 m, ).

### Recorded Isaac mission

| Metric | simple | cand_obj M3 | cand_obj M4 | cand_grid | existing |
|---|---|---|---|---|---|
| recall (final) | 0.889 | 1 | 1 | 1 | 0.778 |
| precision (final) | 1 | 1 | 1 | 1 | 1 |
| recall (per-frame mean) | 0.636 | 0.739 | 0.738 | 0.738 | 0.580 |
| mean IoU (final) | 0.730 | 0.859 | 0.867 | 0.850 | 0.629 |
| min IoU (final) | 0.462 | 0.779 | 0.817 | 0.746 | 0.393 |
| neighbours swallowed (final) | 1 | 0 | 0 | 0 | 2 |
| split trees (final) | 0 | 0 | 0 | 0 | 0 |
| false trees (final) | 0 | 0 | 0 | 0 | 0 |
| centre RMSE m (final) | 1.103 | 0.213 | 0.191 | 0.191 | 1.355 |
| height RMSE m (final) | n/a | n/a | n/a | n/a | n/a |
| height bias m (final) | n/a | n/a | n/a | n/a | n/a |
| spill (final) | 0.141 | 0.052 | 0.064 | 0.093 | 0.164 |
| neighbour spill max (final) | 0.378 | 0.022 | 0.056 | 0.077 | 0.415 |
| centre jumps | 8 | 7 | 0 | 0 | 8 |
| height jumps | 3 | 4 | 0 | 0 | 19 |
| ID switches after lock | 0 | 2 | 0 | 0 | 0 |
| SAFETY total violations | 0 | 0 | 0 | 0 | 5 |
| implausible-height frames | 0 | 0 | 0 | 0 | 0 |
| truth trees sprayed | 7 | 7 | 7 | 7 | 7 |
| scan latency mean s | 2.386 | 2.230 | 2.319 | 2.297 | 1.190 |
| max height drift since lock m (worst) | 0.366 | 0.030 | 0 | 1.751 | 4.212 |
| max centre drift since lock m (worst) | 1.129 | 0.204 | 0.235 | 0.691 | 1.443 |
| target height range in own descent m (worst) | 0.295 | 0.019 | 0 | 0.641 | 1.840 |
| matched trees with abs height error > 1 m (of which under-estimates) / matched | 6 (0) / 8 | 5 (0) / 9 | 6 (0) / 9 | 6 (0) / 9 | 6 (0) / 7 |
| max abs height error m | 1.697 | 1.826 | 1.740 | 1.764 | 1.892 |
| cost ms / frame, mean (worst max) | 0.615 (0.980) | 0.842 (2.863) | 0.860 (1.118) | 0.580 (1.950) | n/a |

- simple final heights (m): 4.77, 4.99, 5.01, 5.53, 5.60, 5.60, 5.66, 5.70
- cand_obj M3 final heights (m): 4.27, 4.39, 4.78, 4.95, 5.03, 5.65, 5.78, 5.83, 5.83
- cand_obj M4 final heights (m): 4.83, 4.85, 4.89, 5.47, 5.56, 5.58, 5.62, 5.62, 5.74
- cand_grid final heights (m): 4.75, 4.80, 4.84, 5.04, 5.35, 5.51, 5.73, 5.75, 5.76

