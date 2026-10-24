# M3 comparison: cand_obj vs cand_grid (vs simple, vs existing)

Final M3 selection, 2026-10-24. Both candidates frozen (`config/cand_obj.cfg`, `config/cand_grid.cfg`, commit d5ce8be);
nothing was tuned here. Scorer: `treemapper_tools.score` with R14 (truth-aware implausible-height test, below).

- **Held-out suite** `/data/work/holdout/suite_holdout`: 30 missions, seeds 100-109 x easy / medium / hard, never seen by either
  candidate. Run with `treemapper_tools.evaluate --system simple,cand_obj,cand_grid --reference simple` once per level
  (outputs `/data/work/eval/m3_select/holdout_{easy,medium,hard}.{md,json}`), merged and paired by
  `python3 -m treemapper_tools.m3_compare` (also cand_grid vs cand_obj, from the same runs). Raw: `results/m3_comparison.json`.
- **Recorded Isaac mission**, all three plus `existing`: `/data/work/eval/m3_select/isaac.{md,json}`.
- Run-to-run noise is 0 (deterministic); seed-to-seed is the noise floor, hence paired by seed. |t| < 2 = within seed noise.

## Decision: **cand_obj** goes to M4 (close call)

Safety (criterion a) is a tie: **0 violations for both on all 30 held-out missions and on the recording**
(simple: 1670 implausible-height frames on 11 missions, real over-estimates / heights of non-trees above the drone;
existing: 5 on the recording). On medium (b) the two are within seed noise on almost everything: cand_obj has
slightly higher recall (+0.016, 3/0/7, t 2.0) and per-frame recall (+0.024, 9/0/1); cand_grid has slightly
lower spill (-0.016, 8/2, t -3.3), higher min IoU (+0.10, 8/2) and a better height tail (0 of 175 matched trees
off by > 1 m vs 4 of 178 for cand_obj, max 0.93 vs 2.53 m). Swallowed neighbours, false trees and centre error are ties.
**Medium does not separate them.** What decided it, in order:

1. **Worst cases (c), hard level**: cand_obj recall 0.866 vs 0.758 (9/1, t 4.8; worst seed 0.815 vs 0.679),
   neighbours swallowed 3.6 vs 5.3 (8/1), false trees 0 vs 0.8 (worst 0 vs 3), precision 0.987 vs 0.952 (7/0).
   cand_grid has the better hard IoU (+0.047, 10/0) and height RMSE (7/3). Missing, merged and invented trees are
   mission failures (unsprayed tree, a double target, a descent onto nothing); a few cm of IoU are not.
2. **Descent reference on the recording** (the only non-dome canopies we have): after the list locks, cand_grid's
   published heights move by up to 1.75 m and mostly **downwards** (ID 3: 4.91 m at lock, 3.16 m minimum, 4.80 m final;
   ID 7 5.00 -> 3.82; ID 8 5.57 -> 4.24 min), and its target moves up to 0.64 m during its own descent. cand_obj's
   locked heights move <= 0.03 m and its target <= 0.02 m. A target height that sinks while the drone descends is
   the unsafe direction. (cand_obj's own height problem is a static low bias on two Isaac trees, 4.27 / 4.39 m where
   simple and cand_grid say 5.4-5.75 m; it is M4 item 1.)
3. **Realistic-noise geometry**: real stereo (~0.1 px) is closer to easy than to medium. On easy cand_obj wins IoU 10/0
   (0.903 vs 0.872, min IoU 0.848 vs 0.761); on the recording its spill is 0.052 vs 0.093 and neighbour spill 0.022 vs 0.077.
4. **Latency (e)**: cand_obj publishes 0.39 s earlier (23/6/1 over 30 pairs; Isaac 2.23 vs 2.30 s).
5. **Cost / memory / simplicity (f, g)**: per-frame cost equal (~0.9-1.0 ms x86 mean, both); cand_obj memory is bounded
   (~4 kB per live instance), cand_grid's tiled grid grows with area (3 MB per mission, no eviction). cand_obj is
   ~990 lines vs ~1320 and has fewer interacting gates (cand_grid's 1.0 m spread gate is noise-scale dependent).

**Against cand_obj, honestly:** cand_grid is clearly more stable in publication (centre jumps 0.7 vs 7.4 per medium
mission, t -11; height jumps 0 vs 1.1), has the tighter height tail on medium, and its multi-frame DSM with range /
off-axis / rotation-rate weighting is the more principled fit for the real 15 deg off-nadir camera and turn blur.
cand_obj had a transient duplicate on the recording (truth tree 2 matched by ID 11 -> 12 -> 11 within 0.6 s at
178 s, second SCAN; scored as 2 ID switches after lock, no safety count). Those are the M4 work list below; the
detection advantage on the worst seeds and the stable descent reference are harder to retrofit into cand_grid
than the stabilisers are into cand_obj.

## Scorer change (R14): implausible heights judged against the truth top

Both candidates showed `implausible_height_frames` was ill-posed: the generator's descent hover clears only the
**target's** top (z = max(ground + 7 m, target top + 1.5 m)), so a taller neighbour or one on higher ground can top out
above the hovering drone (e.g. hard_s100 at 61.4 s: drone 7.31 m, neighbour top 7.48 m), and accurate heights were
flagged. With reliable truth heights an above-the-drone height now counts only if it is also above the owning truth
top + 0.5 m (owner = tallest truth tree holding >= 20 % of the entry; no owner = plain test); `<= 0` always counts;
without truth, or with unreliable truth (Isaac), the definition is unchanged. Effect on the held-out suite
(old -> new frames): cand_obj hard 624 -> 0 (10 missions -> 0), cand_grid hard 527 -> 0 (9 -> 0), simple unchanged
(1605 hard + 65 medium: its flagged entries are owned by no truth tree or are > 0.5 m over the truth). Excused entries:
cand_obj 991, cand_grid 1053, simple 78. Existing-system baseline and its pin: unchanged (Isaac truth is unreliable).
The generator flaw stays (known issue for M4, D30); no suite was regenerated.

## M4 work list (cand_obj hardening; improvements to carry over from cand_grid)

1. **Height statistic** (descent reference): replace the median of the crown-top cell with a high quantile of the
   latched cells' surface (cand_grid: q 0.97 of fused cell heights) and check the two low Isaac trees (4.27 / 4.39 m)
   against the measured 5.4-5.9 m tops; keep the median-of-samples freeze so locked heights do not move. Investigate
   the +-2.5 m tails (medium_s105 trees 7 / 18, IoU 0.25 / 0.40: a crown top sampled from the neighbour).
2. **Publication stability** (from cand_grid): rate-limit published centre and height motion (cand_grid
   `max_height_rate` 0.5 m/s, 0 height jumps) and/or require the crown's peak neighbourhood observed before publishing,
   to cut cand_obj's 7.4 centre jumps per medium mission; measure the latency cost.
3. **Relative-height latch** (from cand_grid `latch_rel_h` 0.6 x peak CHM): trims the low skirt; cand_grid's medium
   spill / neighbour spill max 0.153 / 0.234 vs cand_obj 0.169 / 0.306.
4. **Point and frame weighting** (from cand_grid): range / off-axis weight and rotation-rate frame weight for turn blur;
   most relevant for the real 15 deg off-nadir camera.
5. **Transient duplicate at crown entry** (Isaac 178 s): a fragment instance briefly out-matches the established one;
   M4 must test that a tentative or new instance can never take over a published tree's match.
6. **Generator fix (D30)**: hover altitude = max over trees within the footprint / rotor reach of top + clearance, not
   only the target's; then regenerate a suite for M4 (keep the held-out suite untouched).
7. Keep cand_obj's bounded memory; verify on the Jetson (all timings here are x86).

## Tables
### easy (n = 10 held-out seeds): mean / worst

| Metric | simple | cand_obj | cand_grid |
|---|---|---|---|
| recall (final) | 0.966 / 0.867 | 1 / 1 | 1 / 1 |
| precision (final) | 0.861 / 0.778 | 0.993 / 0.929 | 1 / 1 |
| recall (per-frame mean) | 0.902 / 0.790 | 0.899 / 0.886 | 0.887 / 0.880 |
| mean IoU (final) | 0.583 / 0.551 | 0.903 / 0.895 | 0.872 / 0.848 |
| min IoU (final) | 0.274 / 0.167 | 0.848 / 0.775 | 0.761 / 0.661 |
| neighbours swallowed (final) | 1.800 / 3 | 0 / 0 | 0 / 0 |
| split trees (final) | 1.500 / 2 | 0.100 / 1 | 0 / 0 |
| false trees (final) | 0.800 / 2 | 0 / 0 | 0 / 0 |
| centre RMSE m (final) | 1.048 / 1.249 | 0.141 / 0.162 | 0.153 / 0.262 |
| height RMSE m (final) | 0.169 / 0.240 | 0.050 / 0.068 | 0.056 / 0.110 |
| height bias m (final) | -0.064 / n/a | 0.019 / n/a | -0.005 / n/a |
| spill (final) | 0.434 / 0.482 | 0.060 / 0.071 | 0.064 / 0.079 |
| neighbour spill max (final) | 0.208 / 0.304 | 0.000 / 0.003 | 0.017 / 0.087 |
| geometry retreats | 0 / 0 | 0 / 0 | 0 / 0 |
| centre jumps | 7.500 / 11 | 2.500 / 6 | 0 / 0 |
| height jumps | 0 / 0 | 0.100 / 1 | 0 / 0 |
| SAFETY total violations | 0 / 0 | 0 / 0 | 0 / 0 |
| implausible-height frames | 0 / 0 | 0 / 0 | 0 / 0 |
| truth trees sprayed | 3.600 / 3 | 4 / 4 | 4 / 4 |
| scan latency mean s | 0.564 / 0.629 | 1.172 / 1.247 | 1.400 / 1.600 |
| max centre drift since lock m (worst seed) | 0.857 | 0.139 | 0.305 |
| max height drift since lock m (worst seed) | 0.865 | 0.173 | 0.254 |
| matched trees with abs height error > 1 m (of which under-estimates) / matched | 0 (0) / 141 | 0 (0) / 146 | 0 (0) / 146 |
| max abs height error m (any matched tree) | 0.651 | 0.159 | 0.240 |
| target height range during its own descent m (worst seed) | 0.459 | 0.150 | 0.167 |
| cost ms / frame, mean (worst snapshot max) | 0.544 (1.045) | 0.903 (1.250) | 0.806 (1.700) |

### medium (n = 10 held-out seeds): mean / worst

| Metric | simple | cand_obj | cand_grid |
|---|---|---|---|
| recall (final) | 0.393 / 0.235 | 0.977 / 0.938 | 0.961 / 0.882 |
| precision (final) | 0.478 / 0.250 | 0.979 / 0.895 | 0.989 / 0.938 |
| recall (per-frame mean) | 0.383 / 0.225 | 0.870 / 0.826 | 0.846 / 0.787 |
| mean IoU (final) | 0.315 / 0.199 | 0.755 / 0.715 | 0.771 / 0.758 |
| min IoU (final) | 0.151 / 0.111 | 0.430 / 0.147 | 0.529 / 0.423 |
| neighbours swallowed (final) | 14.300 / 18 | 1 / 3 | 0.700 / 2 |
| split trees (final) | 0.100 / 1 | 0.400 / 2 | 0.200 / 1 |
| false trees (final) | 10 / 15 | 0 / 0 | 0 / 0 |
| centre RMSE m (final) | 2.137 / 3.864 | 0.523 / 0.714 | 0.514 / 0.721 |
| height RMSE m (final) | 0.586 / 1.125 | 0.257 / 0.818 | 0.153 / 0.260 |
| height bias m (final) | 0.114 / n/a | 0.022 / n/a | 0.017 / n/a |
| spill (final) | 0.679 / 0.764 | 0.169 / 0.196 | 0.153 / 0.178 |
| neighbour spill max (final) | 0.362 / 0.516 | 0.306 / 0.526 | 0.234 / 0.393 |
| geometry retreats | 0 / 0 | 0 / 0 | 0 / 0 |
| centre jumps | 15 / 19 | 7.400 / 10 | 0.700 / 2 |
| height jumps | 0.800 / 2 | 1.100 / 3 | 0 / 0 |
| SAFETY total violations | 6.500 / 65 | 0 / 0 | 0 / 0 |
| implausible-height frames | 6.500 / 65 | 0 / 0 | 0 / 0 |
| truth trees sprayed | 2.600 / 1 | 5.100 / 5 | 5.100 / 5 |
| scan latency mean s | 1.178 / 4.031 | 1.847 / 2.920 | 2.383 / 4.440 |
| max centre drift since lock m (worst seed) | 1.998 | 0.469 | 0.850 |
| max height drift since lock m (worst seed) | 0.483 | 1.366 | 0.700 |
| matched trees with abs height error > 1 m (of which under-estimates) / matched | 9 (4) / 72 | 4 (2) / 178 | 0 (0) / 175 |
| max abs height error m (any matched tree) | 2.024 | 2.531 | 0.927 |
| target height range during its own descent m (worst seed) | 0.318 | 0.618 | 0.492 |
| cost ms / frame, mean (worst snapshot max) | 0.496 (16.583) | 0.935 (1.768) | 0.964 (2.283) |

### hard (n = 10 held-out seeds): mean / worst

| Metric | simple | cand_obj | cand_grid |
|---|---|---|---|
| recall (final) | 0.105 / 0 | 0.866 / 0.815 | 0.758 / 0.679 |
| precision (final) | 0.117 / 0 | 0.987 / 0.957 | 0.952 / 0.850 |
| recall (per-frame mean) | 0.114 / 0.007 | 0.777 / 0.724 | 0.678 / 0.609 |
| mean IoU (final) | 0.344 / 0.173 | 0.593 / 0.550 | 0.640 / 0.597 |
| min IoU (final) | 0.268 / 0.108 | 0.275 / 0.132 | 0.375 / 0.304 |
| neighbours swallowed (final) | 29.300 / 41 | 3.600 / 7 | 5.300 / 8 |
| split trees (final) | 0.100 / 1 | 0.500 / 1 | 0.200 / 1 |
| false trees (final) | 18.700 / 26 | 0 / 0 | 0.800 / 3 |
| centre RMSE m (final) | 2.097 / 3.775 | 0.798 / 0.958 | 0.875 / 1.247 |
| height RMSE m (final) | 0.859 / 1.278 | 0.480 / 0.742 | 0.420 / 0.581 |
| height bias m (final) | 0.389 / n/a | 0.109 / n/a | 0.133 / n/a |
| spill (final) | 0.807 / 0.866 | 0.249 / 0.300 | 0.238 / 0.306 |
| neighbour spill max (final) | 0.286 / 0.632 | 0.443 / 0.584 | 0.461 / 0.612 |
| geometry retreats | 0 / 0 | 0 / 0 | 0 / 0 |
| centre jumps | 24 / 29 | 6.300 / 11 | 3.700 / 7 |
| height jumps | 2.900 / 6 | 0.800 / 3 | 0 / 0 |
| SAFETY total violations | 160.500 / 331 | 0 / 0 | 0 / 0 |
| implausible-height frames | 160.500 / 331 | 0 / 0 | 0 / 0 |
| truth trees sprayed | 1.600 / 0 | 5.500 / 3 | 5.800 / 5 |
| scan latency mean s | 0.800 / 1.373 | 2.238 / 3.577 | 2.655 / 5.365 |
| max centre drift since lock m (worst seed) | 4.769 | 1.652 | 0.977 |
| max height drift since lock m (worst seed) | 1.255 | 1.906 | 2 |
| matched trees with abs height error > 1 m (of which under-estimates) / matched | 9 (2) / 27 | 10 (5) / 224 | 5 (3) / 196 |
| max abs height error m (any matched tree) | 2.139 | 2.296 | 2.177 |
| target height range during its own descent m (worst seed) | 0.976 | 1.792 | 1.600 |
| cost ms / frame, mean (worst snapshot max) | 0.441 (1.316) | 0.917 (2.316) | 1.014 (2.605) |

### Paired: cand_obj vs simple (diff = cand_obj - simple; win = cand_obj better)

| Metric | all 30: mean diff (t) | all 30: W / L / T | easy: diff, W/L/T | medium: diff, W/L/T | hard: diff, W/L/T |
|---|---|---|---|---|---|
| recall (final) | +0.459 (+7.7) | 24 / 0 / 6 | +0.034, 4/0/6 | +0.584, 10/0/0 | +0.760, 10/0/0 |
| precision (final) | +0.501 (+8.6) | 30 / 0 / 0 | +0.132, 10/0/0 | +0.501, 10/0/0 | +0.871, 10/0/0 |
| mean IoU (final) | +0.344 (+14.0) | 27 / 1 / 0 | +0.320, 10/0/0 | +0.440, 10/0/0 | +0.254, 7/1/0 |
| neighbours swallowed (final) | -13.600 (-7.0) | 30 / 0 / 0 | -1.800, 10/0/0 | -13.300, 10/0/0 | -25.700, 10/0/0 |
| false trees (final) | -9.833 (-6.8) | 25 / 0 / 5 | -0.800, 5/0/5 | -10, 10/0/0 | -18.700, 10/0/0 |
| centre RMSE m (final) | -1.274 (-8.2) | 27 / 1 / 0 | -0.907, 10/0/0 | -1.614, 10/0/0 | -1.307, 7/1/0 |
| height RMSE m (final) | -0.264 (-4.8) | 25 / 3 / 0 | -0.119, 10/0/0 | -0.329, 9/1/0 | -0.362, 6/2/0 |
| spill (final) | -0.481 (-31.2) | 30 / 0 / 0 | -0.374, 10/0/0 | -0.510, 10/0/0 | -0.558, 10/0/0 |
| centre jumps | -10.100 (-8.5) | 30 / 0 / 0 | -5, 10/0/0 | -7.600, 10/0/0 | -17.700, 10/0/0 |
| height jumps | -0.567 (-1.9) | 12 / 6 / 12 | +0.100, 0/1/9 | +0.300, 3/5/2 | -2.100, 9/0/1 |
| SAFETY total violations | -55.667 (-3.4) | 11 / 0 / 19 | 0, 0/0/10 | -6.500, 1/0/9 | -160.500, 10/0/0 |
| scan latency mean s | +0.905 (+6.6) | 1 / 29 / 0 | +0.607, 0/10/0 | +0.669, 1/9/0 | +1.438, 0/10/0 |

### Paired: cand_grid vs simple (diff = cand_grid - simple; win = cand_grid better)

| Metric | all 30: mean diff (t) | all 30: W / L / T | easy: diff, W/L/T | medium: diff, W/L/T | hard: diff, W/L/T |
|---|---|---|---|---|---|
| recall (final) | +0.418 (+7.9) | 24 / 0 / 6 | +0.034, 4/0/6 | +0.568, 10/0/0 | +0.652, 10/0/0 |
| precision (final) | +0.495 (+8.8) | 30 / 0 / 0 | +0.139, 10/0/0 | +0.511, 10/0/0 | +0.835, 10/0/0 |
| mean IoU (final) | +0.351 (+13.6) | 27 / 1 / 0 | +0.288, 10/0/0 | +0.456, 10/0/0 | +0.297, 7/1/0 |
| neighbours swallowed (final) | -13.133 (-7.4) | 30 / 0 / 0 | -1.800, 10/0/0 | -13.600, 10/0/0 | -24, 10/0/0 |
| false trees (final) | -9.567 (-6.9) | 25 / 0 / 5 | -0.800, 5/0/5 | -10, 10/0/0 | -17.900, 10/0/0 |
| centre RMSE m (final) | -1.252 (-7.6) | 26 / 2 / 0 | -0.894, 10/0/0 | -1.623, 10/0/0 | -1.235, 6/2/0 |
| height RMSE m (final) | -0.323 (-5.9) | 27 / 1 / 0 | -0.113, 10/0/0 | -0.433, 10/0/0 | -0.449, 7/1/0 |
| spill (final) | -0.489 (-27.3) | 30 / 0 / 0 | -0.370, 10/0/0 | -0.526, 10/0/0 | -0.570, 10/0/0 |
| centre jumps | -14.033 (-12.9) | 30 / 0 / 0 | -7.500, 10/0/0 | -14.300, 10/0/0 | -20.300, 10/0/0 |
| height jumps | -1.233 (-4.0) | 15 / 0 / 15 | 0, 0/0/10 | -0.800, 6/0/4 | -2.900, 9/0/1 |
| SAFETY total violations | -55.667 (-3.4) | 11 / 0 / 19 | 0, 0/0/10 | -6.500, 1/0/9 | -160.500, 10/0/0 |
| scan latency mean s | +1.298 (+6.3) | 1 / 29 / 0 | +0.836, 0/10/0 | +1.205, 1/9/0 | +1.854, 0/10/0 |

### Paired: cand_grid vs cand_obj (diff = cand_grid - cand_obj; win = cand_grid better)

| Metric | all 30: mean diff (t) | all 30: W / L / T | easy: diff, W/L/T | medium: diff, W/L/T | hard: diff, W/L/T |
|---|---|---|---|---|---|
| recall (final) | -0.041 (-3.5) | 1 / 12 / 17 | 0, 0/0/10 | -0.016, 0/3/7 | -0.108, 1/9/0 |
| precision (final) | -0.006 (-0.7) | 4 / 9 / 17 | +0.007, 1/0/9 | +0.010, 3/2/5 | -0.035, 0/7/3 |
| mean IoU (final) | +0.010 (+1.3) | 17 / 13 / 0 | -0.032, 0/10/0 | +0.016, 7/3/0 | +0.047, 10/0/0 |
| neighbours swallowed (final) | +0.467 (+1.6) | 5 / 11 / 14 | 0, 0/0/10 | -0.300, 4/3/3 | +1.700, 1/8/1 |
| false trees (final) | +0.267 (+2.1) | 0 / 5 / 25 | 0, 0/0/10 | 0, 0/0/10 | +0.800, 0/5/5 |
| centre RMSE m (final) | +0.026 (+0.8) | 14 / 16 / 0 | +0.012, 5/5/0 | -0.009, 5/5/0 | +0.077, 4/6/0 |
| height RMSE m (final) | -0.053 (-2.1) | 19 / 11 / 0 | +0.006, 4/6/0 | -0.103, 8/2/0 | -0.061, 7/3/0 |
| spill (final) | -0.008 (-1.3) | 18 / 12 / 0 | +0.004, 3/7/0 | -0.016, 8/2/0 | -0.012, 7/3/0 |
| centre jumps | -3.933 (-6.9) | 27 / 2 / 1 | -2.500, 10/0/0 | -6.700, 10/0/0 | -2.600, 7/2/1 |
| height jumps | -0.667 (-4.1) | 14 / 0 / 16 | -0.100, 1/0/9 | -1.100, 8/0/2 | -0.800, 5/0/5 |
| SAFETY total violations | 0 (+0.0) | 0 / 0 / 30 | 0, 0/0/10 | 0, 0/0/10 | 0, 0/0/10 |
| scan latency mean s | +0.394 (+3.0) | 6 / 23 / 1 | +0.228, 0/10/0 | +0.536, 0/9/1 | +0.417, 6/4/0 |

### Per seed, cand_obj | cand_grid

| dataset | recall | precision | mean IoU | neighbours swallowed | false trees | height RMSE m | spill | SAFETY total violations |
|---|---|---|---|---|---|---|---|---|
| easy_s100 | 1 \| 1 | 1 \| 1 | 0.907 \| 0.848 | 0 \| 0 | 0 \| 0 | 0.065 \| 0.051 | 0.057 \| 0.078 | 0 \| 0 |
| easy_s101 | 1 \| 1 | 1 \| 1 | 0.901 \| 0.874 | 0 \| 0 | 0 \| 0 | 0.057 \| 0.036 | 0.067 \| 0.063 | 0 \| 0 |
| easy_s102 | 1 \| 1 | 1 \| 1 | 0.899 \| 0.882 | 0 \| 0 | 0 \| 0 | 0.029 \| 0.054 | 0.071 \| 0.059 | 0 \| 0 |
| easy_s103 | 1 \| 1 | 0.929 \| 1 | 0.908 \| 0.889 | 0 \| 0 | 0 \| 0 | 0.066 \| 0.047 | 0.056 \| 0.058 | 0 \| 0 |
| easy_s104 | 1 \| 1 | 1 \| 1 | 0.903 \| 0.861 | 0 \| 0 | 0 \| 0 | 0.034 \| 0.041 | 0.060 \| 0.071 | 0 \| 0 |
| easy_s105 | 1 \| 1 | 1 \| 1 | 0.906 \| 0.888 | 0 \| 0 | 0 \| 0 | 0.022 \| 0.032 | 0.058 \| 0.047 | 0 \| 0 |
| easy_s106 | 1 \| 1 | 1 \| 1 | 0.895 \| 0.857 | 0 \| 0 | 0 \| 0 | 0.064 \| 0.110 | 0.067 \| 0.079 | 0 \| 0 |
| easy_s107 | 1 \| 1 | 1 \| 1 | 0.901 \| 0.861 | 0 \| 0 | 0 \| 0 | 0.068 \| 0.067 | 0.061 \| 0.066 | 0 \| 0 |
| easy_s108 | 1 \| 1 | 1 \| 1 | 0.901 \| 0.865 | 0 \| 0 | 0 \| 0 | 0.042 \| 0.067 | 0.055 \| 0.067 | 0 \| 0 |
| easy_s109 | 1 \| 1 | 1 \| 1 | 0.914 \| 0.890 | 0 \| 0 | 0 \| 0 | 0.053 \| 0.053 | 0.050 \| 0.056 | 0 \| 0 |
| hard_s100 | 0.880 \| 0.800 | 0.957 \| 0.952 | 0.600 \| 0.662 | 3 \| 3 | 0 \| 1 | 0.742 \| 0.581 | 0.300 \| 0.283 | 0 \| 0 |
| hard_s101 | 0.880 \| 0.760 | 1 \| 1 | 0.596 \| 0.609 | 2 \| 7 | 0 \| 0 | 0.397 \| 0.556 | 0.290 \| 0.245 | 0 \| 0 |
| hard_s102 | 0.815 \| 0.704 | 0.957 \| 0.950 | 0.593 \| 0.636 | 4 \| 7 | 0 \| 1 | 0.570 \| 0.375 | 0.231 \| 0.216 | 0 \| 0 |
| hard_s103 | 0.917 \| 0.708 | 1 \| 0.850 | 0.622 \| 0.656 | 4 \| 6 | 0 \| 3 | 0.441 \| 0.486 | 0.223 \| 0.306 | 0 \| 0 |
| hard_s104 | 0.875 \| 0.750 | 1 \| 0.947 | 0.572 \| 0.657 | 2 \| 3 | 0 \| 0 | 0.475 \| 0.237 | 0.263 \| 0.155 | 0 \| 0 |
| hard_s105 | 0.926 \| 0.741 | 1 \| 0.952 | 0.606 \| 0.633 | 7 \| 8 | 0 \| 1 | 0.583 \| 0.409 | 0.208 \| 0.246 | 0 \| 0 |
| hard_s106 | 0.840 \| 0.880 | 1 \| 1 | 0.578 \| 0.639 | 5 \| 3 | 0 \| 0 | 0.416 \| 0.399 | 0.225 \| 0.211 | 0 \| 0 |
| hard_s107 | 0.815 \| 0.778 | 1 \| 0.913 | 0.618 \| 0.641 | 4 \| 6 | 0 \| 2 | 0.450 \| 0.545 | 0.248 \| 0.292 | 0 \| 0 |
| hard_s108 | 0.821 \| 0.679 | 1 \| 1 | 0.550 \| 0.669 | 4 \| 5 | 0 \| 0 | 0.435 \| 0.361 | 0.231 \| 0.167 | 0 \| 0 |
| hard_s109 | 0.889 \| 0.778 | 0.960 \| 0.955 | 0.595 \| 0.597 | 1 \| 5 | 0 \| 0 | 0.296 \| 0.251 | 0.274 \| 0.255 | 0 \| 0 |
| medium_s100 | 0.941 \| 0.882 | 1 \| 0.938 | 0.715 \| 0.793 | 1 \| 2 | 0 \| 0 | 0.170 \| 0.131 | 0.162 \| 0.137 | 0 \| 0 |
| medium_s101 | 0.938 \| 0.938 | 1 \| 1 | 0.750 \| 0.758 | 3 \| 1 | 0 \| 0 | 0.317 \| 0.104 | 0.148 \| 0.149 | 0 \| 0 |
| medium_s102 | 1 \| 1 | 1 \| 1 | 0.815 \| 0.782 | 0 \| 0 | 0 \| 0 | 0.199 \| 0.165 | 0.160 \| 0.170 | 0 \| 0 |
| medium_s103 | 1 \| 1 | 1 \| 1 | 0.786 \| 0.765 | 0 \| 0 | 0 \| 0 | 0.149 \| 0.149 | 0.190 \| 0.178 | 0 \| 0 |
| medium_s104 | 1 \| 0.947 | 0.950 \| 1 | 0.753 \| 0.762 | 0 \| 1 | 0 \| 0 | 0.076 \| 0.166 | 0.155 \| 0.133 | 0 \| 0 |
| medium_s105 | 0.950 \| 0.900 | 1 \| 1 | 0.720 \| 0.760 | 2 \| 2 | 0 \| 0 | 0.818 \| 0.260 | 0.169 \| 0.168 | 0 \| 0 |
| medium_s106 | 0.944 \| 0.944 | 0.944 \| 1 | 0.744 \| 0.778 | 1 \| 0 | 0 \| 0 | 0.148 \| 0.135 | 0.196 \| 0.161 | 0 \| 0 |
| medium_s107 | 1 \| 1 | 0.895 \| 1 | 0.754 \| 0.790 | 1 \| 0 | 0 \| 0 | 0.455 \| 0.258 | 0.171 \| 0.143 | 0 \| 0 |
| medium_s108 | 1 \| 1 | 1 \| 0.950 | 0.779 \| 0.759 | 0 \| 1 | 0 \| 0 | 0.163 \| 0.086 | 0.157 \| 0.142 | 0 \| 0 |
| medium_s109 | 1 \| 1 | 1 \| 1 | 0.734 \| 0.763 | 2 \| 0 | 0 \| 0 | 0.072 \| 0.078 | 0.181 \| 0.147 | 0 \| 0 |

### Non-zero safety counts (held-out)

| system | dataset | counts | implausible events (first) |
|---|---|---|---|
| simple | hard_s100 | {"implausible_height_frames": 110} | (unowned / over-estimated heights above the drone) |
| simple | hard_s101 | {"implausible_height_frames": 218} | (unowned / over-estimated heights above the drone) |
| simple | hard_s102 | {"implausible_height_frames": 215} | (unowned / over-estimated heights above the drone) |
| simple | hard_s103 | {"implausible_height_frames": 209} | (unowned / over-estimated heights above the drone) |
| simple | hard_s104 | {"implausible_height_frames": 331} | (unowned / over-estimated heights above the drone) |
| simple | hard_s105 | {"implausible_height_frames": 158} | (unowned / over-estimated heights above the drone) |
| simple | hard_s106 | {"implausible_height_frames": 45} | (unowned / over-estimated heights above the drone) |
| simple | hard_s107 | {"implausible_height_frames": 83} | (unowned / over-estimated heights above the drone) |
| simple | hard_s108 | {"implausible_height_frames": 117} | (unowned / over-estimated heights above the drone) |
| simple | hard_s109 | {"implausible_height_frames": 119} | (unowned / over-estimated heights above the drone) |
| simple | medium_s102 | {"implausible_height_frames": 65} | (unowned / over-estimated heights above the drone) |

Excused accurate above-the-drone entries (R14), total over 30 missions: simple 78, cand_obj 991, cand_grid 1053

### Recorded Isaac mission (fixed scorer)

| Metric | simple | cand_obj | cand_grid | existing |
|---|---|---|---|---|
| recall (final) | 0.889 | 1 | 1 | 0.778 |
| precision (final) | 1 | 1 | 1 | 1 |
| published / truth (final) | 8 | 9 | 9 | 7 |
| recall (per-frame mean) | 0.636 | 0.739 | 0.738 | 0.580 |
| mean IoU (final) | 0.730 | 0.859 | 0.850 | 0.629 |
| min IoU (final) | 0.462 | 0.779 | 0.746 | 0.393 |
| merged entries (final) | 1 | 0 | 0 | 2 |
| neighbours swallowed (final) | 1 | 0 | 0 | 2 |
| split trees (final) | 0 | 0 | 0 | 0 |
| false trees (final) | 0 | 0 | 0 | 0 |
| centre RMSE m (final) | 1.103 | 0.213 | 0.191 | 1.355 |
| height bias m (final) | n/a | n/a | n/a | n/a |
| height RMSE m (final) | n/a | n/a | n/a | n/a |
| spill (final) | 0.141 | 0.052 | 0.093 | 0.164 |
| neighbour spill max (final) | 0.378 | 0.022 | 0.077 | 0.415 |
| ID switches after lock | 0 | 2 | 0 | 0 |
| geometry retreats | 0 | 0 | 0 | 0 |
| centre jumps | 8 | 7 | 0 | 8 |
| height jumps | 3 | 4 | 0 | 19 |
| IDs vanished while held by nav | 0 | 0 | 0 | 1 |
| SAFETY total violations | 0 | 0 | 0 | 5 |
| undefined-height frames | 0 | 0 | 0 | 0 |
| new trees after scan | 0 | 0 | 0 | 0 |
| non-target changes in descent | 0 | 0 | 0 | 3 |
| sprayed-tree changes | 0 | 0 | 0 | 1 |
| new tree inside sprayed | 0 | 0 | 0 | 0 |
| invalid-entry frames | 0 | 0 | 0 | 0 |
| duplicate-ID frames | 0 | 0 | 0 | 0 |
| implausible-height frames | 0 | 0 | 0 | 0 |
| locked ID vanished | 0 | 0 | 0 | 1 |
| truth trees sprayed | 7 | 7 | 7 | 7 |
| truth trees sprayed twice | 0 | 0 | 0 | 0 |
| scan latency mean s | 2.386 | 2.230 | 2.297 | 1.190 |
| max centre / height drift since lock m | 1.129 / 0.366 | 0.204 / 0.030 | 0.691 / 1.751 | n/a |
| target height range during its own descent m (max) | 0.295 | 0.019 | 0.641 | n/a |
| cost ms / frame mean (max) | 0.621 (0.939) | 0.869 (1.396) | 0.613 (1.917) | n/a |

